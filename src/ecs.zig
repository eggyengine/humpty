//! humpty in a clutch world. Entities with a `Collider` (and a `RigidBody` to move) get a body
//! in the physics world, a clutch resource; their handles live in a `Link` component, freed by
//! an `onRemove` hook whether it is removed or the entity despawned. Systems in `PhysicsStage`,
//! right after `Update`, build and rebuild what changed, step, move the host's transforms, and
//! report collisions as `CollidingEntities`, the resource's `started`/`ended` lists and events.
//!
//!     humpty.ecs.Plugin(.three, f32, MyHost).install(&world, gpa);
//!
//! The components are plain `f32` data, the same in 2D and 3D, so an editor can list and save
//! them whatever the world is; 2D reads x and y, and turns about z.
const std = @import("std");
const clutch = @import("clutch");
const emath = @import("eggenvector");
const physics_zig = @import("physics.zig");
const space = @import("space.zig");
const BodyKind = @import("body.zig").BodyKind;
const BodyHandle = @import("body.zig").BodyHandle;
const ColliderHandle = @import("collider.zig").ColliderHandle;

const Vec3 = emath.Vec3;
const log = std.log.scoped(.physics);

pub const Precision = enum { f32, f64 };

/// How a scene's physics runs. Put one on the scene's root entity; the world is built from the
/// first found when physics starts, so `dimension` and `precision` take effect from then.
pub const PhysicsSettings = struct {
    /// Marks it in the editor (nf-md-earth).
    pub const icon = "\u{f01e7}";
    pub const category = "Physics";
    dimension: space.Dim = .three,
    precision: Precision = .f32,
    gravity_mps2: Vec3 = .init(0, -9.81, 0),
    steps_per_second: f32 = 60,
};

/// Makes its entity a body the physics world moves, from its `Collider`'s shape.
pub const RigidBody = struct {
    /// Marks it in the editor (nf-md-cube_send).
    pub const icon = "\u{f0a6e}";
    pub const category = "Physics";
    pub const dependencies = .{Collider};
    kind: BodyKind = .dynamic,
    mass_kg: f32 = 1,
    linear_damping: f32 = 0,
    angular_damping: f32 = 0,
    gravity_scale: f32 = 1,
};

/// The shape its entity touches the world with. Alone, it is static scenery.
pub const Collider = struct {
    /// Marks it in the editor (nf-md-shape_outline).
    pub const icon = "\u{f0832}";
    pub const category = "Physics";
    shape: Shape = .{ .cuboid = .{} },
    /// Where the shape sits from the entity, before its scale.
    offset_m: Vec3 = .zero,
    restitution: f32 = 0.2,
    friction: f32 = 0.5,
    /// Reports what it overlaps without pushing anything.
    sensor: bool = false,

    /// Every shape before the entity's scale; along y where it has an axis.
    pub const Shape = union(enum) {
        ball: struct { radius_m: f32 = 0.5 },
        cuboid: struct { half_extents_m: Vec3 = .splat(0.5) },
        capsule: struct { half_height_m: f32 = 0.5, radius_m: f32 = 0.25 },
        /// 3D only.
        cylinder: struct { half_height_m: f32 = 0.5, radius_m: f32 = 0.5 },
        /// 3D only.
        cone: struct { half_height_m: f32 = 0.5, radius_m: f32 = 0.5 },
        /// Everything below the plane through the entity facing `normal`: ground.
        half_space: struct { normal: Vec3 = .init(0, 1, 0) },
        /// The convex shape around the entity's mesh.
        convex_hull_from_mesh,
        /// The entity's mesh itself, triangle by triangle: 3D static scenery only.
        mesh,
    };
};

/// How fast its body moves, in m/s: set it to throw the body, read it to see.
pub const LinearVelocity = struct {
    pub const category = "Physics";
    value: Vec3 = .zero,
};

/// How fast its body turns, in rad/s: about each axis in 3D, about z in 2D.
pub const AngularVelocity = struct {
    pub const category = "Physics";
    value: Vec3 = .zero,
};

/// The entities its collider touches, kept up to date by physics.
pub const CollidingEntities = struct {
    pub const category = "Physics";
    entities: [8]?clutch.EntityId = @splat(null),

    fn add(self: *CollidingEntities, entity: clutch.EntityId) void {
        for (self.entities) |slot| if (slot != null and std.meta.eql(slot.?, entity)) return;
        for (&self.entities) |*slot| if (slot.* == null) {
            slot.* = entity;
            return;
        };
    }

    fn remove(self: *CollidingEntities, entity: clutch.EntityId) void {
        for (&self.entities) |*slot| if (slot.* != null and std.meta.eql(slot.*.?, entity)) {
            slot.* = null;
        };
    }
};

/// Sent when two entities' colliders start touching, and when they stop.
pub const CollisionStarted = struct { a: clutch.EntityId, b: clutch.EntityId };
pub const CollisionEnded = struct { a: clutch.EntityId, b: clutch.EntityId };

/// The stage physics runs in: right after `Update`, so `PostUpdate` sees where bodies went.
pub const PhysicsStage = struct {
    pub fn schedule() clutch.ScheduleStage {
        return clutch.ScheduleStage.after(.Update, PhysicsStage);
    }
};

/// What a host gives physics, as a type with these declarations:
///
///     pub const Transform: type             // the component placing an entity
///     pub const Time: type                  // a resource with the frame's length
///     pub fn deltaSeconds(time: *const Time) f32
///     pub fn place(transform: Transform) Place
///     pub fn setPlace(transform: *Transform, position: Vec3, rotation: emath.UnitQuat) void
///     // Optional, for `convex_hull_from_mesh` and `mesh`:
///     pub fn mesh(world: *clutch.World, entity: clutch.EntityId, gpa: Allocator) !?MeshSource
pub const Place = struct { position: Vec3, rotation: emath.UnitQuat, scale: Vec3 };

/// An entity's mesh: a shape the host knows exactly (a built-in cube), or triangles, both
/// before the entity's scale. Triangles are allocated with the `gpa` given, which physics frees.
pub const MeshSource = union(enum) {
    shape: Collider.Shape,
    triangles: struct { positions: [][3]f32, indices: [][3]u32 },
};

/// Physics in `dim` dimensions with scalar `T` (`f32` or `f64`), placing entities by `Host`.
pub fn Plugin(comptime dim: space.Dim, comptime T: type, comptime Host: type) type {
    return struct {
        pub const World = physics_zig.Physics(dim, T);
        const S = World.S;
        const two = dim == .two;

        /// An entity pair, `a` the lower collider.
        pub const EntityPair = struct { a: clutch.EntityId, b: clutch.EntityId };

        /// The physics world, as a clutch resource.
        pub const Resource = struct {
            world: World,
            /// What touched and stopped touching in the last frame's steps.
            started_pairs: std.ArrayList(EntityPair) = .empty,
            ended_pairs: std.ArrayList(EntityPair) = .empty,
            /// False once torn down, so remove hooks after that leave it alone.
            live: bool = true,

            pub fn started(self: *const Resource) []const EntityPair {
                return self.started_pairs.items;
            }

            pub fn ended(self: *const Resource) []const EntityPair {
                return self.ended_pairs.items;
            }

            pub const Hit = struct { entity: clutch.EntityId, point: Vec3, normal: Vec3, distance_m: f32 };

            /// The nearest entity a ray from `origin` along unit `direction` meets.
            pub fn raycast(self: *const Resource, origin: Vec3, direction: Vec3, max_distance_m: f32) ?Hit {
                const hit = self.world.raycast(.{ .origin = toVec(origin), .direction = toVec(direction), .max_distance_m = max_distance_m }) orelse return null;
                return .{
                    .entity = entityOf(self.world.colliderData(hit.collider)),
                    .point = fromVec(hit.point),
                    .normal = fromVec(hit.normal),
                    .distance_m = @floatCast(hit.distance_m),
                };
            }
        };

        /// An entity's body and collider, and what they were built from, so an edit rebuilds them.
        pub const Link = struct {
            body: ?BodyHandle = null,
            /// Null when its shape couldn't be built; it is tried again once something changes.
            collider: ?ColliderHandle = null,
            rigid_body: ?RigidBody,
            source: Collider,
            /// The transform as physics last left it; any other change is a move by the game.
            transform: Host.Transform,
            linear: Vec3 = .zero,
            angular: Vec3 = .zero,
        };

        /// Adds the physics world as a resource, its systems, and the hook that frees an entity's
        /// body when its `Link` goes. Run `Deinit` before freeing the world to free it.
        pub fn install(world: *clutch.World, gpa: std.mem.Allocator, settings: PhysicsSettings) !void {
            try world.insertResource(Resource{ .world = try World.init(gpa, configOf(settings)) });
            try world.addScheduleFallible(clutch.Schedule(PhysicsStage, .{ sync, unlinkOrphans, step, writeBack, report }));
            try world.addScheduleFallible(clutch.Schedule(.Deinit, .{teardown}));
            world.onRemove(Link, unlink);
        }

        fn configOf(settings: PhysicsSettings) World.Config {
            return .{ .gravity = toVec(settings.gravity_mps2), .fixed_step_s = 1 / @as(T, @max(settings.steps_per_second, 1)) };
        }

        /// Builds bodies for new and edited entities, and moves those the game moved.
        fn sync(res: *Resource, colliders: clutch.Query(.{ *const Collider, *const Host.Transform }), commands: clutch.Commands) !void {
            var query = colliders;
            while (query.next()) |view| {
                const world = view.world;
                const entity = view.entity;
                const collider = view.get(*const Collider).*;
                const transform = view.get(*const Host.Transform).*;
                const rigid_body: ?RigidBody = if (world.get(entity, RigidBody)) |found| found.* else null;
                const link = world.get(entity, Link) orelse {
                    try commands.addComponent(entity, build(res, world, entity, rigid_body, collider, transform));
                    continue;
                };
                const same = std.meta.eql(link.rigid_body, rigid_body) and std.meta.eql(link.source, collider) and
                    Host.place(link.transform).scale.eql(Host.place(transform).scale);
                if (!same) {
                    drop(res, link.*);
                    link.* = build(res, world, entity, rigid_body, collider, transform);
                    continue;
                }
                if (!std.meta.eql(link.transform, transform)) {
                    const pose = poseOf(Host.place(transform));
                    if (link.body) |body| res.world.setPose(body, pose) else if (link.collider) |handle| res.world.setColliderPose(handle, pose.mul(offsetPose(collider, Host.place(transform).scale)));
                    link.transform = transform;
                }
                const body = link.body orelse continue;
                const linear = if (world.get(entity, LinearVelocity)) |v| v.value else link.linear;
                const angular = if (world.get(entity, AngularVelocity)) |v| v.value else link.angular;
                if (!linear.eql(link.linear) or !angular.eql(link.angular)) {
                    res.world.setVelocity(body, .{ .linear = toVec(linear), .angular = toAngular(angular) });
                    link.linear = linear;
                    link.angular = angular;
                }
            }
        }

        /// Entities that lost their `Collider` lose their `Link`, and with it their body.
        fn unlinkOrphans(links: clutch.Query(.{ *const Link, clutch.Without(Collider) }), commands: clutch.Commands) !void {
            var query = links;
            while (query.next()) |view| try commands.removeComponent(view.entity, Link);
        }

        fn step(res: *Resource, time: *const Host.Time, settings: clutch.Query(.{*const PhysicsSettings})) !void {
            var found = settings;
            if (found.next()) |view| res.world.config = configOf(view.get(*const PhysicsSettings).*);
            const delta_s = Host.deltaSeconds(time);
            if (!std.math.isFinite(delta_s)) return;
            try res.world.update(@floatCast(delta_s));
        }

        /// Moving bodies carry their entities' transforms and velocities.
        fn writeBack(res: *Resource, links: clutch.Query(.{ *Link, *Host.Transform })) !void {
            var query = links;
            while (query.next()) |view| {
                const link = view.get(*Link);
                const body = link.body orelse continue;
                const velocity = res.world.velocity(body);
                link.linear = fromVec(velocity.linear);
                link.angular = fromAngular(velocity.angular);
                if (view.world.get(view.entity, LinearVelocity)) |v| v.value = link.linear;
                if (view.world.get(view.entity, AngularVelocity)) |v| v.value = link.angular;
                if (link.rigid_body.?.kind == .static or res.world.isSleeping(body)) continue;
                const transform = view.get(*Host.Transform);
                const place = Host.place(transform.*);
                const pose = res.world.pose(body);
                if (two) {
                    Host.setPlace(transform, Vec3.init(@floatCast(pose.translation.x), @floatCast(pose.translation.y), place.position.z), emath.UnitQuat.fromAxisAngle(Vec3.unit_z, @floatCast(pose.rotation.angle())));
                } else {
                    Host.setPlace(transform, fromVec(pose.translation), fromRotation(pose.rotation));
                }
                view.world.markChanged(view.entity, Host.Transform);
                link.transform = transform.*;
            }
        }

        /// Turns the last steps' started and ended pairs into entity pairs, events and
        /// `CollidingEntities`.
        fn report(res: *Resource, colliding: clutch.Query(.{*CollidingEntities}), commands: clutch.Commands) !void {
            const gpa = res.world.gpa;
            res.started_pairs.clearRetainingCapacity();
            res.ended_pairs.clearRetainingCapacity();
            for (res.world.started()) |pair| {
                const entities = entityPair(res, pair) orelse continue;
                try res.started_pairs.append(gpa, entities);
                try commands.sendEvent(CollisionStarted{ .a = entities.a, .b = entities.b });
            }
            for (res.world.ended()) |pair| {
                const entities = entityPair(res, pair) orelse continue;
                try res.ended_pairs.append(gpa, entities);
                try commands.sendEvent(CollisionEnded{ .a = entities.a, .b = entities.b });
            }
            var query = colliding;
            while (query.next()) |view| {
                const list = view.get(*CollidingEntities);
                for (res.ended_pairs.items) |pair| {
                    if (std.meta.eql(pair.a, view.entity)) list.remove(pair.b);
                    if (std.meta.eql(pair.b, view.entity)) list.remove(pair.a);
                }
                for (res.started_pairs.items) |pair| {
                    if (std.meta.eql(pair.a, view.entity)) list.add(pair.b);
                    if (std.meta.eql(pair.b, view.entity)) list.add(pair.a);
                }
            }
        }

        fn entityPair(res: *const Resource, pair: World.Pair) ?EntityPair {
            if (!res.world.contains(pair.a) or !res.world.contains(pair.b)) return null;
            return .{ .a = entityOf(res.world.colliderData(pair.a)), .b = entityOf(res.world.colliderData(pair.b)) };
        }

        fn teardown(res: *Resource) !void {
            if (!res.live) return;
            const gpa = res.world.gpa;
            res.started_pairs.deinit(gpa);
            res.ended_pairs.deinit(gpa);
            res.world.deinit();
            res.live = false;
        }

        fn unlink(world: *clutch.World, entity: clutch.EntityId) void {
            const res = world.getResource(Resource) orelse return;
            if (!res.live) return;
            drop(res, (world.get(entity, Link) orelse return).*);
        }

        fn drop(res: *Resource, link: Link) void {
            if (link.body) |body| res.world.removeBody(body) else if (link.collider) |collider| res.world.removeCollider(collider);
        }

        /// A body (with a `RigidBody`) and a collider for `entity`. What can't be built is left
        /// null and logged; it is tried again when the entity's physics changes.
        fn build(res: *Resource, world: *clutch.World, entity: clutch.EntityId, rigid_body: ?RigidBody, collider: Collider, transform: Host.Transform) Link {
            var link: Link = .{ .rigid_body = rigid_body, .source = collider, .transform = transform };
            const place = Host.place(transform);
            const pose = poseOf(place);
            const data = dataOf(entity);
            if (rigid_body) |found| {
                if (world.get(entity, LinearVelocity)) |v| link.linear = v.value;
                if (world.get(entity, AngularVelocity)) |v| link.angular = v.value;
                link.body = res.world.addBody(.{
                    .kind = found.kind,
                    .pose = pose,
                    .mass_kg = found.mass_kg,
                    .linear_damping = found.linear_damping,
                    .angular_damping = found.angular_damping,
                    .gravity_scale = found.gravity_scale,
                    .linear_velocity = toVec(link.linear),
                    .angular_velocity = toAngular(link.angular),
                }, data) catch |err| {
                    log.err("cannot add a body: {t}", .{err});
                    return link;
                };
            }
            var shape = shapeOf(collider.shape, place.scale, world, entity, res.world.gpa) catch |err| {
                log.warn("cannot shape a collider ({t}): {t}", .{ collider.shape, err });
                return link;
            };
            defer shape.deinit(res.world.gpa);
            const offset = offsetPose(collider, place.scale);
            link.collider = res.world.addCollider(.{
                .shape = shape.shape,
                // With no body, a collider's own pose is where it sits in the world.
                .local_pose = if (link.body == null) pose.mul(offset) else offset,
                .material = .{ .restitution = collider.restitution, .friction = collider.friction },
                .sensor = collider.sensor,
            }, link.body, data) catch |err| {
                log.warn("cannot add a collider ({t}): {t}", .{ collider.shape, err });
                return link;
            };
            return link;
        }

        fn offsetPose(collider: Collider, scale: Vec3) World.Pose {
            return World.Pose.fromTranslation(toVec(collider.offset_m.mul(scale)));
        }

        /// A built shape, and the vertices it borrows from mesh data, freed once the world has copied them.
        const Built = struct {
            shape: World.Shape,
            vertices: []S.Vec = &.{},
            indices: [][S.n]u32 = &.{},

            fn deinit(self: *Built, gpa: std.mem.Allocator) void {
                gpa.free(self.vertices);
                gpa.free(self.indices);
            }
        };

        fn shapeOf(shape: Collider.Shape, scale: Vec3, world: *clutch.World, entity: clutch.EntityId, gpa: std.mem.Allocator) !Built {
            if (analytic(shape, scale)) |exact| return .{ .shape = exact };
            return fromMesh(shape, scale, world, entity, gpa);
        }

        /// Every shape but those made from a mesh, at the entity's scale.
        fn analytic(shape: Collider.Shape, scale_raw: Vec3) ?World.Shape {
            const scale = scale_raw.abs();
            const across = if (two) scale.x else @max(scale.x, scale.z);
            return switch (shape) {
                .ball => |ball| .{ .ball = .{ .radius = ball.radius_m * (if (two) @max(scale.x, scale.y) else @max(scale.x, scale.y, scale.z)) } },
                .cuboid => |cuboid| .{ .cuboid = .{ .half_extents = toVec(cuboid.half_extents_m.mul(scale)) } },
                .capsule => |capsule| .{ .capsule = .{ .half_height = capsule.half_height_m * scale.y, .radius = capsule.radius_m * across } },
                .cylinder => |cylinder| .{ .cylinder = .{ .half_height = cylinder.half_height_m * scale.y, .radius = cylinder.radius_m * across } },
                .cone => |cone| .{ .cone = .{ .half_height = cone.half_height_m * scale.y, .radius = cone.radius_m * across } },
                .half_space => |half_space| .{ .half_space = .{ .normal = toVec(half_space.normal).normalize() } },
                .convex_hull_from_mesh, .mesh => null,
            };
        }

        fn fromMesh(shape: Collider.Shape, scale: Vec3, world: *clutch.World, entity: clutch.EntityId, gpa: std.mem.Allocator) !Built {
            if (!@hasDecl(Host, "mesh")) return error.NoMeshSource;
            const source = (try Host.mesh(world, entity, gpa)) orelse return error.NoMesh;
            switch (source) {
                // A built-in shape is used as itself, whichever way it was asked for.
                .shape => |exact| return .{ .shape = analytic(exact, scale) orelse return error.NoMesh },
                .triangles => |triangles| {
                    defer gpa.free(triangles.positions);
                    defer gpa.free(triangles.indices);
                    if (triangles.positions.len == 0) return error.NoMesh;
                    const vertices = try gpa.alloc(S.Vec, triangles.positions.len);
                    errdefer gpa.free(vertices);
                    for (vertices, triangles.positions) |*v, p| v.* = toVec(Vec3.fromArray(p).mul(scale));
                    if (shape == .convex_hull_from_mesh) return .{ .shape = .{ .convex_hull = .{ .points = vertices } }, .vertices = vertices };
                    if (two) return error.NoMeshIn2D;
                    const indices = try gpa.alloc([S.n]u32, triangles.indices.len);
                    for (indices, triangles.indices) |*out, in| out.* = in;
                    return .{ .shape = .{ .mesh = .{ .vertices = vertices, .indices = indices } }, .vertices = vertices, .indices = indices };
                },
            }
        }

        fn dataOf(entity: clutch.EntityId) u64 {
            return (@as(u64, entity.id) << 32) | entity.generation;
        }

        fn entityOf(data: u64) clutch.EntityId {
            return clutch.EntityId.init(@truncate(data >> 32), @truncate(data));
        }

        fn poseOf(place: Place) World.Pose {
            if (two) {
                const turned = place.rotation.rotateVector(Vec3.unit_x);
                return World.Pose.init(toVec(place.position), std.math.atan2(@as(T, turned.y), @as(T, turned.x)));
            } else {
                const Quat = @FieldType(emath.UnitQuaternion(T), "quat");
                const parts: @Vector(4, T) = @floatCast(place.rotation.quat.v);
                return World.Pose.fromParts(toVec(place.position), emath.UnitQuaternion(T).newUnchecked(Quat{ .v = parts }));
            }
        }

        fn fromRotation(rotation: emath.UnitQuaternion(T)) emath.UnitQuat {
            const Quat = @FieldType(emath.UnitQuat, "quat");
            const parts: @Vector(4, f32) = @floatCast(rotation.quat.v);
            return emath.UnitQuat.newUnchecked(Quat{ .v = parts });
        }

        fn toVec(v: Vec3) S.Vec {
            return if (two) S.Vec.init(v.x, v.y) else S.Vec.init(v.x, v.y, v.z);
        }

        fn fromVec(v: S.Vec) Vec3 {
            return if (two) Vec3.init(@floatCast(v.x), @floatCast(v.y), 0) else Vec3.init(@floatCast(v.x), @floatCast(v.y), @floatCast(v.z));
        }

        fn toAngular(v: Vec3) S.Angular {
            return if (two) v.z else S.Vec.init(v.x, v.y, v.z);
        }

        fn fromAngular(v: S.Angular) Vec3 {
            return if (two) Vec3.init(0, 0, @floatCast(v)) else fromVec(v);
        }
    };
}

/// A host for tests: its transform is a plain place.
const TestHost = struct {
    pub const Transform = struct { position: Vec3 = .zero, rotation: emath.UnitQuat = .identity(), scale: Vec3 = .one };
    pub const Time = struct { delta_s: f32 };
    pub fn deltaSeconds(time: *const Time) f32 {
        return time.delta_s;
    }
    pub fn place(transform: Transform) Place {
        return .{ .position = transform.position, .rotation = transform.rotation, .scale = transform.scale };
    }
    pub fn setPlace(transform: *Transform, position: Vec3, rotation: emath.UnitQuat) void {
        transform.position = position;
        transform.rotation = rotation;
    }
};

fn runFrames(world: *clutch.World, frames: usize) !void {
    for (0..frames) |_| {
        try world.runStage(PhysicsStage);
        world.tick();
    }
}

test "an entity with a RigidBody falls onto a static Collider, and they collide" {
    inline for (.{ .{ space.Dim.three, f32 }, .{ space.Dim.three, f64 }, .{ space.Dim.two, f32 }, .{ space.Dim.two, f64 } }) |variant| {
        const P = Plugin(variant[0], variant[1], TestHost);
        const gpa = std.testing.allocator;
        var world = clutch.World.init(gpa);
        defer world.deinit();
        try world.insertResource(TestHost.Time{ .delta_s = 1.0 / 60.0 });
        try P.install(&world, gpa, .{ .dimension = variant[0] });
        defer world.runStage(.Deinit) catch {};

        const ground = try world.spawn(.{ TestHost.Transform{ .scale = .init(10, 1, 10) }, Collider{} });
        const crate = try world.spawn(.{ TestHost.Transform{ .position = .init(0, 3, 0) }, Collider{}, RigidBody{}, LinearVelocity{}, CollidingEntities{} });
        try runFrames(&world, 180);

        // The ground's top is at 0.5, so the crate's centre rests a half metre above it.
        try std.testing.expectApproxEqAbs(@as(f32, 1.0), world.get(crate, TestHost.Transform).?.position.y, 0.03);
        try std.testing.expect(std.meta.eql(world.get(crate, CollidingEntities).?.entities[0].?, ground));
    }
}

test "removing a Collider or despawning frees its body" {
    const P = Plugin(.three, f32, TestHost);
    const gpa = std.testing.allocator;
    var world = clutch.World.init(gpa);
    defer world.deinit();
    try world.insertResource(TestHost.Time{ .delta_s = 1.0 / 60.0 });
    try P.install(&world, gpa, .{});
    defer world.runStage(.Deinit) catch {};

    const ball = try world.spawn(.{ TestHost.Transform{}, Collider{ .shape = .{ .ball = .{} } }, RigidBody{} });
    const wall = try world.spawn(.{ TestHost.Transform{ .position = .init(5, 0, 0) }, Collider{} });
    try runFrames(&world, 1);
    const res = world.getResource(P.Resource).?;
    try std.testing.expectEqual(@as(usize, 2), res.world.colliders.entries.items.len - res.world.colliders.free.items.len);
    world.remove(ball, Collider);
    try runFrames(&world, 1);
    world.despawn(wall);
    try std.testing.expectEqual(@as(usize, 0), res.world.colliders.entries.items.len - res.world.colliders.free.items.len);
    try std.testing.expect(world.get(ball, P.Link) == null);
}

test "a game setting a LinearVelocity throws the body" {
    const P = Plugin(.three, f32, TestHost);
    const gpa = std.testing.allocator;
    var world = clutch.World.init(gpa);
    defer world.deinit();
    try world.insertResource(TestHost.Time{ .delta_s = 1.0 / 60.0 });
    try P.install(&world, gpa, .{ .gravity_mps2 = .zero });
    defer world.runStage(.Deinit) catch {};

    const ball = try world.spawn(.{ TestHost.Transform{}, Collider{ .shape = .{ .ball = .{} } }, RigidBody{}, LinearVelocity{} });
    try runFrames(&world, 1);
    world.get(ball, LinearVelocity).?.value = .init(6, 0, 0);
    try runFrames(&world, 30);
    try std.testing.expectApproxEqAbs(@as(f32, 3), world.get(ball, TestHost.Transform).?.position.x, 0.15);
}
