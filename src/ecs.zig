//! humpty in a clutch world. Entities with a `Collider` (and a `RigidBody` to move) get a body
//! in the physics world, a clutch resource; their handles live in a `Link` component, freed by
//! an `onRemove` hook whether it is removed or the entity despawned. Systems in `PhysicsStage`,
//! right after `Update`, build and rebuild what changed, step, move the host's transforms, and
//! report collisions as `CollidingEntities`, the resource's `started`/`ended` lists and events.
//!
//!     try humpty.ecs.Plugin(.three, f32, MyHost).install(&world, gpa, .{});
//!
//! The components are plain `f32` data, the same in 2D and 3D, so an editor can list and save
//! them whatever the world is; 2D reads x and y, and turns about z.
const std = @import("std");
const clutch = @import("clutch");
const emath = @import("eggenvector");
const physics_zig = @import("physics.zig");
const space = @import("space.zig");
const debug_draw = @import("debug_draw.zig");
const BodyKind = @import("body.zig").BodyKind;
const Rotation = @import("body.zig").Rotation;
const BodyHandle = @import("body.zig").BodyHandle;
const ColliderHandle = @import("collider.zig").ColliderHandle;

const Vec3 = emath.Vec3;
const log = std.log.scoped(.physics);

/// The scalar a world computes in: `f64` for worlds far bigger than a few kilometres.
pub const Precision = enum {
    f32,
    f64,

    /// The Zig type it names.
    pub fn Type(comptime self: Precision) type {
        return if (self == .f32) f32 else f64;
    }
};

/// How a scene's physics runs, read each step: put one on the scene's root entity. Whether it
/// is 2D or 3D, and its precision, are the plugin's (`Plugin(dim, T, Host)`), not the scene's.
pub const PhysicsSettings = struct {
    /// Marks it in the editor (nf-md-earth).
    pub const icon = "\u{f01e7}";
    pub const category = "Physics";
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
    /// Locked, it slides but never tips over: for characters.
    rotation: Rotation = .free,
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
    /// The entity whose mesh `convex_hull_from_mesh` and `mesh` take, such as a child holding
    /// the model, placed where it sits from this one; this entity's own when unset.
    mesh_from: EntityRef = .{},

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

/// Another entity, if set. Eggy's editor and scenes take it as an entity reference to any entity
/// (`eggy_ref`, `Required`), so it can be dragged from the Hierarchy and is saved with the scene.
pub const EntityRef = struct {
    entity: ?clutch.EntityId = null,

    pub const eggy_ref = .entity;
    pub const Required = void;

    pub fn init(entity: ?clutch.EntityId) EntityRef {
        return .{ .entity = entity };
    }
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

/// What a raycast leaves out.
pub const RayFilter = struct {
    /// This entity's own collider, such as the one casting from inside it.
    exclude: ?clutch.EntityId = null,
    sensors: enum { hit, skip } = .hit,
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
///     // Where `transform`, the entity's, puts it in the world, through any parents; and the
///     // reverse, setting `transform` so the entity is at `position` turned by `rotation`.
///     pub fn place(world: *clutch.World, entity: clutch.EntityId, transform: Transform) Place
///     pub fn setPlace(world: *clutch.World, entity: clutch.EntityId, transform: *Transform, position: Vec3, rotation: emath.UnitQuat) void
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

            /// The nearest entity a ray from `origin` along unit `direction` meets, leaving out
            /// what `filter` says.
            pub fn raycast(self: *const Resource, origin: Vec3, direction: Vec3, max_distance_m: f32, filter: RayFilter) ?Hit {
                var query: World.QueryFilter = .{ .sensors = switch (filter.sensors) {
                    .hit => .hit,
                    .skip => .skip,
                } };
                // An entity is left out by its body, or its lone collider when it has none.
                if (filter.exclude) |entity| {
                    const resource_world = &self.world;
                    for (resource_world.colliders.entries.items, 0..) |entry, index| {
                        const attached = entry.item orelse continue;
                        if (attached.user_data != dataOf(entity)) continue;
                        if (attached.parent) |body| query.exclude_body = body else query.exclude_collider = resource_world.colliders.handleAt(index);
                        break;
                    }
                }
                const hit = self.world.raycast(.{ .origin = toVec(origin), .direction = toVec(direction), .max_distance_m = max_distance_m }, query) orelse return null;
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
            /// Where its transform put it in the world when physics last left it; any other place
            /// is a move by the game, or by a parent.
            place: Place,
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

        /// Wireframes of the colliders on `only` (every entity with a `Collider` when null),
        /// through `sink`: any value with `fn line(self, from: Vec3, to: Vec3, color: [4]f32) !void`.
        /// Drawn from components and transforms, so it needs no running world (an editor between
        /// plays); when one is running, sleeping bodies are drawn as sleeping. Shapes that can't
        /// be built (a mesh not loaded yet) are skipped.
        pub fn drawColliders(world: *clutch.World, only: ?[]const clutch.EntityId, gpa: std.mem.Allocator, sink: anytype) !void {
            if (only) |entities| {
                for (entities) |entity| try drawCollider(world, entity, gpa, sink);
            } else {
                var query = world.query(.{*const Collider});
                while (query.next()) |view| try drawCollider(world, view.entity, gpa, sink);
            }
        }

        /// The last step's contacts through `sink` (as `drawColliders` takes it): a cross at each
        /// point, its normal drawn from it. Nothing without a running world.
        pub fn drawContacts(world: *clutch.World, sink: anytype) !void {
            const res = world.getResource(Resource) orelse return;
            var lines: Lines(@TypeOf(sink)) = .{ .sink = sink };
            for (res.world.contacts()) |contact| try debug_draw.DebugDraw(S).marker(contact.point, contact.normal, 0.1, debug_draw.palette.contact, &lines);
        }

        fn drawCollider(world: *clutch.World, entity: clutch.EntityId, gpa: std.mem.Allocator, sink: anytype) !void {
            const collider = (world.get(entity, Collider) orelse return).*;
            const transform = (world.get(entity, Host.Transform) orelse return).*;
            const place = Host.place(world, entity, transform);
            var shape = shapeOf(collider, place, world, entity, gpa) catch return;
            defer shape.deinit(gpa);
            const kind: ?BodyKind = if (world.get(entity, RigidBody)) |found| found.kind else null;
            const color = debug_draw.palette.of(kind, sleeping(world, entity), collider.sensor);
            var lines: Lines(@TypeOf(sink)) = .{ .sink = sink };
            try debug_draw.DebugDraw(S).shape(shape.shape, poseOf(place).mul(offsetPose(collider, place.scale)), color, &lines);
        }

        fn sleeping(world: *clutch.World, entity: clutch.EntityId) bool {
            const res = world.getResource(Resource) orelse return false;
            const body = (world.get(entity, Link) orelse return false).body orelse return false;
            return res.world.contains(body) and res.world.isSleeping(body);
        }

        /// A sink taking this space's vectors, passing them on as `Vec3`s.
        fn Lines(comptime Sink: type) type {
            return struct {
                sink: Sink,
                pub fn line(self: *@This(), from: S.Vec, to: S.Vec, color: debug_draw.Color) !void {
                    try self.sink.line(fromVec(from), fromVec(to), color);
                }
            };
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
                const place = Host.place(world, entity, transform);
                const same = std.meta.eql(link.rigid_body, rigid_body) and std.meta.eql(link.source, collider) and link.place.scale.eql(place.scale);
                if (!same) {
                    drop(res, link.*);
                    link.* = build(res, world, entity, rigid_body, collider, transform);
                    continue;
                }
                if (!std.meta.eql(link.place, place)) {
                    const pose = poseOf(place);
                    if (link.body) |body| res.world.setPose(body, pose) else if (link.collider) |handle| res.world.setColliderPose(handle, pose.mul(offsetPose(collider, place.scale)));
                    link.place = place;
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
                const place = Host.place(view.world, view.entity, transform.*);
                const pose = res.world.pose(body);
                if (two) {
                    Host.setPlace(view.world, view.entity, transform, Vec3.init(@floatCast(pose.translation.x), @floatCast(pose.translation.y), place.position.z), emath.UnitQuat.fromAxisAngle(Vec3.unit_z, @floatCast(pose.rotation.angle())));
                } else {
                    Host.setPlace(view.world, view.entity, transform, fromVec(pose.translation), fromRotation(pose.rotation));
                }
                view.world.markChanged(view.entity, Host.Transform);
                link.place = Host.place(view.world, view.entity, transform.*);
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
            const place = Host.place(world, entity, transform);
            var link: Link = .{ .rigid_body = rigid_body, .source = collider, .place = place };
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
                    .rotation = found.rotation,
                    .linear_velocity = toVec(link.linear),
                    .angular_velocity = toAngular(link.angular),
                }, data) catch |err| {
                    log.err("cannot add a body: {t}", .{err});
                    return link;
                };
            }
            var shape = shapeOf(collider, place, world, entity, res.world.gpa) catch |err| {
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

        /// `collider`'s shape on `entity`, which is at `place`.
        fn shapeOf(collider: Collider, place: Place, world: *clutch.World, entity: clutch.EntityId, gpa: std.mem.Allocator) !Built {
            if (analytic(collider.shape, place.scale)) |exact| return .{ .shape = exact };
            return fromMesh(collider, place, world, entity, gpa);
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

        /// The shape made from the mesh `collider` takes: `entity`'s own (at `place`), or that of
        /// the entity its `mesh_from` names, moved from where that one sits into `entity`'s space.
        fn fromMesh(collider: Collider, place: Place, world: *clutch.World, entity: clutch.EntityId, gpa: std.mem.Allocator) !Built {
            if (!@hasDecl(Host, "mesh")) return error.NoMeshSource;
            const shape = collider.shape;
            const scale = place.scale;
            const from = collider.mesh_from.entity orelse entity;
            if (!world.isAlive(from)) return error.NoMesh;
            const elsewhere: ?Place = if (std.meta.eql(from, entity)) null else Host.place(world, from, (world.get(from, Host.Transform) orelse return error.NoMesh).*);
            const source = (try Host.mesh(world, from, gpa)) orelse return error.NoMesh;
            switch (source) {
                // A built-in shape is used as itself, whichever way it was asked for.
                // ponytail: another entity's built-in shape takes its scale but not its offset or turn; make it a hull of its corners if that bites.
                .shape => |exact| return .{ .shape = analytic(exact, if (elsewhere) |other| other.scale else scale) orelse return error.NoMesh },
                .triangles => |triangles| {
                    defer gpa.free(triangles.positions);
                    defer gpa.free(triangles.indices);
                    if (triangles.positions.len == 0) return error.NoMesh;
                    const vertices = try gpa.alloc(S.Vec, triangles.positions.len);
                    errdefer gpa.free(vertices);
                    for (vertices, triangles.positions) |*v, p| v.* = toVec(if (elsewhere) |other| relativeCorner(place, other, Vec3.fromArray(p)) else Vec3.fromArray(p).mul(scale));
                    if (shape == .convex_hull_from_mesh) return .{ .shape = .{ .convex_hull = .{ .points = vertices } }, .vertices = vertices };
                    if (two) return error.NoMeshIn2D;
                    const indices = try gpa.alloc([S.n]u32, triangles.indices.len);
                    for (indices, triangles.indices) |*out, in| out.* = in;
                    return .{ .shape = .{ .mesh = .{ .vertices = vertices, .indices = indices } }, .vertices = vertices, .indices = indices };
                },
            }
        }

        /// Corner `at` of a mesh on an entity at `other`, from `place` as `place`'s collider sees
        /// its own scaled corners: turned into its frame, at the world's size.
        fn relativeCorner(place: Place, other: Place, at: Vec3) Vec3 {
            const in_world = other.position.add(other.rotation.rotateVector(at.mul(other.scale)));
            return place.rotation.inverseRotateVector(in_world.sub(place.position));
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
    pub fn place(_: *clutch.World, _: clutch.EntityId, transform: Transform) Place {
        return .{ .position = transform.position, .rotation = transform.rotation, .scale = transform.scale };
    }
    pub fn setPlace(_: *clutch.World, _: clutch.EntityId, transform: *Transform, position: Vec3, rotation: emath.UnitQuat) void {
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
        try P.install(&world, gpa, .{});
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

test "a body with its rotation locked lands on an edge without tipping" {
    const P = Plugin(.three, f32, TestHost);
    var world = clutch.World.init(std.testing.allocator);
    defer world.deinit();
    try P.install(&world, std.testing.allocator, .{});
    defer world.runStage(.Deinit) catch {};
    try world.insertResource(TestHost.Time{ .delta_s = 1.0 / 60.0 });

    _ = try world.spawn(.{ TestHost.Transform{}, Collider{ .shape = .{ .half_space = .{} } } });
    const tilted = emath.UnitQuat.fromAxisAngle(Vec3.unit_z, std.math.pi / 4.0);
    const crate = try world.spawn(.{ TestHost.Transform{ .position = .init(0, 2, 0), .rotation = tilted }, Collider{}, RigidBody{ .rotation = .locked } });
    try runFrames(&world, 180);
    const rotation = world.get(crate, TestHost.Transform).?.rotation;
    try std.testing.expectApproxEqAbs(@as(f32, 0), rotation.rotateVector(Vec3.unit_x).sub(tilted.rotateVector(Vec3.unit_x)).length(), 1e-3);
    // Resting on its edge: half the diagonal up.
    try std.testing.expectApproxEqAbs(@as(f32, @sqrt(0.5)), world.get(crate, TestHost.Transform).?.position.y, 0.03);
}

test "drawColliders draws a scaled, moved box without a running world, only for the entities asked" {
    const P = Plugin(.three, f32, TestHost);
    const gpa = std.testing.allocator;
    var world = clutch.World.init(gpa);
    defer world.deinit();

    const crate = try world.spawn(.{ TestHost.Transform{ .position = .init(10, 0, 0), .scale = .init(2, 1, 1) }, Collider{}, RigidBody{} });
    _ = try world.spawn(.{ TestHost.Transform{}, Collider{ .shape = .{ .ball = .{} } } });

    const Sink = struct {
        lines: usize = 0,
        min_x: f32 = std.math.inf(f32),
        color: [4]f32 = undefined,
        pub fn line(self: *@This(), from: Vec3, to: Vec3, color: [4]f32) !void {
            self.lines += 1;
            self.min_x = @min(self.min_x, from.x, to.x);
            self.color = color;
        }
    };
    var sink = Sink{};
    try P.drawColliders(&world, &.{crate}, gpa, &sink);
    // Twelve edges, the nearest at 10 - 0.5 * 2, in the dynamic colour.
    try std.testing.expectEqual(@as(usize, 12), sink.lines);
    try std.testing.expectApproxEqAbs(@as(f32, 9), sink.min_x, 1e-4);
    try std.testing.expectEqual(debug_draw.palette.dynamic, sink.color);

    sink = .{};
    try P.drawColliders(&world, null, gpa, &sink);
    try std.testing.expect(sink.lines > 12);
}

/// `TestHost` with meshes: an entity with `HasCube` has a unit cube's corners.
const MeshHost = struct {
    const HasCube = struct {};
    pub const Transform = TestHost.Transform;
    pub const Time = TestHost.Time;
    pub const deltaSeconds = TestHost.deltaSeconds;
    pub const place = TestHost.place;
    pub const setPlace = TestHost.setPlace;
    pub fn mesh(world: *clutch.World, entity: clutch.EntityId, gpa: std.mem.Allocator) !?MeshSource {
        if (world.get(entity, HasCube) == null) return null;
        const positions = try gpa.alloc([3]f32, 8);
        errdefer gpa.free(positions);
        for (positions, 0..) |*corner, i| corner.* = .{ if (i & 1 != 0) 0.5 else -0.5, if (i & 2 != 0) 0.5 else -0.5, if (i & 4 != 0) 0.5 else -0.5 };
        const indices = try gpa.dupe([3]u32, &.{ .{ 0, 1, 2 }, .{ 4, 5, 6 } });
        return .{ .triangles = .{ .positions = positions, .indices = indices } };
    }
};

test "a collider takes its mesh from the entity mesh_from names, where that entity sits" {
    const P = Plugin(.three, f32, MeshHost);
    const gpa = std.testing.allocator;
    var world = clutch.World.init(gpa);
    defer world.deinit();
    try world.insertResource(TestHost.Time{ .delta_s = 1.0 / 60.0 });
    try P.install(&world, gpa, .{});
    defer world.runStage(.Deinit) catch {};

    // The cube sits a metre above the collider's own entity, which has no mesh.
    const cube = try world.spawn(.{ MeshHost.Transform{ .position = .init(0, 1, 0) }, MeshHost.HasCube{} });
    _ = try world.spawn(.{ MeshHost.Transform{}, Collider{ .shape = .convex_hull_from_mesh, .mesh_from = .init(cube) } });
    const ball = try world.spawn(.{ MeshHost.Transform{ .position = .init(0, 4, 0) }, Collider{ .shape = .{ .ball = .{} } }, RigidBody{} });
    try runFrames(&world, 180);

    // The cube's top is at 1.5, so the ball's centre rests half a metre above it.
    try std.testing.expectApproxEqAbs(@as(f32, 2.0), world.get(ball, MeshHost.Transform).?.position.y, 0.05);
}
