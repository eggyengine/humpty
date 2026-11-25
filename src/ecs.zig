//! humpty in a clutch world. Entities with a `RigidBody` get a body in the physics world, a
//! clutch resource; a `Collider` joins the body on its own entity or the nearest one above it,
//! or alone is scenery. Their handles live in a `Link` component, freed by an `onRemove` hook
//! whether it is removed or the entity despawned. Systems in `PhysicsStage`, right after
//! `Update`, build and rebuild what changed, step (running the game's `FixedStep` inside each
//! step), move the host's transforms, and report collisions as `CollidingEntities`, the
//! resource's `started`/`ended` lists and events. Joints (`ecs_joints.zig`) and vehicles
//! (`ecs_vehicles.zig`) ride on the same bodies.
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

const joints_zig = @import("ecs_joints.zig");
const vehicles_zig = @import("ecs_vehicles.zig");

const Vec3 = emath.Vec3;
const log = std.log.scoped(.physics);

pub const Joint = joints_zig.Joint;
pub const JointState = joints_zig.JointState;
pub const Vehicle = vehicles_zig.Vehicle;
pub const VehicleInput = vehicles_zig.VehicleInput;
pub const VehicleState = vehicles_zig.VehicleState;
pub const Wheel = vehicles_zig.Wheel;
pub const WheelState = vehicles_zig.WheelState;

/// Code to run inside each fixed physics step, before its forces, contacts and moves, as a
/// clutch resource: a host's way to give games a fixed-step hook. Forces applied from it last
/// that step alone, and `VehicleInput`s set in it are read that same step. It may read and
/// change components, but not spawn, despawn, or add or remove them.
pub const FixedStep = struct {
    context: *anyopaque,
    call: *const fn (context: *anyopaque, world: *clutch.World, step_s: f32) anyerror!void,
};

/// Where a force or impulse pushes a body.
pub const At = union(enum) {
    center_of_mass,
    world_point: Vec3,
};

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
    /// Raise it (120 or more) for vehicles, whose springs and tyres are worked out each step.
    steps_per_second: f32 = 60,
};

/// Makes its entity a body the physics world moves, shaped by its own `Collider` and those on
/// the entities below it that have no `RigidBody` of their own.
pub const RigidBody = struct {
    /// Marks it in the editor (nf-md-cube_send).
    pub const icon = "\u{f0a6e}";
    pub const category = "Physics";
    pub const dependencies = .{Collider};
    /// Can change while it runs, keeping where it is: a parked trailer turned static is steady
    /// to build on, and turned dynamic again to tow.
    kind: BodyKind = .dynamic,
    /// Spread over its colliders without a mass of their own; those with one add theirs.
    mass_kg: f32 = 1,
    /// Where its mass balances, from the entity; unset, where its colliders balance. Vehicles
    /// want it low.
    center_of_mass_m: ?Vec3 = null,
    linear_damping: f32 = 0,
    angular_damping: f32 = 0,
    gravity_scale: f32 = 1,
    /// Locked, it slides but never tips over: for characters.
    rotation: Rotation = .free,
    /// Off, it stays where its transform puts it and touches nothing, as a player sitting in a cab.
    enabled: bool = true,
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
    /// Its own mass, added to its body's where it sits, as a machine built on a deck adds its
    /// weight; unset, it takes a share of the body's `mass_kg`.
    mass_kg: ?f32 = null,
    /// Off, it touches nothing and adds no mass.
    enabled: bool = true,
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
        /// The height grid of this name (`Host.heightfield`), its first corner at the entity:
        /// 3D static scenery only.
        heightfield: GridName,
    };
};

/// A height grid's name, which the host looks up. Eggy's editor and scenes keep it as text
/// (`eggy_text`).
pub const GridName = struct {
    /// Zeroed past `len`, so two names compare equal by their bytes.
    buffer: [max_len]u8 = @splat(0),
    len: u8 = 0,

    pub const max_len = 64;
    pub const eggy_text = {};

    /// `value`, cut to `max_len` bytes without splitting a UTF-8 character.
    pub fn init(value: []const u8) GridName {
        var end = @min(value.len, max_len);
        while (end > 0 and end < value.len and (value[end] & 0xC0) == 0x80) end -= 1;
        var name = GridName{ .len = @intCast(end) };
        @memcpy(name.buffer[0..end], value[0..end]);
        return name;
    }

    pub fn text(self: *const GridName) []const u8 {
        return self.buffer[0..self.len];
    }
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

/// What a raycast or shape cast leaves out.
pub const RayFilter = struct {
    /// This entity's body and every collider on it, or its own collider when it has no body:
    /// such as the one casting from inside it.
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
///     // Optional, for `heightfield`: the grid named `name`, borrowed while the collider is built.
///     pub fn heightfield(world: *clutch.World, name: []const u8) ?HeightfieldSource
pub const Place = struct { position: Vec3, rotation: emath.UnitQuat, scale: Vec3 };

/// A height grid, before the entity's scale: `heights` row by row, `columns` along x, rows along
/// z, `cell_m` apart both ways.
pub const HeightfieldSource = struct { heights: []const f32, columns: u32, cell_m: f32 };

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
        /// How it places entities, for joints and vehicles.
        pub const PluginHost = Host;
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
                const hit = self.world.raycast(.{ .origin = toVec(origin), .direction = toVec(direction), .max_distance_m = max_distance_m }, self.queryFilter(filter)) orelse return null;
                return .{
                    .entity = entityOf(self.world.colliderData(hit.collider)),
                    .point = fromVec(hit.point),
                    .normal = fromVec(hit.normal),
                    .distance_m = @floatCast(hit.distance_m),
                };
            }

            pub const ShapeHit = struct { entity: clutch.EntityId, point: Vec3, normal: Vec3, distance_m: f32, time_of_impact: f32 };

            /// The first entity `shape` (a ball, cuboid, capsule, cylinder or cone, at its own
            /// size) meets moving from `origin`, turned by `rotation`, `max_distance_m` along unit
            /// `direction`, leaving out what `filter` says. One it starts in is met at 0.
            pub fn shapecast(self: *const Resource, shape: Collider.Shape, origin: Vec3, rotation: emath.UnitQuat, direction: Vec3, max_distance_m: f32, filter: RayFilter) ?ShapeHit {
                const cast_shape = analytic(shape, .one) orelse return null;
                if (!cast_shape.convex() or !cast_shape.supported()) return null;
                const start = poseOf(.{ .position = origin, .rotation = rotation, .scale = .one });
                const hit = self.world.shapecast(cast_shape, start, toVec(direction), max_distance_m, self.queryFilter(filter)) orelse return null;
                return .{
                    .entity = entityOf(self.world.colliderData(hit.collider)),
                    .point = fromVec(hit.point),
                    .normal = fromVec(hit.normal),
                    .distance_m = @floatCast(hit.distance_m),
                    .time_of_impact = @floatCast(hit.time_of_impact),
                };
            }

            /// An entity is left out by its body, or its lone collider when it has none.
            fn queryFilter(self: *const Resource, filter: RayFilter) World.QueryFilter {
                var query: World.QueryFilter = .{ .sensors = switch (filter.sensors) {
                    .hit => .hit,
                    .skip => .skip,
                } };
                const entity = filter.exclude orelse return query;
                for (self.world.bodies.entries.items, 0..) |entry, index| {
                    const attached = entry.item orelse continue;
                    if (attached.user_data != dataOf(entity)) continue;
                    query.exclude_body = self.world.bodies.handleAt(index);
                    return query;
                }
                for (self.world.colliders.entries.items, 0..) |entry, index| {
                    const attached = entry.item orelse continue;
                    if (attached.user_data != dataOf(entity)) continue;
                    if (attached.parent) |body| query.exclude_body = body else query.exclude_collider = self.world.colliders.handleAt(index);
                    break;
                }
                return query;
            }
        };

        /// The body `entity`'s own `RigidBody` makes, or the one its collider is on.
        pub fn bodyOf(world: *clutch.World, entity: clutch.EntityId) ?BodyHandle {
            const link = world.get(entity, Link) orelse return null;
            return link.body orelse link.collider_body;
        }

        /// Pushes `entity`'s body with `force_n` (N) for every step of this frame, or from a
        /// `FixedStep`, for that step. Only dynamic bodies move.
        pub fn applyForce(world: *clutch.World, entity: clutch.EntityId, force_n: Vec3, at: At) void {
            const res = world.getResource(Resource) orelse return;
            res.world.applyForce(bodyOf(world, entity) orelse return, toVec(force_n), impulsePoint(at));
        }

        /// Turns `entity`'s body with `torque_nm` (N·m; about z in 2D) as `applyForce` pushes.
        pub fn applyTorque(world: *clutch.World, entity: clutch.EntityId, torque_nm: Vec3) void {
            const res = world.getResource(Resource) orelse return;
            res.world.applyTorque(bodyOf(world, entity) orelse return, toAngular(torque_nm));
        }

        /// Pushes `entity`'s body by `impulse_ns` (N·s) at once.
        pub fn applyImpulse(world: *clutch.World, entity: clutch.EntityId, impulse_ns: Vec3, at: At) void {
            const res = world.getResource(Resource) orelse return;
            res.world.applyImpulse(bodyOf(world, entity) orelse return, toVec(impulse_ns), impulsePoint(at));
        }

        fn impulsePoint(at: At) World.ImpulsePoint {
            return switch (at) {
                .center_of_mass => .center_of_mass,
                .world_point => |point| .{ .at_world_point = toVec(point) },
            };
        }

        pub const Ground = struct {
            entity: clutch.EntityId,
            point: Vec3,
            /// Out of the ground.
            normal: Vec3,
            /// How fast the ground moves where it is stood on, in m/s: what a character
            /// standing there moves with to ride it.
            velocity: Vec3,
        };

        /// What `entity`'s body stood on in the last step, within `max_slope_deg` of `up`.
        pub fn groundOf(world: *clutch.World, entity: clutch.EntityId, up: Vec3, max_slope_deg: f32) ?Ground {
            const res = world.getResource(Resource) orelse return null;
            const found = res.world.groundOf(bodyOf(world, entity) orelse return null, toVec(up), max_slope_deg * std.math.pi / 180.0) orelse return null;
            return .{ .entity = entityOf(res.world.colliderData(found.collider)), .point = fromVec(found.point), .normal = fromVec(found.normal), .velocity = fromVec(found.velocity) };
        }

        /// An entity's body and collider, and what they were built from, so an edit rebuilds them.
        pub const Link = struct {
            /// Its own body, from its `RigidBody`.
            body: ?BodyHandle = null,
            rigid_body: ?RigidBody = null,
            /// Null when its shape couldn't be built; it is tried again once something changes.
            collider: ?ColliderHandle = null,
            source: ?Collider = null,
            /// The body its collider is on: its own, the nearest above it, or none for scenery.
            collider_body: ?BodyHandle = null,
            /// Where the collider sits on that body, or in the world as scenery.
            collider_pose: World.Pose = .identity(),
            /// The scale its shape was built at.
            collider_scale: Vec3 = .one,
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
            if (!two) {
                JointSync.install(world);
                VehicleSync.install(world);
            }
        }

        const JointSync = joints_zig.Sync(@This());
        const VehicleSync = vehicles_zig.Sync(@This());

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
            const owner = ownerOf(world, entity);
            const kind: ?BodyKind = if (owner) |found| world.get(found, RigidBody).?.kind else null;
            const color = debug_draw.palette.of(kind, if (owner) |found| sleeping(world, found) else false, collider.sensor);
            var lines: Lines(@TypeOf(sink)) = .{ .sink = sink };
            try debug_draw.DebugDraw(S).shape(shape.shape, poseOf(place).mul(offsetPose(collider, place.scale)), color, &lines);
        }

        /// The nearest entity from `entity` up through its parents with a `RigidBody`: whose
        /// body its collider joins.
        pub fn ownerOf(world: *clutch.World, entity: clutch.EntityId) ?clutch.EntityId {
            var at: ?clutch.EntityId = entity;
            while (at) |current| : (at = world.getParent(current)) {
                if (world.get(current, RigidBody) != null) return current;
            }
            return null;
        }

        /// The joints', centres of mass', vehicles' and contact forces' wireframes from the last
        /// step, through `sink` (as `drawColliders` takes it). Nothing without a running world.
        pub fn drawMechanics(world: *clutch.World, sink: anytype) !void {
            const res = world.getResource(Resource) orelse return;
            var lines: Lines(@TypeOf(sink)) = .{ .sink = sink };
            try res.world.debugDraw(.{ .colliders = false, .contacts = true, .force_m_per_n = 1.0 / 5000.0, .centers = true, .joints = true, .vehicles = true }, &lines);
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

        /// Builds bodies and colliders for new and edited entities, moves those the game moved,
        /// then joints and vehicles.
        fn sync(res: *Resource, bodies: clutch.Query(.{ *const RigidBody, *const Host.Transform })) !void {
            const world = bodies.world;
            try syncBodies(res, bodies);
            try syncColliders(res, world);
            if (!two) {
                try JointSync.sync(res, world);
                try VehicleSync.sync(res, world);
            }
        }

        /// A body for each `RigidBody`: built when new, changed in place when edited, so its
        /// colliders, joints and vehicle stay on it.
        fn syncBodies(res: *Resource, bodies: clutch.Query(.{ *const RigidBody, *const Host.Transform })) !void {
            const world = bodies.world;
            const gpa = res.world.gpa;
            var fresh: std.ArrayList(struct { entity: clutch.EntityId, link: Link }) = .empty;
            defer fresh.deinit(gpa);
            var query = bodies;
            while (query.next()) |view| {
                const entity = view.entity;
                const rigid_body = view.get(*const RigidBody).*;
                const place = Host.place(world, entity, view.get(*const Host.Transform).*);
                const link = world.get(entity, Link) orelse {
                    var made: Link = .{ .place = place };
                    made.body = addBody(res, world, entity, rigid_body, place) orelse continue;
                    made.rigid_body = rigid_body;
                    try fresh.append(gpa, .{ .entity = entity, .link = made });
                    continue;
                };
                const body = if (link.body) |handle| if (res.world.contains(handle)) handle else null else null;
                if (body == null) {
                    link.body = addBody(res, world, entity, rigid_body, place) orelse continue;
                    link.rigid_body = rigid_body;
                    link.place = place;
                    continue;
                }
                if (!std.meta.eql(link.rigid_body, rigid_body)) {
                    res.world.setProperties(body.?, propertiesOf(rigid_body));
                    link.rigid_body = rigid_body;
                }
                if (!std.meta.eql(link.place, place)) {
                    res.world.setPose(body.?, poseOf(place));
                    link.place = place;
                }
                const linear = if (world.get(entity, LinearVelocity)) |v| v.value else link.linear;
                const angular = if (world.get(entity, AngularVelocity)) |v| v.value else link.angular;
                if (!linear.eql(link.linear) or !angular.eql(link.angular)) {
                    res.world.setVelocity(body.?, .{ .linear = toVec(linear), .angular = toAngular(angular) });
                    link.linear = linear;
                    link.angular = angular;
                }
            }
            for (fresh.items) |made| try world.addComponent(made.entity, made.link);

            // An entity that lost its `RigidBody` loses its body; its colliders are built again
            // on whatever is left.
            var unbodied = world.query(.{ *Link, clutch.Without(RigidBody) });
            while (unbodied.next()) |view| {
                const link = view.get(*Link);
                const body = link.body orelse continue;
                res.world.removeBody(body);
                link.body = null;
                link.rigid_body = null;
            }
        }

        fn addBody(res: *Resource, world: *clutch.World, entity: clutch.EntityId, rigid_body: RigidBody, place: Place) ?BodyHandle {
            const linear = if (world.get(entity, LinearVelocity)) |v| v.value else Vec3.zero;
            const angular = if (world.get(entity, AngularVelocity)) |v| v.value else Vec3.zero;
            const properties = propertiesOf(rigid_body);
            return res.world.addBody(.{
                .kind = properties.kind,
                .pose = poseOf(place),
                .mass_kg = properties.mass_kg,
                .center_of_mass_m = properties.center_of_mass_m,
                .linear_damping = properties.linear_damping,
                .angular_damping = properties.angular_damping,
                .gravity_scale = properties.gravity_scale,
                .rotation = properties.rotation,
                .enabled = properties.enabled,
                .linear_velocity = toVec(linear),
                .angular_velocity = toAngular(angular),
            }, dataOf(entity)) catch |err| {
                log.err("cannot add a body: {t}", .{err});
                return null;
            };
        }

        fn propertiesOf(rigid_body: RigidBody) World.Properties {
            return .{
                .kind = rigid_body.kind,
                .mass_kg = rigid_body.mass_kg,
                .center_of_mass_m = if (rigid_body.center_of_mass_m) |centre| toVec(centre) else null,
                .linear_damping = rigid_body.linear_damping,
                .angular_damping = rigid_body.angular_damping,
                .gravity_scale = rigid_body.gravity_scale,
                .rotation = rigid_body.rotation,
                .enabled = rigid_body.enabled,
            };
        }

        /// A collider for each `Collider`, on the body of the nearest entity at or above it with
        /// one; rebuilt when its shape, scale or body changes, moved when it moves on its body.
        fn syncColliders(res: *Resource, world: *clutch.World) !void {
            const gpa = res.world.gpa;
            var fresh: std.ArrayList(struct { entity: clutch.EntityId, link: Link }) = .empty;
            defer fresh.deinit(gpa);
            var query = world.query(.{ *const Collider, *const Host.Transform });
            while (query.next()) |view| {
                const entity = view.entity;
                const collider = view.get(*const Collider).*;
                const place = Host.place(world, entity, view.get(*const Host.Transform).*);
                const owner = ownerOf(world, entity);
                const owner_body: ?BodyHandle = if (owner) |found| if (world.get(found, Link)) |owner_link| owner_link.body else null else null;
                // An owner whose body isn't built yet waits for it rather than being scenery a frame.
                if (owner != null and owner_body == null) continue;
                const pose = colliderPose(world, collider, place, owner);
                const link = world.get(entity, Link) orelse {
                    var made: Link = .{ .place = place };
                    attachCollider(res, world, entity, &made, collider, place, owner_body, pose);
                    try fresh.append(gpa, .{ .entity = entity, .link = made });
                    continue;
                };
                const live = if (link.collider) |handle| res.world.contains(handle) else false;
                var unchanged = collider;
                if (link.source) |old| unchanged.enabled = old.enabled;
                const same = live and link.source != null and std.meta.eql(link.source.?, unchanged) and
                    link.collider_scale.eql(place.scale) and std.meta.eql(link.collider_body, owner_body);
                if (!same) {
                    if (link.collider) |handle| if (live) res.world.removeCollider(handle);
                    attachCollider(res, world, entity, link, collider, place, owner_body, pose);
                } else {
                    if (link.source.?.enabled != collider.enabled) res.world.setColliderEnabled(link.collider.?, collider.enabled);
                    link.source = collider;
                    if (!samePose(link.collider_pose, pose)) {
                        res.world.setColliderPose(link.collider.?, pose);
                        link.collider_pose = pose;
                    }
                }
                if (link.body == null) link.place = place;
            }
            for (fresh.items) |made| try world.addComponent(made.entity, made.link);
        }

        /// Where `entity`'s collider sits on `owner`'s body, or in the world without one.
        fn colliderPose(world: *clutch.World, collider: Collider, place: Place, owner: ?clutch.EntityId) World.Pose {
            const offset = offsetPose(collider, place.scale);
            const found = owner orelse return poseOf(place).mul(offset);
            const owner_place = Host.place(world, found, (world.get(found, Host.Transform) orelse return offset).*);
            if (std.meta.eql(owner_place, place)) return offset;
            return poseOf(owner_place).inverse().mul(poseOf(place)).mul(offset);
        }

        /// Whether two collider poses differ by less than the noise of working them out from
        /// transforms again, so a child collider isn't moved, and its body woken, every frame.
        fn samePose(a: World.Pose, b: World.Pose) bool {
            const near_m: T = 1e-5;
            if (a.translation.sub(b.translation).lengthSquared() > near_m * near_m) return false;
            if (two) return @abs(a.rotation.angle() - b.rotation.angle()) < 1e-5;
            return @abs(@reduce(.Add, a.rotation.quat.v * b.rotation.quat.v)) > 1 - 1e-9;
        }

        /// Builds `collider`'s shape and attaches it to `body` (scenery when null) at `pose`.
        /// What can't be built is left null and logged; it is tried again when it changes.
        fn attachCollider(res: *Resource, world: *clutch.World, entity: clutch.EntityId, link: *Link, collider: Collider, place: Place, body: ?BodyHandle, pose: World.Pose) void {
            link.source = collider;
            link.collider = null;
            link.collider_body = body;
            link.collider_pose = pose;
            link.collider_scale = place.scale;
            var shape = shapeOf(collider, place, world, entity, res.world.gpa) catch |err| {
                log.warn("cannot shape a collider ({t}): {t}", .{ collider.shape, err });
                return;
            };
            // The shape's memory is the world's from here, without another copy.
            link.collider = res.world.addColliderOwned(.{
                .shape = shape.shape,
                .local_pose = pose,
                .material = .{ .restitution = collider.restitution, .friction = collider.friction },
                .sensor = collider.sensor,
                .mass_kg = if (collider.mass_kg) |kg| kg else null,
                .enabled = collider.enabled,
            }, body, dataOf(entity)) catch |err| {
                shape.deinit(res.world.gpa);
                log.warn("cannot add a collider ({t}): {t}", .{ collider.shape, err });
                return;
            };
        }

        /// Entities that lost their `Collider` lose their collider, and those with neither it nor
        /// a `RigidBody` their `Link`.
        fn unlinkOrphans(res: *Resource, links: clutch.Query(.{ *Link, clutch.Without(Collider) }), commands: clutch.Commands) !void {
            var query = links;
            while (query.next()) |view| {
                const link = view.get(*Link);
                if (link.collider) |handle| res.world.removeCollider(handle);
                link.collider = null;
                link.source = null;
                link.collider_body = null;
                if (link.body == null) try commands.removeComponent(view.entity, Link);
            }
        }

        fn step(res: *Resource, time: *const Host.Time, settings: clutch.Query(.{*const PhysicsSettings})) !void {
            var found = settings;
            if (found.next()) |view| res.world.config = configOf(view.get(*const PhysicsSettings).*);
            const delta_s = Host.deltaSeconds(time);
            if (!std.math.isFinite(delta_s)) return;
            const world = settings.world;
            var inside: Inside = .{ .world = world, .fixed = if (world.getResource(FixedStep)) |fixed| fixed.* else null };
            try res.world.updateWith(@floatCast(delta_s), .{ .context = &inside, .call = Inside.run });
        }

        /// What runs inside each step: the game's `FixedStep`, then the vehicles' input it set.
        const Inside = struct {
            world: *clutch.World,
            fixed: ?FixedStep,

            fn run(context: *anyopaque, _: *World, dt_s: T) anyerror!void {
                const self: *Inside = @ptrCast(@alignCast(context));
                const fixed = self.fixed orelse return;
                try fixed.call(fixed.context, self.world, @floatCast(dt_s));
                if (!two) VehicleSync.readInput(self.world);
            }
        };

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
                if (link.rigid_body.?.kind == .static or !link.rigid_body.?.enabled or res.world.isSleeping(body)) continue;
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
            if (!two) {
                JointSync.writeBack(res, links.world);
                VehicleSync.writeBack(res, links.world);
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

        fn offsetPose(collider: Collider, scale: Vec3) World.Pose {
            return World.Pose.fromTranslation(toVec(collider.offset_m.mul(scale)));
        }

        /// A built shape, and the memory its points, vertices or heights are in, which the world
        /// takes (`addColliderOwned`) or `deinit` frees.
        const Built = struct {
            shape: World.Shape,
            vertices: []S.Vec = &.{},
            indices: [][S.n]u32 = &.{},
            heights: []T = &.{},

            fn deinit(self: *Built, gpa: std.mem.Allocator) void {
                gpa.free(self.vertices);
                gpa.free(self.indices);
                gpa.free(self.heights);
            }
        };

        /// `collider`'s shape on `entity`, which is at `place`.
        fn shapeOf(collider: Collider, place: Place, world: *clutch.World, entity: clutch.EntityId, gpa: std.mem.Allocator) !Built {
            if (analytic(collider.shape, place.scale)) |exact| return .{ .shape = exact };
            return switch (collider.shape) {
                .heightfield => |*name| fromHeightfield(name.text(), place.scale, world, gpa),
                else => fromMesh(collider, place, world, entity, gpa),
            };
        }

        /// The grid named `name`, at the entity's `scale`.
        fn fromHeightfield(name: []const u8, scale: Vec3, world: *clutch.World, gpa: std.mem.Allocator) !Built {
            if (!@hasDecl(Host, "heightfield")) return error.NoHeightfieldSource;
            if (two) return error.NoHeightfieldIn2D;
            const grid = Host.heightfield(world, name) orelse return error.NoHeightfield;
            if (grid.columns < 2 or grid.heights.len < 2 * @as(usize, grid.columns) or grid.heights.len % grid.columns != 0) return error.NoHeightfield;
            const heights = try gpa.alloc(T, grid.heights.len);
            for (heights, grid.heights) |*out, h| out.* = h * scale.y;
            const cell_x: T = grid.cell_m * @abs(scale.x);
            const cell_z: T = grid.cell_m * @abs(scale.z);
            return .{ .shape = .{ .heightfield = .init(heights, grid.columns, cell_x, cell_z) }, .heights = heights };
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
                .convex_hull_from_mesh, .mesh, .heightfield => null,
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

        pub fn dataOf(entity: clutch.EntityId) u64 {
            return (@as(u64, entity.id) << 32) | entity.generation;
        }

        pub fn entityOf(data: u64) clutch.EntityId {
            return clutch.EntityId.init(@truncate(data >> 32), @truncate(data));
        }

        pub fn poseOf(place: Place) World.Pose {
            if (two) {
                const turned = place.rotation.rotateVector(Vec3.unit_x);
                return World.Pose.init(toVec(place.position), std.math.atan2(@as(T, turned.y), @as(T, turned.x)));
            } else {
                const Quat = @FieldType(emath.UnitQuaternion(T), "quat");
                const parts: @Vector(4, T) = @floatCast(place.rotation.quat.v);
                return World.Pose.fromParts(toVec(place.position), emath.UnitQuaternion(T).newUnchecked(Quat{ .v = parts }));
            }
        }

        pub fn fromRotation(rotation: emath.UnitQuaternion(T)) emath.UnitQuat {
            const Quat = @FieldType(emath.UnitQuat, "quat");
            const parts: @Vector(4, f32) = @floatCast(rotation.quat.v);
            return emath.UnitQuat.newUnchecked(Quat{ .v = parts });
        }

        pub fn toVec(v: Vec3) S.Vec {
            return if (two) S.Vec.init(v.x, v.y) else S.Vec.init(v.x, v.y, v.z);
        }

        pub fn fromVec(v: S.Vec) Vec3 {
            return if (two) Vec3.init(@floatCast(v.x), @floatCast(v.y), 0) else Vec3.init(@floatCast(v.x), @floatCast(v.y), @floatCast(v.z));
        }

        pub fn toAngular(v: Vec3) S.Angular {
            return if (two) v.z else S.Vec.init(v.x, v.y, v.z);
        }

        pub fn fromAngular(v: S.Angular) Vec3 {
            return if (two) Vec3.init(0, 0, @floatCast(v)) else fromVec(v);
        }
    };
}

/// A host for tests: its transform is a place within its parent's.
const TestHost = struct {
    pub const Transform = struct { position: Vec3 = .zero, rotation: emath.UnitQuat = .identity(), scale: Vec3 = .one };
    pub const Time = struct { delta_s: f32 };
    pub fn deltaSeconds(time: *const Time) f32 {
        return time.delta_s;
    }
    pub fn place(world: *clutch.World, entity: clutch.EntityId, transform: Transform) Place {
        const local: Place = .{ .position = transform.position, .rotation = transform.rotation, .scale = transform.scale };
        const parent = world.getParent(entity) orelse return local;
        const above = place(world, parent, (world.get(parent, Transform) orelse return local).*);
        return .{ .position = above.position.add(above.rotation.rotateVector(local.position.mul(above.scale))), .rotation = above.rotation.mul(local.rotation), .scale = above.scale.mul(local.scale) };
    }
    pub fn setPlace(world: *clutch.World, entity: clutch.EntityId, transform: *Transform, position: Vec3, rotation: emath.UnitQuat) void {
        transform.position = position;
        transform.rotation = rotation;
        const parent = world.getParent(entity) orelse return;
        const above = place(world, parent, (world.get(parent, Transform) orelse return).*);
        transform.position = above.rotation.inverseRotateVector(position.sub(above.position)).div(above.scale);
        transform.rotation = above.rotation.inverse().mul(rotation);
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

test "removing a Collider frees it, removing the RigidBody frees its body, and despawning frees both" {
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
    try std.testing.expectEqual(@as(usize, 1), res.world.colliders.entries.items.len - res.world.colliders.free.items.len);
    try std.testing.expect(res.world.contains(world.get(ball, P.Link).?.body.?));
    world.remove(ball, RigidBody);
    try runFrames(&world, 1);
    world.despawn(wall);
    try std.testing.expectEqual(@as(usize, 0), res.world.colliders.entries.items.len - res.world.colliders.free.items.len);
    try std.testing.expectEqual(@as(usize, 0), res.world.bodies.entries.items.len - res.world.bodies.free.items.len);
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

test "a truck made of components drives on its wheels from a FixedStep, carries a machine, and tows a hitched trailer" {
    const P = Plugin(.three, f32, TestHost);
    const gpa = std.testing.allocator;
    var world = clutch.World.init(gpa);
    defer world.deinit();
    try world.insertResource(TestHost.Time{ .delta_s = 1.0 / 60.0 });
    try P.install(&world, gpa, .{ .steps_per_second = 120 });
    defer world.runStage(.Deinit) catch {};

    // Each step after the second second, the game's own hook asks for half throttle.
    const Game = struct {
        steps: usize = 0,
        fn fixed(context: *anyopaque, clutch_world: *clutch.World, step_s: f32) anyerror!void {
            const game: *@This() = @ptrCast(@alignCast(context));
            game.steps += 1;
            if (@as(f32, @floatFromInt(game.steps)) * step_s < 2) return;
            var drivers = clutch_world.query(.{ *VehicleInput, *const VehicleState });
            while (drivers.next()) |view| view.get(*VehicleInput).throttle = 0.5;
        }
    };
    var game: Game = .{};
    try world.insertResource(FixedStep{ .context = &game, .call = Game.fixed });

    _ = try world.spawn(.{ TestHost.Transform{}, Collider{ .shape = .{ .half_space = .{} } } });
    const truck = try world.spawn(.{ TestHost.Transform{ .position = .init(0, 1.2, 0) }, RigidBody{ .mass_kg = 3000, .center_of_mass_m = .init(0, -0.3, 0) }, Collider{ .shape = .{ .cuboid = .{ .half_extents_m = .init(1.1, 0.5, 2.8) } } }, Vehicle{}, VehicleInput{}, VehicleState{} });
    const machine = try world.spawn(.{ TestHost.Transform{ .position = .init(0, 0.75, 1) }, Collider{ .shape = .{ .cuboid = .{ .half_extents_m = .splat(0.25) } }, .mass_kg = 500 } });
    try world.setParent(machine, truck);
    const trailer = try world.spawn(.{ TestHost.Transform{ .position = .init(0, 1.2, 7) }, RigidBody{ .mass_kg = 1500, .center_of_mass_m = .init(0, -0.3, 0) }, Collider{ .shape = .{ .cuboid = .{ .half_extents_m = .init(1.1, 0.4, 3) } } }, Vehicle{}, VehicleInput{}, Joint{ .other = .init(truck), .kind = .{ .ball = .{ .axis = .init(0, 0, 1), .cone_limit_deg = 75 } }, .anchor_m = .init(0, -0.3, -3.8), .other_anchor_m = .init(0, -0.3, 3.2) }, JointState{} });
    var front_left: clutch.EntityId = undefined;
    var spinner: clutch.EntityId = undefined;
    for ([_][3]f32{ .{ -1, -1.8, 0 }, .{ 1, -1.8, 0 }, .{ -1, 1.8, 1 }, .{ 1, 1.8, 1 }, .{ -1, 1.8, 2 }, .{ 1, 1.8, 2 } }, 0..) |corner, i| {
        const on_trailer = corner[2] == 2;
        const wheel = try world.spawn(.{ TestHost.Transform{ .position = .init(corner[0], -0.3, corner[1]) }, Wheel{ .steers = corner[2] == 0, .driven = corner[2] == 1, .handbrake = corner[2] != 0 }, WheelState{} });
        try world.setParent(wheel, if (on_trailer) trailer else truck);
        if (i == 0) {
            front_left = wheel;
            spinner = try world.spawn(.{TestHost.Transform{ .position = .init(-1, 0.4, -1.8) }});
            world.get(wheel, Wheel).?.visual = .init(spinner);
        }
    }
    try runFrames(&world, 120);

    const res = world.getResource(P.Resource).?;
    try std.testing.expectApproxEqAbs(@as(f32, 3500), res.world.massOf(world.get(truck, P.Link).?.body.?).mass_kg, 1e-2);
    const state = world.get(front_left, WheelState).?;
    try std.testing.expect(state.grounded and state.suspension_force_n > 5000);
    // The wheel's visual sits at its hub, a radius above the ground.
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), world.get(spinner, TestHost.Transform).?.position.y, 0.02);

    try runFrames(&world, 180);
    try std.testing.expectEqual(@as(usize, 600), game.steps);
    try std.testing.expect(world.get(truck, VehicleState).?.speed_mps > 2);
    try std.testing.expect(world.get(truck, TestHost.Transform).?.position.z < -2);
    // The trailer came along on the hitch.
    try std.testing.expectApproxEqAbs(world.get(truck, TestHost.Transform).?.position.z + 7, world.get(trailer, TestHost.Transform).?.position.z, 0.1);
    try std.testing.expect(world.get(trailer, JointState).?.force_n > 100);
}
