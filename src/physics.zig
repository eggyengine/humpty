//! The physics world: bodies, the colliders they touch it with, the joints and vehicles that
//! hold and drive them, and the fixed step that moves them under gravity and pushes them apart
//! where they meet. One world type per space: 2D or 3D, in `f32` or `f64`; joints and vehicles
//! are 3D only.
const std = @import("std");
const space = @import("space.zig");
const b = @import("body.zig");
const c = @import("collider.zig");
const shapes_zig = @import("shapes.zig");
const narrowphase_zig = @import("narrowphase.zig");
const solver_zig = @import("solver.zig");
const ray_zig = @import("ray.zig");
const shapecast_zig = @import("shapecast.zig");
const joint_zig = @import("joint.zig");
const vehicle_zig = @import("vehicle.zig");
const debug_draw = @import("debug_draw.zig");
const Slots = @import("slots.zig").Slots;

/// A world in `dim` dimensions with scalar `T` (`f32` or `f64`).
pub fn Physics(comptime dim: space.Dim, comptime T: type) type {
    return struct {
        const Self = @This();
        pub const S = space.Space(dim, T);
        pub const Vec = S.Vec;
        pub const Pose = S.Pose;
        pub const Shape = shapes_zig.Shapes(S).Shape;
        pub const Heightfield = shapes_zig.Shapes(S).Heightfield;
        pub const Body = b.Bodies(S).Body;
        pub const Collider = c.Colliders(S).Collider;
        pub const Material = c.Colliders(S).Material;
        pub const Ray = ray_zig.Rays(S).Ray;
        pub const RayHit = ray_zig.Rays(S).RayHit;
        pub const State = b.Bodies(S).State;
        const Narrowphase = narrowphase_zig.Narrowphase(S);
        const Solver = solver_zig.Solver(S);
        const Rays = ray_zig.Rays(S);
        const Casts = shapecast_zig.Casts(S);
        const three = dim == .three;
        /// Joints and vehicles, in 3D; nothing in 2D.
        pub const Joints = if (three) joint_zig.Joints(S) else struct {
            pub const Stored = void;
            pub const Row = void;
            pub const Joint = void;
        };
        pub const Vehicles = if (three) vehicle_zig.Vehicles(S) else struct {
            pub const Vehicle = struct { body: b.BodyHandle };
        };
        pub const Joint = Joints.Joint;
        pub const Vehicle = Vehicles.Vehicle;
        pub const JointHandle = joint_zig.JointHandle;
        pub const VehicleHandle = vehicle_zig.VehicleHandle;

        /// What can change about a body after it is added, keeping where it is and how it moves.
        pub const Properties = struct {
            kind: b.BodyKind,
            mass_kg: T = 0,
            center_of_mass_m: ?Vec = null,
            linear_damping: T = 0,
            angular_damping: T = 0,
            gravity_scale: T = 1,
            rotation: b.Rotation = .free,
            enabled: bool = true,
        };

        /// Code run inside each fixed step, before the step's forces, contacts and moves, such
        /// as a game's own suspension. Forces applied from it last that step alone.
        pub const StepHook = struct {
            context: *anyopaque,
            call: *const fn (context: *anyopaque, world: *Self, dt_s: T) anyerror!void,
        };

        pub const ImpulsePoint = union(enum) {
            center_of_mass,
            at_world_point: Vec,
        };

        pub const Velocity = struct {
            /// m/s.
            linear: Vec,
            /// rad/s: about z in 2D, axis times speed in 3D.
            angular: S.Angular,
        };

        pub const Contact = struct {
            a: c.ColliderHandle,
            b: c.ColliderHandle,
            /// World space.
            point: Vec,
            /// From `a` towards `b`.
            normal: Vec,
            depth_m: T,
            /// How hard the step pushed them apart here, in N·s; divide by the step for newtons.
            impulse_ns: T = 0,

            pub fn involves(self: Contact, collider: c.ColliderHandle) bool {
                return self.a == collider or self.b == collider;
            }
        };

        /// Two colliders that started or stopped touching, the lower handle first.
        pub const Pair = struct { a: c.ColliderHandle, b: c.ColliderHandle };

        pub const Config = struct {
            /// m/s², world space: `.init(0, -9.81, 0)` for Earth with y up.
            gravity: Vec,
            /// How long each step is: `1.0 / 60.0` steps sixty times a second of game time.
            fixed_step_s: T,
        };

        gpa: std.mem.Allocator,
        config: Config,
        bodies: Slots(Attached(State), b.BodyHandle) = .{},
        colliders: Slots(Attached(Collider), c.ColliderHandle) = .{},
        joints: Slots(StoredJoint, JointHandle) = .{},
        vehicles: Slots(Attached(Vehicle), VehicleHandle) = .{},
        /// The step being taken, while one is: forces applied now last only it.
        stepping_dt_s: ?T = null,

        accumulator_s: T = 0,
        /// Per-step scratch, cleared at the start of each step but kept allocated,
        /// so after the first few steps the solver stops asking `gpa` for memory.
        placed: std.ArrayList(Placed) = .empty,
        pairs: std.ArrayList(ColliderPair) = .empty,
        constraints: std.ArrayList(Solver.Constraint) = .empty,
        rows: std.ArrayList(Joints.Row) = .empty,
        contact_list: std.ArrayList(Contact) = .empty,
        touching: std.AutoArrayHashMapUnmanaged(Pair, void) = .empty,
        touched: std.AutoArrayHashMapUnmanaged(Pair, void) = .empty,
        started_list: std.ArrayList(Pair) = .empty,
        ended_list: std.ArrayList(Pair) = .empty,

        /// Longest frame `update` will catch up on, so a stall slows the
        /// simulation instead of running hundreds of steps at once.
        const max_frame_dt_s: T = 0.25;
        /// Passes the solver makes over the contacts each step.
        const solver_iterations = 16;
        /// A body slower than these for `sleep_after_s` sleeps until something touches or pushes it.
        const sleep_linear_mps: T = 0.05;
        const sleep_angular_radps: T = 0.05;
        const sleep_after_s: T = 0.5;

        /// A stored item with the number its owner tagged it with (`user_data`).
        fn Attached(comptime Item: type) type {
            return struct {
                item: Item,
                user_data: u64,
                /// A collider's body; always null for a body.
                parent: ?b.BodyHandle = null,
            };
        }

        /// A collider where this step has it, with the body it moves with.
        const Placed = struct {
            handle: c.ColliderHandle,
            collider: *const Collider,
            pose: Pose,
            bounds: S.Aabb,
            body: ?*State,
        };

        /// Pairs of colliders whose bounds overlap, found by the broadphase, as indices into `placed`.
        const ColliderPair = struct { a: u32, b: u32 };

        /// A joint with the bodies it holds; `b` null holds `a` to the world.
        pub const StoredJoint = struct {
            stored: Joints.Stored,
            a: b.BodyHandle,
            b: ?b.BodyHandle,
            user_data: u64,
        };

        pub fn init(gpa: std.mem.Allocator, config: Config) !Self {
            return .{ .gpa = gpa, .config = config };
        }

        pub fn deinit(self: *Self) void {
            for (self.colliders.entries.items) |entry| if (entry.item) |attached| self.freeShape(attached.item.shape);
            self.bodies.deinit(self.gpa);
            self.colliders.deinit(self.gpa);
            self.joints.deinit(self.gpa);
            self.vehicles.deinit(self.gpa);
            self.placed.deinit(self.gpa);
            self.pairs.deinit(self.gpa);
            self.constraints.deinit(self.gpa);
            self.rows.deinit(self.gpa);
            self.contact_list.deinit(self.gpa);
            self.touching.deinit(self.gpa);
            self.touched.deinit(self.gpa);
            self.started_list.deinit(self.gpa);
            self.ended_list.deinit(self.gpa);
        }

        /// Adds a body with no colliders yet: it moves, but touches nothing until one is attached.
        /// `user_data` is the caller's own tag for it, read back with `bodyData`.
        pub fn addBody(self: *Self, body: Body, user_data: u64) !b.BodyHandle {
            return self.bodies.add(self.gpa, .{ .item = State.init(body), .user_data = user_data });
        }

        /// Removes the body with every collider, joint and vehicle on it. A stale handle is ignored.
        pub fn removeBody(self: *Self, body: b.BodyHandle) void {
            for (self.colliders.entries.items, 0..) |entry, index| {
                const attached = entry.item orelse continue;
                if (attached.parent != null and attached.parent.? == body) self.removeCollider(self.colliders.handleAt(index));
            }
            for (self.joints.entries.items, 0..) |entry, index| {
                const joint = entry.item orelse continue;
                if (joint.a == body or (joint.b != null and joint.b.? == body)) _ = self.joints.remove(self.joints.handleAt(index));
            }
            for (self.vehicles.entries.items, 0..) |entry, index| {
                const vehicle = entry.item orelse continue;
                if (vehicle.item.body == body) _ = self.vehicles.remove(self.vehicles.handleAt(index));
            }
            _ = self.bodies.remove(body);
        }

        /// Whether `handle`, a body's, collider's, joint's or vehicle's, still names something
        /// in the world.
        pub fn contains(self: *const Self, handle: anytype) bool {
            return switch (@TypeOf(handle)) {
                b.BodyHandle => self.bodies.get(handle) != null,
                c.ColliderHandle => self.colliders.get(handle) != null,
                JointHandle => self.joints.get(handle) != null,
                VehicleHandle => self.vehicles.get(handle) != null,
                else => @compileError("contains takes a body, collider, joint or vehicle handle"),
            };
        }

        /// Attaches `collider` to `parent`, or with no parent makes it static scenery. A hull's
        /// points, a mesh's vertices and indices and a heightfield's heights are copied. The
        /// parent's mass, centre of mass and inertia are worked out again with it.
        pub fn addCollider(self: *Self, collider: Collider, parent: ?b.BodyHandle, user_data: u64) !c.ColliderHandle {
            if (!collider.shape.supported()) return error.UnsupportedShape;
            var owned = collider;
            owned.shape = try self.copyShape(collider.shape);
            errdefer self.freeShape(owned.shape);
            return self.addColliderOwned(owned, parent, user_data);
        }

        /// `addCollider` without the copy: the world takes the shape's points, vertices and
        /// indices or heights, which must come from its allocator, and frees them when the
        /// collider goes. On an error they are still the caller's.
        pub fn addColliderOwned(self: *Self, collider: Collider, parent: ?b.BodyHandle, user_data: u64) !c.ColliderHandle {
            if (!collider.shape.supported()) return error.UnsupportedShape;
            if (parent) |body| if (self.bodies.get(body) == null) return error.StaleBody;
            const handle = try self.colliders.add(self.gpa, .{ .item = collider, .user_data = user_data, .parent = parent });
            if (parent) |body| self.updateMass(body);
            return handle;
        }

        /// Detaches and removes `collider`. A stale handle is ignored.
        pub fn removeCollider(self: *Self, collider: c.ColliderHandle) void {
            const attached = (self.colliders.get(collider) orelse return).*;
            _ = self.colliders.remove(collider);
            self.freeShape(attached.item.shape);
            if (attached.parent) |body| if (self.bodies.get(body) != null) self.updateMass(body);
        }

        fn copyShape(self: *Self, shape: Shape) !Shape {
            return switch (shape) {
                .convex_hull => |hull| .{ .convex_hull = .{ .points = try self.gpa.dupe(Vec, hull.points) } },
                .mesh => |mesh| blk: {
                    const vertices = try self.gpa.dupe(Vec, mesh.vertices);
                    errdefer self.gpa.free(vertices);
                    break :blk .{ .mesh = .{ .vertices = vertices, .indices = try self.gpa.dupe([S.n]u32, mesh.indices) } };
                },
                .heightfield => |field| blk: {
                    var copied = field;
                    copied.heights = try self.gpa.dupe(T, field.heights);
                    break :blk .{ .heightfield = copied };
                },
                else => shape,
            };
        }

        fn freeShape(self: *Self, shape: Shape) void {
            switch (shape) {
                .convex_hull => |hull| self.gpa.free(hull.points),
                .mesh => |mesh| {
                    self.gpa.free(mesh.vertices);
                    self.gpa.free(mesh.indices);
                },
                .heightfield => |field| self.gpa.free(field.heights),
                else => {},
            }
        }

        /// The body `collider` is attached to, or null for static scenery or a stale handle.
        pub fn colliderBody(self: *const Self, collider: c.ColliderHandle) ?b.BodyHandle {
            const attached = self.colliders.get(collider) orelse return null;
            return attached.parent;
        }

        /// The tag `collider` was added with. It must be in the world.
        pub fn colliderData(self: *const Self, collider: c.ColliderHandle) u64 {
            return self.colliders.get(collider).?.user_data;
        }

        /// The tag `body` was added with. It must be in the world.
        pub fn bodyData(self: *const Self, body: b.BodyHandle) u64 {
            return self.bodies.get(body).?.user_data;
        }

        /// Advances the world by a frame of `frame_dt_s` seconds, in as many fixed steps as fit
        /// (the remainder carries to the next frame, and a long frame is capped so a stall
        /// doesn't run away). Contacts and started/ended pairs are the last step's.
        pub fn update(self: *Self, frame_dt_s: T) !void {
            try self.updateWith(frame_dt_s, null);
        }

        /// `update`, running `hook` inside each fixed step. Forces applied before it last every
        /// step of this frame; on a frame too short for a step, their push carries to the next.
        pub fn updateWith(self: *Self, frame_dt_s: T, hook: ?StepHook) !void {
            const frame_s = @min(frame_dt_s, max_frame_dt_s);
            self.accumulator_s += frame_s;
            var steps: usize = 0;
            while (self.accumulator_s >= self.config.fixed_step_s) : (steps += 1) {
                try self.step(self.config.fixed_step_s, hook);
                self.accumulator_s -= self.config.fixed_step_s;
            }
            for (self.bodies.entries.items) |*entry| {
                const state = &(entry.item orelse continue).item;
                if (steps == 0) {
                    state.carried = state.carried.add(state.force.scale(frame_s));
                    state.carried_turn = S.angularAdd(state.carried_turn, S.angularScale(state.torque, frame_s));
                }
                state.force = .zero;
                state.torque = S.angular_zero;
            }
        }

        fn step(self: *Self, dt_s: T, hook: ?StepHook) !void {
            // Cleared here rather than at the end, so `contacts()` still shows the
            // last step's contacts until the next one runs.
            self.placed.clearRetainingCapacity();
            self.pairs.clearRetainingCapacity();
            self.constraints.clearRetainingCapacity();
            self.contact_list.clearRetainingCapacity();
            self.rows.clearRetainingCapacity();
            std.mem.swap(std.AutoArrayHashMapUnmanaged(Pair, void), &self.touching, &self.touched);
            self.touching.clearRetainingCapacity();

            if (hook) |run| {
                self.stepping_dt_s = dt_s;
                defer self.stepping_dt_s = null;
                try run.call(run.context, self, dt_s);
            }

            // Gravity and applied forces speed up every awake dynamic body before anything
            // pushes back; damping slows it.
            for (self.bodies.entries.items) |*entry| {
                const state = &(entry.item orelse continue).item;
                if (!state.pushable()) continue;
                state.linear = state.linear.add(self.config.gravity.scale(state.gravity_scale * dt_s));
                state.applyImpulse(state.force.scale(dt_s).add(state.carried), .zero);
                state.applyAngularImpulse(S.angularAdd(S.angularScale(state.torque, dt_s), state.carried_turn));
                state.carried = .zero;
                state.carried_turn = S.angular_zero;
                state.linear = state.linear.scale(1 / (1 + state.linear_damping * dt_s));
                state.angular = S.angularScale(state.angular, 1 / (1 + state.angular_damping * dt_s));
            }

            // Vehicles push off the ground through their wheels, from where gravity left them.
            if (three) for (self.vehicles.entries.items) |*entry| {
                const vehicle = &(entry.item orelse continue).item;
                const chassis = &(self.bodies.get(vehicle.body) orelse continue).item;
                Vehicles.step(vehicle, chassis, dt_s, WheelCaster{ .world = self, .chassis = vehicle.body });
            };

            try self.placeColliders();
            try self.findPairs();

            // Narrowphase: each overlapping pair's contacts, and a constraint for each to solve.
            for (self.pairs.items) |pair| {
                const a = self.placed.items[pair.a];
                const bb = self.placed.items[pair.b];
                const manifold = Narrowphase.collide(a.collider.shape, a.pose, bb.collider.shape, bb.pose);
                if (manifold.len == 0) continue;
                try self.touching.put(self.gpa, orderedPair(a.handle, bb.handle), {});
                const sensor = a.collider.sensor or bb.collider.sensor;
                if (!sensor) wakeOnTouch(a.body, bb.body);
                const friction = @sqrt(a.collider.material.friction * bb.collider.material.friction);
                const restitution = @max(a.collider.material.restitution, bb.collider.material.restitution);
                for (manifold.slice()) |point| {
                    try self.contact_list.append(self.gpa, .{ .a = a.handle, .b = bb.handle, .point = point.point, .normal = point.normal, .depth_m = point.depth_m });
                    if (sensor) continue;
                    var constraint = Solver.Constraint.init(a.body, bb.body, point.point, point.normal, point.depth_m, friction, restitution, dt_s);
                    constraint.contact = @intCast(self.contact_list.items.len - 1);
                    try self.constraints.append(self.gpa, constraint);
                }
            }

            if (three) try self.buildJoints(dt_s);
            for (0..solver_iterations) |_| {
                for (self.constraints.items) |*constraint| constraint.solveOnce();
                if (three) for (self.rows.items) |*joint_row| joint_row.solveOnce();
            }
            for (self.constraints.items) |constraint| self.contact_list.items[constraint.contact].impulse_ns = constraint.normal_impulse;
            if (three) self.finishJoints(dt_s);

            for (self.bodies.entries.items) |*entry| {
                const state = &(entry.item orelse continue).item;
                if (!state.moving()) continue;
                state.integrate(dt_s);
                if (state.kind == .dynamic) settle(state, dt_s);
            }

            try self.compareTouching();
        }

        /// Each joint's rows for this step, started with last step's impulses. A joint whose
        /// body moves wakes the other, and one on a body that is off or gone holds nothing.
        fn buildJoints(self: *Self, dt_s: T) !void {
            for (self.joints.entries.items) |*entry| {
                const joint = &(entry.item orelse continue);
                if (joint.stored.broken) continue;
                const a = &(self.bodies.get(joint.a) orelse continue).item;
                const other: ?*State = if (joint.b) |handle| &(self.bodies.get(handle) orelse continue).item else null;
                if (!a.enabled or (other != null and !other.?.enabled)) continue;
                if (other) |held| {
                    if (stirring(a) and held.sleeping and held.kind == .dynamic) held.wake();
                    if (stirring(held) and a.sleeping and a.kind == .dynamic) a.wake();
                }
                const first = self.rows.items.len;
                try Joints.build(&joint.stored, a, other, dt_s, &self.rows, self.gpa);
                Joints.warmStart(self.rows.items[first..]);
            }
        }

        /// Keeps each joint's impulses for the next step, and breaks those pulled too hard.
        fn finishJoints(self: *Self, dt_s: T) void {
            Joints.store(self.rows.items);
            for (self.joints.entries.items) |*entry| {
                const joint = &(entry.item orelse continue);
                if (joint.stored.broken) continue;
                joint.stored.force_n = Joints.pointForce(&joint.stored, dt_s);
                const limit = joint.stored.joint.break_force_n orelse continue;
                if (joint.stored.force_n > limit) {
                    joint.stored.broken = true;
                    joint.stored.impulses = @splat(0);
                }
            }
        }

        /// Whether a body moves faster than a sleeping one may.
        fn stirring(state: *const State) bool {
            return state.moving() and (state.linear.lengthSquared() > sleep_linear_mps * sleep_linear_mps or
                S.angularLengthSquared(state.angular) > sleep_angular_radps * sleep_angular_radps);
        }

        /// How vehicles cast their wheels: a ball the wheel's size, past their own chassis and
        /// through sensors.
        const WheelCaster = struct {
            world: *Self,
            chassis: b.BodyHandle,

            pub fn castWheel(self: WheelCaster, from: Vec, direction: Vec, max_m: T, radius_m: T) ?Vehicles.Contact {
                const hit = self.world.shapecast(.{ .ball = .{ .radius = radius_m } }, Pose.fromTranslation(from), direction, max_m, .{ .exclude_body = self.chassis, .sensors = .skip }) orelse return null;
                const ground = if (self.world.colliderBody(hit.collider)) |handle| &self.world.bodies.get(handle).?.item else null;
                return .{ .collider = hit.collider, .ground = ground, .point = hit.point, .normal = hit.normal, .distance_m = hit.distance_m };
            }
        };

        /// Which pairs started touching this step, and which stopped.
        fn compareTouching(self: *Self) !void {
            self.started_list.clearRetainingCapacity();
            self.ended_list.clearRetainingCapacity();
            for (self.touching.keys()) |pair| if (!self.touched.contains(pair)) try self.started_list.append(self.gpa, pair);
            for (self.touched.keys()) |pair| if (!self.touching.contains(pair)) try self.ended_list.append(self.gpa, pair);
        }

        fn orderedPair(x: c.ColliderHandle, y: c.ColliderHandle) Pair {
            return if (@as(u64, @bitCast(x)) < @as(u64, @bitCast(y))) .{ .a = x, .b = y } else .{ .a = y, .b = x };
        }

        /// Fills `placed` with every collider where its body has it this step.
        fn placeColliders(self: *Self) !void {
            for (self.colliders.entries.items, 0..) |*entry, index| {
                const attached = &(entry.item orelse continue);
                if (!attached.item.enabled) continue;
                const body = if (attached.parent) |parent| &self.bodies.get(parent).?.item else null;
                if (body) |state| if (!state.enabled) continue;
                const placed_pose = if (body) |state| state.pose.mul(attached.item.local_pose) else attached.item.local_pose;
                try self.placed.append(self.gpa, .{
                    .handle = self.colliders.handleAt(index),
                    .collider = &attached.item,
                    .pose = placed_pose,
                    .bounds = attached.item.shape.aabb(placed_pose),
                    .body = body,
                });
            }
        }

        // ponytail: every pair is tested, O(n²); add a sweep-and-prune or tree past a few hundred colliders.
        fn findPairs(self: *Self) !void {
            for (self.placed.items, 0..) |a, i| {
                for (self.placed.items[i + 1 ..], i + 1..) |bb, j| {
                    if (!canTouch(a, bb) or !a.bounds.overlaps(bb.bounds)) continue;
                    try self.pairs.append(self.gpa, .{ .a = @intCast(i), .b = @intCast(j) });
                }
            }
        }

        /// Whether two colliders can meet: not on the same body, and one of them moves (or senses).
        fn canTouch(a: Placed, bb: Placed) bool {
            if (a.body != null and a.body == bb.body) return false;
            const a_moves = if (a.body) |s| s.kind != .static else false;
            const b_moves = if (bb.body) |s| s.kind != .static else false;
            return a_moves or b_moves or a.collider.sensor or bb.collider.sensor;
        }

        /// Something awake touching a sleeping body wakes it.
        fn wakeOnTouch(a: ?*State, bb: ?*State) void {
            const a_awake = if (a) |s| s.moving() else false;
            const b_awake = if (bb) |s| s.moving() else false;
            if (a_awake) if (bb) |s| if (s.kind == .dynamic) s.wake();
            if (b_awake) if (a) |s| if (s.kind == .dynamic) s.wake();
        }

        /// Sleeps a body once it has been nearly still for `sleep_after_s`.
        fn settle(state: *State, dt_s: T) void {
            const still = state.linear.lengthSquared() < sleep_linear_mps * sleep_linear_mps and
                S.angularLengthSquared(state.angular) < sleep_angular_radps * sleep_angular_radps;
            state.still_s = if (still) state.still_s + dt_s else 0;
            if (state.still_s < sleep_after_s) return;
            state.sleeping = true;
            state.linear = .zero;
            state.angular = S.angular_zero;
        }

        /// Works out `body`'s mass, centre of mass and inertia from its own mass, spread over
        /// its colliders without one of their own by volume, and the masses of those with one.
        // ponytail: colliders' own rotations and products of inertia are left out, so a part
        // tilted on its body turns as if square to it; take the full tensor when that shows.
        fn updateMass(self: *Self, body: b.BodyHandle) void {
            const state = &self.bodies.get(body).?.item;
            var shared_volume: T = 0;
            var sharing: T = 0;
            var own_kg: T = 0;
            for (self.colliders.entries.items) |entry| {
                const part = self.partOf(entry, body) orelse continue;
                if (part.mass_kg) |kg| own_kg += kg else {
                    shared_volume += part.shape.volume();
                    sharing += 1;
                }
            }
            const total_kg = (if (sharing > 0 or own_kg == 0) state.own_mass_kg else 0) + own_kg;

            // The centre is where the parts balance, unless the body says otherwise.
            var weighted: Vec = .zero;
            for (self.colliders.entries.items) |entry| {
                const part = self.partOf(entry, body) orelse continue;
                weighted = weighted.add(part.local_pose.translation.scale(partMass(part, state.own_mass_kg, shared_volume, sharing)));
            }
            state.local_center = state.center_override orelse if (total_kg > 0 and sharing + own_kg > 0) weighted.scale(1 / total_kg) else Vec.zero;

            const dynamic = state.kind == .dynamic and total_kg > 0;
            state.inv_mass = if (dynamic) 1 / total_kg else 0;
            var inertia: S.Inertia = S.angular_zero;
            for (self.colliders.entries.items) |entry| {
                const part = self.partOf(entry, body) orelse continue;
                const part_kg = partMass(part, state.own_mass_kg, shared_volume, sharing);
                inertia = S.angularAdd(inertia, part.shape.inertia(part_kg));
                // Parallel axis: a part off the centre is harder to turn about the axes it is away from.
                const offset = part.local_pose.translation.sub(state.local_center);
                const away: S.Inertia = if (dim == .two) part_kg * offset.lengthSquared() else Vec.splat(offset.lengthSquared()).sub(offset.mul(offset)).scale(part_kg);
                inertia = S.angularAdd(inertia, away);
            }
            state.setInertia(inertia);
        }

        /// The collider in `entry` when it is on `body` and on.
        fn partOf(_: *const Self, entry: anytype, body: b.BodyHandle) ?Collider {
            const attached = entry.item orelse return null;
            if (attached.parent == null or attached.parent.? != body or !attached.item.enabled) return null;
            return attached.item;
        }

        /// A part's mass: its own, or its share by volume of what its body has to spread.
        fn partMass(part: Collider, spread_kg: T, shared_volume: T, sharing: T) T {
            if (part.mass_kg) |kg| return kg;
            return spread_kg * (if (shared_volume > 0) part.shape.volume() / shared_volume else 1 / sharing);
        }

        /// Pushes `body` by `impulse_ns` (N·s, world space), waking it. Only dynamic bodies move.
        pub fn applyImpulse(self: *Self, body: b.BodyHandle, impulse_ns: Vec, at: ImpulsePoint) void {
            const state = &(self.bodies.get(body) orelse return).item;
            if (state.kind != .dynamic) return;
            state.wake();
            state.applyImpulse(impulse_ns, offsetOf(state.*, at));
        }

        /// Pushes `body` with `force_n` (N, world space) through every step of this frame, or,
        /// from a `StepHook`, through this step. Only dynamic bodies move.
        pub fn applyForce(self: *Self, body: b.BodyHandle, force_n: Vec, at: ImpulsePoint) void {
            const state = &(self.bodies.get(body) orelse return).item;
            if (state.kind != .dynamic or force_n.lengthSquared() == 0) return;
            state.wake();
            const offset = offsetOf(state.*, at);
            if (self.stepping_dt_s) |dt_s| return state.applyImpulse(force_n.scale(dt_s), offset);
            state.force = state.force.add(force_n);
            state.torque = S.angularAdd(state.torque, S.cross(offset, force_n));
        }

        /// Turns `body` with `torque_nm` (N·m, world space; about z in 2D) as `applyForce` pushes.
        pub fn applyTorque(self: *Self, body: b.BodyHandle, torque_nm: S.Angular) void {
            const state = &(self.bodies.get(body) orelse return).item;
            if (state.kind != .dynamic or S.angularLengthSquared(torque_nm) == 0) return;
            state.wake();
            if (self.stepping_dt_s) |dt_s| return state.applyAngularImpulse(S.angularScale(torque_nm, dt_s));
            state.torque = S.angularAdd(state.torque, torque_nm);
        }

        fn offsetOf(state: State, at: ImpulsePoint) Vec {
            return switch (at) {
                .center_of_mass => Vec.zero,
                .at_world_point => |point| point.sub(state.center()),
            };
        }

        /// Changes what `body` is, keeping where it is. Turned static it stops; turned on it
        /// wakes. A stale handle is ignored.
        pub fn setProperties(self: *Self, body: b.BodyHandle, properties: Properties) void {
            const state = &(self.bodies.get(body) orelse return).item;
            state.kind = properties.kind;
            state.own_mass_kg = properties.mass_kg;
            state.center_override = properties.center_of_mass_m;
            state.linear_damping = properties.linear_damping;
            state.angular_damping = properties.angular_damping;
            state.gravity_scale = properties.gravity_scale;
            state.rotation = properties.rotation;
            state.enabled = properties.enabled;
            if (state.kind == .static) {
                state.linear = .zero;
                state.angular = S.angular_zero;
            }
            state.wake();
            self.updateMass(body);
        }

        /// Turns `collider` on or off, changing its body's mass to match.
        pub fn setColliderEnabled(self: *Self, collider: c.ColliderHandle, enabled: bool) void {
            const attached = self.colliders.get(collider) orelse return;
            if (attached.item.enabled == enabled) return;
            attached.item.enabled = enabled;
            if (attached.parent) |body| if (self.bodies.get(body)) |state| {
                state.item.wake();
                self.updateMass(body);
            };
        }

        /// Moves `body` to `new_pose` at once, keeping its velocity and waking it.
        pub fn setPose(self: *Self, body: b.BodyHandle, new_pose: Pose) void {
            const state = &(self.bodies.get(body) orelse return).item;
            state.pose = new_pose;
            state.wake();
        }

        /// Moves `collider` on its body, which balances anew, or in the world when it has none.
        pub fn setColliderPose(self: *Self, collider: c.ColliderHandle, local_pose: Pose) void {
            const attached = self.colliders.get(collider) orelse return;
            attached.item.local_pose = local_pose;
            if (attached.parent) |body| if (self.bodies.get(body)) |state| {
                state.item.wake();
                self.updateMass(body);
            };
        }

        /// Sets how fast `body` moves and turns, waking it.
        pub fn setVelocity(self: *Self, body: b.BodyHandle, new_velocity: Velocity) void {
            const state = &(self.bodies.get(body) orelse return).item;
            state.linear = new_velocity.linear;
            state.angular = new_velocity.angular;
            state.wake();
        }

        /// Where `body` is. It must be in the world.
        pub fn pose(self: *const Self, body: b.BodyHandle) Pose {
            return self.bodies.get(body).?.item.pose;
        }

        /// How fast `body` moves and turns. It must be in the world.
        pub fn velocity(self: *const Self, body: b.BodyHandle) Velocity {
            const state = self.bodies.get(body).?.item;
            return .{ .linear = state.linear, .angular = state.angular };
        }

        /// Whether `body` has settled and stopped being stepped. It must be in the world.
        pub fn isSleeping(self: *const Self, body: b.BodyHandle) bool {
            return self.bodies.get(body).?.item.sleeping;
        }

        /// `body`'s mass in kg (0 unless dynamic), and where it balances in world space. It
        /// must be in the world.
        pub fn massOf(self: *const Self, body: b.BodyHandle) struct { mass_kg: T, center: Vec } {
            const state = self.bodies.get(body).?.item;
            return .{ .mass_kg = if (state.inv_mass > 0) 1 / state.inv_mass else 0, .center = state.center() };
        }

        /// How fast the point of `body` at `point` (world space) moves. It must be in the world.
        pub fn pointVelocity(self: *const Self, body: b.BodyHandle, point: Vec) Vec {
            const state = self.bodies.get(body).?.item;
            if (!state.moving()) return .zero;
            return state.velocityAt(point.sub(state.center()));
        }

        /// What a body stood on in the last step (`groundOf`).
        pub const Ground = struct {
            collider: c.ColliderHandle,
            /// The body it is on, null for scenery.
            body: ?b.BodyHandle,
            point: Vec,
            /// Out of the ground, towards the body standing on it.
            normal: Vec,
            /// How fast the ground moves where it is stood on: what to move with to ride it.
            velocity: Vec,
        };

        /// What `body` stood on in the last step: of its contacts whose normal leans within
        /// `max_slope_rad` of `up`, the most upright. Null when it stood on nothing.
        pub fn groundOf(self: *const Self, body: b.BodyHandle, up: Vec, max_slope_rad: T) ?Ground {
            var best: ?Ground = null;
            var best_lean: T = @cos(max_slope_rad);
            for (self.contact_list.items) |contact| {
                const on_a = self.colliderBody(contact.a);
                const on_b = self.colliderBody(contact.b);
                const mine_a = on_a != null and on_a.? == body;
                const mine_b = on_b != null and on_b.? == body;
                if (mine_a == mine_b) continue;
                const under = if (mine_a) contact.b else contact.a;
                const normal = if (mine_b) contact.normal else contact.normal.negate();
                const lean = normal.dot(up);
                if (lean < best_lean) continue;
                best_lean = lean;
                const under_body = self.colliderBody(under);
                best = .{
                    .collider = under,
                    .body = under_body,
                    .point = contact.point,
                    .normal = normal,
                    .velocity = if (under_body) |handle| self.pointVelocity(handle, contact.point) else .zero,
                };
            }
            return best;
        }

        /// Holds `a` and `b` together by `joint`, from where they are now; `b` null holds `a`
        /// to the world, at `joint.anchor_b` in world space. 3D only.
        pub fn addJoint(self: *Self, joint: Joint, a: b.BodyHandle, other: ?b.BodyHandle, user_data: u64) !JointHandle {
            if (!three) return error.UnsupportedIn2D;
            const first = &(self.bodies.get(a) orelse return error.StaleBody).item;
            const second: ?*State = if (other) |handle| &(self.bodies.get(handle) orelse return error.StaleBody).item else null;
            first.wake();
            if (second) |state| state.wake();
            return self.joints.add(self.gpa, .{ .stored = Joints.Stored.init(joint, first, second), .a = a, .b = other, .user_data = user_data });
        }

        /// Lets go of what `joint` held, waking both bodies. A stale handle is ignored.
        pub fn removeJoint(self: *Self, joint: JointHandle) void {
            const stored = (self.joints.get(joint) orelse return).*;
            if (self.bodies.get(stored.a)) |state| state.item.wake();
            if (stored.b) |other| if (self.bodies.get(other)) |state| state.item.wake();
            _ = self.joints.remove(joint);
        }

        /// The joint as the world keeps it: what it was made with, whether it broke, its last
        /// position and force. Null for a stale handle; valid until the next `addJoint`.
        pub fn jointState(self: *const Self, joint: JointHandle) ?*StoredJoint {
            return self.joints.get(joint);
        }

        /// Puts `vehicle` (with its wheels) on its body. 3D only.
        pub fn addVehicle(self: *Self, vehicle: Vehicle, user_data: u64) !VehicleHandle {
            if (!three) return error.UnsupportedIn2D;
            if (self.bodies.get(vehicle.body) == null) return error.StaleBody;
            return self.vehicles.add(self.gpa, .{ .item = vehicle, .user_data = user_data });
        }

        pub fn removeVehicle(self: *Self, vehicle: VehicleHandle) void {
            _ = self.vehicles.remove(vehicle);
        }

        /// The vehicle, to set its input and wheels or read how they went. Null for a stale
        /// handle; valid until the next `addVehicle`.
        pub fn vehicleOf(self: *const Self, handle: VehicleHandle) ?*Vehicle {
            return &(self.vehicles.get(handle) orelse return null).item;
        }

        /// The contacts found by the last step. Valid until the next `update`.
        pub fn contacts(self: *const Self) []const Contact {
            return self.contact_list.items;
        }

        /// The pairs that started touching in the last step. Valid until the next `update`.
        pub fn started(self: *const Self) []const Pair {
            return self.started_list.items;
        }

        /// The pairs that stopped touching in the last step. Valid until the next `update`.
        pub fn ended(self: *const Self) []const Pair {
            return self.ended_list.items;
        }

        /// What `debugDraw` draws.
        pub const DebugDrawOptions = struct {
            colliders: bool = true,
            /// The last step's contacts: a cross at each point, its normal drawn from it.
            contacts: bool = true,
            /// A contact marker's size.
            marker_m: T = 0.1,
            /// Metres of line per newton a contact or wheel pushed with in the last step; 0 draws
            /// contacts' normals at the marker's size, and no wheel forces.
            force_m_per_n: T = 0,
            /// Each moving body's centre of mass.
            centers: bool = false,
            /// Joints' anchors, axes and limits.
            joints: bool = false,
            /// Vehicles' wheels, suspension and casts.
            vehicles: bool = false,
        };

        /// The world as wireframes through `sink` (see `debug_draw.DebugDraw`): each collider
        /// in its `debug_draw.palette.of` colour where it is now, and the last step's contacts.
        pub fn debugDraw(self: *const Self, options: DebugDrawOptions, sink: anytype) !void {
            const Draw = debug_draw.DebugDraw(S);
            if (options.colliders) for (self.colliders.entries.items) |entry| {
                const attached = entry.item orelse continue;
                const body = if (attached.parent) |parent| self.bodies.get(parent).?.item else null;
                const placed_pose = if (body) |state| state.pose.mul(attached.item.local_pose) else attached.item.local_pose;
                const color = debug_draw.palette.of(if (body) |state| state.kind else null, if (body) |state| state.sleeping else false, attached.item.sensor);
                try Draw.shape(attached.item.shape, placed_pose, color, sink);
            };
            if (options.contacts) for (self.contact_list.items) |contact| {
                try Draw.marker(contact.point, contact.normal, options.marker_m, debug_draw.palette.contact, sink);
                if (options.force_m_per_n > 0) try sink.line(contact.point, contact.point.add(contact.normal.scale(contact.impulse_ns / self.config.fixed_step_s * options.force_m_per_n)), debug_draw.palette.force);
            };
            if (options.centers) for (self.bodies.entries.items) |entry| {
                const state = (entry.item orelse continue).item;
                if (state.kind != .dynamic or !state.enabled) continue;
                try Draw.marker(state.center(), .zero, options.marker_m * 3, debug_draw.palette.center, sink);
            };
            if (three and options.joints) for (self.joints.entries.items) |entry| {
                const joint = entry.item orelse continue;
                try self.drawJoint(joint, options, sink);
            };
            if (three and options.vehicles) for (self.vehicles.entries.items) |entry| {
                const attached = entry.item orelse continue;
                try self.drawVehicle(attached.item, options, sink);
            };
        }

        fn drawJoint(self: *const Self, joint: StoredJoint, options: DebugDrawOptions, sink: anytype) !void {
            const Draw = debug_draw.DebugDraw(S);
            const a = (self.bodies.get(joint.a) orelse return).item;
            const other: ?State = if (joint.b) |handle| (self.bodies.get(handle) orelse return).item else null;
            const color = if (joint.stored.broken) debug_draw.palette.contact else debug_draw.palette.joint;
            const pa = a.pose.transformPoint(joint.stored.joint.anchor_a);
            const pb = if (other) |state| state.pose.transformPoint(joint.stored.joint.anchor_b) else joint.stored.joint.anchor_b;
            try Draw.marker(pa, .zero, options.marker_m * 2, color, sink);
            try Draw.marker(pb, .zero, options.marker_m * 2, color, sink);
            try sink.line(a.center(), pa, color);
            if (other) |state| try sink.line(state.center(), pb, color);
            const reach = options.marker_m * 5;
            switch (joint.stored.joint.kind) {
                .fixed => {},
                .ball => |ball| try sink.line(pa, pa.add(a.pose.rotation.rotateVector(ball.axis.normalize()).scale(reach)), color),
                .hinge => |hinge| {
                    const axis = a.pose.rotation.rotateVector(hinge.axis.normalize());
                    try sink.line(pa.sub(axis.scale(reach)), pa.add(axis.scale(reach)), color);
                    const limits = hinge.limits orelse return;
                    const across = a.pose.rotation.rotateVector(S.tangents(hinge.axis.normalize())[0]);
                    for ([_]T{ limits.lower, limits.upper }) |angle| {
                        try sink.line(pa, pa.add(S.Rotation.fromAxisAngle(axis, angle).rotateVector(across).scale(reach)), debug_draw.palette.limit);
                    }
                },
                .slider => |slider| {
                    const axis = a.pose.rotation.rotateVector(slider.axis.normalize());
                    const limits = slider.limits orelse return sink.line(pa.sub(axis.scale(reach)), pa.add(axis.scale(reach)), color);
                    try sink.line(pa.add(axis.scale(limits.lower)), pa.add(axis.scale(limits.upper)), debug_draw.palette.limit);
                },
            }
        }

        fn drawVehicle(self: *const Self, drawn: Vehicle, options: DebugDrawOptions, sink: anytype) !void {
            const Draw = debug_draw.DebugDraw(S);
            const chassis = (self.bodies.get(drawn.body) orelse return).item;
            const down = chassis.pose.rotation.rotateVector(Vec.init(0, -1, 0));
            for (drawn.wheels[0..drawn.wheel_count]) |wheel| {
                const setup = wheel.config;
                const mount = chassis.pose.transformPoint(setup.position);
                const hub = mount.add(down.scale(wheel.state.suspension_m));
                // The cast's full reach, then the spring to where the wheel is.
                try sink.line(mount, mount.add(down.scale(setup.suspension_rest_m + setup.suspension_travel_m + setup.radius_m)), debug_draw.palette.limit);
                try sink.line(mount, hub, if (wheel.state.grounded) debug_draw.palette.suspension else debug_draw.palette.limit);
                const steer = S.Rotation.fromAxisAngle(down.negate(), -wheel.state.steer_rad);
                try Draw.ring(Pose.fromParts(hub, steer.mul(chassis.pose.rotation)), 1, 2, setup.radius_m, debug_draw.palette.suspension, sink);
                if (!wheel.state.grounded) continue;
                const length: T = if (options.force_m_per_n > 0) wheel.state.suspension_force_n * options.force_m_per_n else options.marker_m * 2;
                try Draw.marker(wheel.state.contact_point, .zero, options.marker_m, debug_draw.palette.contact, sink);
                try sink.line(wheel.state.contact_point, wheel.state.contact_point.add(wheel.state.contact_normal.scale(length)), debug_draw.palette.force);
            }
        }

        /// Which colliders a query leaves out, as Rapier's `QueryFilter` does.
        pub const QueryFilter = struct {
            /// Every collider on this body, such as the one casting.
            exclude_body: ?b.BodyHandle = null,
            exclude_collider: ?c.ColliderHandle = null,
            sensors: enum { hit, skip } = .hit,

            fn passes(self: QueryFilter, handle: c.ColliderHandle, attached: Attached(Collider)) bool {
                if (!attached.item.enabled) return false;
                if (self.exclude_collider) |excluded| if (excluded == handle) return false;
                if (self.exclude_body) |excluded| if (attached.parent) |parent| if (parent == excluded) return false;
                return !(self.sensors == .skip and attached.item.sensor);
            }
        };

        /// The nearest collider `ray` meets that `filter` lets through, as things stand now. A ray
        /// starting inside a collider meets it at 0, so leave out the one casting it.
        ///
        /// ```zig
        /// const below = world.raycast(.{ .origin = feet, .direction = down, .max_distance_m = 0.1 }, .{ .exclude_body = me });
        /// ```
        pub fn raycast(self: *const Self, ray: Ray, filter: QueryFilter) ?RayHit {
            var nearest: ?RayHit = null;
            for (self.colliders.entries.items, 0..) |entry, index| {
                const attached = entry.item orelse continue;
                const placed_pose = self.queryPose(self.colliders.handleAt(index), attached, filter) orelse continue;
                const hit = Rays.cast(ray, attached.item.shape, placed_pose) orelse continue;
                if (nearest != null and nearest.?.distance_m <= hit.distance_m) continue;
                nearest = .{
                    .collider = self.colliders.handleAt(index),
                    .point = ray.origin.add(ray.direction.scale(hit.distance_m)),
                    .normal = hit.normal,
                    .distance_m = hit.distance_m,
                };
            }
            return nearest;
        }

        /// Where a collider is for a query that `filter` lets it into, or null when it is left
        /// out or off.
        fn queryPose(self: *const Self, handle: c.ColliderHandle, attached: Attached(Collider), filter: QueryFilter) ?Pose {
            if (!filter.passes(handle, attached)) return null;
            const body = if (attached.parent) |parent| self.bodies.get(parent).?.item else return attached.item.local_pose;
            if (!body.enabled) return null;
            return body.pose.mul(attached.item.local_pose);
        }

        /// What a shape cast met.
        pub const ShapeHit = struct {
            collider: c.ColliderHandle,
            /// Where they touched, on the collider met.
            point: Vec,
            /// The collider's surface there, facing back towards the cast.
            normal: Vec,
            /// How far the shape moved before touching.
            distance_m: T,
            /// The share of the way it got: `distance_m` over the cast's length.
            time_of_impact: T,
        };

        /// The first collider convex `shape`, starting at `start` and moving `max_distance_m`
        /// along unit `direction`, touches, as things stand now; one it starts touching is met
        /// at 0. `filter.exclude_body` leaves out every collider of a compound body, such as a
        /// wheel's own chassis.
        ///
        /// ```zig
        /// const under = world.shapecast(.{ .ball = .{ .radius = 0.4 } }, wheel_top, down, 1, .{ .exclude_body = chassis });
        /// ```
        pub fn shapecast(self: *const Self, shape: Shape, start: Pose, direction: Vec, max_distance_m: T, filter: QueryFilter) ?ShapeHit {
            if (!shape.convex() or !shape.supported()) return null;
            var nearest: ?ShapeHit = null;
            for (self.colliders.entries.items, 0..) |entry, index| {
                const attached = entry.item orelse continue;
                const handle = self.colliders.handleAt(index);
                const placed_pose = self.queryPose(handle, attached, filter) orelse continue;
                const reach = if (nearest) |hit| hit.distance_m else max_distance_m;
                const hit = Casts.cast(shape, start, direction, reach, attached.item.shape, placed_pose) orelse continue;
                if (nearest != null and nearest.?.distance_m <= hit.distance_m) continue;
                nearest = .{
                    .collider = handle,
                    .point = hit.point,
                    .normal = hit.normal,
                    .distance_m = hit.distance_m,
                    .time_of_impact = if (max_distance_m > 0) hit.distance_m / max_distance_m else 0,
                };
            }
            return nearest;
        }
    };
}
