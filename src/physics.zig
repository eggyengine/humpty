//! The physics world: bodies, the colliders they touch it with, and the fixed step that moves
//! them under gravity and pushes them apart where they meet. One world type per space: 2D or
//! 3D, in `f32` or `f64`.
const std = @import("std");
const space = @import("space.zig");
const b = @import("body.zig");
const c = @import("collider.zig");
const shapes_zig = @import("shapes.zig");
const narrowphase_zig = @import("narrowphase.zig");
const solver_zig = @import("solver.zig");
const ray_zig = @import("ray.zig");
const Slots = @import("slots.zig").Slots;

/// A world in `dim` dimensions with scalar `T` (`f32` or `f64`).
pub fn Physics(comptime dim: space.Dim, comptime T: type) type {
    return struct {
        const Self = @This();
        pub const S = space.Space(dim, T);
        pub const Vec = S.Vec;
        pub const Pose = S.Pose;
        pub const Shape = shapes_zig.Shapes(S).Shape;
        pub const Body = b.Bodies(S).Body;
        pub const Collider = c.Colliders(S).Collider;
        pub const Material = c.Colliders(S).Material;
        pub const Ray = ray_zig.Rays(S).Ray;
        pub const RayHit = ray_zig.Rays(S).RayHit;
        const State = b.Bodies(S).State;
        const Narrowphase = narrowphase_zig.Narrowphase(S);
        const Solver = solver_zig.Solver(S);
        const Rays = ray_zig.Rays(S);

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

            pub fn involves(self: Contact, collider: c.ColliderHandle) bool {
                return self.a == collider or self.b == collider;
            }
        };

        /// Two colliders that started or stopped touching, the lower handle first.
        pub const Pair = struct { a: c.ColliderHandle, b: c.ColliderHandle };

        pub const Config = struct {
            gravity: Vec,
            fixed_step_s: T,
        };

        gpa: std.mem.Allocator,
        config: Config,
        bodies: Slots(Attached(State), b.BodyHandle) = .{},
        colliders: Slots(Attached(Collider), c.ColliderHandle) = .{},

        accumulator_s: T = 0,
        /// Per-step scratch, cleared at the start of each step but kept allocated,
        /// so after the first few steps the solver stops asking `gpa` for memory.
        placed: std.ArrayList(Placed) = .empty,
        pairs: std.ArrayList(ColliderPair) = .empty,
        constraints: std.ArrayList(Solver.Constraint) = .empty,
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

        pub fn init(gpa: std.mem.Allocator, config: Config) !Self {
            return .{ .gpa = gpa, .config = config };
        }

        pub fn deinit(self: *Self) void {
            for (self.colliders.entries.items) |entry| if (entry.item) |attached| self.freeShape(attached.item.shape);
            self.bodies.deinit(self.gpa);
            self.colliders.deinit(self.gpa);
            self.placed.deinit(self.gpa);
            self.pairs.deinit(self.gpa);
            self.constraints.deinit(self.gpa);
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

        /// Removes the body and every collider attached to it. A stale handle is ignored.
        pub fn removeBody(self: *Self, body: b.BodyHandle) void {
            for (self.colliders.entries.items, 0..) |entry, index| {
                const attached = entry.item orelse continue;
                if (attached.parent != null and attached.parent.? == body) self.removeCollider(self.colliders.handleAt(index));
            }
            _ = self.bodies.remove(body);
        }

        /// Whether `handle`, a `BodyHandle` or `ColliderHandle`, still names something in the world.
        pub fn contains(self: *const Self, handle: anytype) bool {
            return switch (@TypeOf(handle)) {
                b.BodyHandle => self.bodies.get(handle) != null,
                c.ColliderHandle => self.colliders.get(handle) != null,
                else => @compileError("contains takes a BodyHandle or ColliderHandle"),
            };
        }

        /// Attaches `collider` to `parent`, or with no parent makes it static scenery. A hull's
        /// points and a mesh's vertices are copied. The parent's mass is spread over its colliders
        /// by volume, for how it turns.
        pub fn addCollider(self: *Self, collider: Collider, parent: ?b.BodyHandle, user_data: u64) !c.ColliderHandle {
            if (!collider.shape.supported()) return error.UnsupportedShape;
            if (parent) |body| if (self.bodies.get(body) == null) return error.StaleBody;
            var owned = collider;
            owned.shape = try self.copyShape(collider.shape);
            errdefer self.freeShape(owned.shape);
            const handle = try self.colliders.add(self.gpa, .{ .item = owned, .user_data = user_data, .parent = parent });
            if (parent) |body| self.updateInertia(body);
            return handle;
        }

        /// Detaches and removes `collider`. A stale handle is ignored.
        pub fn removeCollider(self: *Self, collider: c.ColliderHandle) void {
            const attached = (self.colliders.get(collider) orelse return).*;
            _ = self.colliders.remove(collider);
            self.freeShape(attached.item.shape);
            if (attached.parent) |body| if (self.bodies.get(body) != null) self.updateInertia(body);
        }

        fn copyShape(self: *Self, shape: Shape) !Shape {
            return switch (shape) {
                .convex_hull => |hull| .{ .convex_hull = .{ .points = try self.gpa.dupe(Vec, hull.points) } },
                .mesh => |mesh| blk: {
                    const vertices = try self.gpa.dupe(Vec, mesh.vertices);
                    errdefer self.gpa.free(vertices);
                    break :blk .{ .mesh = .{ .vertices = vertices, .indices = try self.gpa.dupe([S.n]u32, mesh.indices) } };
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

        pub fn update(self: *Self, frame_dt_s: T) !void {
            self.accumulator_s += @min(frame_dt_s, max_frame_dt_s);
            while (self.accumulator_s >= self.config.fixed_step_s) {
                try self.step(self.config.fixed_step_s);
                self.accumulator_s -= self.config.fixed_step_s;
            }
        }

        fn step(self: *Self, dt_s: T) !void {
            // Cleared here rather than at the end, so `contacts()` still shows the
            // last step's contacts until the next one runs.
            self.placed.clearRetainingCapacity();
            self.pairs.clearRetainingCapacity();
            self.constraints.clearRetainingCapacity();
            self.contact_list.clearRetainingCapacity();
            std.mem.swap(std.AutoArrayHashMapUnmanaged(Pair, void), &self.touching, &self.touched);
            self.touching.clearRetainingCapacity();

            // Gravity speeds up every awake dynamic body before anything pushes back; damping slows it.
            for (self.bodies.entries.items) |*entry| {
                const state = &(entry.item orelse continue).item;
                if (!state.pushable()) continue;
                state.linear = state.linear.add(self.config.gravity.scale(state.gravity_scale * dt_s));
                state.linear = state.linear.scale(1 / (1 + state.linear_damping * dt_s));
                state.angular = S.angularScale(state.angular, 1 / (1 + state.angular_damping * dt_s));
            }

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
                    try self.constraints.append(self.gpa, Solver.Constraint.init(a.body, bb.body, point.point, point.normal, point.depth_m, friction, restitution, dt_s));
                }
            }

            Solver.solve(self.constraints.items, solver_iterations);

            for (self.bodies.entries.items) |*entry| {
                const state = &(entry.item orelse continue).item;
                if (!state.moving()) continue;
                state.pose.translation = state.pose.translation.add(state.linear.scale(dt_s));
                state.pose.rotation = S.integrate(state.pose.rotation, state.angular, dt_s);
                if (state.kind == .dynamic) settle(state, dt_s);
            }

            try self.compareTouching();
        }

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
                const body = if (attached.parent) |parent| &self.bodies.get(parent).?.item else null;
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

        /// Spreads `body`'s mass over its colliders by volume, each about its place on the body.
        // ponytail: colliders' own rotations and an off-centre centre of mass are ignored; take the
        // full inertia tensor when tilted or lopsided compounds need to turn right.
        fn updateInertia(self: *Self, body: b.BodyHandle) void {
            const state = &self.bodies.get(body).?.item;
            if (state.inv_mass == 0) return;
            const mass_kg = 1 / state.inv_mass;

            var total_volume: T = 0;
            var count: T = 0;
            for (self.colliders.entries.items) |entry| {
                const attached = entry.item orelse continue;
                if (attached.parent == null or attached.parent.? != body) continue;
                total_volume += attached.item.shape.volume();
                count += 1;
            }

            var inertia: S.Inertia = S.angular_zero;
            for (self.colliders.entries.items) |entry| {
                const attached = entry.item orelse continue;
                if (attached.parent == null or attached.parent.? != body) continue;
                const share = if (total_volume > 0) attached.item.shape.volume() / total_volume else 1 / count;
                const part_kg = mass_kg * share;
                inertia = S.angularAdd(inertia, attached.item.shape.inertia(part_kg));
                // Parallel axis: a part off the centre is harder to turn about the axes it is away from.
                const offset = attached.item.local_pose.translation;
                const away: S.Inertia = if (dim == .two) part_kg * offset.lengthSquared() else Vec.splat(offset.lengthSquared()).sub(offset.mul(offset)).scale(part_kg);
                inertia = S.angularAdd(inertia, away);
            }
            state.setInertia(inertia);
        }

        /// Pushes `body` by `impulse_ns` (N·s, world space), waking it. Only dynamic bodies move.
        pub fn applyImpulse(self: *Self, body: b.BodyHandle, impulse_ns: Vec, at: ImpulsePoint) void {
            const state = &(self.bodies.get(body) orelse return).item;
            if (state.kind != .dynamic) return;
            state.wake();
            const offset = switch (at) {
                .center_of_mass => Vec.zero,
                .at_world_point => |point| point.sub(state.pose.translation),
            };
            state.applyImpulse(impulse_ns, offset);
        }

        /// Moves `body` to `new_pose` at once, keeping its velocity and waking it.
        pub fn setPose(self: *Self, body: b.BodyHandle, new_pose: Pose) void {
            const state = &(self.bodies.get(body) orelse return).item;
            state.pose = new_pose;
            state.wake();
        }

        /// Moves `collider` on its body, or in the world when it has none.
        pub fn setColliderPose(self: *Self, collider: c.ColliderHandle, local_pose: Pose) void {
            const attached = self.colliders.get(collider) orelse return;
            attached.item.local_pose = local_pose;
            if (attached.parent) |body| if (self.bodies.get(body)) |state| state.item.wake();
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

        /// The nearest collider `ray` meets, as things stand now.
        pub fn raycast(self: *const Self, ray: Ray) ?RayHit {
            var nearest: ?RayHit = null;
            for (self.colliders.entries.items, 0..) |entry, index| {
                const attached = entry.item orelse continue;
                const body_pose = if (attached.parent) |parent| self.bodies.get(parent).?.item.pose else null;
                const placed_pose = if (body_pose) |p| p.mul(attached.item.local_pose) else attached.item.local_pose;
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
    };
}
