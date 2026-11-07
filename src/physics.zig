//! The physics world: bodies, the colliders they touch it with, and the fixed step that moves
//! them under gravity and pushes them apart where they meet.
const std = @import("std");
const emath = @import("eggenvector");
const math = @import("math.zig");
const b = @import("body.zig");
const c = @import("collider.zig");
const r = @import("ray.zig");
const shapes = @import("shapes.zig");
const narrowphase = @import("narrowphase.zig");
const solver = @import("solver.zig");
const Slots = @import("slots.zig").Slots;

const Vec3 = emath.Vec3;

pub const Physics = struct {
    gpa: std.mem.Allocator,
    config: Config,
    bodies: Slots(b.State, b.BodyHandle) = .{},
    colliders: Slots(Attached, c.ColliderHandle) = .{},

    accumulator_s: f32 = 0,
    /// Per-step scratch, cleared at the start of each step but kept allocated,
    /// so after the first few steps the solver stops asking `gpa` for memory.
    placed: std.ArrayList(Placed) = .empty,
    pairs: std.ArrayList(ColliderPair) = .empty,
    constraints: std.ArrayList(solver.Constraint) = .empty,
    contact_list: std.ArrayList(math.Contact) = .empty,

    /// Longest frame `update` will catch up on, so a stall slows the
    /// simulation instead of running hundreds of steps at once.
    const max_frame_dt_s: f32 = 0.25;
    /// Passes the solver makes over the contacts each step.
    const solver_iterations = 16;
    /// A body slower than these for `sleep_after_s` sleeps until something touches or pushes it.
    const sleep_linear_mps: f32 = 0.05;
    const sleep_angular_radps: f32 = 0.05;
    const sleep_after_s: f32 = 0.5;

    pub const Config = struct {
        gravity: emath.Vec3,
        fixed_step_s: f32,

        pub fn init(gravity: emath.Vec3, fixed_step_s: f32) Config {
            return .{
                .gravity = gravity,
                .fixed_step_s = fixed_step_s,
            };
        }
    };

    const Attached = struct { collider: c.Collider, parent: ?b.BodyHandle };

    /// A collider where this step has it, with the body it moves with.
    const Placed = struct {
        handle: c.ColliderHandle,
        shape: shapes.Shape,
        pose: math.Pose,
        bounds: shapes.Aabb,
        material: c.Material,
        body: ?*b.State,
    };

    /// Pairs of colliders whose bounds overlap, found by the broadphase, as indices into `placed`.
    const ColliderPair = struct { a: u32, b: u32 };

    pub fn init(gpa: std.mem.Allocator, config: Config) !Physics {
        return .{ .gpa = gpa, .config = config };
    }

    pub fn deinit(self: *Physics) void {
        self.bodies.deinit(self.gpa);
        self.colliders.deinit(self.gpa);
        self.placed.deinit(self.gpa);
        self.pairs.deinit(self.gpa);
        self.constraints.deinit(self.gpa);
        self.contact_list.deinit(self.gpa);
    }

    /// Adds a body with no colliders yet: it moves, but touches nothing until one is attached.
    pub fn addBody(self: *Physics, body: b.Body) !b.BodyHandle {
        return self.bodies.add(self.gpa, b.State.init(body));
    }

    /// Removes the body and every collider attached to it. A stale handle is ignored.
    pub fn removeBody(self: *Physics, body: b.BodyHandle) void {
        for (self.colliders.entries.items, 0..) |entry, index| {
            const attached = entry.item orelse continue;
            if (attached.parent != null and attached.parent.? == body) _ = self.colliders.remove(self.colliders.handleAt(index));
        }
        _ = self.bodies.remove(body);
    }

    /// Whether `handle`, a `BodyHandle` or `ColliderHandle`, still names something in the world.
    pub fn contains(self: *const Physics, handle: anytype) bool {
        return switch (@TypeOf(handle)) {
            b.BodyHandle => self.bodies.get(handle) != null,
            c.ColliderHandle => self.colliders.get(handle) != null,
            else => @compileError("contains takes a BodyHandle or ColliderHandle"),
        };
    }

    /// Attaches `collider` to `parent`, or with no parent makes it static scenery.
    /// The parent's mass is spread over its colliders by volume, for how it turns.
    pub fn addCollider(self: *Physics, collider: c.Collider, parent: ?b.BodyHandle) !c.ColliderHandle {
        if (parent) |body| if (self.bodies.get(body) == null) return error.StaleBody;
        const handle = try self.colliders.add(self.gpa, .{ .collider = collider, .parent = parent });
        if (parent) |body| self.updateInertia(body);
        return handle;
    }

    /// Detaches and removes `collider`. A stale handle is ignored.
    pub fn removeCollider(self: *Physics, collider: c.ColliderHandle) void {
        const attached = (self.colliders.get(collider) orelse return).*;
        _ = self.colliders.remove(collider);
        if (attached.parent) |body| if (self.bodies.get(body) != null) self.updateInertia(body);
    }

    /// The body `collider` is attached to, or null for static scenery or a stale handle.
    pub fn colliderBody(self: *const Physics, collider: c.ColliderHandle) ?b.BodyHandle {
        const attached = self.colliders.get(collider) orelse return null;
        return attached.parent;
    }

    pub fn update(self: *Physics, frame_dt_s: f32) !void {
        self.accumulator_s += @min(frame_dt_s, max_frame_dt_s);
        while (self.accumulator_s >= self.config.fixed_step_s) {
            try self.step(self.config.fixed_step_s);
            self.accumulator_s -= self.config.fixed_step_s;
        }
    }

    fn step(self: *Physics, dt_s: f32) !void {
        // Cleared here rather than at the end, so `contacts()` still shows the
        // last step's contacts until the next one runs.
        self.placed.clearRetainingCapacity();
        self.pairs.clearRetainingCapacity();
        self.constraints.clearRetainingCapacity();
        self.contact_list.clearRetainingCapacity();

        // Gravity speeds up every awake body before anything pushes back.
        for (self.bodies.entries.items) |*entry| {
            const state = &(entry.item orelse continue);
            if (state.moves()) state.linear = state.linear.add(self.config.gravity.scale(dt_s));
        }

        try self.placeColliders();
        try self.findPairs();

        // Narrowphase: each overlapping pair's contacts, and a constraint for each to solve.
        for (self.pairs.items) |pair| {
            const a = self.placed.items[pair.a];
            const bb = self.placed.items[pair.b];
            const manifold = narrowphase.collide(a.shape, a.pose, bb.shape, bb.pose);
            if (manifold.len == 0) continue;
            wakeOnTouch(a.body, bb.body);
            const friction = @sqrt(a.material.friction * bb.material.friction);
            const restitution = @max(a.material.restitution, bb.material.restitution);
            for (manifold.slice()) |point| {
                try self.contact_list.append(self.gpa, .{ .a = a.handle, .b = bb.handle, .point = point.point, .normal = point.normal, .depth_m = point.depth_m });
                try self.constraints.append(self.gpa, solver.Constraint.init(a.body, bb.body, point.point, point.normal, point.depth_m, friction, restitution, dt_s));
            }
        }

        solver.solve(self.constraints.items, solver_iterations);

        for (self.bodies.entries.items) |*entry| {
            const state = &(entry.item orelse continue);
            if (!state.moves()) continue;
            state.pose.translation = state.pose.translation.add(state.linear.scale(dt_s));
            state.pose.rotation = emath.UnitQuat.fromScaledAxis(state.angular.scale(dt_s)).mul(state.pose.rotation).renormalize();
            settle(state, dt_s);
        }
    }

    /// Fills `placed` with every collider where its body has it this step.
    fn placeColliders(self: *Physics) !void {
        for (self.colliders.entries.items, 0..) |entry, index| {
            const attached = entry.item orelse continue;
            const body = if (attached.parent) |parent| self.bodies.get(parent) else null;
            const placed_pose = if (body) |state| state.pose.mul(attached.collider.local_pose) else attached.collider.local_pose;
            try self.placed.append(self.gpa, .{
                .handle = self.colliders.handleAt(index),
                .shape = attached.collider.shape,
                .pose = placed_pose,
                .bounds = attached.collider.shape.aabb(placed_pose),
                .material = attached.collider.material,
                .body = body,
            });
        }
    }

    // ponytail: every pair is tested, O(n²); add a sweep-and-prune or tree past a few hundred colliders.
    fn findPairs(self: *Physics) !void {
        for (self.placed.items, 0..) |a, i| {
            for (self.placed.items[i + 1 ..], i + 1..) |bb, j| {
                if (!canTouch(a.body, bb.body) or !a.bounds.overlaps(bb.bounds)) continue;
                try self.pairs.append(self.gpa, .{ .a = @intCast(i), .b = @intCast(j) });
            }
        }
    }

    /// Whether colliders on these bodies can push each other: one is dynamic, and they aren't
    /// the same body.
    fn canTouch(a: ?*b.State, bb: ?*b.State) bool {
        if (a != null and a == bb) return false;
        const a_dynamic = if (a) |s| s.kind == .dynamic else false;
        const b_dynamic = if (bb) |s| s.kind == .dynamic else false;
        return a_dynamic or b_dynamic;
    }

    /// Something awake touching a sleeping body wakes it.
    fn wakeOnTouch(a: ?*b.State, bb: ?*b.State) void {
        const a_awake = if (a) |s| s.moves() else false;
        const b_awake = if (bb) |s| s.moves() else false;
        if (a_awake) if (bb) |s| if (s.kind == .dynamic) s.wake();
        if (b_awake) if (a) |s| if (s.kind == .dynamic) s.wake();
    }

    /// Sleeps a body once it has been nearly still for `sleep_after_s`.
    fn settle(state: *b.State, dt_s: f32) void {
        const still = state.linear.lengthSquared() < sleep_linear_mps * sleep_linear_mps and
            state.angular.lengthSquared() < sleep_angular_radps * sleep_angular_radps;
        state.still_s = if (still) state.still_s + dt_s else 0;
        if (state.still_s < sleep_after_s) return;
        state.sleeping = true;
        state.linear = .zero;
        state.angular = .zero;
    }

    /// Spreads `body`'s mass over its colliders by volume, each about its place on the body.
    // ponytail: colliders' own rotations and an off-centre centre of mass are ignored; take the
    // full inertia tensor when tilted or lopsided compounds need to turn right.
    fn updateInertia(self: *Physics, body: b.BodyHandle) void {
        const state = self.bodies.get(body).?;
        if (state.inv_mass == 0) return;
        const mass_kg = 1 / state.inv_mass;

        var total_volume: f32 = 0;
        var count: f32 = 0;
        for (self.colliders.entries.items) |entry| {
            const attached = entry.item orelse continue;
            if (attached.parent == null or attached.parent.? != body) continue;
            total_volume += attached.collider.shape.volume();
            count += 1;
        }

        var inertia: Vec3 = .zero;
        for (self.colliders.entries.items) |entry| {
            const attached = entry.item orelse continue;
            if (attached.parent == null or attached.parent.? != body) continue;
            const share = if (total_volume > 0) attached.collider.shape.volume() / total_volume else 1 / count;
            const part_kg = mass_kg * share;
            // Parallel axis: a part off the centre is harder to turn about the axes it is away from.
            const offset = attached.collider.local_pose.translation;
            const away = Vec3.splat(offset.lengthSquared()).sub(offset.mul(offset)).scale(part_kg);
            inertia = inertia.add(attached.collider.shape.inertia(part_kg)).add(away);
        }
        state.setInertia(inertia);
    }

    /// Pushes `body` by `impulse_ns` (N·s, world space), waking it. Static bodies don't move.
    pub fn applyImpulse(self: *Physics, body: b.BodyHandle, impulse_ns: emath.Vec3, at: math.ImpulsePoint) void {
        const state = self.bodies.get(body) orelse return;
        if (state.kind != .dynamic) return;
        state.wake();
        const offset = switch (at) {
            .center_of_mass => Vec3.zero,
            .at_world_point => |point| point.sub(state.pose.translation),
        };
        state.applyImpulse(impulse_ns, offset);
    }

    /// Moves `body` to `new_pose` at once, keeping its velocity and waking it.
    pub fn setPose(self: *Physics, body: b.BodyHandle, new_pose: math.Pose) void {
        const state = self.bodies.get(body) orelse return;
        state.pose = new_pose;
        state.wake();
    }

    /// Where `body` is. It must be in the world.
    pub fn pose(self: *const Physics, body: b.BodyHandle) math.Pose {
        return self.bodies.get(body).?.pose;
    }

    /// How fast `body` moves and turns. It must be in the world.
    pub fn velocity(self: *const Physics, body: b.BodyHandle) math.Velocity {
        const state = self.bodies.get(body).?;
        return .{ .linear = state.linear, .angular = state.angular };
    }

    /// Whether `body` has settled and stopped being stepped. It must be in the world.
    pub fn isSleeping(self: *const Physics, body: b.BodyHandle) bool {
        return self.bodies.get(body).?.sleeping;
    }

    /// The contacts found by the last step. Valid until the next `update`.
    pub fn contacts(self: *const Physics) []const math.Contact {
        return self.contact_list.items;
    }

    /// The nearest collider `ray` meets, as things stand now.
    pub fn raycast(self: *const Physics, ray: r.Ray) ?r.RayHit {
        var nearest: ?r.RayHit = null;
        for (self.colliders.entries.items, 0..) |entry, index| {
            const attached = entry.item orelse continue;
            const body = if (attached.parent) |parent| self.bodies.get(parent) else null;
            const placed_pose = if (body) |state| state.pose.mul(attached.collider.local_pose) else attached.collider.local_pose;
            const hit = r.cast(ray, attached.collider.shape, placed_pose) orelse continue;
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
