//! Joints: two bodies held together at an anchor on each, solved beside contacts by sequential
//! impulses, one row per direction held. Fixed, ball (with a cone limit, for a hitch), hinge
//! and slider (each with limits and a motor). Errors are pushed out a share a step (Baumgarte)
//! and last step's impulses start the next (warm starting), so loaded chains hold still. 3D only.
const std = @import("std");
const body = @import("body.zig");
const math = @import("math.zig");

pub const JointHandle = math.Handle(.joint);

/// Joints in 3D space `S` (a `space.Space`).
pub fn Joints(comptime S: type) type {
    const T = S.Real;
    const Vec = S.Vec;
    const State = body.Bodies(S).State;
    // The share of a joint's error pushed out each step.
    const correction: T = 0.2;
    // Below this, an angle or length counts as none.
    const tiny: T = 1e-6;
    // How quickly a position motor closes on its target: it aims to be there in this long.
    const position_motor_s: T = 0.1;

    return struct {
        /// How a hinge or slider drives the second body against the first, pushing with up to
        /// `max` (N·m for a hinge, N for a slider).
        pub const Motor = union(enum) {
            off,
            /// Turns (rad/s) or slides (m/s) at `speed`.
            velocity: struct { speed: T, max: T },
            /// Goes to `target` (rad or m from where the joint was made) at up to `speed`, and
            /// holds it there.
            position: struct { target: T, speed: T, max: T },
        };

        /// How far a hinge turns or a slider slides either way: rad or m from where it was made,
        /// the second body against the first.
        pub const Limits = struct { lower: T, upper: T };

        pub const Kind = union(enum) {
            /// Holds both together as one.
            fixed,
            /// Turns freely about the anchor. With a `cone_rad`, `axis` (in the first body's
            /// space) stays within that angle of where it pointed on the second when made.
            ball: struct { axis: Vec = Vec.init(0, 0, -1), cone_rad: ?T = null },
            /// Turns only about `axis`, in the first body's space.
            hinge: struct { axis: Vec, limits: ?Limits = null, motor: Motor = .off },
            /// Slides only along `axis`, in the first body's space, without turning.
            slider: struct { axis: Vec, limits: ?Limits = null, motor: Motor = .off },
        };

        pub const Joint = struct {
            kind: Kind,
            /// Where it holds each body, in that body's own space; the second's in world space
            /// when it is held to the world.
            anchor_a: Vec = .zero,
            anchor_b: Vec = .zero,
            /// It breaks holding with more force than this (N); null never breaks.
            break_force_n: ?T = null,
        };

        /// The rows one joint can need: three to hold a point, three to hold a turn, two limits,
        /// a motor and a cone.
        const max_rows = 10;
        const RowSlot = enum(u8) { point0, point1, point2, turn0, turn1, turn2, lower, upper, motor, cone };

        /// A joint as the world keeps it, with what it measured when made.
        pub const Stored = struct {
            joint: Joint,
            /// The second body's turn relative to the first's when made.
            rest: S.Rotation,
            /// Last step's impulses, to start the next with.
            impulses: [max_rows]T = @splat(0),
            broken: bool = false,
            /// Last step's angle (rad, hinge) or offset (m, slider) from where it was made.
            position: T = 0,
            /// Last step's force holding its anchors together, in N.
            force_n: T = 0,

            pub fn init(joint: Joint, a: *const State, b: ?*const State) Stored {
                const turn_b = if (b) |s| s.pose.rotation else S.Rotation.identity();
                return .{ .joint = joint, .rest = a.pose.rotation.inverse().mul(turn_b) };
            }
        };

        /// One direction a joint holds this step: the relative velocity along `linear` at the
        /// anchors, plus about `angular`, driven towards `target` with a total impulse kept
        /// within `lower`..`upper`.
        pub const Row = struct {
            a: ?*State,
            b: ?*State,
            moving_a: ?*State,
            moving_b: ?*State,
            linear: Vec,
            /// What turning each body changes the row's speed by: the anchor's lever plus `angular`.
            turn_a: Vec,
            turn_b: Vec,
            mass: T,
            target: T,
            lower: T = -std.math.inf(T),
            upper: T = std.math.inf(T),
            impulse: T = 0,
            stored: *T,

            fn velocity(self: Row) T {
                var out: T = 0;
                if (self.moving_b) |s| out += self.linear.dot(s.linear) + self.turn_b.dot(s.angular);
                if (self.moving_a) |s| out -= self.linear.dot(s.linear) + self.turn_a.dot(s.angular);
                return out;
            }

            fn apply(self: Row, impulse: T) void {
                if (self.a) |s| {
                    s.linear = s.linear.sub(self.linear.scale(impulse * s.inv_mass));
                    s.angular = s.angular.sub(s.turnBy(self.turn_a.scale(impulse)));
                }
                if (self.b) |s| {
                    s.linear = s.linear.add(self.linear.scale(impulse * s.inv_mass));
                    s.angular = s.angular.add(s.turnBy(self.turn_b.scale(impulse)));
                }
            }

            pub fn solveOnce(self: *Row) void {
                const change = self.mass * (self.target - self.velocity());
                const total = std.math.clamp(self.impulse + change, self.lower, self.upper);
                self.apply(total - self.impulse);
                self.impulse = total;
            }
        };

        /// The anchors and frames of a joint where its bodies are this step.
        const Frame = struct {
            a: *State,
            b: ?*State,
            ra: Vec,
            rb: Vec,
            /// From the first anchor to the second.
            gap: Vec,

            fn of(joint: Joint, a: *State, b: ?*State) Frame {
                const pa = a.pose.transformPoint(joint.anchor_a);
                const pb = if (b) |s| s.pose.transformPoint(joint.anchor_b) else joint.anchor_b;
                return .{
                    .a = a,
                    .b = b,
                    .ra = pa.sub(a.center()),
                    .rb = if (b) |s| pb.sub(s.center()) else Vec.zero,
                    .gap = pb.sub(pa),
                };
            }

            fn turnB(self: Frame) S.Rotation {
                return if (self.b) |s| s.pose.rotation else S.Rotation.identity();
            }
        };

        /// Adds `stored`'s rows for this step to `rows`, started with last step's impulses.
        pub fn build(stored: *Stored, a: *State, b: ?*State, dt_s: T, rows: anytype, gpa: std.mem.Allocator) !void {
            const frame = Frame.of(stored.joint, a, b);
            const bias = correction / dt_s;
            const turn_a = a.pose.rotation;
            switch (stored.joint.kind) {
                .fixed => {
                    try holdPoint(stored, frame, &.{ S.axis(0), S.axis(1), S.axis(2) }, bias, rows, gpa);
                    try holdTurn(stored, frame, bias, rows, gpa);
                },
                .ball => |ball| {
                    try holdPoint(stored, frame, &.{ S.axis(0), S.axis(1), S.axis(2) }, bias, rows, gpa);
                    if (ball.cone_rad) |cone| {
                        const on_a = turn_a.rotateVector(ball.axis.normalize());
                        const on_b = frame.turnB().rotateVector(stored.rest.inverseRotateVector(ball.axis.normalize()));
                        const away = on_a.cross(on_b);
                        const sin = away.length();
                        if (sin > tiny) {
                            const angle = std.math.atan2(sin, on_a.dot(on_b));
                            // Turning `b` about `away` opens the angle; the row closes it.
                            try rows.append(gpa, limitRow(stored, frame, .cone, .zero, away.scale(-1 / sin), cone - angle, dt_s));
                        }
                    }
                },
                .hinge => |hinge| {
                    try holdPoint(stored, frame, &.{ S.axis(0), S.axis(1), S.axis(2) }, bias, rows, gpa);
                    const axis_a = turn_a.rotateVector(hinge.axis.normalize());
                    const axis_b = frame.turnB().rotateVector(stored.rest.inverseRotateVector(hinge.axis.normalize()));
                    const across = S.tangents(axis_a);
                    const misaligned = axis_a.cross(axis_b);
                    for (across, 0..) |direction, i| try rows.append(gpa, row(stored, frame, @fromBackingInt(@intCast(@backingInt(RowSlot.turn0) + i)), .zero, direction, -bias * misaligned.dot(direction)));
                    // The angle about the axis from a reference across it, as each body carries it.
                    const reference = S.tangents(hinge.axis.normalize())[0];
                    const ref_a = turn_a.rotateVector(reference);
                    const ref_b = frame.turnB().rotateVector(stored.rest.inverseRotateVector(reference));
                    stored.position = std.math.atan2(axis_a.dot(ref_a.cross(ref_b)), ref_a.dot(ref_b));
                    try freeAxis(stored, frame, .zero, axis_a, hinge.limits, hinge.motor, dt_s, rows, gpa);
                },
                .slider => |slider| {
                    const axis_a = turn_a.rotateVector(slider.axis.normalize());
                    const across = S.tangents(axis_a);
                    try holdPoint(stored, frame, &across, bias, rows, gpa);
                    try holdTurn(stored, frame, bias, rows, gpa);
                    stored.position = frame.gap.dot(axis_a);
                    try freeAxis(stored, frame, axis_a, .zero, slider.limits, slider.motor, dt_s, rows, gpa);
                },
            }
        }

        /// Rows keeping the anchors together along each of `directions`.
        fn holdPoint(stored: *Stored, frame: Frame, directions: []const Vec, bias: T, rows: anytype, gpa: std.mem.Allocator) !void {
            for (directions, 0..) |direction, i| try rows.append(gpa, row(stored, frame, @fromBackingInt(@intCast(i)), direction, .zero, -bias * frame.gap.dot(direction)));
        }

        /// Rows keeping the second body turned as it was against the first.
        fn holdTurn(stored: *Stored, frame: Frame, bias: T, rows: anytype, gpa: std.mem.Allocator) !void {
            const wanted = frame.a.pose.rotation.mul(stored.rest);
            const off = frame.turnB().mul(wanted.inverse()).quat.v;
            // Twice the vector part is the small turn from where it should be to where it is.
            const sign: T = if (off[3] < 0) -2 else 2;
            const error_turn = Vec.init(off[0] * sign, off[1] * sign, off[2] * sign);
            for (0..3) |i| try rows.append(gpa, row(stored, frame, @fromBackingInt(@intCast(@backingInt(RowSlot.turn0) + i)), .zero, S.axis(i), -bias * error_turn.dot(S.axis(i))));
        }

        /// A hinge's or slider's limits and motor along its free `linear` or `angular` axis.
        fn freeAxis(stored: *Stored, frame: Frame, linear: Vec, angular: Vec, limits: ?Limits, motor: Motor, dt_s: T, rows: anytype, gpa: std.mem.Allocator) !void {
            if (limits) |range| {
                try rows.append(gpa, limitRow(stored, frame, .lower, linear, angular, stored.position - range.lower, dt_s));
                try rows.append(gpa, limitRow(stored, frame, .upper, linear.negate(), angular.negate(), range.upper - stored.position, dt_s));
            }
            switch (motor) {
                .off => stored.impulses[@backingInt(RowSlot.motor)] = 0,
                .velocity => |drive| {
                    var out = row(stored, frame, .motor, linear, angular, drive.speed);
                    out.lower = -drive.max * dt_s;
                    out.upper = drive.max * dt_s;
                    try rows.append(gpa, out);
                },
                .position => |drive| {
                    const speed = std.math.clamp((drive.target - stored.position) / @max(position_motor_s, dt_s), -drive.speed, drive.speed);
                    var out = row(stored, frame, .motor, linear, angular, speed);
                    out.lower = -drive.max * dt_s;
                    out.upper = drive.max * dt_s;
                    try rows.append(gpa, out);
                },
            }
        }

        /// A row that only pushes, keeping `room` (how far inside the limit it is) from going
        /// below zero: it lets the bodies close what room is left in a step, and pushes back out
        /// a share of any overshoot.
        fn limitRow(stored: *Stored, frame: Frame, slot: RowSlot, linear: Vec, angular: Vec, room: T, dt_s: T) Row {
            const target = if (room < 0) -correction / dt_s * room else -room / dt_s;
            var out = row(stored, frame, slot, linear, angular, target);
            out.lower = 0;
            out.impulse = @max(out.impulse, 0);
            return out;
        }

        fn row(stored: *Stored, frame: Frame, slot: RowSlot, linear: Vec, angular: Vec, target: T) Row {
            const a = frame.a;
            const b = frame.b;
            var out: Row = .{
                .a = if (a.pushable()) a else null,
                .b = if (b) |s| if (s.pushable()) s else null else null,
                .moving_a = if (a.moving()) a else null,
                .moving_b = if (b) |s| if (s.moving()) s else null else null,
                .linear = linear,
                .turn_a = frame.ra.cross(linear).add(angular),
                .turn_b = frame.rb.cross(linear).add(angular),
                .mass = 0,
                .target = target,
                .stored = &stored.impulses[@backingInt(slot)],
            };
            var inverse: T = 0;
            if (out.a) |s| inverse += s.inv_mass * linear.lengthSquared() + out.turn_a.dot(s.turnBy(out.turn_a));
            if (out.b) |s| inverse += s.inv_mass * linear.lengthSquared() + out.turn_b.dot(s.turnBy(out.turn_b));
            out.mass = if (inverse > 0) 1 / inverse else 0;
            out.impulse = out.stored.*;
            return out;
        }

        /// Starts each row with last step's impulse.
        pub fn warmStart(rows: []Row) void {
            for (rows) |r| r.apply(r.impulse);
        }

        /// Keeps each row's impulse for next step.
        pub fn store(rows: []const Row) void {
            for (rows) |r| r.stored.* = r.impulse;
        }

        /// The force holding a joint's anchors together last step, in N.
        pub fn pointForce(stored: *const Stored, dt_s: T) T {
            const i = stored.impulses;
            const slot = @backingInt(RowSlot.point0);
            return @sqrt(i[slot] * i[slot] + i[slot + 1] * i[slot + 1] + i[slot + 2] * i[slot + 2]) / dt_s;
        }
    };
}

const TestWorld = @import("physics.zig").Physics(.three, f32);

fn testWorld() !TestWorld {
    return TestWorld.init(std.testing.allocator, .{ .gravity = TestWorld.Vec.init(0, -9.81, 0), .fixed_step_s = 1.0 / 120.0 });
}

fn box(world: *TestWorld, at: TestWorld.Vec, mass_kg: f32) !body.BodyHandle {
    const handle = try world.addBody(.{ .kind = .dynamic, .pose = TestWorld.Pose.fromTranslation(at), .mass_kg = mass_kg }, 0);
    _ = try world.addCollider(.{ .shape = .{ .cuboid = .{ .half_extents = TestWorld.Vec.splat(0.25) } } }, handle, 0);
    return handle;
}

fn run(world: *TestWorld, seconds: f32) !void {
    for (0..@intFromFloat(seconds * 120)) |_| try world.update(1.0 / 120.0);
}

test "a ball joint holds a heavy body under a light one, and hinge and slider limits hold against gravity" {
    const Vec = TestWorld.Vec;
    var world = try testWorld();
    defer world.deinit();

    // A 1500 kg body on a ball joint under a 100 kg one held to the world: the ratio a trailer
    // and its wing have.
    const top = try box(&world, Vec.init(0, 5, 0), 100);
    const hanging = try box(&world, Vec.init(0, 3, 0), 1500);
    _ = try world.addJoint(.{ .kind = .{ .ball = .{} }, .anchor_b = Vec.init(0, 5, 0) }, top, null, 0);
    const ball = try world.addJoint(.{ .kind = .{ .ball = .{} }, .anchor_a = Vec.init(0, -1, 0), .anchor_b = Vec.init(0, 1, 0) }, top, hanging, 0);
    // A hinge about z that would swing a bar down 90°, stopped at 30°: the world turns up
    // against the bar as the bar turns down.
    const bar = try box(&world, Vec.init(10, 5, 0), 50);
    const hinge = try world.addJoint(.{ .kind = .{ .hinge = .{ .axis = Vec.init(0, 0, 1), .limits = .{ .lower = -0.5, .upper = std.math.pi / 6.0 } } }, .anchor_a = Vec.init(-1, 0, 0), .anchor_b = Vec.init(9, 5, 0) }, bar, null, 0);
    // A slider down y that would drop forever, stopped a metre down.
    const drawer = try box(&world, Vec.init(20, 5, 0), 50);
    const slider = try world.addJoint(.{ .kind = .{ .slider = .{ .axis = Vec.init(0, 1, 0), .limits = .{ .lower = -1, .upper = 1 } } }, .anchor_b = Vec.init(20, 5, 0) }, drawer, null, 0);
    try run(&world, 3);

    try std.testing.expectApproxEqAbs(@as(f32, 3), world.pose(hanging).translation.y, 0.02);
    try std.testing.expect(world.velocity(hanging).linear.length() < 0.05);
    try std.testing.expect(world.jointState(ball).?.stored.force_n > 1500 * 9.81 * 0.95);
    try std.testing.expectApproxEqAbs(@as(f32, std.math.pi / 6.0), world.jointState(hinge).?.stored.position, 0.02);
    try std.testing.expectApproxEqAbs(@as(f32, 1), world.jointState(slider).?.stored.position, 0.02);
    try std.testing.expectApproxEqAbs(@as(f32, 4), world.pose(drawer).translation.y, 0.02);
    try std.testing.expectApproxEqAbs(@as(f32, 20), world.pose(drawer).translation.x, 0.01);
}

test "a slider motor reaches its target and holds it under load, a hinge motor turns at its speed, and a fixed joint holds" {
    const Vec = TestWorld.Vec;
    var world = try testWorld();
    defer world.deinit();

    // A 1500 kg trailer bed, held still, pushes a 100 kg deck section up and out 0.8 m along a
    // slider tilted 45°, against gravity.
    const bed = try world.addBody(.{ .kind = .static, .pose = TestWorld.Pose.identity() }, 0);
    const deck = try box(&world, Vec.init(0, 0, 0), 100);
    const out = Vec.init(1, 1, 0).normalize();
    const slide = try world.addJoint(.{ .kind = .{ .slider = .{ .axis = out, .motor = .{ .position = .{ .target = 0.8, .speed = 0.5, .max = 5000 } } } } }, bed, deck, 0);
    // A wing turning at 1 rad/s with no gravity to fight: the world turns forward against it, so
    // the wing turns back.
    const wing = try box(&world, Vec.init(5, 0, 0), 100);
    world.bodies.get(wing).?.item.gravity_scale = 0;
    _ = try world.addJoint(.{ .kind = .{ .hinge = .{ .axis = Vec.init(0, 1, 0), .motor = .{ .velocity = .{ .speed = 1, .max = 2000 } } } }, .anchor_b = Vec.init(5, 0, 0) }, wing, null, 0);
    // A crate welded to a falling lever arm keeps its place on it.
    const lever = try box(&world, Vec.init(10, 5, 0), 20);
    const crate = try box(&world, Vec.init(11, 5, 0), 20);
    _ = try world.addJoint(.{ .kind = .fixed, .anchor_a = Vec.init(0.5, 0, 0), .anchor_b = Vec.init(-0.5, 0, 0) }, lever, crate, 0);
    try run(&world, 4);

    try std.testing.expectApproxEqAbs(@as(f32, 0.8), world.jointState(slide).?.stored.position, 0.01);
    try std.testing.expect(world.pose(deck).translation.sub(out.scale(0.8)).length() < 0.01);
    try std.testing.expectApproxEqAbs(@as(f32, -1), world.velocity(wing).angular.y, 0.02);
    const apart = world.pose(lever).inverse().mul(world.pose(crate)).translation;
    try std.testing.expect(apart.sub(Vec.init(1, 0, 0)).length() < 0.01);
}
