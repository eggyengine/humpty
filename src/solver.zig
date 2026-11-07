//! Contact solver: sequential impulses. Each contact pushes its two bodies apart along its
//! normal, holds them by friction along the surface, and is revisited until they agree.
const std = @import("std");
const math = @import("math.zig");
const body = @import("body.zig");

const Vec3 = math.emath.Vec3;
const State = body.State;

/// Overlap left alone, in metres, so resting contacts stay touching instead of jittering.
const slop_m: f32 = 0.005;
/// The share of the remaining overlap pushed out each step.
const correction: f32 = 0.2;
/// Slower approaches than this, in m/s, don't bounce, so resting bodies settle.
const bounce_threshold_mps: f32 = 1;

pub const Constraint = struct {
    /// Null for what doesn't move: scenery, static bodies, and sleeping ones.
    a: ?*State,
    b: ?*State,
    offset_a: Vec3,
    offset_b: Vec3,
    /// From `a` towards `b`, and the two directions along the surface.
    normal: Vec3,
    tangents: [2]Vec3,
    normal_mass: f32,
    tangent_mass: [2]f32,
    /// The separation speed along the normal the solver aims for, in m/s.
    target_mps: f32,
    friction: f32,
    normal_impulse: f32 = 0,
    tangent_impulse: [2]f32 = .{ 0, 0 },

    /// A contact at `point` between `a` and `b` (either null when it doesn't move).
    pub fn init(a: ?*State, b: ?*State, point: Vec3, normal: Vec3, depth_m: f32, friction: f32, restitution: f32, dt_s: f32) Constraint {
        const moving_a = if (a) |s| if (s.moves()) s else null else null;
        const moving_b = if (b) |s| if (s.moves()) s else null else null;
        const offset_a = if (a) |s| point.sub(s.pose.translation) else Vec3.zero;
        const offset_b = if (b) |s| point.sub(s.pose.translation) else Vec3.zero;
        const tangents = surfaceAxes(normal);

        var out: Constraint = .{
            .a = moving_a,
            .b = moving_b,
            .offset_a = offset_a,
            .offset_b = offset_b,
            .normal = normal,
            .tangents = tangents,
            .normal_mass = 0,
            .tangent_mass = .{ 0, 0 },
            .target_mps = 0,
            .friction = friction,
        };
        out.normal_mass = out.massAlong(normal);
        out.tangent_mass = .{ out.massAlong(tangents[0]), out.massAlong(tangents[1]) };

        // Push out the overlap beyond the slop, or bounce back a fast approach, whichever is more.
        const approach = out.relativeVelocity().dot(normal);
        const push = correction / dt_s * @max(depth_m - slop_m, 0);
        const bounce = if (approach < -bounce_threshold_mps) -restitution * approach else 0;
        out.target_mps = @max(push, bounce);
        return out;
    }

    /// The impulse along `direction` that changes the bodies' relative speed by 1 m/s.
    fn massAlong(self: Constraint, direction: Vec3) f32 {
        var inverse: f32 = 0;
        if (self.a) |s| inverse += s.inv_mass + s.turnBy(self.offset_a.cross(direction)).cross(self.offset_a).dot(direction);
        if (self.b) |s| inverse += s.inv_mass + s.turnBy(self.offset_b.cross(direction)).cross(self.offset_b).dot(direction);
        return if (inverse > 0) 1 / inverse else 0;
    }

    /// How fast `b`'s contact point moves relative to `a`'s.
    fn relativeVelocity(self: Constraint) Vec3 {
        const va = if (self.a) |s| s.velocityAt(self.offset_a) else Vec3.zero;
        const vb = if (self.b) |s| s.velocityAt(self.offset_b) else Vec3.zero;
        return vb.sub(va);
    }

    fn apply(self: *Constraint, impulse: Vec3) void {
        if (self.a) |s| s.applyImpulse(impulse.negate(), self.offset_a);
        if (self.b) |s| s.applyImpulse(impulse, self.offset_b);
    }

    fn solveOnce(self: *Constraint) void {
        // Friction first, limited by the normal impulse so far, so the push has the last word.
        for (self.tangents, self.tangent_mass, &self.tangent_impulse) |tangent, mass, *total| {
            const limit = self.friction * self.normal_impulse;
            const change = -mass * self.relativeVelocity().dot(tangent);
            const new_total = std.math.clamp(total.* + change, -limit, limit);
            self.apply(tangent.scale(new_total - total.*));
            total.* = new_total;
        }
        // Totals stay non-negative: a contact pushes, never pulls.
        const change = self.normal_mass * (self.target_mps - self.relativeVelocity().dot(self.normal));
        const new_total = @max(self.normal_impulse + change, 0);
        self.apply(self.normal.scale(new_total - self.normal_impulse));
        self.normal_impulse = new_total;
    }
};

/// Revisits every contact `iterations` times, changing the bodies' velocities.
pub fn solve(constraints: []Constraint, iterations: u32) void {
    for (0..iterations) |_| {
        for (constraints) |*constraint| constraint.solveOnce();
    }
}

/// Two unit directions along a surface with unit normal `normal`, at right angles.
fn surfaceAxes(normal: Vec3) [2]Vec3 {
    const first = if (@abs(normal.x) > 0.57) Vec3.init(normal.y, -normal.x, 0).normalize() else Vec3.init(0, normal.z, -normal.y).normalize();
    return .{ first, normal.cross(first) };
}

test "a ball falling onto still ground stops instead of sinking" {
    var ball = State.init(.{ .kind = .dynamic, .pose = math.Pose.fromTranslation(Vec3.init(0, 0.5, 0)), .mass_kg = 1 });
    ball.linear = Vec3.init(0, -0.5, 0);
    var contacts = [_]Constraint{Constraint.init(&ball, null, Vec3.zero, Vec3.down, 0, 0.5, 0, 1.0 / 60.0)};
    solve(&contacts, 4);
    try std.testing.expectApproxEqAbs(@as(f32, 0), ball.linear.y, 1e-5);
}
