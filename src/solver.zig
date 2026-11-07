//! Contact solver: sequential impulses. Each contact pushes its two bodies apart along its
//! normal, holds them by friction along the surface, and is revisited until they agree.
const std = @import("std");
const body = @import("body.zig");

/// The solver for space `S` (a `space.Space`).
pub fn Solver(comptime S: type) type {
    const T = S.Real;
    const Vec = S.Vec;
    const State = body.Bodies(S).State;
    // Overlap left alone, in metres, so resting contacts stay touching instead of jittering.
    const slop_m: T = 0.005;
    // The share of the remaining overlap pushed out each step.
    const correction: T = 0.2;
    // Slower approaches than this, in m/s, don't bounce, so resting bodies settle.
    const bounce_threshold_mps: T = 1;

    return struct {
        pub const Constraint = struct {
            /// Null for what contacts don't push: scenery, static, kinematic and sleeping bodies.
            a: ?*State,
            b: ?*State,
            /// Every body involved, pushed or not, for the velocity of a kinematic one.
            moving_a: ?*State,
            moving_b: ?*State,
            offset_a: Vec,
            offset_b: Vec,
            /// From `a` towards `b`, and the directions along the surface.
            normal: Vec,
            tangents: [S.n - 1]Vec,
            normal_mass: T,
            tangent_mass: [S.n - 1]T,
            /// The separation speed along the normal the solver aims for, in m/s.
            target_mps: T,
            friction: T,
            normal_impulse: T = 0,
            tangent_impulse: [S.n - 1]T = @splat(0),

            /// A contact at `point` between `a` and `b` (either null for scenery).
            pub fn init(a: ?*State, b: ?*State, point: Vec, normal: Vec, depth_m: T, friction: T, restitution: T, dt_s: T) Constraint {
                const tangents = S.tangents(normal);
                var out: Constraint = .{
                    .a = if (a) |s| if (s.pushable()) s else null else null,
                    .b = if (b) |s| if (s.pushable()) s else null else null,
                    .moving_a = if (a) |s| if (s.moving()) s else null else null,
                    .moving_b = if (b) |s| if (s.moving()) s else null else null,
                    .offset_a = if (a) |s| point.sub(s.pose.translation) else Vec.zero,
                    .offset_b = if (b) |s| point.sub(s.pose.translation) else Vec.zero,
                    .normal = normal,
                    .tangents = tangents,
                    .normal_mass = 0,
                    .tangent_mass = undefined,
                    .target_mps = 0,
                    .friction = friction,
                };
                out.normal_mass = out.massAlong(normal);
                for (&out.tangent_mass, tangents) |*mass, tangent| mass.* = out.massAlong(tangent);

                // Push out the overlap beyond the slop, or bounce back a fast approach, whichever is more.
                const approach = out.relativeVelocity().dot(normal);
                const push = correction / dt_s * @max(depth_m - slop_m, 0);
                const bounce = if (approach < -bounce_threshold_mps) -restitution * approach else 0;
                out.target_mps = @max(push, bounce);
                return out;
            }

            /// The impulse along `direction` that changes the bodies' relative speed by 1 m/s.
            fn massAlong(self: Constraint, direction: Vec) T {
                var inverse: T = 0;
                if (self.a) |s| inverse += s.inv_mass + S.angularDot(S.cross(self.offset_a, direction), s.turnBy(S.cross(self.offset_a, direction)));
                if (self.b) |s| inverse += s.inv_mass + S.angularDot(S.cross(self.offset_b, direction), s.turnBy(S.cross(self.offset_b, direction)));
                return if (inverse > 0) 1 / inverse else 0;
            }

            /// How fast `b`'s contact point moves relative to `a`'s.
            fn relativeVelocity(self: Constraint) Vec {
                const va = if (self.moving_a) |s| s.velocityAt(self.offset_a) else Vec.zero;
                const vb = if (self.moving_b) |s| s.velocityAt(self.offset_b) else Vec.zero;
                return vb.sub(va);
            }

            fn apply(self: *Constraint, impulse: Vec) void {
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
    };
}

test "a ball falling onto still ground stops instead of sinking" {
    const S = @import("space.zig").Space(.three, f32);
    const Sv = Solver(S);
    var ball = body.Bodies(S).State.init(.{ .kind = .dynamic, .pose = S.Pose.fromTranslation(S.Vec.init(0, 0.5, 0)), .mass_kg = 1 });
    ball.linear = S.Vec.init(0, -0.5, 0);
    var contacts = [_]Sv.Constraint{Sv.Constraint.init(&ball, null, S.Vec.zero, S.Vec.init(0, -1, 0), 0, 0.5, 0, 1.0 / 60.0)};
    Sv.solve(&contacts, 4);
    try std.testing.expectApproxEqAbs(@as(f32, 0), ball.linear.y, 1e-5);
}
