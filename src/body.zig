//! Bodies: what moves. A body's shape comes from the colliders attached to it.
const math = @import("math.zig");

pub const BodyHandle = math.Handle(.body);

/// Static bodies never move; dynamic ones are pushed by gravity, contacts and impulses;
/// kinematic ones move at the velocity they are given and push dynamic ones aside.
pub const BodyKind = enum { static, dynamic, kinematic };

/// Bodies in space `S` (a `space.Space`).
pub fn Bodies(comptime S: type) type {
    const T = S.Real;
    const Vec = S.Vec;

    return struct {
        pub const Body = struct {
            kind: BodyKind,
            pose: S.Pose,
            /// Spread over its colliders' shapes for inertia. Only dynamic bodies have mass.
            mass_kg: T = 0,
            /// The share of its speed it loses each second, to air and the like.
            linear_damping: T = 0,
            angular_damping: T = 0,
            /// How strongly gravity pulls it: 0 floats, 1 falls as everything else does.
            gravity_scale: T = 1,
            linear_velocity: Vec = .zero,
            angular_velocity: S.Angular = S.angular_zero,
        };

        /// A body as the world steps it: where it is, how it moves, and how hard it is to push.
        pub const State = struct {
            kind: BodyKind,
            pose: S.Pose,
            inv_mass: T,
            /// Inverse principal moments; zero where it can't turn.
            inv_inertia: S.Inertia = S.angular_zero,
            linear: Vec,
            angular: S.Angular,
            linear_damping: T,
            angular_damping: T,
            gravity_scale: T,
            /// How long it has been nearly still, in seconds.
            still_s: T = 0,
            sleeping: bool = false,

            pub fn init(body: Body) State {
                const dynamic = body.kind == .dynamic and body.mass_kg > 0;
                return .{
                    .kind = body.kind,
                    .pose = body.pose,
                    .inv_mass = if (dynamic) 1 / body.mass_kg else 0,
                    .linear = body.linear_velocity,
                    .angular = body.angular_velocity,
                    .linear_damping = body.linear_damping,
                    .angular_damping = body.angular_damping,
                    .gravity_scale = body.gravity_scale,
                };
            }

            /// Whether contacts and impulses move it: dynamic and awake.
            pub fn pushable(self: State) bool {
                return self.kind == .dynamic and !self.sleeping;
            }

            /// Whether it moves this step: awake and not static.
            pub fn moving(self: State) bool {
                return self.kind != .static and !self.sleeping;
            }

            pub fn setInertia(self: *State, inertia: S.Inertia) void {
                self.inv_inertia = if (self.inv_mass > 0) S.invert(inertia) else S.angular_zero;
            }

            /// The angular velocity change a world-space angular impulse makes.
            pub fn turnBy(self: State, angular_impulse: S.Angular) S.Angular {
                return S.turnBy(self.pose.rotation, self.inv_inertia, angular_impulse);
            }

            /// The velocity of the point `offset` (world space) from its centre.
            pub fn velocityAt(self: State, offset: Vec) Vec {
                return self.linear.add(S.turnVelocity(self.angular, offset));
            }

            /// Applies `impulse` (N·s, world space) at `offset` from its centre.
            pub fn applyImpulse(self: *State, impulse: Vec, offset: Vec) void {
                self.linear = self.linear.add(impulse.scale(self.inv_mass));
                self.angular = S.angularAdd(self.angular, self.turnBy(S.cross(offset, impulse)));
            }

            pub fn wake(self: *State) void {
                self.sleeping = false;
                self.still_s = 0;
            }
        };
    };
}
