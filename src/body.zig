//! Bodies: what moves. A body's shape comes from the colliders attached to it.
const math = @import("math.zig");

pub const BodyHandle = math.Handle(.body);

/// Static bodies never move; dynamic ones are pushed by gravity, contacts and impulses;
/// kinematic ones move at the velocity they are given and push dynamic ones aside.
pub const BodyKind = enum { static, dynamic, kinematic };

/// Whether contacts and impulses can turn a body; a locked one only slides, as a character does.
pub const Rotation = enum { free, locked };

/// Bodies in space `S` (a `space.Space`).
pub fn Bodies(comptime S: type) type {
    const T = S.Real;
    const Vec = S.Vec;

    return struct {
        pub const Body = struct {
            kind: BodyKind,
            pose: S.Pose,
            /// Spread over its colliders without a mass of their own, by volume; colliders with
            /// one add theirs. Only dynamic bodies have mass.
            mass_kg: T = 0,
            /// Where its mass balances, in its own space; null takes it from its colliders.
            center_of_mass_m: ?Vec = null,
            /// Off, it neither moves nor touches anything, and queries pass through it.
            enabled: bool = true,
            /// The share of its speed it loses each second, to air and the like.
            linear_damping: T = 0,
            angular_damping: T = 0,
            /// How strongly gravity pulls it: 0 floats, 1 falls as everything else does.
            gravity_scale: T = 1,
            rotation: Rotation = .free,
            linear_velocity: Vec = .zero,
            angular_velocity: S.Angular = S.angular_zero,
        };

        /// A body as the world steps it: where it is, how it moves, and how hard it is to push.
        pub const State = struct {
            kind: BodyKind,
            /// Where its origin is; its centre of mass is `center()`.
            pose: S.Pose,
            /// Its own mass and centre, before its colliders' (`Body.mass_kg`, `center_of_mass_m`).
            own_mass_kg: T,
            center_override: ?Vec,
            /// Where its mass balances, in its own space, from its colliders.
            local_center: Vec = .zero,
            inv_mass: T,
            /// Inverse principal moments; zero where it can't turn.
            inv_inertia: S.Inertia = S.angular_zero,
            linear: Vec,
            angular: S.Angular,
            linear_damping: T,
            angular_damping: T,
            gravity_scale: T,
            rotation: Rotation = .free,
            /// How long it has been nearly still, in seconds.
            still_s: T = 0,
            sleeping: bool = false,
            enabled: bool = true,
            /// Forces and torques for the rest of this frame's steps (`Physics.applyForce`).
            force: Vec = .zero,
            torque: S.Angular = S.angular_zero,
            /// The push of forces from a frame too short for a step, for the next step (N·s).
            carried: Vec = .zero,
            carried_turn: S.Angular = S.angular_zero,

            pub fn init(body: Body) State {
                const dynamic = body.kind == .dynamic and body.mass_kg > 0;
                return .{
                    .kind = body.kind,
                    .pose = body.pose,
                    .own_mass_kg = body.mass_kg,
                    .center_override = body.center_of_mass_m,
                    .local_center = body.center_of_mass_m orelse .zero,
                    .enabled = body.enabled,
                    .inv_mass = if (dynamic) 1 / body.mass_kg else 0,
                    .linear = body.linear_velocity,
                    .angular = body.angular_velocity,
                    .linear_damping = body.linear_damping,
                    .angular_damping = body.angular_damping,
                    .gravity_scale = body.gravity_scale,
                    .rotation = body.rotation,
                };
            }

            /// Whether contacts and impulses move it: dynamic, awake and on.
            pub fn pushable(self: State) bool {
                return self.kind == .dynamic and !self.sleeping and self.enabled;
            }

            /// Whether it moves this step: awake, on and not static.
            pub fn moving(self: State) bool {
                return self.kind != .static and !self.sleeping and self.enabled;
            }

            /// Its centre of mass in world space, which it turns about.
            pub fn center(self: State) Vec {
                return self.pose.transformPoint(self.local_center);
            }

            /// Moves it by its velocities for `dt_s`, turning it about its centre of mass.
            pub fn integrate(self: *State, dt_s: T) void {
                const centre = self.center().add(self.linear.scale(dt_s));
                self.pose.rotation = S.integrate(self.pose.rotation, self.angular, dt_s);
                self.pose.translation = centre.sub(self.pose.rotation.rotateVector(self.local_center));
            }

            pub fn setInertia(self: *State, inertia: S.Inertia) void {
                self.inv_inertia = if (self.inv_mass > 0 and self.rotation == .free) S.invert(inertia) else S.angular_zero;
            }

            /// The angular velocity change a world-space angular impulse makes.
            pub fn turnBy(self: State, angular_impulse: S.Angular) S.Angular {
                return S.turnBy(self.pose.rotation, self.inv_inertia, angular_impulse);
            }

            /// The velocity of the point `offset` (world space) from its centre of mass.
            pub fn velocityAt(self: State, offset: Vec) Vec {
                return self.linear.add(S.turnVelocity(self.angular, offset));
            }

            /// Applies `impulse` (N·s, world space) at `offset` from its centre of mass.
            pub fn applyImpulse(self: *State, impulse: Vec, offset: Vec) void {
                self.linear = self.linear.add(impulse.scale(self.inv_mass));
                self.angular = S.angularAdd(self.angular, self.turnBy(S.cross(offset, impulse)));
            }

            /// Applies an angular impulse (N·m·s, world space).
            pub fn applyAngularImpulse(self: *State, angular_impulse: S.Angular) void {
                self.angular = S.angularAdd(self.angular, self.turnBy(angular_impulse));
            }

            pub fn wake(self: *State) void {
                self.sleeping = false;
                self.still_s = 0;
            }
        };
    };
}
