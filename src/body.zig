//! Bodies: what moves. A body's shape comes from the colliders attached to it.
const math = @import("math.zig");
const Vec3 = math.emath.Vec3;

pub const BodyHandle = math.Handle(.body);

pub const Body = struct {
    kind: BodyKind,
    pose: math.Pose,
    /// Spread over its colliders' shapes for inertia. Ignored for static bodies, which never move.
    mass_kg: f32 = 0,
};

pub const BodyKind = enum { static, dynamic };

/// A body as the world steps it: where it is, how it moves, and how hard it is to push.
pub const State = struct {
    kind: BodyKind,
    pose: math.Pose,
    inv_mass: f32,
    /// Inverse principal moments in the body's own axes; zero on an axis it can't turn about.
    inv_inertia: Vec3 = .zero,
    linear: Vec3 = .zero,
    angular: Vec3 = .zero,
    /// How long it has been nearly still, in seconds.
    still_s: f32 = 0,
    sleeping: bool = false,

    pub fn init(body: Body) State {
        const dynamic = body.kind == .dynamic and body.mass_kg > 0;
        return .{ .kind = body.kind, .pose = body.pose, .inv_mass = if (dynamic) 1 / body.mass_kg else 0 };
    }

    /// Whether impulses move it: dynamic and awake.
    pub fn moves(self: State) bool {
        return self.kind == .dynamic and !self.sleeping;
    }

    /// Sets its principal moments of inertia, in kg·m² about its own axes.
    pub fn setInertia(self: *State, inertia: Vec3) void {
        const arr = inertia.toArray();
        var inv: [3]f32 = undefined;
        for (&inv, arr) |*slot, moment| slot.* = if (self.inv_mass > 0 and moment > 0) 1 / moment else 0;
        self.inv_inertia = Vec3.fromArray(inv);
    }

    /// The angular velocity change a world-space angular impulse makes.
    pub fn turnBy(self: State, angular_impulse: Vec3) Vec3 {
        const local = self.pose.rotation.inverseRotateVector(angular_impulse);
        return self.pose.rotation.rotateVector(local.mul(self.inv_inertia));
    }

    /// The velocity of the point `offset` (world space) from its centre.
    pub fn velocityAt(self: State, offset: Vec3) Vec3 {
        return self.linear.add(self.angular.cross(offset));
    }

    /// Applies `impulse` (N·s, world space) at `offset` from its centre.
    pub fn applyImpulse(self: *State, impulse: Vec3, offset: Vec3) void {
        self.linear = self.linear.add(impulse.scale(self.inv_mass));
        self.angular = self.angular.add(self.turnBy(offset.cross(impulse)));
    }

    pub fn wake(self: *State) void {
        self.sleeping = false;
        self.still_s = 0;
    }
};
