pub const emath = @import("eggenvector");
const c = @import("collider.zig");

pub const Pose = emath.Isometry3(f32);

/// A handle to a body or collider: its slot and the generation it was added in, so a handle to
/// a removed one never matches whatever reuses its slot. Each `of` makes a distinct type, so a
/// body handle can't be passed as a collider's. Packed so handles compare with `==`.
pub fn Handle(comptime of: enum { body, collider }) type {
    return packed struct(u64) {
        index: u32,
        generation: u32,

        pub const target = of;
    };
}

pub const ImpulsePoint = union(enum) {
    center_of_mass,
    at_world_point: emath.Vec3,
};

pub const Velocity = struct {
    linear: emath.Vec3, // m/s
    angular: emath.Vec3, // rad/s, axis * speed
};

pub const Contact = struct {
    a: c.ColliderHandle,
    b: c.ColliderHandle,
    point: emath.Vec3, // world space
    normal: emath.Vec3, // from a towards b
    depth_m: f32,

    pub fn involves(self: Contact, collider: c.ColliderHandle) bool {
        return self.a == collider or self.b == collider;
    }
};
