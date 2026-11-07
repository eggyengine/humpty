//! The components Eggy adds to every project for humpty's physics: plain data, free of Eggy,
//! listed in `components` so Eggy discovers them as it does a plugin's.
const emath = @import("eggenvector");

pub const components = .{ RigidBody, Collider };

/// Makes its entity a body the physics world moves (or, when static, collides with).
pub const RigidBody = struct {
    /// Marks it in the editor (nf-md-cube_send).
    pub const icon = "\u{f0a6e}";
    pub const category = "Physics";
    kind: Kind = .dynamic,
    mass_kg: f32 = 1,

    pub const Kind = enum { static, dynamic };
};

/// The shape a body collides with, centred on its entity.
pub const Collider = struct {
    pub const icon = "\u{f0832}";
    pub const category = "Physics";
    shape: Shape = .{ .box = .{} },
    restitution: f32 = 0.2,
    friction: f32 = 0.5,

    pub const Shape = union(enum) {
        sphere: struct { radius_m: f32 = 0.5 },
        box: struct { half_extents_m: emath.Vec3 = .splat(0.5) },
    };
};
