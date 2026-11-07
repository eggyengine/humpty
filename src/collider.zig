//! Colliders: the shapes bodies touch the world with. One attached to a body moves with it and
//! adds to its inertia; one with no body is static scenery.
const math = @import("math.zig");
const shape = @import("shapes.zig");

pub const ColliderHandle = math.Handle(.collider);

pub const Collider = struct {
    shape: shape.Shape,
    /// Where it sits on its body, in the body's space; in world space when it has none.
    local_pose: math.Pose = math.Pose.identity(),
    material: Material = .{},
};

pub const Material = struct {
    restitution: f32 = 0,
    friction: f32 = 0.5,
};
