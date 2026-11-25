//! Colliders: the shapes bodies touch the world with. One attached to a body moves with it and
//! adds to its inertia; one with no body is static scenery.
const math = @import("math.zig");
const shapes = @import("shapes.zig");

pub const ColliderHandle = math.Handle(.collider);

/// Colliders in space `S` (a `space.Space`).
pub fn Colliders(comptime S: type) type {
    return struct {
        pub const Collider = struct {
            shape: shapes.Shapes(S).Shape,
            /// Where it sits on its body, in the body's space; in world space when it has none.
            local_pose: S.Pose = S.Pose.identity(),
            material: Material = .{},
            /// A sensor reports what it overlaps but pushes nothing.
            sensor: bool = false,
            /// Its own mass, added to its body's at its place; null takes a share of the body's
            /// own mass by volume.
            mass_kg: ?S.Real = null,
            /// Off, it touches nothing, adds no mass and queries pass through it.
            enabled: bool = true,
        };

        pub const Material = struct {
            /// How much of an approach speed bounces back: 0 thuds, 1 bounces fully.
            restitution: S.Real = 0,
            friction: S.Real = 0.5,
        };
    };
}
