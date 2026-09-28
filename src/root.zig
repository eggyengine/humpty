const std = @import("std");
const emath = @import("eggenvector");

pub const Object = struct {
    position: emath.Vec3,
    velocity: emath.Vec3,
    force: emath.Vec3,
    mass: f32,
};

// pub const PhysicsWorld = struct { objects: std.ArrayList() };
