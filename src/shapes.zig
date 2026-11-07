//! Collision shapes: a closed set of primitives behind one tagged union.
const std = @import("std");
const math = @import("math.zig");

const Vec3 = math.emath.Vec3;
const Pose = math.Pose;

/// An axis-aligned bounding box in world space, used by the broadphase.
pub const Aabb = struct {
    min: Vec3,
    max: Vec3,

    pub fn overlaps(self: Aabb, other: Aabb) bool {
        return self.min.x <= other.max.x and self.max.x >= other.min.x and
            self.min.y <= other.max.y and self.max.y >= other.min.y and
            self.min.z <= other.max.z and self.max.z >= other.min.z;
    }
};

pub const Sphere = struct {
    radius: f32,

    pub fn aabb(self: Sphere, pose: Pose) Aabb {
        const extent = Vec3.splat(self.radius);
        return .{ .min = pose.translation.sub(extent), .max = pose.translation.add(extent) };
    }

    pub fn volume(self: Sphere) f32 {
        return 4.0 / 3.0 * std.math.pi * self.radius * self.radius * self.radius;
    }

    /// Principal moments of inertia of a solid sphere, in kg·m².
    pub fn inertia(self: Sphere, mass_kg: f32) Vec3 {
        return Vec3.splat(0.4 * mass_kg * self.radius * self.radius);
    }
};

pub const Box = struct {
    half_extents: Vec3,

    pub fn aabb(self: Box, pose: Pose) Aabb {
        // Each rotated half-axis adds its absolute reach on every world axis.
        const h = self.half_extents;
        const extent = pose.transformVector(Vec3.init(h.x, 0, 0)).abs()
            .add(pose.transformVector(Vec3.init(0, h.y, 0)).abs())
            .add(pose.transformVector(Vec3.init(0, 0, h.z)).abs());
        return .{ .min = pose.translation.sub(extent), .max = pose.translation.add(extent) };
    }

    pub fn volume(self: Box) f32 {
        return 8 * self.half_extents.x * self.half_extents.y * self.half_extents.z;
    }

    /// Principal moments of inertia of a solid box, in kg·m².
    pub fn inertia(self: Box, mass_kg: f32) Vec3 {
        const sq = self.half_extents.mul(self.half_extents);
        return Vec3.init(sq.y + sq.z, sq.x + sq.z, sq.x + sq.y).scale(mass_kg / 3.0);
    }
};

/// An infinite plane: points `p` with `dot(normal, p) == offset` in the body's
/// space. It has no finite mass, so it only belongs on static bodies.
pub const Plane = struct {
    normal: Vec3,
    offset: f32,

    pub fn aabb(self: Plane, pose: Pose) Aabb {
        _ = self;
        _ = pose;
        // ponytail: unbounded on every axis, fine for brute-force pairs; clip
        // it to the world normal's axes when a tree broadphase arrives.
        const inf = std.math.inf(f32);
        return .{ .min = Vec3.splat(-inf), .max = Vec3.splat(inf) };
    }

    pub fn volume(self: Plane) f32 {
        _ = self;
        return 0;
    }

    /// The plane placed at `pose`: its world normal and offset along it.
    pub fn placed(self: Plane, pose: Pose) Plane {
        const normal = pose.transformVector(self.normal);
        return .{ .normal = normal, .offset = self.offset + normal.dot(pose.translation) };
    }

    pub fn inertia(self: Plane, mass_kg: f32) Vec3 {
        _ = self;
        _ = mass_kg;
        return Vec3.zero;
    }
};

pub const Shape = union(enum) {
    sphere: Sphere,
    box: Box,
    plane: Plane,

    /// World-space bounds of the shape placed at `pose`.
    pub fn aabb(self: Shape, pose: Pose) Aabb {
        return switch (self) {
            inline else => |shape| shape.aabb(pose),
        };
    }

    /// Its volume in m³; zero for a plane.
    pub fn volume(self: Shape) f32 {
        return switch (self) {
            inline else => |shape| shape.volume(),
        };
    }

    /// Principal moments of inertia in the body's local axes, in kg·m².
    pub fn inertia(self: Shape, mass_kg: f32) Vec3 {
        return switch (self) {
            inline else => |shape| shape.inertia(mass_kg),
        };
    }
};
