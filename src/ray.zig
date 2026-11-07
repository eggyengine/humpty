//! Rays: where a line from a point first meets a shape.
const std = @import("std");
const emath = @import("eggenvector");
const math = @import("math.zig");
const c = @import("collider.zig");
const shapes = @import("shapes.zig");

const Vec3 = emath.Vec3;

pub const Ray = struct {
    origin: emath.Vec3,
    direction: emath.Vec3, // unit length
    max_distance_m: f32,
};

pub const RayHit = struct {
    collider: c.ColliderHandle,
    point: emath.Vec3,
    normal: emath.Vec3,
    distance_m: f32,
};

/// Where `ray` first meets `shape` placed at `pose`: its distance and the surface normal there.
/// A ray starting inside a shape meets it at distance 0.
pub fn cast(ray: Ray, shape: shapes.Shape, pose: math.Pose) ?struct { distance_m: f32, normal: Vec3 } {
    const hit = switch (shape) {
        .sphere => |s| castSphere(ray, s, pose),
        .box => |s| castBox(ray, s, pose),
        .plane => |s| castPlane(ray, s.placed(pose)),
    } orelse return null;
    if (hit.distance_m > ray.max_distance_m) return null;
    return .{ .distance_m = hit.distance_m, .normal = hit.normal };
}

const Hit = struct { distance_m: f32, normal: Vec3 };

fn castSphere(ray: Ray, sphere: shapes.Sphere, pose: math.Pose) ?Hit {
    const from_centre = ray.origin.sub(pose.translation);
    const along = from_centre.dot(ray.direction);
    const beyond = from_centre.dot(from_centre) - sphere.radius * sphere.radius;
    if (beyond <= 0) return .{ .distance_m = 0, .normal = ray.direction.negate() };
    const discriminant = along * along - beyond;
    if (along > 0 or discriminant < 0) return null;
    const distance = -along - @sqrt(discriminant);
    const point = ray.origin.add(ray.direction.scale(distance));
    return .{ .distance_m = distance, .normal = point.sub(pose.translation).scale(1 / sphere.radius) };
}

fn castBox(ray: Ray, box: shapes.Box, pose: math.Pose) ?Hit {
    // Slabs in the box's own space: the ray is inside once it has entered all three.
    const origin = pose.inverseTransformPoint(ray.origin).toArray();
    const direction = pose.inverseTransformVector(ray.direction).toArray();
    const h = box.half_extents.toArray();
    var enter: f32 = -std.math.inf(f32);
    var leave: f32 = std.math.inf(f32);
    var enter_axis: usize = 0;
    for (0..3) |i| {
        if (@abs(direction[i]) < 1e-8) {
            if (@abs(origin[i]) > h[i]) return null;
            continue;
        }
        const t1 = (-h[i] - origin[i]) / direction[i];
        const t2 = (h[i] - origin[i]) / direction[i];
        if (@min(t1, t2) > enter) {
            enter = @min(t1, t2);
            enter_axis = i;
        }
        leave = @min(leave, @max(t1, t2));
    }
    if (leave < @max(enter, 0)) return null;
    if (enter < 0) return .{ .distance_m = 0, .normal = ray.direction.negate() };
    var normal = [3]f32{ 0, 0, 0 };
    normal[enter_axis] = if (direction[enter_axis] > 0) -1 else 1;
    return .{ .distance_m = enter, .normal = pose.transformVector(Vec3.fromArray(normal)) };
}

fn castPlane(ray: Ray, plane: shapes.Plane) ?Hit {
    // A plane is solid below its surface.
    const height = plane.normal.dot(ray.origin) - plane.offset;
    if (height <= 0) return .{ .distance_m = 0, .normal = plane.normal };
    const closing = plane.normal.dot(ray.direction);
    if (closing >= 0) return null;
    return .{ .distance_m = -height / closing, .normal = plane.normal };
}

test "a ray down onto a box hits its top face" {
    const ray: Ray = .{ .origin = Vec3.init(0, 5, 0), .direction = Vec3.down, .max_distance_m = 10 };
    const hit = cast(ray, .{ .box = .{ .half_extents = Vec3.splat(0.5) } }, math.Pose.identity()).?;
    try std.testing.expectApproxEqAbs(@as(f32, 4.5), hit.distance_m, 1e-5);
    try std.testing.expect(hit.normal.approxEql(Vec3.up, 1e-5));
}
