//! Narrowphase: the points where two placed shapes overlap, with the normal and depth that
//! push them apart.
const std = @import("std");
const math = @import("math.zig");
const shapes = @import("shapes.zig");

const Vec3 = math.emath.Vec3;
const Pose = math.Pose;
const Shape = shapes.Shape;

pub const Point = struct {
    /// World space, midway through the overlap.
    point: Vec3,
    /// Unit length, from the first shape towards the second.
    normal: Vec3,
    depth_m: f32,
};

pub const Manifold = struct {
    points: [8]Point = undefined,
    len: usize = 0,

    fn add(self: *Manifold, point: Point) void {
        if (self.len == self.points.len) return;
        self.points[self.len] = point;
        self.len += 1;
    }

    fn flipped(self: Manifold) Manifold {
        var out = self;
        for (out.points[0..out.len]) |*p| p.normal = p.normal.negate();
        return out;
    }

    pub fn slice(self: *const Manifold) []const Point {
        return self.points[0..self.len];
    }
};

/// Where `a` placed at `pose_a` overlaps `b` placed at `pose_b`; empty when they don't.
pub fn collide(a: Shape, pose_a: Pose, b: Shape, pose_b: Pose) Manifold {
    // Each pair is written once, first shape lower in `Shape`'s order; the rest swap and flip.
    if (@backingInt(std.meta.activeTag(a)) > @backingInt(std.meta.activeTag(b))) return collide(b, pose_b, a, pose_a).flipped();
    var out: Manifold = .{};
    switch (a) {
        .sphere => |sa| switch (b) {
            .sphere => |sb| sphereSphere(&out, sa, pose_a, sb, pose_b),
            .box => |bb| sphereBox(&out, sa, pose_a, bb, pose_b),
            .plane => |pb| spherePlane(&out, sa, pose_a, pb.placed(pose_b)),
        },
        .box => |ba| switch (b) {
            .sphere => unreachable,
            .box => |bb| boxBox(&out, ba, pose_a, bb, pose_b),
            .plane => |pb| boxPlane(&out, ba, pose_a, pb.placed(pose_b)),
        },
        // Planes only belong on static scenery, which never needs to collide with itself.
        .plane => {},
    }
    return out;
}

fn sphereSphere(out: *Manifold, a: shapes.Sphere, pose_a: Pose, b: shapes.Sphere, pose_b: Pose) void {
    const between = pose_b.translation.sub(pose_a.translation);
    const distance = between.length();
    const depth = a.radius + b.radius - distance;
    if (depth <= 0) return;
    const normal = if (distance > 1e-6) between.scale(1 / distance) else Vec3.up;
    out.add(.{ .point = pose_a.translation.add(normal.scale(a.radius - depth / 2)), .normal = normal, .depth_m = depth });
}

fn spherePlane(out: *Manifold, a: shapes.Sphere, pose_a: Pose, plane: shapes.Plane) void {
    const height = plane.normal.dot(pose_a.translation) - plane.offset;
    const depth = a.radius - height;
    if (depth <= 0) return;
    out.add(.{ .point = pose_a.translation.sub(plane.normal.scale(a.radius - depth / 2)), .normal = plane.normal.negate(), .depth_m = depth });
}

fn sphereBox(out: *Manifold, a: shapes.Sphere, pose_a: Pose, b: shapes.Box, pose_b: Pose) void {
    const centre = pose_b.inverseTransformPoint(pose_a.translation);
    const h = b.half_extents;
    const closest = centre.clamp(h.negate(), h);
    const outside = centre.sub(closest);
    const distance = outside.length();
    if (distance > 1e-6) {
        // Centre outside the box: pushed out from the nearest point on its surface.
        const depth = a.radius - distance;
        if (depth <= 0) return;
        const outward = pose_b.transformVector(outside.scale(1 / distance));
        out.add(.{ .point = pose_b.transformPoint(closest), .normal = outward.negate(), .depth_m = depth });
        return;
    }
    // Centre inside the box: pushed out through the nearest face.
    const face = nearestFace(centre, h);
    out.add(.{ .point = pose_a.translation, .normal = pose_b.transformVector(face.outward).negate(), .depth_m = a.radius + face.depth_m });
}

fn boxPlane(out: *Manifold, a: shapes.Box, pose_a: Pose, plane: shapes.Plane) void {
    for (corners(a, pose_a)) |corner| {
        const height = plane.normal.dot(corner) - plane.offset;
        if (height >= 0) continue;
        out.add(.{ .point = corner.sub(plane.normal.scale(height / 2)), .normal = plane.normal.negate(), .depth_m = -height });
    }
}

// ponytail: corners inside the other box only; edge-on-edge crossings are missed, so boxes can
// sink edge-first into each other. Swap for SAT with face clipping when stacking matters.
fn boxBox(out: *Manifold, a: shapes.Box, pose_a: Pose, b: shapes.Box, pose_b: Pose) void {
    for (corners(a, pose_a)) |corner| {
        const local = pose_b.inverseTransformPoint(corner);
        if (!inside(local, b.half_extents)) continue;
        const face = nearestFace(local, b.half_extents);
        out.add(.{ .point = corner, .normal = pose_b.transformVector(face.outward).negate(), .depth_m = face.depth_m });
    }
    for (corners(b, pose_b)) |corner| {
        const local = pose_a.inverseTransformPoint(corner);
        if (!inside(local, a.half_extents)) continue;
        const face = nearestFace(local, a.half_extents);
        out.add(.{ .point = corner, .normal = pose_a.transformVector(face.outward), .depth_m = face.depth_m });
    }
}

fn corners(box: shapes.Box, pose: Pose) [8]Vec3 {
    var out: [8]Vec3 = undefined;
    for (&out, 0..) |*corner, i| {
        const sign = Vec3.init(if (i & 1 == 0) -1 else 1, if (i & 2 == 0) -1 else 1, if (i & 4 == 0) -1 else 1);
        corner.* = pose.transformPoint(box.half_extents.mul(sign));
    }
    return out;
}

fn inside(local: Vec3, half_extents: Vec3) bool {
    return @abs(local.x) < half_extents.x and @abs(local.y) < half_extents.y and @abs(local.z) < half_extents.z;
}

/// The face of a box (half extents `h`) nearest a point inside it, in the box's space.
fn nearestFace(local: Vec3, h: Vec3) struct { outward: Vec3, depth_m: f32 } {
    const p = local.toArray();
    const extents = h.toArray();
    var axis: usize = 0;
    var depth = std.math.inf(f32);
    for (0..3) |i| {
        const to_face = extents[i] - @abs(p[i]);
        if (to_face < depth) {
            depth = to_face;
            axis = i;
        }
    }
    var outward = [3]f32{ 0, 0, 0 };
    outward[axis] = if (p[axis] < 0) -1 else 1;
    return .{ .outward = Vec3.fromArray(outward), .depth_m = depth };
}

test "a sphere sunk into a plane touches it once, pushed up" {
    const sphere: Shape = .{ .sphere = .{ .radius = 0.5 } };
    const ground: Shape = .{ .plane = .{ .normal = Vec3.up, .offset = 0 } };
    const manifold = collide(sphere, Pose.fromTranslation(Vec3.init(0, 0.4, 0)), ground, Pose.identity());
    try std.testing.expectEqual(@as(usize, 1), manifold.len);
    try std.testing.expectApproxEqAbs(@as(f32, 0.1), manifold.points[0].depth_m, 1e-5);
    // From the sphere towards the ground: down.
    try std.testing.expect(manifold.points[0].normal.approxEql(Vec3.down, 1e-5));
    // Swapped, the normal flips.
    try std.testing.expect(collide(ground, Pose.identity(), sphere, Pose.fromTranslation(Vec3.init(0, 0.4, 0))).points[0].normal.approxEql(Vec3.up, 1e-5));
}

test "a box resting into a plane touches it at its four lower corners" {
    const box: Shape = .{ .box = .{ .half_extents = Vec3.splat(0.5) } };
    const ground: Shape = .{ .plane = .{ .normal = Vec3.up, .offset = 0 } };
    const manifold = collide(box, Pose.fromTranslation(Vec3.init(0, 0.49, 0)), ground, Pose.identity());
    try std.testing.expectEqual(@as(usize, 4), manifold.len);
}
