//! Rays: where a line from a point first meets a shape. Balls, cuboids, half-spaces and mesh
//! facets are solved exactly; other convex shapes are stepped towards by their GJK distance.
const std = @import("std");
const c = @import("collider.zig");
const shapes_zig = @import("shapes.zig");
const gjk_zig = @import("gjk.zig");

/// Rays in space `S` (a `space.Space`).
pub fn Rays(comptime S: type) type {
    const T = S.Real;
    const Vec = S.Vec;
    const Pose = S.Pose;
    const Shapes = shapes_zig.Shapes(S);
    const G = gjk_zig.Gjk(S);

    return struct {
        pub const Ray = struct {
            origin: Vec,
            /// Unit length.
            direction: Vec,
            max_distance_m: T,
        };

        pub const RayHit = struct {
            collider: c.ColliderHandle,
            point: Vec,
            normal: Vec,
            distance_m: T,
        };

        pub const Hit = struct { distance_m: T, normal: Vec };

        /// Where `ray` first meets `shape` placed at `pose`. A ray starting inside meets it at 0.
        pub fn cast(ray: Ray, shape: Shapes.Shape, pose: Pose) ?Hit {
            const hit = switch (shape) {
                .ball => |ball| castBall(ray, ball.radius, pose.translation),
                .cuboid => |cuboid| castCuboid(ray, cuboid, pose),
                .half_space => |half_space| castPlane(ray, Shapes.placedPlane(half_space, pose)),
                .mesh => |mesh| castMesh(ray, mesh, pose),
                else => castConvex(ray, shape, pose),
            } orelse return null;
            if (hit.distance_m > ray.max_distance_m) return null;
            return hit;
        }

        fn castBall(ray: Ray, radius: T, centre: Vec) ?Hit {
            const from_centre = ray.origin.sub(centre);
            const along = from_centre.dot(ray.direction);
            const beyond = from_centre.dot(from_centre) - radius * radius;
            if (beyond <= 0) return .{ .distance_m = 0, .normal = ray.direction.negate() };
            const discriminant = along * along - beyond;
            if (along > 0 or discriminant < 0) return null;
            const distance = -along - @sqrt(discriminant);
            const point = ray.origin.add(ray.direction.scale(distance));
            return .{ .distance_m = distance, .normal = point.sub(centre).scale(1 / radius) };
        }

        fn castCuboid(ray: Ray, cuboid: Shapes.Cuboid, pose: Pose) ?Hit {
            // Slabs in its own space: the ray is inside once it has entered every one.
            const origin = pose.inverseTransformPoint(ray.origin).toArray();
            const direction = pose.inverseTransformVector(ray.direction).toArray();
            const h = cuboid.half_extents.toArray();
            var enter: T = -std.math.inf(T);
            var leave: T = std.math.inf(T);
            var enter_axis: usize = 0;
            for (0..S.n) |i| {
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
            const outward = S.axis(enter_axis).scale(if (direction[enter_axis] > 0) -1 else 1);
            return .{ .distance_m = enter, .normal = pose.transformVector(outward) };
        }

        fn castPlane(ray: Ray, plane: anytype) ?Hit {
            // A half-space is solid behind its surface.
            const height = plane.normal.dot(ray.origin) - plane.offset;
            if (height <= 0) return .{ .distance_m = 0, .normal = plane.normal };
            const closing = plane.normal.dot(ray.direction);
            if (closing >= 0) return null;
            return .{ .distance_m = -height / closing, .normal = plane.normal };
        }

        // ponytail: every facet is tried; add a bounding-volume tree when meshes get big.
        fn castMesh(ray: Ray, mesh: Shapes.Mesh, pose: Pose) ?Hit {
            var nearest: ?Hit = null;
            for (mesh.indices) |facet| {
                var points: [S.n]Vec = undefined;
                for (&points, facet) |*p, i| p.* = pose.transformPoint(mesh.vertices[i]);
                const hit = castFacet(ray, points) orelse continue;
                if (nearest == null or hit.distance_m < nearest.?.distance_m) nearest = hit;
            }
            return nearest;
        }

        fn castFacet(ray: Ray, p: [S.n]Vec) ?Hit {
            if (S.dim == .two) {
                // Where the ray crosses the segment's line, if that is between its ends.
                const edge = p[1].sub(p[0]);
                const denom = S.cross(ray.direction, edge);
                if (@abs(denom) < 1e-12) return null;
                const to_start = p[0].sub(ray.origin);
                const t = S.cross(to_start, edge) / denom;
                const u = S.cross(to_start, ray.direction) / denom;
                if (t < 0 or u < 0 or u > 1) return null;
                var normal = edge.perpendicular().normalize();
                if (normal.dot(ray.direction) > 0) normal = normal.negate();
                return .{ .distance_m = t, .normal = normal };
            } else {
                // Möller–Trumbore.
                const e1 = p[1].sub(p[0]);
                const e2 = p[2].sub(p[0]);
                const h = ray.direction.cross(e2);
                const det = e1.dot(h);
                if (@abs(det) < 1e-12) return null;
                const s = ray.origin.sub(p[0]);
                const u = s.dot(h) / det;
                if (u < 0 or u > 1) return null;
                const q = s.cross(e1);
                const v = ray.direction.dot(q) / det;
                if (v < 0 or u + v > 1) return null;
                const t = e2.dot(q) / det;
                if (t < 0) return null;
                var normal = e1.cross(e2).normalize();
                if (normal.dot(ray.direction) > 0) normal = normal.negate();
                return .{ .distance_m = t, .normal = normal };
            }
        }

        const Placed = struct {
            shape: Shapes.Shape,
            pose: Pose,
            pub fn support(self: Placed, direction: Vec) Vec {
                return self.pose.transformPoint(self.shape.support(self.pose.inverseTransformVector(direction)));
            }
        };

        const Point = struct {
            point: Vec,
            pub fn support(self: Point, _: Vec) Vec {
                return self.point;
            }
        };

        /// Steps along the ray to the plane the shape's nearest point lies behind, until it touches.
        fn castConvex(ray: Ray, shape: Shapes.Shape, pose: Pose) ?Hit {
            const placed: Placed = .{ .shape = shape, .pose = pose };
            const close_m: T = 1e-4;
            var t: T = 0;
            var normal = ray.direction.negate();
            for (0..64) |_| {
                const at = ray.origin.add(ray.direction.scale(t));
                const gap = G.distance(Point{ .point = at }, placed);
                if (gap.distance < close_m) return .{ .distance_m = t, .normal = normal };
                normal = gap.on_a.sub(gap.on_b).scale(1 / gap.distance);
                const closing = -ray.direction.dot(normal);
                if (closing <= 0) return null;
                t += gap.distance / closing;
                if (t > ray.max_distance_m) return null;
            }
            return null;
        }
    };
}

test "a ray down onto a box hits its top face" {
    const S = @import("space.zig").Space(.three, f32);
    const R = Rays(S);
    const ray: R.Ray = .{ .origin = S.Vec.init(0, 5, 0), .direction = S.Vec.init(0, -1, 0), .max_distance_m = 10 };
    const hit = R.cast(ray, .{ .cuboid = .{ .half_extents = S.Vec.splat(0.5) } }, S.Pose.identity()).?;
    try std.testing.expectApproxEqAbs(@as(f32, 4.5), hit.distance_m, 1e-5);
    try std.testing.expect(hit.normal.approxEql(S.Vec.init(0, 1, 0), 1e-5));
}

test "a ray meets a cylinder's side and a cone's tip" {
    const S = @import("space.zig").Space(.three, f64);
    const R = Rays(S);
    const sideways: R.Ray = .{ .origin = S.Vec.init(-5, 0, 0), .direction = S.Vec.init(1, 0, 0), .max_distance_m = 10 };
    const side = R.cast(sideways, .{ .cylinder = .{ .half_height = 1, .radius = 0.5 } }, S.Pose.identity()).?;
    try std.testing.expectApproxEqAbs(@as(f64, 4.5), side.distance_m, 1e-3);
    const down: R.Ray = .{ .origin = S.Vec.init(0, 5, 0), .direction = S.Vec.init(0, -1, 0), .max_distance_m = 10 };
    try std.testing.expectApproxEqAbs(@as(f64, 4), R.cast(down, .{ .cone = .{ .half_height = 1, .radius = 0.5 } }, S.Pose.identity()).?.distance_m, 1e-3);
}

test "a 2D ray crosses a mesh segment" {
    const S = @import("space.zig").Space(.two, f32);
    const R = Rays(S);
    const vertices = [_]S.Vec{ .init(-1, 0), .init(1, 0) };
    const indices = [_][2]u32{.{ 0, 1 }};
    const ray: R.Ray = .{ .origin = S.Vec.init(0, 3), .direction = S.Vec.init(0, -1), .max_distance_m = 10 };
    try std.testing.expectApproxEqAbs(@as(f32, 3), R.cast(ray, .{ .mesh = .{ .vertices = &vertices, .indices = &indices } }, S.Pose.identity()).?.distance_m, 1e-5);
}
