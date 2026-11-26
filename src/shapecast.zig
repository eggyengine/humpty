//! Shape casts: how far a convex shape can move in a straight line before it touches another,
//! by conservative advancement over GJK distances: step to the plane the nearest points lie
//! on, measure again, until touching. Meshes and heightfields are cast at facet by facet, only
//! those the sweep passes.
const std = @import("std");
const shapes_zig = @import("shapes.zig");
const gjk_zig = @import("gjk.zig");

/// Casts in space `S` (a `space.Space`).
pub fn Casts(comptime S: type) type {
    const T = S.Real;
    const Vec = S.Vec;
    const Pose = S.Pose;
    const Shapes = shapes_zig.Shapes(S);
    const Shape = Shapes.Shape;
    const G = gjk_zig.Gjk(S);
    // Closer than this counts as touching.
    const touching_m: T = if (T == f32) 1e-4 else 1e-6;
    const max_steps = 48;

    return struct {
        pub const Hit = struct {
            /// How far the shape moved before touching.
            distance_m: T,
            /// Where it touched, on the shape it met, in world space.
            point: Vec,
            /// The surface it met, facing back towards the cast.
            normal: Vec,
        };

        /// A convex shape placed in the world, moved along by `offset`.
        const Moved = struct {
            shape: Shape,
            pose: Pose,
            offset: Vec = .zero,

            pub fn support(self: Moved, direction: Vec) Vec {
                return self.pose.transformPoint(self.shape.support(self.pose.inverseTransformVector(direction))).add(self.offset);
            }
        };

        /// One triangle (segment in 2D) as a convex shape.
        const Facet = struct {
            points: [S.n]Vec,

            pub fn support(self: Facet, direction: Vec) Vec {
                var best = self.points[0];
                for (self.points[1..]) |p| if (p.dot(direction) > best.dot(direction)) {
                    best = p;
                };
                return best;
            }
        };

        /// Where convex `shape`, starting at `pose` and moving along unit `direction` for up to
        /// `max_m`, first touches `target` at `target_pose`. Already touching is a hit at 0.
        pub fn cast(shape: Shape, pose: Pose, direction: Vec, max_m: T, target: Shape, target_pose: Pose) ?Hit {
            std.debug.assert(shape.convex());
            const moving: Moved = .{ .shape = shape, .pose = pose };
            switch (target) {
                .half_space => |half_space| {
                    const plane = Shapes.placedPlane(half_space, target_pose);
                    const deepest = moving.support(plane.normal.negate());
                    const height = plane.normal.dot(deepest) - plane.offset;
                    if (height <= 0) return .{ .distance_m = 0, .point = deepest, .normal = plane.normal };
                    const closing = -direction.dot(plane.normal);
                    if (closing <= 0 or height / closing > max_m) return null;
                    const distance = height / closing;
                    return .{ .distance_m = distance, .point = deepest.add(direction.scale(distance)), .normal = plane.normal };
                },
                .mesh => |mesh| {
                    const swept = sweptBounds(shape, pose, direction, max_m);
                    var nearest: ?Hit = null;
                    for (mesh.indices) |facet_indices| {
                        var facet: Facet = undefined;
                        for (&facet.points, facet_indices) |*p, i| p.* = target_pose.transformPoint(mesh.vertices[i]);
                        nearest = closer(nearest, againstFacet(moving, direction, if (nearest) |hit| hit.distance_m else max_m, swept, facet));
                    }
                    return nearest;
                },
                .heightfield => |field| {
                    if (S.n != 3) return null;
                    // Its cells under the sweep, found in its own space.
                    const local = target_pose.inverse().mul(pose);
                    const local_direction = target_pose.inverseTransformVector(direction);
                    const cells = field.cellsUnder(sweptBounds(shape, local, local_direction, max_m)) orelse return null;
                    const swept = sweptBounds(shape, pose, direction, max_m);
                    var nearest: ?Hit = null;
                    for (cells.first[1]..cells.last[1] + 1) |row| for (cells.first[0]..cells.last[0] + 1) |column| {
                        for (field.cellTriangles(@intCast(column), @intCast(row))) |triangle| {
                            var facet: Facet = undefined;
                            for (&facet.points, triangle) |*p, corner| p.* = target_pose.transformPoint(corner);
                            nearest = closer(nearest, againstFacet(moving, direction, if (nearest) |hit| hit.distance_m else max_m, swept, facet));
                        }
                    };
                    return nearest;
                },
                else => return advance(moving, direction, max_m, Moved{ .shape = target, .pose = target_pose }),
            }
        }

        fn againstFacet(moving: Moved, direction: Vec, max_m: T, swept: S.Aabb, facet: Facet) ?Hit {
            if (!swept.overlaps(S.Aabb.around(&facet.points))) return null;
            // A ball (a wheel, most often) needs no advancing: where it meets is worked out.
            if (S.n == 3 and moving.shape == .ball) switch (ballFacet(moving.pose.translation.add(moving.offset), moving.shape.ball.radius, direction, max_m, facet.points)) {
                .met => |hit| return hit,
                .degenerate => {},
            };
            return advance(moving, direction, max_m, facet);
        }

        /// Where a ball at `centre` cast along unit `direction` first meets a triangle: its face,
        /// an edge (a capsule round it) or a corner (a ball round it), found exactly, and
        /// reported as `advance` would. A triangle without area is `.degenerate`, for `advance`.
        fn ballFacet(centre: Vec, radius: T, direction: Vec, max_m: T, points: [3]Vec) union(enum) { degenerate, met: ?Hit } {
            const a, const b, const c = points;
            const across = b.sub(a).cross(c.sub(a));
            const area = across.length();
            if (!(area > 1e-12)) return .degenerate;
            const normal = across.scale(1 / area);

            // Touching already: met where it starts.
            const start_nearest = nearestOnTriangle(centre, a, b, c);
            const start_gap = start_nearest.sub(centre).length() - radius;
            if (start_gap < touching_m) return .{ .met = .{ .distance_m = 0, .point = if (start_gap > 0) start_nearest else centre, .normal = direction.negate() } };

            var first: ?T = null;
            // The face, met from the side the ball is on.
            const side = normal.dot(centre.sub(a));
            const facing = if (side < 0) normal.negate() else normal;
            const approach = -facing.dot(direction);
            if (@abs(side) >= radius and approach > 1e-9) {
                const t = (@abs(side) - radius) / approach;
                if (inTriangle(centre.add(direction.scale(t)).sub(facing.scale(radius)), a, b, c, normal)) first = t;
            }
            for ([_][2]Vec{ .{ a, b }, .{ b, c }, .{ c, a } }) |edge| first = earlier(first, edgeTime(centre, direction, edge[0], edge[1], radius));
            for (points) |corner| first = earlier(first, cornerTime(centre, direction, corner, radius));

            const t = first orelse return .{ .met = null };
            if (t > max_m) return .{ .met = null };
            const at = centre.add(direction.scale(t));
            const point = nearestOnTriangle(at, a, b, c);
            const away = at.sub(point);
            const apart = away.length();
            return .{ .met = .{ .distance_m = t, .point = point, .normal = if (apart > 1e-9) away.scale(1 / apart) else facing } };
        }

        fn earlier(kept: ?T, next: ?T) ?T {
            const found = next orelse return kept;
            return if (kept) |so_far| @min(so_far, found) else found;
        }

        /// Whether `p`, on the plane of triangle `a`, `b`, `c` (unit `normal`), is inside it.
        fn inTriangle(p: Vec, a: Vec, b: Vec, c: Vec, normal: Vec) bool {
            const slack: T = -1e-6;
            return b.sub(a).cross(p.sub(a)).dot(normal) >= slack and
                c.sub(b).cross(p.sub(b)).dot(normal) >= slack and
                a.sub(c).cross(p.sub(c)).dot(normal) >= slack;
        }

        /// How far along unit `direction` a ball at `centre` meets the capsule of `radius` round
        /// the edge from `from` to `to`, along its side; its ends are the corners'.
        fn edgeTime(centre: Vec, direction: Vec, from: Vec, to: Vec, radius: T) ?T {
            const edge = to.sub(from);
            const m = centre.sub(from);
            const ee = edge.dot(edge);
            const ed = edge.dot(direction);
            const em = edge.dot(m);
            const qa = ee - ed * ed;
            // Along the edge: it meets a corner first.
            if (!(qa > 1e-9 * ee)) return null;
            const qb = ee * m.dot(direction) - em * ed;
            const qc = ee * (m.dot(m) - radius * radius) - em * em;
            const discriminant = qb * qb - qa * qc;
            if (discriminant < 0) return null;
            const t = (-qb - @sqrt(discriminant)) / qa;
            if (t < 0) return null;
            const along = (em + t * ed) / ee;
            return if (along >= 0 and along <= 1) t else null;
        }

        /// How far along unit `direction` a ball at `centre` meets one of `radius` at `corner`.
        fn cornerTime(centre: Vec, direction: Vec, corner: Vec, radius: T) ?T {
            const m = centre.sub(corner);
            const closing = m.dot(direction);
            if (closing > 0) return null;
            const discriminant = closing * closing - (m.dot(m) - radius * radius);
            if (discriminant < 0) return null;
            const t = -closing - @sqrt(discriminant);
            return if (t >= 0) t else null;
        }

        /// The point of triangle `a`, `b`, `c` nearest `p` (Ericson, Real-Time Collision Detection).
        fn nearestOnTriangle(p: Vec, a: Vec, b: Vec, c: Vec) Vec {
            const ab = b.sub(a);
            const ac = c.sub(a);
            const ap = p.sub(a);
            const d1 = ab.dot(ap);
            const d2 = ac.dot(ap);
            if (d1 <= 0 and d2 <= 0) return a;
            const bp = p.sub(b);
            const d3 = ab.dot(bp);
            const d4 = ac.dot(bp);
            if (d3 >= 0 and d4 <= d3) return b;
            const vc = d1 * d4 - d3 * d2;
            if (vc <= 0 and d1 >= 0 and d3 <= 0) return a.add(ab.scale(d1 / (d1 - d3)));
            const cp = p.sub(c);
            const d5 = ab.dot(cp);
            const d6 = ac.dot(cp);
            if (d6 >= 0 and d5 <= d6) return c;
            const vb = d5 * d2 - d1 * d6;
            if (vb <= 0 and d2 >= 0 and d6 <= 0) return a.add(ac.scale(d2 / (d2 - d6)));
            const va = d3 * d6 - d5 * d4;
            if (va <= 0 and d4 - d3 >= 0 and d5 - d6 >= 0) return b.add(c.sub(b).scale((d4 - d3) / ((d4 - d3) + (d5 - d6))));
            const scale = 1 / (va + vb + vc);
            return a.add(ab.scale(vb * scale)).add(ac.scale(vc * scale));
        }

        fn closer(best: ?Hit, next: ?Hit) ?Hit {
            const found = next orelse return best;
            if (best) |kept| if (kept.distance_m <= found.distance_m) return kept;
            return found;
        }

        /// Everywhere `shape` at `pose` passes moving `max_m` along `direction`.
        pub fn sweptBounds(shape: Shape, pose: Pose, direction: Vec, max_m: T) S.Aabb {
            const start = shape.aabb(pose);
            var end_pose = pose;
            end_pose.translation = pose.translation.add(direction.scale(max_m));
            return start.merge(shape.aabb(end_pose));
        }

        /// Conservative advancement of `moving` along `direction` towards convex `target`.
        fn advance(moving: Moved, direction: Vec, max_m: T, target: anytype) ?Hit {
            var travelled: T = 0;
            var normal = direction.negate();
            var point = moving.pose.translation;
            for (0..max_steps) |_| {
                var at = moving;
                at.offset = direction.scale(travelled);
                const gap = G.distance(at, target);
                // Overlapping reports no points: keep the last ones, or none at the start.
                if (gap.distance > 0) point = gap.on_b;
                if (gap.distance < touching_m) return .{ .distance_m = travelled, .point = point, .normal = normal };
                normal = gap.on_a.sub(gap.on_b).scale(1 / gap.distance);
                const closing = -direction.dot(normal);
                if (closing <= 1e-6) return null;
                travelled += gap.distance / closing;
                if (travelled > max_m) return null;
            }
            return null;
        }
    };
}

test "a ball cast down meets a box's top, a plane, and a heightfield where a ray from its bottom would" {
    const S = @import("space.zig").Space(.three, f32);
    const C = Casts(S);
    const Shapes = shapes_zig.Shapes(S);
    const ball: Shapes.Shape = .{ .ball = .{ .radius = 0.5 } };
    const from = S.Pose.fromTranslation(S.Vec.init(0.2, 5, 0.1));
    const down = S.Vec.init(0, -1, 0);

    const box = C.cast(ball, from, down, 10, .{ .cuboid = .{ .half_extents = S.Vec.splat(1) } }, S.Pose.identity()).?;
    try std.testing.expectApproxEqAbs(@as(f32, 3.5), box.distance_m, 1e-3);
    try std.testing.expect(box.normal.approxEql(S.Vec.init(0, 1, 0), 1e-2));

    const plane = C.cast(ball, from, down, 10, .{ .half_space = .{ .normal = S.Vec.init(0, 1, 0) } }, S.Pose.identity()).?;
    try std.testing.expectApproxEqAbs(@as(f32, 4.5), plane.distance_m, 1e-5);

    const heights = [_]f32{ 1, 1, 1, 1, 1, 1, 1, 1, 1 };
    const field: Shapes.Heightfield = .init(&heights, 3, 1, 1);
    const ground = C.cast(ball, from, down, 10, .{ .heightfield = field }, S.Pose.fromTranslation(S.Vec.init(-1, 0, -1))).?;
    try std.testing.expectApproxEqAbs(@as(f32, 3.5), ground.distance_m, 1e-3);
    try std.testing.expectApproxEqAbs(@as(f32, 1), ground.point.y, 1e-3);
    // Too short a cast meets nothing.
    try std.testing.expect(C.cast(ball, from, down, 3, .{ .heightfield = field }, S.Pose.fromTranslation(S.Vec.init(-1, 0, -1))) == null);
}
