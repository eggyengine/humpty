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
            return advance(moving, direction, max_m, facet);
        }

        fn closer(best: ?Hit, next: ?Hit) ?Hit {
            const found = next orelse return best;
            if (best) |kept| if (kept.distance_m <= found.distance_m) return kept;
            return found;
        }

        /// Everywhere `shape` at `pose` passes moving `max_m` along `direction`.
        fn sweptBounds(shape: Shape, pose: Pose, direction: Vec, max_m: T) S.Aabb {
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
