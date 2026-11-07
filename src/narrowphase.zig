//! Narrowphase: the points where two placed shapes overlap, with the normal and depth that
//! push them apart. Convex pairs go through GJK and EPA; resting contacts get several points,
//! found from support points a little either side of the normal, so boxes and cylinders sit flat.
const std = @import("std");
const shapes_zig = @import("shapes.zig");
const gjk_zig = @import("gjk.zig");

/// The narrowphase for space `S` (a `space.Space`).
pub fn Narrowphase(comptime S: type) type {
    const T = S.Real;
    const Vec = S.Vec;
    const Pose = S.Pose;
    const Shapes = shapes_zig.Shapes(S);
    const Shape = Shapes.Shape;
    const G = gjk_zig.Gjk(S);
    // Points closer than this to an existing one, or to the other shape, count as on it.
    const near_m: T = 1e-3;
    // How far support directions tip off the normal to find a resting face's corners.
    const tilt: T = 0.25;

    return struct {
        pub const Point = struct {
            /// World space, midway through the overlap.
            point: Vec,
            /// Unit length, from the first shape towards the second.
            normal: Vec,
            depth_m: T,
        };

        pub const Manifold = struct {
            points: [8]Point = undefined,
            len: usize = 0,

            fn add(self: *Manifold, point: Point) void {
                if (self.len == self.points.len) return;
                for (self.points[0..self.len]) |p| if (p.point.distanceSquared(point.point) < near_m * near_m) return;
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

        /// A convex shape placed in the world.
        pub const Placed = struct {
            shape: Shape,
            pose: Pose,

            pub fn support(self: Placed, direction: Vec) Vec {
                return self.pose.transformPoint(self.shape.support(self.pose.inverseTransformVector(direction)));
            }
        };

        /// One point, or one triangle (segment in 2D), as a convex shape.
        const Points = struct {
            points: []const Vec,

            pub fn support(self: Points, direction: Vec) Vec {
                var best = self.points[0];
                for (self.points[1..]) |p| if (p.dot(direction) > best.dot(direction)) {
                    best = p;
                };
                return best;
            }
        };

        /// Where `a` placed at `pose_a` overlaps `b` placed at `pose_b`; empty when they don't.
        pub fn collide(a: Shape, pose_a: Pose, b: Shape, pose_b: Pose) Manifold {
            // Written for a convex first shape; swap the rest and flip their normals.
            if (!a.convex()) {
                if (!b.convex()) return .{};
                return collide(b, pose_b, a, pose_a).flipped();
            }
            var out: Manifold = .{};
            const placed_a: Placed = .{ .shape = a, .pose = pose_a };
            switch (b) {
                .half_space => |half_space| convexPlane(&out, placed_a, Shapes.placedPlane(half_space, pose_b)),
                .mesh => |mesh| {
                    const bounds = a.aabb(pose_a);
                    for (mesh.indices) |facet_indices| {
                        var facet: [S.n]Vec = undefined;
                        for (&facet, facet_indices) |*p, i| p.* = pose_b.transformPoint(mesh.vertices[i]);
                        if (!bounds.overlaps(S.Aabb.around(&facet))) continue;
                        convexConvex(&out, placed_a, Points{ .points = &facet }, a == .ball);
                    }
                },
                else => {
                    const placed_b: Placed = .{ .shape = b, .pose = pose_b };
                    if (a == .ball and b == .ball) {
                        ballBall(&out, a.ball, pose_a, b.ball, pose_b);
                    } else if (a == .ball) {
                        ballConvex(&out, a.ball, pose_a, placed_b);
                    } else if (b == .ball) {
                        var flipped: Manifold = .{};
                        ballConvex(&flipped, b.ball, pose_b, placed_a);
                        out = flipped.flipped();
                    } else {
                        convexConvex(&out, placed_a, placed_b, false);
                    }
                },
            }
            return out;
        }

        fn ballBall(out: *Manifold, a: Shapes.Ball, pose_a: Pose, b: Shapes.Ball, pose_b: Pose) void {
            const between = pose_b.translation.sub(pose_a.translation);
            const distance = between.length();
            const depth = a.radius + b.radius - distance;
            if (depth <= 0) return;
            const normal = if (distance > 1e-6) between.scale(1 / distance) else S.axis(1);
            out.add(.{ .point = pose_a.translation.add(normal.scale(a.radius - depth / 2)), .normal = normal, .depth_m = depth });
        }

        /// A ball against a convex shape: the gap from its centre, or EPA when the centre is inside.
        fn ballConvex(out: *Manifold, ball: Shapes.Ball, pose_a: Pose, b: anytype) void {
            const centre = [_]Vec{pose_a.translation};
            const gap = G.distance(Points{ .points = &centre }, b);
            if (gap.distance > 0) {
                const depth = ball.radius - gap.distance;
                if (depth <= 0) return;
                const normal = gap.on_b.sub(pose_a.translation).scale(1 / gap.distance);
                out.add(.{ .point = gap.on_b.sub(normal.scale(depth / 2)), .normal = normal, .depth_m = depth });
                return;
            }
            convexConvex(out, Placed{ .shape = .{ .ball = ball }, .pose = pose_a }, b, true);
        }

        fn convexPlane(out: *Manifold, a: Placed, plane: anytype) void {
            const into = plane.normal.negate();
            const dirs = directions(into, a.shape == .ball);
            for (dirs.items[0..dirs.len]) |direction| {
                const deepest = a.support(direction);
                const height = plane.normal.dot(deepest) - plane.offset;
                if (height >= 0) continue;
                out.add(.{ .point = deepest.sub(plane.normal.scale(height / 2)), .normal = into, .depth_m = -height });
            }
        }

        fn convexConvex(out: *Manifold, a: anytype, b: anytype, single_point: bool) void {
            const gap = G.distance(a, b);
            if (gap.distance > 0) return;
            const deep = G.penetration(a, b, gap.simplex) orelse return;
            const normal = deep.normal;
            const first = out.len;
            if (!single_point) {
                // A's points past B's nearest face, and B's past A's, that lie in the other shape.
                const face_b = b.support(normal.negate());
                const face_a = a.support(normal);
                const towards_b = directions(normal, false);
                for (towards_b.items[0..towards_b.len]) |direction| {
                    const p = a.support(direction);
                    const depth = p.sub(face_b).dot(normal);
                    if (depth > 0 and touches(b, p)) out.add(.{ .point = p.sub(normal.scale(depth / 2)), .normal = normal, .depth_m = depth });
                }
                const towards_a = directions(normal.negate(), false);
                for (towards_a.items[0..towards_a.len]) |direction| {
                    const q = b.support(direction);
                    const depth = face_a.sub(q).dot(normal);
                    if (depth > 0 and touches(a, q)) out.add(.{ .point = q.add(normal.scale(depth / 2)), .normal = normal, .depth_m = depth });
                }
            }
            if (out.len == first) out.add(.{ .point = deep.on_a.add(deep.on_b).scale(0.5), .normal = normal, .depth_m = deep.depth });
        }

        fn touches(shape: anytype, point: Vec) bool {
            const single = [_]Vec{point};
            return G.distance(Points{ .points = &single }, shape).distance < near_m;
        }

        /// `normal`, and unless `only_normal`, it tipped a little along and between the surface axes.
        const Directions = struct { items: [if (S.n == 2) 3 else 9]Vec, len: usize };

        fn directions(normal: Vec, only_normal: bool) Directions {
            var out: Directions = .{ .items = undefined, .len = 1 };
            out.items[0] = normal;
            if (only_normal) return out;
            const tangents = S.tangents(normal);
            if (S.n == 2) {
                for ([_]T{ -1, 1 }) |sign| {
                    out.items[out.len] = normal.add(tangents[0].scale(sign * tilt)).normalize();
                    out.len += 1;
                }
            } else {
                // Straight along each surface axis finds the ends of a line contact (a cylinder on
                // its side), diagonally the corners of a face.
                for (tangents) |tangent| for ([_]T{ -1, 1 }) |sign| {
                    out.items[out.len] = normal.add(tangent.scale(sign * tilt)).normalize();
                    out.len += 1;
                };
                for ([_]T{ -1, 1 }) |s1| for ([_]T{ -1, 1 }) |s2| {
                    out.items[out.len] = normal.add(tangents[0].scale(s1 * tilt)).add(tangents[S.n - 2].scale(s2 * tilt)).normalize();
                    out.len += 1;
                };
            }
            return out;
        }
    };
}

const space = @import("space.zig");

test "a box resting into a half-space touches it at its four lower corners" {
    const S = space.Space(.three, f32);
    const N = Narrowphase(S);
    const manifold = N.collide(.{ .cuboid = .{ .half_extents = S.Vec.splat(0.5) } }, S.Pose.fromTranslation(S.Vec.init(0, 0.49, 0)), .{ .half_space = .{ .normal = S.Vec.init(0, 1, 0) } }, S.Pose.identity());
    try std.testing.expectEqual(@as(usize, 4), manifold.len);
    // From the box towards the ground: down.
    try std.testing.expect(manifold.points[0].normal.approxEql(S.Vec.init(0, -1, 0), 1e-5));
}

test "a box resting on a bigger box touches at its own corners, not the big one's" {
    const S = space.Space(.three, f64);
    const N = Narrowphase(S);
    const small: S.Pose = .fromTranslation(S.Vec.init(0, 0.99, 0));
    const manifold = N.collide(.{ .cuboid = .{ .half_extents = S.Vec.splat(0.5) } }, small, .{ .cuboid = .{ .half_extents = S.Vec.init(5, 0.5, 5) } }, S.Pose.identity());
    try std.testing.expectEqual(@as(usize, 4), manifold.len);
    for (manifold.slice()) |p| try std.testing.expect(@abs(p.point.x) <= 0.51 and @abs(p.point.z) <= 0.51);
}

test "a cylinder on its side touches a half-space along its length" {
    const S = space.Space(.three, f32);
    const N = Narrowphase(S);
    const lying = S.Pose.init(S.Vec.init(0, 0.49, 0), S.Vec.init(0, 0, std.math.pi / 2.0));
    const manifold = N.collide(.{ .cylinder = .{ .half_height = 1, .radius = 0.5 } }, lying, .{ .half_space = .{ .normal = S.Vec.init(0, 1, 0) } }, S.Pose.identity());
    try std.testing.expect(manifold.len >= 2);
}

test "a circle overlapping a 2D mesh segment is pushed off it" {
    const S = space.Space(.two, f32);
    const N = Narrowphase(S);
    const vertices = [_]S.Vec{ .init(-5, 0), .init(5, 0) };
    const indices = [_][2]u32{.{ 0, 1 }};
    const manifold = N.collide(.{ .ball = .{ .radius = 0.5 } }, S.Pose.fromTranslation(S.Vec.init(0, 0.4)), .{ .mesh = .{ .vertices = &vertices, .indices = &indices } }, S.Pose.identity());
    try std.testing.expectEqual(@as(usize, 1), manifold.len);
    try std.testing.expectApproxEqAbs(@as(f32, 0.1), manifold.points[0].depth_m, 1e-4);
}
