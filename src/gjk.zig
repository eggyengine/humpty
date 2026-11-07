//! GJK and EPA: how far apart two convex shapes are, or how deep they overlap, from nothing but
//! their support points. One algorithm for every convex pair, in 2D and 3D.
const std = @import("std");

/// The algorithms for space `S` (a `space.Space`).
pub fn Gjk(comptime S: type) type {
    const T = S.Real;
    const Vec = S.Vec;
    const n = S.n;
    const max_iterations = 64;
    const tolerance: T = if (T == f32) 1e-5 else 1e-9;

    return struct {
        /// A point of the Minkowski difference A − B, with the points of A and B it came from.
        pub const Vertex = struct { w: Vec, a: Vec, b: Vec };

        pub const Simplex = struct {
            points: [n + 1]Vertex = undefined,
            len: usize = 0,
        };

        pub const Distance = struct {
            /// Zero when they overlap.
            distance: T,
            /// The nearest points on A and on B; meaningless when they overlap.
            on_a: Vec,
            on_b: Vec,
            simplex: Simplex,
        };

        pub const Penetration = struct {
            /// Unit length, from A towards B: moving B along it by `depth` separates them.
            normal: Vec,
            depth: T,
            /// The deepest points of A in B and of B in A.
            on_a: Vec,
            on_b: Vec,
        };

        fn vertex(a: anytype, b: anytype, direction: Vec) Vertex {
            const pa = a.support(direction);
            const pb = b.support(direction.negate());
            return .{ .w = pa.sub(pb), .a = pa, .b = pb };
        }

        /// How far apart convex `a` and `b` are, with their nearest points. Each is anything with
        /// `support(direction: Vec) Vec`, its farthest point that way in world space.
        pub fn distance(a: anytype, b: anytype) Distance {
            var simplex: Simplex = .{};
            simplex.points[0] = vertex(a, b, S.axis(0));
            simplex.len = 1;
            var nearest: Nearest = .{ .point = simplex.points[0].w, .weights = [_]T{1} ++ @as([n]T, @splat(0)) };
            for (0..max_iterations) |_| {
                const v = nearest.point;
                const v_len_sq = v.lengthSquared();
                if (v_len_sq < tolerance * tolerance) return overlap(simplex);
                const w = vertex(a, b, v.negate());
                // No support point gets closer to the origin than the nearest we have: done.
                if (v_len_sq - v.dot(w.w) <= tolerance * @max(v_len_sq, 1)) break;
                simplex.points[simplex.len] = w;
                simplex.len += 1;
                const next = nearestOnSimplex(&simplex) orelse break;
                if (simplex.len == n + 1) return overlap(simplex);
                if (next.point.lengthSquared() >= v_len_sq) break;
                nearest = next;
            }
            var on_a: Vec = .zero;
            var on_b: Vec = .zero;
            for (simplex.points[0..simplex.len], nearest.weights[0..simplex.len]) |p, weight| {
                on_a = on_a.add(p.a.scale(weight));
                on_b = on_b.add(p.b.scale(weight));
            }
            return .{ .distance = nearest.point.length(), .on_a = on_a, .on_b = on_b, .simplex = simplex };
        }

        fn overlap(simplex: Simplex) Distance {
            return .{ .distance = 0, .on_a = .zero, .on_b = .zero, .simplex = simplex };
        }

        const Nearest = struct { point: Vec, weights: [n + 1]T };

        /// Shrinks `simplex` to the part nearest the origin and returns that nearest point.
        /// Tries every face of it: tiny, and the same code in 2D and 3D.
        fn nearestOnSimplex(simplex: *Simplex) ?Nearest {
            const count = simplex.len;
            var best: ?Nearest = null;
            var best_mask: usize = 0;
            var mask: usize = 1;
            while (mask < (@as(usize, 1) << @intCast(count))) : (mask += 1) {
                var members: [n + 1]Vec = undefined;
                var size: usize = 0;
                for (0..count) |i| if (mask & (@as(usize, 1) << @intCast(i)) != 0) {
                    members[size] = simplex.points[i].w;
                    size += 1;
                };
                const found = affineNearest(members[0..size]) orelse continue;
                if (best == null or found.point.lengthSquared() < best.?.point.lengthSquared()) {
                    // Weights back in the order of the whole simplex.
                    var weights: [n + 1]T = @splat(0);
                    var j: usize = 0;
                    for (0..count) |i| if (mask & (@as(usize, 1) << @intCast(i)) != 0) {
                        weights[i] = found.weights[j];
                        j += 1;
                    };
                    best = .{ .point = found.point, .weights = weights };
                    best_mask = mask;
                }
            }
            const nearest = best orelse return null;
            // Keep only the vertices it leans on.
            var kept: Simplex = .{};
            var weights: [n + 1]T = @splat(0);
            for (0..count) |i| if (best_mask & (@as(usize, 1) << @intCast(i)) != 0) {
                kept.points[kept.len] = simplex.points[i];
                weights[kept.len] = nearest.weights[i];
                kept.len += 1;
            };
            simplex.* = kept;
            return .{ .point = nearest.point, .weights = weights };
        }

        /// The point nearest the origin on the flat through `points`, if it lies strictly inside
        /// them (every weight positive).
        fn affineNearest(points: []const Vec) ?Nearest {
            const k = points.len;
            var weights: [n + 1]T = @splat(0);
            if (k == 1) {
                weights[0] = 1;
                return .{ .point = points[0], .weights = weights };
            }
            // Solve G·μ = r for the edges from the first point, in at most three unknowns.
            const m = k - 1;
            var g: [n][n + 1]T = undefined;
            for (0..m) |row| {
                const e_row = points[row + 1].sub(points[0]);
                for (0..m) |col| g[row][col] = e_row.dot(points[col + 1].sub(points[0]));
                g[row][m] = -e_row.dot(points[0]);
            }
            var mu: [n]T = undefined;
            if (!solve(m, &g, &mu)) return null;
            var sum: T = 0;
            var point = points[0];
            for (0..m) |j| {
                if (mu[j] <= 0) return null;
                sum += mu[j];
                weights[j + 1] = mu[j];
                point = point.add(points[j + 1].sub(points[0]).scale(mu[j]));
            }
            weights[0] = 1 - sum;
            if (weights[0] <= 0) return null;
            return .{ .point = point, .weights = weights };
        }

        /// Gaussian elimination with partial pivoting on an augmented `m`×(`m`+1) system.
        fn solve(m: usize, g: *[n][n + 1]T, out: *[n]T) bool {
            var scale: T = 0;
            for (0..m) |i| scale = @max(scale, @abs(g[i][i]));
            for (0..m) |col| {
                var pivot = col;
                for (col + 1..m) |row| if (@abs(g[row][col]) > @abs(g[pivot][col])) {
                    pivot = row;
                };
                if (@abs(g[pivot][col]) <= tolerance * tolerance * @max(scale, 1)) return false;
                std.mem.swap([n + 1]T, &g[col], &g[pivot]);
                for (col + 1..m) |row| {
                    const factor = g[row][col] / g[col][col];
                    for (col..m + 1) |c| g[row][c] -= factor * g[col][c];
                }
            }
            var row = m;
            while (row > 0) {
                row -= 1;
                var value = g[row][m];
                for (row + 1..m) |c| value -= g[row][c] * out[c];
                out[row] = value / g[row][row];
            }
            return true;
        }

        /// How deep overlapping convex `a` and `b` go, from the simplex `distance` ended on.
        /// Null when they only touch.
        pub fn penetration(a: anytype, b: anytype, start: Simplex) ?Penetration {
            var simplex = start;
            if (!fill(a, b, &simplex)) return null;
            return if (S.dim == .two) epa2(a, b, simplex) else epa3(a, b, simplex);
        }

        /// Grows `simplex` to n + 1 points around the origin, from support points along the axes.
        fn fill(a: anytype, b: anytype, simplex: *Simplex) bool {
            var axis: usize = 0;
            while (simplex.len < n + 1 and axis < 2 * n) : (axis += 1) {
                const direction = if (axis < n) S.axis(axis) else S.axis(axis - n).negate();
                const candidate = vertex(a, b, direction);
                if (!independent(simplex.*, candidate.w)) continue;
                simplex.points[simplex.len] = candidate;
                simplex.len += 1;
            }
            if (simplex.len < n + 1) return false;
            // The origin must be inside for the polytope to grow around it.
            var points: [n + 1]Vec = undefined;
            for (&points, simplex.points) |*p, v| p.* = v.w;
            return contains(&points);
        }

        fn independent(simplex: Simplex, w: Vec) bool {
            var points: [n + 1]Vec = undefined;
            for (simplex.points[0..simplex.len], 0..) |p, i| points[i] = p.w;
            points[simplex.len] = w;
            const k = simplex.len + 1;
            if (k == 1) return true;
            var g: [n][n + 1]T = undefined;
            for (0..k - 1) |row| {
                const e_row = points[row + 1].sub(points[0]);
                for (0..k - 1) |col| g[row][col] = e_row.dot(points[col + 1].sub(points[0]));
                g[row][k - 1] = 0;
            }
            var out: [n]T = undefined;
            return solve(k - 1, &g, &out);
        }

        fn contains(points: *const [n + 1]Vec) bool {
            var g: [n][n + 1]T = undefined;
            for (0..n) |row| {
                const e_row = points[row + 1].sub(points[0]);
                for (0..n) |col| g[row][col] = e_row.dot(points[col + 1].sub(points[0]));
                g[row][n] = -e_row.dot(points[0]);
            }
            var mu: [n]T = undefined;
            if (!solve(n, &g, &mu)) return false;
            var sum: T = 0;
            for (mu[0..n]) |m| {
                if (m < -tolerance) return false;
                sum += m;
            }
            return sum <= 1 + tolerance;
        }

        fn epa2(a: anytype, b: anytype, simplex: Simplex) ?Penetration {
            var polygon: [96]Vertex = undefined;
            var len: usize = 3;
            @memcpy(polygon[0..3], simplex.points[0..3]);
            // Counter-clockwise, so each edge's outward normal is its right-hand perpendicular.
            if (S.cross(polygon[1].w.sub(polygon[0].w), polygon[2].w.sub(polygon[0].w)) < 0) std.mem.swap(Vertex, &polygon[1], &polygon[2]);
            var best_edge: usize = 0;
            var best_normal: Vec = .zero;
            var best_distance: T = 0;
            for (0..max_iterations) |_| {
                best_distance = std.math.inf(T);
                for (0..len) |i| {
                    const p = polygon[i].w;
                    const q = polygon[(i + 1) % len].w;
                    const edge = q.sub(p);
                    const normal = S.Vec.init(edge.y, -edge.x).normalize();
                    const along = normal.dot(p);
                    if (along < best_distance) {
                        best_distance = along;
                        best_normal = normal;
                        best_edge = i;
                    }
                }
                const w = vertex(a, b, best_normal);
                if (w.w.dot(best_normal) - best_distance < tolerance * 10 or len == polygon.len) break;
                std.mem.copyBackwards(Vertex, polygon[best_edge + 2 .. len + 1], polygon[best_edge + 1 .. len]);
                polygon[best_edge + 1] = w;
                len += 1;
            }
            const p = polygon[best_edge];
            const q = polygon[(best_edge + 1) % len];
            const along_edge = q.w.sub(p.w);
            const t = if (along_edge.lengthSquared() > 0) std.math.clamp(-p.w.dot(along_edge) / along_edge.lengthSquared(), 0, 1) else 0;
            return .{
                .normal = best_normal,
                .depth = best_distance,
                .on_a = p.a.lerp(q.a, t),
                .on_b = p.b.lerp(q.b, t),
            };
        }

        const Face = struct { v: [3]u16, normal: Vec, distance: T };

        fn epa3(a: anytype, b: anytype, simplex: Simplex) ?Penetration {
            var vertices: [128]Vertex = undefined;
            var vertex_count: usize = 4;
            @memcpy(vertices[0..4], simplex.points[0..4]);
            var faces: [256]Face = undefined;
            var face_count: usize = 0;
            const start = [4][3]u16{ .{ 0, 1, 2 }, .{ 0, 3, 1 }, .{ 0, 2, 3 }, .{ 1, 3, 2 } };
            for (start) |f| {
                faces[face_count] = makeFace(&vertices, f) orelse return null;
                face_count += 1;
            }
            var best: usize = 0;
            for (0..max_iterations) |_| {
                best = 0;
                for (faces[1..face_count], 1..) |face, i| if (face.distance < faces[best].distance) {
                    best = i;
                };
                const w = vertex(a, b, faces[best].normal);
                if (w.w.dot(faces[best].normal) - faces[best].distance < tolerance * 10 or vertex_count == vertices.len) break;
                vertices[vertex_count] = w;
                const added: u16 = @intCast(vertex_count);
                vertex_count += 1;

                // Remove every face the new point sees; the edges they leave once are the horizon.
                var horizon: [128][2]u16 = undefined;
                var horizon_len: usize = 0;
                var i: usize = 0;
                while (i < face_count) {
                    const face = faces[i];
                    if (face.normal.dot(w.w.sub(vertices[face.v[0]].w)) <= 0) {
                        i += 1;
                        continue;
                    }
                    for (0..3) |e| {
                        const edge = [2]u16{ face.v[e], face.v[(e + 1) % 3] };
                        var shared = false;
                        for (horizon[0..horizon_len], 0..) |h, hi| if (h[0] == edge[1] and h[1] == edge[0]) {
                            horizon[hi] = horizon[horizon_len - 1];
                            horizon_len -= 1;
                            shared = true;
                            break;
                        };
                        if (!shared and horizon_len < horizon.len) {
                            horizon[horizon_len] = edge;
                            horizon_len += 1;
                        }
                    }
                    faces[i] = faces[face_count - 1];
                    face_count -= 1;
                }
                for (horizon[0..horizon_len]) |edge| {
                    if (face_count == faces.len) break;
                    faces[face_count] = makeFace(&vertices, .{ edge[0], edge[1], added }) orelse continue;
                    face_count += 1;
                }
                if (face_count == 0) return null;
            }
            const face = faces[best];
            const p0 = vertices[face.v[0]];
            const p1 = vertices[face.v[1]];
            const p2 = vertices[face.v[2]];
            // Where the origin's projection sits on the face, to place the points on A and B.
            const projected = face.normal.scale(face.distance);
            const weights = barycentric(projected, p0.w, p1.w, p2.w);
            return .{
                .normal = face.normal,
                .depth = face.distance,
                .on_a = p0.a.scale(weights[0]).add(p1.a.scale(weights[1])).add(p2.a.scale(weights[2])),
                .on_b = p0.b.scale(weights[0]).add(p1.b.scale(weights[1])).add(p2.b.scale(weights[2])),
            };
        }

        fn makeFace(vertices: []const Vertex, v: [3]u16) ?Face {
            const a = vertices[v[0]].w;
            const normal_raw = vertices[v[1]].w.sub(a).cross(vertices[v[2]].w.sub(a));
            if (normal_raw.lengthSquared() < tolerance * tolerance) return null;
            var normal = normal_raw.normalize();
            var order = v;
            // Outward: away from the origin, which is inside.
            if (normal.dot(a) < 0) {
                normal = normal.negate();
                std.mem.swap(u16, &order[1], &order[2]);
            }
            return .{ .v = order, .normal = normal, .distance = normal.dot(a) };
        }

        fn barycentric(p: Vec, a: Vec, b: Vec, c: Vec) [3]T {
            const v0 = b.sub(a);
            const v1 = c.sub(a);
            const v2 = p.sub(a);
            const d00 = v0.dot(v0);
            const d01 = v0.dot(v1);
            const d11 = v1.dot(v1);
            const d20 = v2.dot(v0);
            const d21 = v2.dot(v1);
            const denom = d00 * d11 - d01 * d01;
            if (@abs(denom) < tolerance * tolerance) return .{ 1, 0, 0 };
            const v = (d11 * d20 - d01 * d21) / denom;
            const w = (d00 * d21 - d01 * d20) / denom;
            return .{ 1 - v - w, v, w };
        }
    };
}

/// A convex shape placed in world space, for tests.
fn Placed(comptime S: type) type {
    return struct {
        shape: @import("shapes.zig").Shapes(S).Shape,
        pose: S.Pose,
        pub fn support(self: @This(), direction: S.Vec) S.Vec {
            return self.pose.transformPoint(self.shape.support(self.pose.inverseTransformVector(direction)));
        }
    };
}

test "GJK measures the gap between two boxes, and EPA how far they overlap" {
    const S = @import("space.zig").Space(.three, f64);
    const G = Gjk(S);
    const P = Placed(S);
    const box: @import("shapes.zig").Shapes(S).Shape = .{ .cuboid = .{ .half_extents = S.Vec.splat(0.5) } };
    const a: P = .{ .shape = box, .pose = S.Pose.identity() };
    const apart: P = .{ .shape = box, .pose = S.Pose.fromTranslation(S.Vec.init(2, 0, 0)) };
    try std.testing.expectApproxEqAbs(@as(f64, 1), G.distance(a, apart).distance, 1e-6);

    const sunk: P = .{ .shape = box, .pose = S.Pose.fromTranslation(S.Vec.init(0.8, 0.1, 0)) };
    const result = G.distance(a, sunk);
    try std.testing.expectEqual(@as(f64, 0), result.distance);
    const deep = G.penetration(a, sunk, result.simplex).?;
    try std.testing.expectApproxEqAbs(@as(f64, 0.2), deep.depth, 1e-6);
    try std.testing.expect(deep.normal.approxEql(S.Vec.init(1, 0, 0), 1e-6));
}

test "GJK and EPA work the same way in 2D" {
    const S = @import("space.zig").Space(.two, f32);
    const G = Gjk(S);
    const P = Placed(S);
    const a: P = .{ .shape = .{ .ball = .{ .radius = 1 } }, .pose = S.Pose.identity() };
    const b: P = .{ .shape = .{ .cuboid = .{ .half_extents = S.Vec.splat(0.5) } }, .pose = S.Pose.fromTranslation(S.Vec.init(0, 1.3)) };
    const result = G.distance(a, b);
    try std.testing.expectEqual(@as(f32, 0), result.distance);
    const deep = G.penetration(a, b, result.simplex).?;
    try std.testing.expectApproxEqAbs(@as(f32, 0.2), deep.depth, 1e-3);
    try std.testing.expect(deep.normal.approxEql(S.Vec.init(0, 1), 1e-3));
}
