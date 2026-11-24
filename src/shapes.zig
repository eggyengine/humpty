//! Collision shapes, after Rapier's and Avian's: a ball, cuboid, capsule, cylinder and cone, a
//! convex hull of points, a mesh of triangles (segments in 2D), a heightfield and a half-space,
//! behind one tagged union. Every shape but the mesh, heightfield and half-space is convex,
//! found by its support point.
const std = @import("std");

/// The shapes of space `S` (a `space.Space`).
pub fn Shapes(comptime S: type) type {
    const T = S.Real;
    const Vec = S.Vec;
    const Pose = S.Pose;
    const Aabb = S.Aabb;
    const two = S.dim == .two;

    return struct {
        /// A sphere, or a circle in 2D.
        pub const Ball = struct { radius: T };
        /// A box, or a rectangle in 2D.
        pub const Cuboid = struct { half_extents: Vec };
        /// A segment from -`half_height` to `half_height` along y, rounded by `radius`.
        pub const Capsule = struct { half_height: T, radius: T };
        /// 3D only: a disc of `radius` swept from -`half_height` to `half_height` along y.
        pub const Cylinder = struct { half_height: T, radius: T };
        /// 3D only: its tip at `half_height` up y, its base of `radius` at -`half_height`.
        pub const Cone = struct { half_height: T, radius: T };
        /// The smallest convex shape around `points`. The world copies them when it is added.
        pub const ConvexHull = struct { points: []const Vec };
        /// Triangles (segments in 2D), each three (two) indices into `vertices`. For static
        /// scenery: it has no inside, so it has no mass. The world copies them when it is added.
        pub const Mesh = struct { vertices: []const Vec, indices: []const [S.n]u32 };
        /// Everything on the far side of the plane (line in 2D) through the origin facing `normal`.
        pub const HalfSpace = struct { normal: Vec };
        /// 3D only: a grid of heights along y, `columns` along x by `rows` along z, `cell_x` and
        /// `cell_z` metres apart, its first corner at the origin. Each cell is two triangles,
        /// split from its first corner to the one across. For static scenery, like a mesh; the
        /// world copies the heights when it is added. Make one with `init`, which finds its lowest
        /// and highest points once.
        pub const Heightfield = struct {
            /// Row by row: the height at column `c`, row `r` is `heights[r * columns + c]`.
            heights: []const T,
            columns: u32,
            rows: u32,
            cell_x: T,
            cell_z: T,
            min_height: T,
            max_height: T,

            /// The grid of `heights`, `columns` wide; at least two columns and two rows.
            pub fn init(heights: []const T, columns: u32, cell_x: T, cell_z: T) Heightfield {
                var low = heights[0];
                var high = heights[0];
                for (heights[1..]) |h| {
                    low = @min(low, h);
                    high = @max(high, h);
                }
                return .{ .heights = heights, .columns = columns, .rows = @intCast(heights.len / columns), .cell_x = cell_x, .cell_z = cell_z, .min_height = low, .max_height = high };
            }

            /// The corner at `column`, `row`, in its own space.
            pub fn corner(self: Heightfield, column: u32, row: u32) Vec {
                if (two) unreachable;
                return Vec.init(@as(T, @floatFromInt(column)) * self.cell_x, self.heights[row * self.columns + column], @as(T, @floatFromInt(row)) * self.cell_z);
            }

            /// The two triangles of the cell from `column`, `row`, counter-clockwise from above.
            pub fn cellTriangles(self: Heightfield, column: u32, row: u32) [2][3]Vec {
                const here = self.corner(column, row);
                const right = self.corner(column + 1, row);
                const below = self.corner(column, row + 1);
                const across = self.corner(column + 1, row + 1);
                return .{ .{ here, across, right }, .{ here, below, across } };
            }

            /// The cells `first` to `last` (inclusive), by column and row.
            pub const Cells = struct { first: [2]u32, last: [2]u32 };

            /// The cells whose triangles may reach into `bounds`, in its own space; null when
            /// none can, beside the grid or above or below every height.
            pub fn cellsUnder(self: Heightfield, bounds: Aabb) ?Cells {
                if (two) unreachable;
                if (bounds.min.y > self.max_height or bounds.max.y < self.min_height) return null;
                const columns = cellRange(bounds.min.x, bounds.max.x, self.cell_x, self.columns - 1) orelse return null;
                const rows = cellRange(bounds.min.z, bounds.max.z, self.cell_z, self.rows - 1) orelse return null;
                return .{ .first = .{ columns[0], rows[0] }, .last = .{ columns[1], rows[1] } };
            }

            /// Its bounds in its own space.
            pub fn localBounds(self: Heightfield) Aabb {
                if (two) unreachable;
                return .{
                    .min = Vec.init(0, self.min_height, 0),
                    .max = Vec.init(@as(T, @floatFromInt(self.columns - 1)) * self.cell_x, self.max_height, @as(T, @floatFromInt(self.rows - 1)) * self.cell_z),
                };
            }

            /// The cells from `low` to `high` along an axis of `count` cells `cell` apart.
            fn cellRange(low: T, high: T, cell: T, count: u32) ?[2]u32 {
                const end = @as(T, @floatFromInt(count)) * cell;
                if (high < 0 or low > end) return null;
                const first: u32 = @intFromFloat(@max(@floor(low / cell), 0));
                const last: u32 = @intFromFloat(@min(@floor(high / cell), @as(T, @floatFromInt(count - 1))));
                return .{ @min(first, count - 1), last };
            }
        };
        /// One triangle (segment in 2D) of a mesh, in world space.
        pub const Facet = struct { points: [S.n]Vec };

        pub const Shape = union(enum) {
            ball: Ball,
            cuboid: Cuboid,
            capsule: Capsule,
            cylinder: Cylinder,
            cone: Cone,
            convex_hull: ConvexHull,
            mesh: Mesh,
            heightfield: Heightfield,
            half_space: HalfSpace,

            /// Whether this space has it: cylinders, cones and heightfields are 3D only.
            pub fn supported(self: Shape) bool {
                return switch (self) {
                    .cylinder, .cone => !two,
                    .heightfield => |field| !two and field.columns >= 2 and field.rows >= 2 and field.heights.len == @as(usize, field.columns) * field.rows,
                    .convex_hull => |hull| hull.points.len > 0,
                    .mesh => |mesh| mesh.indices.len > 0,
                    else => true,
                };
            }

            /// Whether `support` describes it: everything but a mesh, heightfield or half-space.
            pub fn convex(self: Shape) bool {
                return switch (self) {
                    .mesh, .heightfield, .half_space => false,
                    else => true,
                };
            }

            /// Its point farthest along `direction`, in its own space. Convex shapes only.
            pub fn support(self: Shape, direction: Vec) Vec {
                return switch (self) {
                    .ball => |ball| direction.normalize().scale(ball.radius),
                    .cuboid => |cuboid| cuboid.half_extents.mul(signs(direction)),
                    .capsule => |capsule| axial(direction, capsule.half_height).add(direction.normalize().scale(capsule.radius)),
                    .cylinder => |cylinder| if (two) unreachable else axial(direction, cylinder.half_height).add(radial(direction).scale(cylinder.radius)),
                    .cone => |cone| if (two) unreachable else coneSupport(cone, direction),
                    .convex_hull => |hull| farthest(hull.points, direction),
                    .mesh, .heightfield, .half_space => unreachable,
                };
            }

            /// Its bounds placed at `pose`.
            pub fn aabb(self: Shape, pose: Pose) Aabb {
                switch (self) {
                    .half_space => return Aabb.everywhere,
                    .mesh => |mesh| {
                        var out: Aabb = .{ .min = pose.transformPoint(mesh.vertices[0]), .max = pose.transformPoint(mesh.vertices[0]) };
                        for (mesh.vertices[1..]) |v| {
                            const p = pose.transformPoint(v);
                            out = out.merge(.{ .min = p, .max = p });
                        }
                        return out;
                    },
                    .heightfield => |field| {
                        if (two) unreachable;
                        // The corners of its box, placed.
                        const local = field.localBounds();
                        var out: ?Aabb = null;
                        for (0..8) |i| {
                            const p = pose.transformPoint(Vec.init(
                                if (i & 1 != 0) local.max.x else local.min.x,
                                if (i & 2 != 0) local.max.y else local.min.y,
                                if (i & 4 != 0) local.max.z else local.min.z,
                            ));
                            const point: Aabb = .{ .min = p, .max = p };
                            out = if (out) |so_far| so_far.merge(point) else point;
                        }
                        return out.?;
                    },
                    else => {
                        // Along each world axis, its farthest point either way.
                        var min: [S.n]T = undefined;
                        var max: [S.n]T = undefined;
                        for (0..S.n) |i| {
                            const axis = S.axis(i);
                            max[i] = S.get(pose.transformPoint(self.support(pose.inverseTransformVector(axis))), i);
                            min[i] = S.get(pose.transformPoint(self.support(pose.inverseTransformVector(axis.negate()))), i);
                        }
                        return .{ .min = Vec.fromArray(min), .max = Vec.fromArray(max) };
                    },
                }
            }

            /// Its volume in m³ (area in m² in 2D); zero for a mesh, heightfield or half-space,
            /// which hold no mass.
            // ponytail: a convex hull counts as its bounding box; integrate over its faces when
            // hull-heavy bodies need exact mass.
            pub fn volume(self: Shape) T {
                return switch (self) {
                    .ball => |ball| if (two) std.math.pi * ball.radius * ball.radius else 4.0 / 3.0 * std.math.pi * ball.radius * ball.radius * ball.radius,
                    .cuboid => |cuboid| boxVolume(cuboid.half_extents),
                    .capsule => |capsule| if (two)
                        4 * capsule.half_height * capsule.radius + std.math.pi * capsule.radius * capsule.radius
                    else
                        2 * std.math.pi * capsule.radius * capsule.radius * capsule.half_height + 4.0 / 3.0 * std.math.pi * capsule.radius * capsule.radius * capsule.radius,
                    .cylinder => |cylinder| 2 * std.math.pi * cylinder.radius * cylinder.radius * cylinder.half_height,
                    .cone => |cone| 2.0 / 3.0 * std.math.pi * cone.radius * cone.radius * cone.half_height,
                    .convex_hull => |hull| boxVolume(halfSize(hull.points)),
                    .mesh, .heightfield, .half_space => 0,
                };
            }

            /// Principal moments of inertia in its own axes for `mass_kg` spread through it, in kg·m².
            // ponytail: capsules count as their bounding cylinder (box in 2D), cones about their
            // middle rather than their centre of mass, and hulls as their bounding box.
            pub fn inertia(self: Shape, mass_kg: T) S.Inertia {
                return switch (self) {
                    .ball => |ball| if (two) 0.5 * mass_kg * ball.radius * ball.radius else S.Inertia.splat(0.4 * mass_kg * ball.radius * ball.radius),
                    .cuboid => |cuboid| boxInertia(cuboid.half_extents, mass_kg),
                    .capsule => |capsule| if (two)
                        boxInertia(Vec.init(capsule.radius, capsule.half_height + capsule.radius), mass_kg)
                    else
                        cylinderInertia(capsule.half_height + capsule.radius, capsule.radius, mass_kg),
                    .cylinder => |cylinder| if (two) unreachable else cylinderInertia(cylinder.half_height, cylinder.radius, mass_kg),
                    .cone => |cone| if (two) unreachable else coneInertia(cone, mass_kg),
                    .convex_hull => |hull| boxInertia(halfSize(hull.points), mass_kg),
                    .mesh, .heightfield, .half_space => S.angular_zero,
                };
            }
        };

        /// The half-space placed at `pose`: its world normal and its offset along it.
        pub fn placedPlane(half_space: HalfSpace, pose: Pose) struct { normal: Vec, offset: T } {
            const normal = pose.transformVector(half_space.normal).normalize();
            return .{ .normal = normal, .offset = normal.dot(pose.translation) };
        }

        fn signs(direction: Vec) Vec {
            var parts = direction.toArray();
            for (&parts) |*part| part.* = if (part.* < 0) -1 else 1;
            return Vec.fromArray(parts);
        }

        /// The end of a y segment of `half_height` towards `direction`.
        fn axial(direction: Vec, half_height: T) Vec {
            return S.axis(1).scale(if (direction.y < 0) -half_height else half_height);
        }

        /// `direction` flattened onto the xz plane, at unit length (zero when it points along y).
        fn radial(direction: Vec) Vec {
            return Vec.init(direction.x, 0, direction.z).normalize();
        }

        fn coneSupport(cone: Cone, direction: Vec) Vec {
            const tip = S.axis(1).scale(cone.half_height);
            const rim = radial(direction).scale(cone.radius).sub(S.axis(1).scale(cone.half_height));
            return if (tip.dot(direction) >= rim.dot(direction)) tip else rim;
        }

        fn farthest(points: []const Vec, direction: Vec) Vec {
            var best = points[0];
            var best_along = best.dot(direction);
            for (points[1..]) |p| {
                const along = p.dot(direction);
                if (along > best_along) {
                    best = p;
                    best_along = along;
                }
            }
            return best;
        }

        fn halfSize(points: []const Vec) Vec {
            const bounds = Aabb.around(points);
            return bounds.max.sub(bounds.min).scale(0.5);
        }

        fn boxVolume(h: Vec) T {
            var out: T = 1;
            for (h.toArray()) |part| out *= 2 * part;
            return out;
        }

        fn boxInertia(h: Vec, mass_kg: T) S.Inertia {
            const sq = h.mul(h);
            if (two) {
                return mass_kg * (sq.x + sq.y) / 3.0;
            } else {
                return Vec.init(sq.y + sq.z, sq.x + sq.z, sq.x + sq.y).scale(mass_kg / 3.0);
            }
        }

        fn cylinderInertia(half_height: T, radius: T, mass_kg: T) S.Inertia {
            const height = 2 * half_height;
            const across = mass_kg * (3 * radius * radius + height * height) / 12.0;
            return Vec.init(across, 0.5 * mass_kg * radius * radius, across);
        }

        fn coneInertia(cone: Cone, mass_kg: T) S.Inertia {
            const height = 2 * cone.half_height;
            const across = 3.0 / 80.0 * mass_kg * (4 * cone.radius * cone.radius + height * height);
            return Vec.init(across, 0.3 * mass_kg * cone.radius * cone.radius, across);
        }
    };
}

test "each shape's support point lies on its surface, the way asked" {
    const S = @import("space.zig").Space(.three, f64);
    const Shape = Shapes(S).Shape;
    const up = S.Vec.init(0, 1, 0);
    try std.testing.expect((Shape{ .cylinder = .{ .half_height = 1, .radius = 0.5 } }).support(S.Vec.init(1, 1, 0)).approxEql(S.Vec.init(0.5, 1, 0), 1e-9));
    try std.testing.expect((Shape{ .cone = .{ .half_height = 1, .radius = 0.5 } }).support(up).approxEql(up, 1e-9));
    try std.testing.expect((Shape{ .capsule = .{ .half_height = 1, .radius = 0.5 } }).support(up).approxEql(S.Vec.init(0, 1.5, 0), 1e-9));
    const points = [_]S.Vec{ .init(0, 0, 0), .init(2, 0, 0), .init(0, 3, 0) };
    try std.testing.expect((Shape{ .convex_hull = .{ .points = &points } }).support(up).approxEql(S.Vec.init(0, 3, 0), 1e-9));
}
