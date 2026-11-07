//! Collision shapes, after Rapier's and Avian's: a ball, cuboid, capsule, cylinder and cone, a
//! convex hull of points, a mesh of triangles (segments in 2D) and a half-space, behind one
//! tagged union. Every shape but the mesh and half-space is convex, found by its support point.
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
            half_space: HalfSpace,

            /// Whether this space has it: cylinders and cones are 3D only.
            pub fn supported(self: Shape) bool {
                return switch (self) {
                    .cylinder, .cone => !two,
                    .convex_hull => |hull| hull.points.len > 0,
                    .mesh => |mesh| mesh.indices.len > 0,
                    else => true,
                };
            }

            /// Whether `support` describes it: everything but a mesh or half-space.
            pub fn convex(self: Shape) bool {
                return switch (self) {
                    .mesh, .half_space => false,
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
                    .mesh, .half_space => unreachable,
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

            /// Its volume in m³ (area in m² in 2D); zero for a mesh or half-space, which hold no mass.
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
                    .mesh, .half_space => 0,
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
                    .mesh, .half_space => S.angular_zero,
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
