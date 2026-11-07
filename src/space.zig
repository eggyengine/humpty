//! The space a world simulates: 2D or 3D, in `f32` or `f64`. Everything that differs between
//! them (vectors, poses, how things turn) lives here, so the rest of humpty is written once.
const std = @import("std");
const emath = @import("eggenvector");

pub const Dim = enum(u8) { two = 2, three = 3 };

/// Vectors, poses and turning for `dim` dimensions in scalar `T` (`f32` or `f64`).
pub fn Space(comptime dim_value: Dim, comptime T: type) type {
    comptime std.debug.assert(T == f32 or T == f64);
    return struct {
        pub const dim = dim_value;
        pub const n: usize = @backingInt(dim_value);
        pub const Real = T;
        pub const Pose = if (dim == .two) emath.Isometry2(T) else emath.Isometry3(T);
        pub const Vec = @FieldType(Pose, "translation");
        pub const Rotation = if (dim == .two) emath.UnitComplex(T) else emath.UnitQuaternion(T);
        /// How fast something turns: radians per second about z in 2D, an axis times speed in 3D.
        pub const Angular = if (dim == .two) T else @FieldType(emath.Isometry3(T), "translation");
        /// Principal moments of inertia (kg·m²): one in 2D, one per body axis in 3D.
        pub const Inertia = Angular;

        pub const angular_zero: Angular = if (dim == .two) 0 else .zero;

        /// Unit vector along axis `i`.
        pub fn axis(i: usize) Vec {
            var parts: [n]T = @splat(0);
            parts[i] = 1;
            return Vec.fromArray(parts);
        }

        pub fn get(v: Vec, i: usize) T {
            return v.toArray()[i];
        }

        /// `a × b`: a scalar in 2D, a vector in 3D.
        pub fn cross(a: Vec, b: Vec) Angular {
            return if (dim == .two) a.x * b.y - a.y * b.x else a.cross(b);
        }

        /// `w × r`: the velocity of a point `r` from the centre of something turning at `w`.
        pub fn turnVelocity(w: Angular, r: Vec) Vec {
            return if (dim == .two) Vec.init(-w * r.y, w * r.x) else w.cross(r);
        }

        pub fn angularAdd(a: Angular, b: Angular) Angular {
            return if (dim == .two) a + b else a.add(b);
        }

        pub fn angularScale(a: Angular, s: T) Angular {
            return if (dim == .two) a * s else a.scale(s);
        }

        pub fn angularDot(a: Angular, b: Angular) T {
            return if (dim == .two) a * b else a.dot(b);
        }

        pub fn angularLengthSquared(a: Angular) T {
            return angularDot(a, a);
        }

        /// The angular velocity change an angular impulse makes on something turned by `rotation`
        /// with inverse principal moments `inv_inertia`.
        pub fn turnBy(rotation: Rotation, inv_inertia: Inertia, angular_impulse: Angular) Angular {
            if (dim == .two) {
                return inv_inertia * angular_impulse;
            } else {
                const local = rotation.inverseRotateVector(angular_impulse);
                return rotation.rotateVector(local.mul(inv_inertia));
            }
        }

        /// `rotation` turned for `dt_s` at `w`.
        pub fn integrate(rotation: Rotation, w: Angular, dt_s: T) Rotation {
            const turn = if (dim == .two) Rotation.fromAngle(w * dt_s) else Rotation.fromScaledAxis(w.scale(dt_s));
            return turn.mul(rotation).renormalize();
        }

        /// 1/x per moment, or 0 where it can't turn.
        pub fn invert(inertia: Inertia) Inertia {
            if (dim == .two) {
                return if (inertia > 0) 1 / inertia else 0;
            } else {
                var parts = inertia.toArray();
                for (&parts) |*part| part.* = if (part.* > 0) 1 / part.* else 0;
                return Inertia.fromArray(parts);
            }
        }

        /// Two (3D) or one (2D) unit directions along a surface with unit normal `normal`.
        pub fn tangents(normal: Vec) [n - 1]Vec {
            if (dim == .two) {
                return .{normal.perpendicular()};
            } else {
                const first = if (@abs(normal.x) > 0.57) Vec.init(normal.y, -normal.x, 0).normalize() else Vec.init(0, normal.z, -normal.y).normalize();
                return .{ first, normal.cross(first) };
            }
        }

        /// A box around something, along the world axes.
        pub const Aabb = struct {
            min: Vec,
            max: Vec,

            pub const everywhere: Aabb = .{ .min = Vec.splat(-std.math.inf(T)), .max = Vec.splat(std.math.inf(T)) };

            pub fn overlaps(self: Aabb, other: Aabb) bool {
                const a_min = self.min.toArray();
                const a_max = self.max.toArray();
                const b_min = other.min.toArray();
                const b_max = other.max.toArray();
                for (0..n) |i| if (a_min[i] > b_max[i] or a_max[i] < b_min[i]) return false;
                return true;
            }

            pub fn merge(self: Aabb, other: Aabb) Aabb {
                return .{ .min = self.min.min(other.min), .max = self.max.max(other.max) };
            }

            pub fn around(points: []const Vec) Aabb {
                var out: Aabb = .{ .min = points[0], .max = points[0] };
                for (points[1..]) |p| out = out.merge(.{ .min = p, .max = p });
                return out;
            }
        };
    };
}

test "turning integrates the same way in 2D and 3D" {
    const S2 = Space(.two, f64);
    const S3 = Space(.three, f32);
    const quarter = std.math.pi / 2.0;
    const turned2 = S2.integrate(S2.Rotation.identity(), quarter, 1).rotateVector(S2.Vec.unit_x);
    try std.testing.expect(turned2.approxEql(S2.Vec.unit_y, 1e-9));
    const turned3 = S3.integrate(S3.Rotation.identity(), S3.Vec.init(0, 0, quarter), 1).rotateVector(S3.Vec.unit_x);
    try std.testing.expect(turned3.approxEql(S3.Vec.unit_y, 1e-5));
}
