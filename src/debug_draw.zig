//! Wireframes of shapes, as line segments handed to a sink the caller owns, so any renderer can
//! draw them. Balls and boxes are drawn exactly; other convex shapes by their outline in each
//! axis plane (found through `support`), meshes edge by edge, and a half-space as a patch of
//! its surface with its normal.
const std = @import("std");
const shapes_zig = @import("shapes.zig");
const BodyKind = @import("body.zig").BodyKind;

/// Linear RGBA, each 0 to 1.
pub const Color = [4]f32;

/// The colours `Physics.debugDraw` and engines' collider drawing use, by what a collider is on.
pub const palette = struct {
    pub const static: Color = .{ 0.55, 0.58, 0.62, 1 };
    pub const dynamic: Color = .{ 0.35, 0.85, 0.45, 1 };
    pub const kinematic: Color = .{ 0.7, 0.5, 1, 1 };
    pub const sleeping: Color = .{ 0.35, 0.55, 0.95, 1 };
    pub const sensor: Color = .{ 1, 0.8, 0.25, 1 };
    pub const contact: Color = .{ 1, 0.3, 0.25, 1 };

    /// A collider's colour: a sensor's whatever it is on, else by its body's kind and whether it
    /// sleeps (`null` body: static scenery).
    pub fn of(body: ?BodyKind, asleep: bool, is_sensor: bool) Color {
        if (is_sensor) return palette.sensor;
        const kind = body orelse return palette.static;
        if (asleep and kind != .static) return palette.sleeping;
        return switch (kind) {
            .static => palette.static,
            .dynamic => palette.dynamic,
            .kinematic => palette.kinematic,
        };
    }
};

/// Drawing in space `S` (a `space.Space`). A sink is any value with
/// `fn line(self, from: S.Vec, to: S.Vec, color: Color) !void`; its error stops the drawing.
pub fn DebugDraw(comptime S: type) type {
    const T = S.Real;
    const Vec = S.Vec;
    const Pose = S.Pose;
    const Shapes = shapes_zig.Shapes(S);
    const two = S.dim == .two;

    return struct {
        /// Segments per full circle: a ball's rings, an outline's samples.
        pub const segments = 24;
        /// How far a half-space's patch reaches from the point nearest the origin, in metres.
        pub const plane_reach_m: T = 5;

        /// `shape` placed at `pose`, in `color`.
        ///
        /// ```zig
        /// try Draw.shape(.{ .ball = .{ .radius = 0.5 } }, pose, palette.dynamic, &sink);
        /// ```
        pub fn shape(drawn: Shapes.Shape, pose: Pose, color: Color, sink: anytype) !void {
            switch (drawn) {
                .ball => |ball| try drawBall(ball.radius, pose, color, sink),
                .cuboid => |cuboid| try drawCuboid(cuboid.half_extents, pose, color, sink),
                .mesh => |mesh| for (mesh.indices) |facet| {
                    var points: [S.n]Vec = undefined;
                    for (&points, facet) |*p, i| p.* = pose.transformPoint(mesh.vertices[i]);
                    if (two) try sink.line(points[0], points[1], color) else {
                        try sink.line(points[0], points[1], color);
                        try sink.line(points[1], points[2], color);
                        try sink.line(points[2], points[0], color);
                    }
                },
                .half_space => |half_space| try drawPlane(half_space.normal, pose, color, sink),
                else => try outline(drawn, pose, color, sink),
            }
        }

        /// A cross where `point` is, `size_m` across, with `normal` drawn from it: a contact.
        pub fn marker(point: Vec, normal: Vec, size_m: T, color: Color, sink: anytype) !void {
            for (0..S.n) |axis| {
                const step = S.axis(axis).scale(size_m / 2);
                try sink.line(point.sub(step), point.add(step), color);
            }
            try sink.line(point, point.add(normal.scale(size_m * 2)), color);
        }

        fn drawBall(radius: T, pose: Pose, color: Color, sink: anytype) !void {
            if (two) {
                try ring(pose, 0, 1, radius, color, sink);
                // A spoke shows how far it has rolled.
                try sink.line(pose.translation, pose.transformPoint(S.axis(0).scale(radius)), color);
            } else {
                try ring(pose, 0, 1, radius, color, sink);
                try ring(pose, 1, 2, radius, color, sink);
                try ring(pose, 0, 2, radius, color, sink);
            }
        }

        /// A circle of `radius` in the plane of local axes `u` and `v`.
        fn ring(pose: Pose, u: usize, v: usize, radius: T, color: Color, sink: anytype) !void {
            var last = pose.transformPoint(S.axis(u).scale(radius));
            for (1..segments + 1) |i| {
                const angle = @as(T, @floatFromInt(i)) / segments * std.math.tau;
                const local = S.axis(u).scale(@cos(angle) * radius).add(S.axis(v).scale(@sin(angle) * radius));
                const next = pose.transformPoint(local);
                try sink.line(last, next, color);
                last = next;
            }
        }

        fn drawCuboid(half: Vec, pose: Pose, color: Color, sink: anytype) !void {
            const h = half.toArray();
            const corners = 1 << S.n;
            // Each edge joins a corner to the one across a single axis, drawn once from its low end.
            for (0..corners) |corner| for (0..S.n) |axis| {
                if (corner & (@as(usize, 1) << @intCast(axis)) != 0) continue;
                const from = cornerPoint(h, corner);
                const to = cornerPoint(h, corner | (@as(usize, 1) << @intCast(axis)));
                try sink.line(pose.transformPoint(from), pose.transformPoint(to), color);
            };
        }

        fn cornerPoint(h: [S.n]T, corner: usize) Vec {
            var point: [S.n]T = undefined;
            for (&point, h, 0..) |*p, extent, axis| p.* = if (corner & (@as(usize, 1) << @intCast(axis)) != 0) extent else -extent;
            return Vec.fromArray(point);
        }

        /// The outline of a convex shape in each local axis plane: its farthest points around.
        fn outline(drawn: Shapes.Shape, pose: Pose, color: Color, sink: anytype) !void {
            const planes: []const [2]usize = if (two) &.{.{ 0, 1 }} else &.{ .{ 0, 1 }, .{ 1, 2 }, .{ 0, 2 } };
            for (planes) |plane| {
                var first: ?Vec = null;
                var last: Vec = undefined;
                for (0..segments) |i| {
                    const angle = @as(T, @floatFromInt(i)) / segments * std.math.tau;
                    const direction = S.axis(plane[0]).scale(@cos(angle)).add(S.axis(plane[1]).scale(@sin(angle)));
                    const point = pose.transformPoint(drawn.support(direction));
                    if (first == null) first = point else try sink.line(last, point, color);
                    last = point;
                }
                try sink.line(last, first.?, color);
            }
        }

        fn drawPlane(normal: Vec, pose: Pose, color: Color, sink: anytype) !void {
            const plane = Shapes.placedPlane(.{ .normal = normal }, pose);
            const centre = plane.normal.scale(plane.offset);
            const across = S.tangents(plane.normal);
            if (two) {
                const reach = across[0].scale(plane_reach_m);
                try sink.line(centre.sub(reach), centre.add(reach), color);
            } else {
                // A square patch, crossed so its middle is plain.
                const u = across[0].scale(plane_reach_m);
                const v = across[1].scale(plane_reach_m);
                const square = [_]Vec{ centre.add(u).add(v), centre.sub(u).add(v), centre.sub(u).sub(v), centre.add(u).sub(v) };
                for (square, 0..) |corner, i| try sink.line(corner, square[(i + 1) % 4], color);
                try sink.line(square[0], square[2], color);
                try sink.line(square[1], square[3], color);
            }
            try sink.line(centre, centre.add(plane.normal), color);
        }
    };
}

/// A sink for tests: counts lines, and keeps how far from the origin any end reached.
const Counter = struct {
    lines: usize = 0,
    farthest: f32 = 0,
    pub fn line(self: *Counter, from: anytype, to: anytype, _: Color) !void {
        self.lines += 1;
        self.farthest = @max(self.farthest, @as(f32, @floatCast(@max(from.length(), to.length()))));
    }
};

test "a box is twelve edges, a ball three rings, and a cylinder's outline stays inside its bounds" {
    const S = @import("space.zig").Space(.three, f32);
    const Draw = DebugDraw(S);
    var counter = Counter{};
    try Draw.shape(.{ .cuboid = .{ .half_extents = S.Vec.splat(1) } }, S.Pose.identity(), palette.static, &counter);
    try std.testing.expectEqual(@as(usize, 12), counter.lines);
    try std.testing.expectApproxEqAbs(@sqrt(3.0), counter.farthest, 1e-4);

    counter = .{};
    try Draw.shape(.{ .ball = .{ .radius = 2 } }, S.Pose.identity(), palette.dynamic, &counter);
    try std.testing.expectEqual(@as(usize, 3 * Draw.segments), counter.lines);

    counter = .{};
    try Draw.shape(.{ .cylinder = .{ .half_height = 1, .radius = 1 } }, S.Pose.identity(), palette.dynamic, &counter);
    try std.testing.expectEqual(@as(usize, 3 * Draw.segments), counter.lines);
    try std.testing.expect(counter.farthest <= @sqrt(2.0) + 1e-4);
}

test "in 2D a box is four edges and a mesh one line a segment" {
    const S = @import("space.zig").Space(.two, f64);
    const Draw = DebugDraw(S);
    var counter = Counter{};
    try Draw.shape(.{ .cuboid = .{ .half_extents = S.Vec.splat(1) } }, S.Pose.identity(), palette.static, &counter);
    try std.testing.expectEqual(@as(usize, 4), counter.lines);

    counter = .{};
    const vertices = [_]S.Vec{ .init(-1, 0), .init(0, 0), .init(1, 0) };
    const indices = [_][2]u32{ .{ 0, 1 }, .{ 1, 2 } };
    try Draw.shape(.{ .mesh = .{ .vertices = &vertices, .indices = &indices } }, S.Pose.identity(), palette.static, &counter);
    try std.testing.expectEqual(@as(usize, 2), counter.lines);
}
