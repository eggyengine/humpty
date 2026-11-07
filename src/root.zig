//! humpty: rigid-body physics in 2D and 3D, in `f32` or `f64`, after Rapier and Avian. A world
//! (`Physics`) owns bodies and the colliders they touch it with; `ecs` runs one inside a clutch
//! world, its bodies and colliders components of the entities they belong to; `debug_draw`
//! turns shapes into lines for whatever renders them. It needs no engine.
const std = @import("std");
pub const emath = @import("eggenvector");

const physics = @import("physics.zig");
pub const space = @import("space.zig");
pub const Dim = space.Dim;
pub const Physics = physics.Physics;
pub const Physics2 = Physics(.two, f32);
pub const Physics3 = Physics(.three, f32);
pub const BodyHandle = @import("body.zig").BodyHandle;
pub const BodyKind = @import("body.zig").BodyKind;
pub const Rotation = @import("body.zig").Rotation;
pub const ColliderHandle = @import("collider.zig").ColliderHandle;
pub const shapes = @import("shapes.zig");
pub const ecs = @import("ecs.zig");
pub const debug_draw = @import("debug_draw.zig");

/// The whole API at once, for any 3D precision: a ball and a crate dropped on the ground
/// settle, touch it, sleep, and can be found by a ray.
fn dropOnGround(comptime T: type) !void {
    const World = Physics(.three, T);
    const Vec = World.Vec;
    const Pose = World.Pose;
    const gpa = std.testing.allocator;

    var world = try World.init(gpa, .{ .gravity = Vec.init(0, -9.81, 0), .fixed_step_s = 1.0 / 60.0 });
    defer world.deinit();

    // Static scenery is a collider with no body: it never moves and costs nothing to step.
    const ground = try world.addCollider(.{ .shape = .{ .half_space = .{ .normal = Vec.init(0, 1, 0) } } }, null, 0);

    const ball = try world.addBody(.{ .kind = .dynamic, .pose = Pose.fromTranslation(Vec.init(0, 10, 0)), .mass_kg = 1 }, 1);
    const ball_shape = try world.addCollider(.{
        .shape = .{ .ball = .{ .radius = 0.5 } },
        .material = .{ .restitution = 0.2, .friction = 0.5 },
    }, ball, 1);

    // Tipped onto an edge so it has to rotate to settle flat.
    const crate = try world.addBody(.{ .kind = .dynamic, .pose = Pose.init(Vec.init(3, 2, 0), Vec.init(0, 0, std.math.pi / 4.0)), .mass_kg = 10 }, 2);
    _ = try world.addCollider(.{ .shape = .{ .cuboid = .{ .half_extents = Vec.splat(0.5) } } }, crate, 2);
    world.applyImpulse(crate, Vec.init(-5, 0, 0), .{ .at_world_point = Vec.init(3, 2.5, 0) });

    // Frames don't line up with the fixed step; the accumulator must absorb that.
    for (0..10 * 144) |_| try world.update(1.0 / 144.0);

    // The ball rests on the ground: not sunk into it, not still bouncing, asleep.
    const ball_pose = world.pose(ball);
    try std.testing.expectApproxEqAbs(@as(T, 0.5), ball_pose.translation.y, 0.01);
    try std.testing.expect(world.velocity(ball).linear.length() < 0.01);
    try std.testing.expect(world.isSleeping(ball));

    // The crate settled on a face, so its bottom sits on the ground.
    try std.testing.expectApproxEqAbs(@as(T, 0.5), world.pose(crate).translation.y, 0.01);
    try std.testing.expect(world.velocity(crate).angular.length() < 0.01);

    // Contacts name both colliders, so games can react to them.
    var touching_ground = false;
    for (world.contacts()) |contact| {
        if (contact.involves(ball_shape) and contact.involves(ground)) touching_ground = true;
    }
    try std.testing.expect(touching_ground);

    // A ray straight down from above the ball hits the ball first.
    const hit = world.raycast(.{
        .origin = ball_pose.translation.add(Vec.init(0, 5, 0)),
        .direction = Vec.init(0, -1, 0),
        .max_distance_m = 100,
    }, .{}) orelse return error.TestExpectedHit;
    try std.testing.expectEqual(ball_shape, hit.collider);
    try std.testing.expectEqual(@as(?BodyHandle, ball), world.colliderBody(hit.collider));
    try std.testing.expectEqual(@as(u64, 1), world.colliderData(hit.collider));
    try std.testing.expectApproxEqAbs(@as(T, 4.5), hit.distance_m, 0.01);

    // Leaving the ball out, the same ray reaches the ground beneath it.
    const past = world.raycast(.{
        .origin = ball_pose.translation.add(Vec.init(0, 5, 0)),
        .direction = Vec.init(0, -1, 0),
        .max_distance_m = 100,
    }, .{ .exclude_body = ball }) orelse return error.TestExpectedHit;
    try std.testing.expectEqual(ground, past.collider);

    // Removing a body takes its colliders with it, and its handle goes stale rather than
    // pointing at a reused slot.
    world.removeBody(ball);
    try std.testing.expect(!world.contains(ball));
    try std.testing.expect(!world.contains(ball_shape));
    const replacement = try world.addBody(.{ .kind = .dynamic, .pose = Pose.identity(), .mass_kg = 1 }, 3);
    try std.testing.expect(replacement != ball);
    try std.testing.expect(!world.contains(ball));
}

test "a ball and a crate settle on the ground in single precision" {
    try dropOnGround(f32);
}

test "a ball and a crate settle on the ground in double precision" {
    try dropOnGround(f64);
}

test "in 2D, a box falls onto a static mesh floor, and a cylinder is refused" {
    const World = Physics2;
    const Vec = World.Vec;
    const gpa = std.testing.allocator;
    var world = try World.init(gpa, .{ .gravity = Vec.init(0, -9.81), .fixed_step_s = 1.0 / 60.0 });
    defer world.deinit();

    const floor = [_]Vec{ .init(-10, 0), .init(0, 0), .init(10, 0) };
    const segments = [_][2]u32{ .{ 0, 1 }, .{ 1, 2 } };
    _ = try world.addCollider(.{ .shape = .{ .mesh = .{ .vertices = &floor, .indices = &segments } } }, null, 0);
    const box = try world.addBody(.{ .kind = .dynamic, .pose = World.Pose.fromTranslation(Vec.init(0.3, 3)), .mass_kg = 2 }, 1);
    _ = try world.addCollider(.{ .shape = .{ .cuboid = .{ .half_extents = Vec.splat(0.5) } } }, box, 1);
    try std.testing.expectError(error.UnsupportedShape, world.addCollider(.{ .shape = .{ .cylinder = .{ .half_height = 1, .radius = 1 } } }, box, 1));

    for (0..240) |_| try world.update(1.0 / 60.0);
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), world.pose(box).translation.y, 0.02);
    try std.testing.expectApproxEqAbs(@as(f32, 0), world.pose(box).rotation.angle(), 0.02);
}

test "a capsule, cylinder, cone and hull each come to rest on the ground" {
    const World = Physics3;
    const Vec = World.Vec;
    const gpa = std.testing.allocator;
    var world = try World.init(gpa, .{ .gravity = Vec.init(0, -9.81, 0), .fixed_step_s = 1.0 / 60.0 });
    defer world.deinit();
    _ = try world.addCollider(.{ .shape = .{ .half_space = .{ .normal = Vec.init(0, 1, 0) } } }, null, 0);

    const hull = [_]Vec{ .init(-0.5, -0.5, -0.5), .init(0.5, -0.5, -0.5), .init(0, -0.5, 0.5), .init(0, 0.5, 0) };
    const shapes_to_drop = [_]World.Shape{
        .{ .capsule = .{ .half_height = 0.5, .radius = 0.3 } },
        .{ .cylinder = .{ .half_height = 0.5, .radius = 0.4 } },
        .{ .cone = .{ .half_height = 0.5, .radius = 0.4 } },
        .{ .convex_hull = .{ .points = &hull } },
    };
    var bodies: [shapes_to_drop.len]BodyHandle = undefined;
    for (shapes_to_drop, &bodies, 0..) |shape, *body, i| {
        body.* = try world.addBody(.{ .kind = .dynamic, .pose = World.Pose.fromTranslation(Vec.init(@as(f32, @floatFromInt(i)) * 3, 2, 0)), .mass_kg = 1 }, i);
        _ = try world.addCollider(.{ .shape = shape }, body.*, i);
    }
    for (0..600) |_| try world.update(1.0 / 60.0);
    for (bodies) |body| {
        try std.testing.expect(world.pose(body).translation.y > 0.1);
        try std.testing.expect(world.pose(body).translation.y < 1.0);
        try std.testing.expect(world.isSleeping(body));
    }
}

test {
    std.testing.refAllDecls(@This());
    _ = @import("slots.zig");
    _ = @import("narrowphase.zig");
    _ = @import("gjk.zig");
    _ = @import("solver.zig");
    _ = @import("ray.zig");
    _ = @import("shapes.zig");
    _ = @import("space.zig");
}
