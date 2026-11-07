const std = @import("std");
const clutch = @import("clutch");
pub const emath = @import("eggenvector");

pub const eggy_types = @import("eggy.zig");

// imports
const physics = @import("physics.zig");
pub const Physics = physics.Physics;
pub const Body = @import("body.zig").Body;
pub const BodyHandle = @import("body.zig").BodyHandle;
pub const Collider = @import("collider.zig").Collider;
pub const Shape = @import("shapes.zig").Shape;
pub const ColliderHandle = @import("collider.zig").ColliderHandle;

// The whole API at once: a ball and a crate dropped on the ground settle, touch it, sleep,
// and can be found by a ray.
test "main" {
    const Vec3 = emath.Vec3;
    const Pose = emath.Isometry3(f32);
    const gpa = std.testing.allocator;

    // The world owns every body; the game only ever holds handles.
    var world = try Physics.init(gpa, .{
        .gravity = Vec3.init(0, -9.81, 0),
        .fixed_step_s = 1.0 / 60.0,
    });
    defer world.deinit();

    // Static scenery is a collider with no body: it never moves and costs nothing to step.
    const ground = try world.addCollider(.{
        .shape = .{ .plane = .{ .normal = Vec3.init(0, 1, 0), .offset = 0 } },
    }, null);

    const ball = try world.addBody(.{
        .kind = .dynamic,
        .pose = Pose.fromTranslation(Vec3.init(0, 10, 0)),
        .mass_kg = 1.0,
    });
    const ball_shape = try world.addCollider(.{
        .shape = .{ .sphere = .{ .radius = 0.5 } },
        .material = .{ .restitution = 0.2, .friction = 0.5 },
    }, ball);

    // Tipped onto an edge so it has to rotate to settle flat.
    const crate = try world.addBody(.{
        .kind = .dynamic,
        .pose = Pose.init(Vec3.init(3, 2, 0), Vec3.init(0, 0, std.math.pi / 4.0)),
        .mass_kg = 10.0,
    });
    _ = try world.addCollider(.{ .shape = .{ .box = .{ .half_extents = Vec3.init(0.5, 0.5, 0.5) } } }, crate);
    world.applyImpulse(crate, Vec3.init(-5, 0, 0), .{ .at_world_point = Vec3.init(3, 2.5, 0) });

    // Frames don't line up with the fixed step; the accumulator must absorb that.
    for (0..10 * 144) |_| try world.update(1.0 / 144.0);

    // The ball rests on the plane: not sunk into it, not still bouncing, asleep.
    const ball_pose = world.pose(ball);
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), ball_pose.translation.y, 0.01);
    try std.testing.expect(world.velocity(ball).linear.length() < 0.01);
    try std.testing.expect(world.isSleeping(ball));

    // The crate settled on a face, so its bottom sits on the ground.
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), world.pose(crate).translation.y, 0.01);
    try std.testing.expect(world.velocity(crate).angular.length() < 0.01);

    // Contacts name both colliders, so games can react to them.
    var touching_ground = false;
    for (world.contacts()) |contact| {
        if (contact.involves(ball_shape) and contact.involves(ground)) touching_ground = true;
    }
    try std.testing.expect(touching_ground);

    // A ray straight down from above the ball hits the ball first.
    const hit = world.raycast(.{
        .origin = ball_pose.translation.add(Vec3.init(0, 5, 0)),
        .direction = Vec3.init(0, -1, 0),
        .max_distance_m = 100,
    }) orelse return error.TestExpectedHit;
    try std.testing.expectEqual(ball_shape, hit.collider);
    try std.testing.expectEqual(@as(?BodyHandle, ball), world.colliderBody(hit.collider));
    try std.testing.expectApproxEqAbs(@as(f32, 4.5), hit.distance_m, 0.01);

    // Removing a body takes its colliders with it, and its handle goes stale rather than
    // pointing at a reused slot.
    world.removeBody(ball);
    try std.testing.expect(!world.contains(ball));
    try std.testing.expect(!world.contains(ball_shape));
    const replacement = try world.addBody(.{ .kind = .dynamic, .pose = Pose.identity(), .mass_kg = 1.0 });
    try std.testing.expect(replacement != ball);
    try std.testing.expect(!world.contains(ball));
}

test {
    // Pulls in the tests written beside the code in physics.zig.
    std.testing.refAllDecls(@This());
    _ = physics;
    _ = @import("slots.zig");
    _ = @import("narrowphase.zig");
    _ = @import("solver.zig");
    _ = @import("ray.zig");
}
