//! humpty: physics for clutch worlds. Give entities a `RigidBody`, call `addTo` once, and the
//! world's `.Update` stage moves them.
const std = @import("std");
const emath = @import("eggenvector");
const clutch = @import("clutch");

/// A body that moves under gravity and the forces applied to it.
pub const RigidBody = struct {
    position: emath.Vec3 = .zero,
    velocity: emath.Vec3 = .zero,
    /// Forces applied since the last step; each step consumes and clears them.
    force: emath.Vec3 = .zero,
    /// In kilograms. Zero or less makes the body static: it never moves.
    mass: f32 = 1,

    pub fn applyForce(self: *RigidBody, force: emath.Vec3) void {
        self.force = self.force.add(force);
    }
};

/// World-wide physics settings, stored as a clutch resource.
pub const Settings = struct {
    /// In metres per second squared.
    gravity: emath.Vec3 = .init(0, -9.81, 0),
    /// Simulated time per step, in seconds.
    step_seconds: f32 = 1.0 / 60.0,
};

/// Advance every body by one step (semi-implicit Euler: forces change velocity, then velocity
/// changes position).
pub fn integrate(bodies: clutch.Query(.{*RigidBody}), settings: clutch.Res(Settings)) !void {
    const dt = settings.step_seconds;
    var iter = bodies;
    while (iter.next()) |view| {
        const body = view.get(*RigidBody);
        if (body.mass <= 0) continue;

        const acceleration = settings.gravity.add(body.force.scale(1 / body.mass));
        body.velocity = body.velocity.add(acceleration.scale(dt));
        body.position = body.position.add(body.velocity.scale(dt));
        body.force = .zero;
    }
}

/// humpty's systems, in clutch's `.Update` stage.
pub const Physics = clutch.Schedule(.Update, .{integrate});

/// Register physics in `world`, with default `Settings` unless the world already has some.
pub fn addTo(world: *clutch.World) !void {
    if (world.getResource(Settings) == null) try world.insertResource(Settings{});
    try world.addScheduleFallible(Physics);
}

test "a dynamic body falls one step under gravity, a static one stays put" {
    var world = clutch.World.init(std.testing.allocator);
    defer world.deinit();
    try addTo(&world);
    const settings = world.getResource(Settings).?;

    const ball = try world.spawn(.{RigidBody{ .position = .init(0, 10, 0) }});
    const floor = try world.spawn(.{RigidBody{ .mass = 0 }});
    try world.runStage(.Update);

    const dt = settings.step_seconds;
    const falling = world.get(ball, RigidBody).?;
    try std.testing.expectApproxEqAbs(-9.81 * dt, falling.velocity.y, 1e-6);
    try std.testing.expectApproxEqAbs(10 - 9.81 * dt * dt, falling.position.y, 1e-6);
    try std.testing.expectEqual(emath.Vec3.zero, world.get(floor, RigidBody).?.position);
}

test "applied forces push for one step and are then cleared" {
    var world = clutch.World.init(std.testing.allocator);
    defer world.deinit();
    try world.insertResource(Settings{ .gravity = .zero, .step_seconds = 0.5 });
    try addTo(&world);

    const crate = try world.spawn(.{RigidBody{ .mass = 2 }});
    world.get(crate, RigidBody).?.applyForce(.init(4, 0, 0));
    try world.runStage(.Update);

    // a = F / m = 2 m/s², so after 0.5 s: v = 1 m/s and x = v * dt = 0.5 m.
    const body = world.get(crate, RigidBody).?;
    try std.testing.expectApproxEqAbs(@as(f32, 1), body.velocity.x, 1e-6);
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), body.position.x, 1e-6);
    try std.testing.expectEqual(emath.Vec3.zero, body.force);
}
