const std = @import("std");
const emath = @import("eggenvector");
const clutch = @import("clutch");

const Position = struct { value: emath.Vec2 };
const Velocity = struct { value: emath.Vec2 };
const Acceleration = struct { value: emath.Vec2 };

const Time = struct {
    delta_seconds: f32,
};

var gravity: f32 = 9.81;

fn physicsStep(
    query: clutch.Query(.{
        *Position,
        *Velocity,
        *const Acceleration,
    }),
    time: clutch.Res(Time),
) !void {
    const dt = time.delta_seconds;

    var iter = query;
    while (iter.next()) |view| {
        const position = &view.get(*Position).value;
        const velocity = &view.get(*Velocity).value;
        const acceleration = &view.get(*const Acceleration).value;

        velocity.x += acceleration.x * dt;
        velocity.y += acceleration.y * dt;

        position.x += velocity.x * dt;
        position.y += velocity.y * dt;
    }
}

test "the ball is affected by gravity" {
    var world = clutch.World.init(std.testing.allocator);
    defer world.deinit();

    const angle = emath.Angle.fromRadians(3.14 / 4.0);
    _ = try world.spawn(.{
        Position{ .value = emath.Vec2.init(0, 100) },
        Velocity{ .value = emath.Vec2.init(10 * angle.cos(), 10 * angle.sin()) },
        Acceleration{ .value = emath.Vec2.init(0, -gravity) },
    });

    world.addSchedule(clutch.Schedule(.Update, .{physicsStep}));

    const dt: f32 = 1.0 / 60.0;
    try world.insertResource(Time{ .delta_seconds = dt });

    simulation: for (0..600) |step| {
        try world.runStage(.Update);
        world.tick();

        var query = world.query(.{*const Position});
        while (query.next()) |view| {
            const position = &view.get(*const Position).value;
            const elapsed_seconds = @as(f32, @floatFromInt(step + 1)) * dt;

            if ((step + 1) % 60 == 0 or position.y <= 0.0) {
                std.debug.print("t={d:.2}s: position=({d:.2}, {d:.2})\n", .{
                    elapsed_seconds,
                    position.x,
                    position.y,
                });
            }

            if (position.y <= 0) {
                break :simulation;
            }
        }
    }
}
