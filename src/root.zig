const std = @import("std");
const emath = @import("eggenvector");
const clutch = @import("clutch");

const Position = struct { value: emath.Vec2 };
const Velocity = struct { value: emath.Vec2 };

const Time = struct {
    delta_seconds: f32,
};

const Gravity = struct { value: emath.Vec2 = .{ .x = 0.0, .y = -9.81 } };

const Force = struct {
    value: emath.Vec2 = .zero,
};

const Mass = struct {
    value: f32,
};

fn physicsStep(
    query: clutch.Query(.{
        *Position,
        *Velocity,
        *Force,
        *Mass,
    }),
    time: clutch.Res(Time),
    gravity: clutch.Res(Gravity),
) !void {
    const dt = time.delta_seconds;
    const grav_force = gravity.value;

    var iter = query;
    while (iter.next()) |view| {
        const position = &view.get(*Position).value;
        const velocity = &view.get(*Velocity).value;
        const force = &view.get(*Force).value;
        const mass = &view.get(*Mass).value;

        const acceleration = grav_force.add(force.div(emath.Vec2.splat(mass.*)));

        velocity.x += acceleration.x * dt;
        velocity.y += acceleration.y * dt;

        position.x += velocity.x * dt;
        position.y += velocity.y * dt;

        force.* = emath.Vec2.zero;
    }
}

fn applyThrust(query: clutch.Query(.{*Force})) !void {
    var iter = query;
    while (iter.next()) |view| {
        const force = &view.get(*Force).value;
        force.* = force.add(emath.Vec2.init(0, 20));
    }
}

test "the ball is affected by gravity" {
    var world = clutch.World.init(std.testing.allocator);
    defer world.deinit();

    _ = try world.spawn(.{
        Position{ .value = emath.Vec2.init(0, 100) },
        Velocity{ .value = emath.Vec2.zero },
        Force{ .value = emath.Vec2.zero },
        Mass{ .value = 1.0 },
    });

    // world.addSchedule(clutch.Schedule(.Update, .{
    //     applyThrust,
    // }));
    world.addSchedule(clutch.Schedule(.PostUpdate, .{physicsStep}));

    const dt: f32 = 1.0 / 60.0;
    try world.insertResource(Time{ .delta_seconds = dt });
    try world.insertResource(Gravity{});

    simulation: for (0..600) |step| {
        try world.runStage(.Update);
        try world.runStage(.PostUpdate);
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
