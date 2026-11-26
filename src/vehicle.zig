//! Vehicles: a chassis body on wheels that are casts, not bodies, after Bullet's
//! `btRaycastVehicle` and Rapier's `DynamicRayCastVehicleController`. Each fixed step a ball the
//! wheel's size is cast down each suspension; a spring and damper push the chassis up off what
//! it meets, and friction impulses at the contact drive, brake and hold it from sliding, within
//! the grip the spring's load gives. The ground is pushed back as hard. 3D only.
//!
//! The chassis faces -z with +y up and +x to its right, in its own space.
const std = @import("std");
const body = @import("body.zig");
const math = @import("math.zig");
const ColliderHandle = @import("collider.zig").ColliderHandle;

pub const VehicleHandle = math.Handle(.vehicle);

/// Vehicles in 3D space `S` (a `space.Space`).
pub fn Vehicles(comptime S: type) type {
    const T = S.Real;
    const Vec = S.Vec;
    const State = body.Bodies(S).State;
    // Passes over the wheels' friction each step, so wheels sharing a chassis agree.
    const friction_passes = 4;

    return struct {
        pub const max_wheels = 16;
        pub const max_gears = 8;

        /// The whole vehicle's drive. Forces are shared among the wheels that take them. The
        /// defaults drive a 3000 kg truck.
        pub const Config = struct {
            /// Pushing forward at full throttle, in N.
            engine_force_n: T = 12000,
            /// The engine stops pushing past this, in m/s.
            max_speed_mps: T = 30,
            /// Reverse's share of the engine's force and top speed.
            reverse_factor: T = 0.5,
            /// Braking at full brake, in N.
            brake_force_n: T = 30000,
            handbrake_force_n: T = 20000,
            /// The share of a wheel's load that holds it back as it rolls.
            rolling_resistance: T = 0.015,
            /// Air holding it back: this times its speed squared, in N·s²/m².
            drag_n_s2_per_m2: T = 2.5,
            /// At this speed, in m/s, steering turns the wheels half as far; 0 keeps them turning fully.
            steer_falloff_mps: T = 25,
            /// Forward gears, lowest first, each as many times top gear's pull as it gives: in
            /// gear `i` (from 1) the engine pushes up to `engine_force_n * gear_ratios[i - 1]`,
            /// fading out towards that gear's top speed, `max_speed_mps / gear_ratios[i - 1]`.
            /// The first zero ends the list; none at all is one gear, pushing evenly up to
            /// `max_speed_mps`, with no clutch. Reverse uses first gear's ratio.
            gear_ratios: [max_gears]T = @splat(0),
            /// Shifts by itself; otherwise only as `Vehicle.shift` asks. Either way it shifts
            /// into reverse, and back, when stopped and the throttle asks to go the other way.
            automatic: bool = true,
            /// How long the clutch is out on each shift, with nothing driving the wheels, in s.
            shift_s: T = 0.5,
            /// Shifting by itself, up past this share of a gear's top speed (with the throttle
            /// down), and down below this share.
            shift_up_share: T = 0.9,
            shift_down_share: T = 0.4,
            /// The engine's speed idling and at the top of each gear, in rpm, for its readers.
            idle_rpm: T = 800,
            max_rpm: T = 2600,

            /// How many forward gears it has; 0 without a gearbox.
            pub fn gearCount(self: Config) u8 {
                for (self.gear_ratios, 0..) |ratio, i| if (!(ratio > 0)) return @intCast(i);
                return max_gears;
            }
        };

        pub const WheelConfig = struct {
            /// Where its suspension is mounted on the chassis, in the chassis's own space.
            position: Vec = .zero,
            radius_m: T = 0.5,
            /// The spring's length unloaded, from the mount to the wheel's centre.
            suspension_rest_m: T = 0.5,
            /// How far the spring squashes or stretches either side of its rest length.
            suspension_travel_m: T = 0.3,
            stiffness_n_per_m: T = 60000,
            /// N·s/m against the spring squashing, and stretching back.
            damping_compression: T = 4500,
            damping_rebound: T = 6500,
            max_suspension_force_n: T = 60000,
            /// Grip along the wheel and across it, as shares of its load.
            forward_friction: T = 1.2,
            side_friction: T = 1.0,
            steers: bool = false,
            driven: bool = false,
            brakes: bool = true,
            handbrake: bool = false,
            max_steer_rad: T = 0.6,
            /// Wheels with the same axle number and an anti-roll stiffness are linked by an
            /// anti-roll bar of the larger stiffness, in N/m.
            axle: u8 = 0,
            anti_roll_n_per_m: T = 0,
        };

        /// What the driver asks for, read each step.
        pub const Input = struct {
            /// -1 to 1: forward, or backward when negative.
            throttle: T = 0,
            /// -1 to 1: right when positive.
            steer: T = 0,
            /// 0 to 1.
            brake: T = 0,
            handbrake: bool = false,

            fn clamped(self: Input) Input {
                return .{
                    .throttle = std.math.clamp(self.throttle, -1, 1),
                    .steer = std.math.clamp(self.steer, -1, 1),
                    .brake = std.math.clamp(self.brake, 0, 1),
                    .handbrake = self.handbrake,
                };
            }
        };

        /// How a wheel was in the last step.
        pub const WheelState = struct {
            grounded: bool = false,
            /// What it stands on, while grounded.
            contact: ?ColliderHandle = null,
            contact_point: Vec = .zero,
            contact_normal: Vec = .zero,
            /// The spring's length, from the mount to the wheel's centre.
            suspension_m: T = 0,
            suspension_force_n: T = 0,
            /// 0 while gripping; towards 1 the more of what was asked of it slid.
            slip: T = 0,
            spin_rad_per_s: T = 0,
            /// How far it has rolled, in radians, within a turn.
            spin_rad: T = 0,
            /// Turned right when positive.
            steer_rad: T = 0,
        };

        pub const Wheel = struct {
            config: WheelConfig,
            state: WheelState = .{},
            /// The caller's own tag for it.
            user_data: u64 = 0,
        };

        pub const Vehicle = struct {
            body: body.BodyHandle,
            config: Config = .{},
            input: Input = .{},
            wheels: [max_wheels]Wheel = undefined,
            wheel_count: usize = 0,
            /// Forward speed in the last step, in m/s; negative reversing.
            speed_mps: T = 0,
            /// The input it last stepped with: asleep, it wakes when that changes.
            stepped_input: Input = .{},
            /// The gear it is in: 1 and up forward, -1 reverse. Without gears, always 1.
            gear: i8 = 1,
            /// While the clutch is out, the gear it is shifting into, and how long until it is in.
            shifting_to: i8 = 1,
            shift_left_s: T = 0,
            /// The engine's speed, in rpm (`Config.idle_rpm` to `max_rpm`).
            rpm: T = 0,
            /// A shift asked for and not yet made: 1 up, -1 down. `shift` sets it.
            shift_asked: i8 = 0,

            /// Asks for the next gear up (`1`) or down (`-1`) without `Config.automatic`.
            pub fn shift(self: *Vehicle, way: i8) void {
                self.shift_asked = std.math.sign(way);
            }

            /// Whether the clutch is out, mid-shift.
            pub fn shifting(self: *const Vehicle) bool {
                return self.shift_left_s > 0;
            }

            pub fn wheelSlice(self: *Vehicle) []Wheel {
                return self.wheels[0..self.wheel_count];
            }
        };

        /// What a wheel's cast met.
        pub const Contact = struct {
            collider: ColliderHandle,
            /// The body it is on, null for scenery.
            ground: ?*State,
            point: Vec,
            normal: Vec,
            /// How far the wheel's centre moved down from the mount before touching.
            distance_m: T,
        };

        /// One wheel's friction for a step, solved over several passes.
        const Grip = struct {
            offset: Vec,
            ground_offset: Vec,
            forward: Vec,
            right: Vec,
            forward_mass: T,
            right_mass: T,
            drive: T,
            brake_limit: T,
            forward_limit: T,
            right_limit: T,
            brake: T = 0,
            right_total: T = 0,
            forward_applied: T = 0,
            right_applied: T = 0,
            /// What was asked of it before grip limited it.
            wanted: T = 0,
        };

        /// Steps `vehicle` on `chassis` for `dt_s`: its wheels' suspension, drive and grip.
        /// `ground.castWheel(from, direction, max_m, radius_m) ?Contact` casts a wheel past the
        /// chassis. A sleeping chassis wakes only when the input changes.
        pub fn step(vehicle: *Vehicle, chassis: *State, dt_s: T, ground: anytype) void {
            const input = vehicle.input.clamped();
            defer vehicle.stepped_input = input;
            if (chassis.sleeping and chassis.enabled) {
                if (std.meta.eql(input, vehicle.stepped_input) and vehicle.shift_asked == 0) return;
                chassis.wake();
            }
            if (!chassis.pushable()) return;

            const turn = chassis.pose.rotation;
            const up = turn.rotateVector(Vec.init(0, 1, 0));
            const down = up.negate();
            const ahead = turn.rotateVector(Vec.init(0, 0, -1));
            const speed = chassis.linear.dot(ahead);
            vehicle.speed_mps = speed;
            const wheels = vehicle.wheelSlice();
            const config = vehicle.config;

            const falloff: T = if (config.steer_falloff_mps > 0) 1 / (1 + @abs(speed) / config.steer_falloff_mps) else 1;
            var driven: T = 0;
            var braking: T = 0;
            var holding: T = 0;
            for (wheels) |wheel| {
                if (wheel.config.driven) driven += 1;
                if (wheel.config.brakes) braking += 1;
                if (wheel.config.handbrake) holding += 1;
            }
            const engine_n = drive(vehicle, input, speed, dt_s);

            // Suspension: where each wheel meets the ground, and how hard its spring pushes there.
            var contacts: [max_wheels]?Contact = @splat(null);
            for (wheels, contacts[0..wheels.len]) |*wheel, *contact| {
                const setup = wheel.config;
                const state = &wheel.state;
                state.steer_rad = if (setup.steers) input.steer * setup.max_steer_rad * falloff else 0;
                const longest = setup.suspension_rest_m + setup.suspension_travel_m;
                const shortest = @max(setup.suspension_rest_m - setup.suspension_travel_m, 0);
                const mount = chassis.pose.transformPoint(setup.position);
                contact.* = ground.castWheel(mount, down, longest, setup.radius_m);
                const met = contact.* orelse {
                    state.* = .{ .suspension_m = longest, .spin_rad_per_s = state.spin_rad_per_s, .spin_rad = state.spin_rad, .steer_rad = state.steer_rad };
                    continue;
                };
                const length = std.math.clamp(met.distance_m, shortest, longest);
                const squash_mps = -pointVelocity(chassis, met.ground, met.point).dot(up);
                const damping = if (squash_mps > 0) setup.damping_compression else setup.damping_rebound;
                state.grounded = true;
                state.contact = met.collider;
                state.contact_point = met.point;
                state.contact_normal = met.normal;
                state.suspension_m = length;
                state.suspension_force_n = setup.stiffness_n_per_m * (setup.suspension_rest_m - length) + damping * squash_mps;
            }
            antiRoll(wheels);
            for (wheels, contacts[0..wheels.len]) |*wheel, contact| {
                const met = contact orelse continue;
                const force_n = std.math.clamp(wheel.state.suspension_force_n, 0, wheel.config.max_suspension_force_n);
                wheel.state.suspension_force_n = force_n;
                push(chassis, met.ground, met.point, met.normal.scale(force_n * dt_s));
            }

            // Grip: drive and brakes along each wheel, holding against sliding across it.
            var grips: [max_wheels]Grip = undefined;
            for (wheels, contacts[0..wheels.len], grips[0..wheels.len]) |wheel, contact, *grip| {
                const met = contact orelse continue;
                const setup = wheel.config;
                const facing = S.Rotation.fromAxisAngle(up, -wheel.state.steer_rad).rotateVector(ahead);
                const forward = facing.sub(met.normal.scale(facing.dot(met.normal))).normalize();
                const right = forward.cross(met.normal);
                const load_n = wheel.state.suspension_force_n;
                const offset = met.point.sub(chassis.center());
                const ground_offset = if (met.ground) |g| met.point.sub(g.center()) else Vec.zero;
                var brake_n = config.rolling_resistance * load_n;
                if (setup.brakes and braking > 0) brake_n += config.brake_force_n * input.brake / braking;
                if (setup.handbrake and input.handbrake and holding > 0) brake_n += config.handbrake_force_n / holding;
                const forward_limit = setup.forward_friction * load_n * dt_s;
                grip.* = .{
                    .offset = offset,
                    .ground_offset = ground_offset,
                    .forward = forward,
                    .right = right,
                    .forward_mass = massAlong(chassis, met.ground, offset, ground_offset, forward),
                    .right_mass = massAlong(chassis, met.ground, offset, ground_offset, right),
                    .drive = if (setup.driven and driven > 0) std.math.clamp(engine_n / driven * dt_s, -forward_limit, forward_limit) else 0,
                    .brake_limit = brake_n * dt_s,
                    .forward_limit = forward_limit,
                    .right_limit = setup.side_friction * load_n * dt_s,
                };
            }
            for (0..friction_passes) |_| {
                for (contacts[0..wheels.len], grips[0..wheels.len]) |contact, *grip| {
                    const met = contact orelse continue;
                    const velocity = pointVelocity(chassis, met.ground, met.point);
                    // Brakes and rolling resistance hold it still along the wheel, as far as
                    // they can and the grip left after driving allows.
                    const brake_room_low = @max(-grip.brake_limit, -grip.forward_limit - grip.drive);
                    const brake_room_high = @min(grip.brake_limit, grip.forward_limit - grip.drive);
                    grip.brake = std.math.clamp(grip.brake - grip.forward_mass * velocity.dot(grip.forward), @min(brake_room_low, 0), @max(brake_room_high, 0));
                    const forward_total = grip.drive + grip.brake;
                    // Across it, what grip the drive and brakes leave, on the friction circle.
                    const used = if (grip.forward_limit > 0) forward_total / grip.forward_limit else 0;
                    const right_room = grip.right_limit * @sqrt(@max(1 - used * used, 0));
                    const right_wanted = grip.right_total - grip.right_mass * velocity.dot(grip.right);
                    grip.right_total = std.math.clamp(right_wanted, -right_room, right_room);
                    grip.wanted = @sqrt(forward_total * forward_total + right_wanted * right_wanted);
                    const impulse = grip.forward.scale(forward_total - grip.forward_applied).add(grip.right.scale(grip.right_total - grip.right_applied));
                    push(chassis, met.ground, met.point, impulse);
                    grip.forward_applied = forward_total;
                    grip.right_applied = grip.right_total;
                }
            }

            // Air holds it back by the square of its speed.
            const air_mps = chassis.linear.length();
            if (air_mps > 0) chassis.applyImpulse(chassis.linear.scale(-config.drag_n_s2_per_m2 * air_mps * dt_s), .zero);

            // What each wheel did, for its readers and its drawn spin.
            for (wheels, contacts[0..wheels.len], grips[0..wheels.len]) |*wheel, contact, grip| {
                const state = &wheel.state;
                if (contact) |met| {
                    const delivered = @sqrt(grip.forward_applied * grip.forward_applied + grip.right_applied * grip.right_applied);
                    state.slip = if (grip.wanted > 1e-6) std.math.clamp(1 - delivered / grip.wanted, 0, 1) else 0;
                    const locked = wheel.config.handbrake and input.handbrake;
                    state.spin_rad_per_s = if (locked) 0 else pointVelocity(chassis, met.ground, met.point).dot(grip.forward) / wheel.config.radius_m;
                } else {
                    state.slip = 0;
                    state.spin_rad_per_s *= @max(1 - dt_s, 0);
                }
                state.spin_rad = @mod(state.spin_rad + state.spin_rad_per_s * dt_s, std.math.tau);
            }
        }

        /// The engine's push along the vehicle this step, in N (backwards when negative), with
        /// the gearbox shifted as it needs.
        fn drive(vehicle: *Vehicle, input: Input, speed: T, dt_s: T) T {
            const config = vehicle.config;
            const gears = config.gearCount();
            if (gears == 0) {
                vehicle.gear = 1;
                vehicle.shift_asked = 0;
                const top_speed = if (input.throttle >= 0) config.max_speed_mps else config.max_speed_mps * config.reverse_factor;
                const pushing = if (@abs(speed) < top_speed or std.math.sign(speed) != std.math.sign(input.throttle)) input.throttle else 0;
                vehicle.rpm = config.idle_rpm + (config.max_rpm - config.idle_rpm) * std.math.clamp(@abs(speed) / top_speed, 0, 1);
                return config.engine_force_n * pushing * (if (input.throttle < 0) config.reverse_factor else 1);
            }
            vehicle.gear = std.math.clamp(vehicle.gear, -1, @as(i8, @intCast(gears)));
            if (vehicle.gear == 0) vehicle.gear = 1;

            // The clutch is out: nothing drives, and the engine falls to the next gear's speed.
            if (vehicle.shift_left_s > 0) {
                vehicle.shift_asked = 0;
                const target = rpmIn(config, vehicle.shifting_to, speed, input.throttle);
                vehicle.rpm += (target - vehicle.rpm) * @min(dt_s / vehicle.shift_left_s, 1);
                vehicle.shift_left_s -= dt_s;
                if (vehicle.shift_left_s <= 0) {
                    vehicle.shift_left_s = 0;
                    vehicle.gear = vehicle.shifting_to;
                }
                return 0;
            }

            const want = nextGear(vehicle, input, speed, gears);
            vehicle.shift_asked = 0;
            if (want != vehicle.gear and config.shift_s > 0) {
                vehicle.shifting_to = want;
                vehicle.shift_left_s = config.shift_s;
                return 0;
            }
            vehicle.gear = want;
            vehicle.rpm = rpmIn(config, vehicle.gear, speed, input.throttle);

            // Full pull until 85% of the gear's top speed, fading to none at it.
            const share = gearShare(config, vehicle.gear, speed);
            const pull = std.math.clamp((1 - share) / 0.15, 0, 1);
            const ratio = config.gear_ratios[0];
            return if (vehicle.gear > 0)
                config.engine_force_n * config.gear_ratios[@intCast(vehicle.gear - 1)] * pull * @max(input.throttle, 0)
            else
                -config.engine_force_n * ratio * config.reverse_factor * pull * @max(-input.throttle, 0);
        }

        /// The gear to be in: reverse and back when stopped and the throttle asks to go the
        /// other way, else the driver's ask or, shifting by itself, by speed.
        fn nextGear(vehicle: *const Vehicle, input: Input, speed: T, gears: u8) i8 {
            const config = vehicle.config;
            const gear = vehicle.gear;
            const stopped_mps = 0.5;
            if (gear > 0 and input.throttle < -0.05 and speed < stopped_mps) return -1;
            if (gear < 0 and input.throttle > 0.05 and speed > -stopped_mps) return 1;
            if (gear < 0) return gear;
            if (!config.automatic) return std.math.clamp(gear + vehicle.shift_asked, 1, @as(i8, @intCast(gears)));
            const share = gearShare(config, gear, speed);
            if (share > config.shift_up_share and gear < gears and input.throttle > 0.05) return gear + 1;
            if (share < config.shift_down_share and gear > 1) {
                // Straight to the highest gear that has the engine back in its range.
                var lower = gear - 1;
                while (lower > 1 and gearShare(config, lower, speed) < config.shift_down_share) lower -= 1;
                return lower;
            }
            return gear;
        }

        /// How far through `gear`'s speeds `speed` is: 0 stopped, 1 at its top.
        fn gearShare(config: Config, gear: i8, speed: T) T {
            if (gear < 0) return @max(-speed, 0) / (config.max_speed_mps * config.reverse_factor / config.gear_ratios[0]);
            const ratio = config.gear_ratios[@intCast(gear - 1)];
            return @max(speed, 0) / (config.max_speed_mps / ratio);
        }

        /// The engine's speed in `gear` at `speed`: with the throttle down and slow, it slips
        /// the clutch and revs up to a third of its range.
        fn rpmIn(config: Config, gear: i8, speed: T, throttle: T) T {
            const range = config.max_rpm - config.idle_rpm;
            const turning = std.math.clamp(gearShare(config, gear, speed), 0, 1);
            const slipping = @abs(throttle) * 0.35;
            return config.idle_rpm + range * @max(turning, slipping);
        }

        /// Anti-roll bars: each pair of wheels on an axle shares the difference in how far
        /// their springs squash, so the chassis leans less in turns.
        fn antiRoll(wheels: []Wheel) void {
            for (wheels, 0..) |*left, i| {
                if (left.config.anti_roll_n_per_m <= 0) continue;
                for (wheels[i + 1 ..]) |*right| {
                    if (right.config.axle != left.config.axle or right.config.anti_roll_n_per_m <= 0) continue;
                    const stiffness = @max(left.config.anti_roll_n_per_m, right.config.anti_roll_n_per_m);
                    const squash_left = left.config.suspension_rest_m - left.state.suspension_m;
                    const squash_right = right.config.suspension_rest_m - right.state.suspension_m;
                    const shared = stiffness * (squash_left - squash_right);
                    if (left.state.grounded) left.state.suspension_force_n += shared;
                    if (right.state.grounded) right.state.suspension_force_n -= shared;
                    break;
                }
            }
        }

        /// How fast the chassis moves at `point` against the ground beneath it.
        fn pointVelocity(chassis: *const State, ground: ?*State, point: Vec) Vec {
            const own = chassis.velocityAt(point.sub(chassis.center()));
            const under = if (ground) |g| if (g.moving()) g.velocityAt(point.sub(g.center())) else Vec.zero else Vec.zero;
            return own.sub(under);
        }

        /// `impulse` on the chassis at `point`, and back on a ground that moves.
        fn push(chassis: *State, ground: ?*State, point: Vec, impulse: Vec) void {
            chassis.applyImpulse(impulse, point.sub(chassis.center()));
            if (ground) |g| if (g.pushable()) g.applyImpulse(impulse.negate(), point.sub(g.center()));
        }

        /// The impulse along `direction` that changes the chassis's speed against the ground by
        /// 1 m/s at the contact.
        fn massAlong(chassis: *const State, ground: ?*State, offset: Vec, ground_offset: Vec, direction: Vec) T {
            var inverse = chassis.inv_mass + S.angularDot(S.cross(offset, direction), chassis.turnBy(S.cross(offset, direction)));
            if (ground) |g| if (g.pushable()) {
                inverse += g.inv_mass + S.angularDot(S.cross(ground_offset, direction), g.turnBy(S.cross(ground_offset, direction)));
            };
            return if (inverse > 0) 1 / inverse else 0;
        }
    };
}

const TestWorld = @import("physics.zig").Physics(.three, f32);
const TestVec = TestWorld.Vec;
const V = Vehicles(TestWorld.S);

/// A 3000 kg truck, its front wheels steering and its rear ones driven and holding the
/// handbrake, or a 1500 kg trailer on two wheels at the back, its front end resting on a hitch.
fn addVehicle(world: *TestWorld, at: TestWorld.Pose, kind: enum { truck, trailer }) !struct { body: body.BodyHandle, vehicle: VehicleHandle } {
    const truck = kind == .truck;
    const chassis = try world.addBody(.{ .kind = .dynamic, .pose = at, .mass_kg = if (truck) 3000 else 1500, .center_of_mass_m = TestVec.init(0, -0.3, 0) }, 0);
    _ = try world.addCollider(.{ .shape = .{ .cuboid = .{ .half_extents = if (truck) TestVec.init(1.1, 0.5, 2.8) else TestVec.init(1.1, 0.4, 3) } } }, chassis, 0);
    var vehicle: V.Vehicle = .{ .body = chassis };
    const corners: []const [2]f32 = if (truck) &.{ .{ -1, -1.8 }, .{ 1, -1.8 }, .{ -1, 1.8 }, .{ 1, 1.8 } } else &.{ .{ -1, 1.8 }, .{ 1, 1.8 } };
    for (corners, 0..) |corner, i| {
        const front = corner[1] < 0;
        vehicle.wheels[i] = .{ .config = .{
            .position = TestVec.init(corner[0], -0.3, corner[1]),
            .steers = truck and front,
            .driven = truck and !front,
            .handbrake = !front,
            .axle = if (front) 0 else 1,
            .anti_roll_n_per_m = 20000,
        } };
    }
    vehicle.wheel_count = corners.len;
    return .{ .body = chassis, .vehicle = try world.addVehicle(vehicle, 0) };
}

fn stepFor(world: *TestWorld, seconds: f32) !void {
    for (0..@intFromFloat(seconds * 120)) |_| try world.update(1.0 / 120.0);
}

test "a truck settles at its loaded suspension height, reaches 25 m/s on a heightfield without bouncing, and brakes to a stop" {
    var world = try TestWorld.init(std.testing.allocator, .{ .gravity = TestVec.init(0, -9.81, 0), .fixed_step_s = 1.0 / 120.0 });
    defer world.deinit();
    // Ground 40 m wide and 600 m long, gently rolling.
    const columns = 11;
    const rows = 151;
    var heights: [columns * rows]f32 = undefined;
    for (0..rows) |row| for (0..columns) |column| {
        heights[row * columns + column] = 0.15 * @sin(@as(f32, @floatFromInt(row)) * 0.7);
    };
    _ = try world.addCollider(.{ .shape = .{ .heightfield = .init(&heights, columns, 4, 4) } }, null, 0);
    const truck = try addVehicle(&world, TestWorld.Pose.fromTranslation(TestVec.init(20, 1.5, 590)), .truck);
    const wheels = world.vehicleOf(truck.vehicle).?.wheelSlice();

    // At rest each spring squashes under its share of the weight: front and back carry alike.
    try stepFor(&world, 4);
    const share_n = 3000 * 9.81 / 4.0;
    for (wheels) |wheel| {
        try std.testing.expect(wheel.state.grounded);
        try std.testing.expectApproxEqAbs(share_n, wheel.state.suspension_force_n, share_n * 0.05);
        try std.testing.expectApproxEqAbs(@as(f32, 0.5 - share_n / 60000.0), wheel.state.suspension_m, 0.02);
    }

    // Full throttle: ahead is -z. It holds its height over the bumps the whole way.
    world.vehicleOf(truck.vehicle).?.input = .{ .throttle = 1 };
    var lowest: f32 = std.math.inf(f32);
    var highest: f32 = -std.math.inf(f32);
    var seconds: f32 = 0;
    while (world.vehicleOf(truck.vehicle).?.speed_mps < 25) : (seconds += 1.0 / 120.0) {
        if (seconds > 15) return error.TestTooSlow;
        try world.update(1.0 / 120.0);
        const clearance = world.pose(truck.body).translation.y;
        lowest = @min(lowest, clearance);
        highest = @max(highest, clearance);
    }
    // 12 kN on 3 t is 4 m/s² before drag and rolling: about seven seconds.
    try std.testing.expect(seconds > 6 and seconds < 10);
    try std.testing.expect(highest - lowest < 0.5);
    try std.testing.expect(@abs(world.pose(truck.body).translation.x - 20) < 0.5);

    // Full brake: 30 kN on 3 t would stop it from 25 m/s in 31 m, but braking tips its weight
    // forward off the rear wheels, which then slide a little.
    const braking_from = world.pose(truck.body).translation.z;
    world.vehicleOf(truck.vehicle).?.input = .{ .brake = 1 };
    try stepFor(&world, 5);
    const stopped_in = braking_from - world.pose(truck.body).translation.z;
    try std.testing.expect(stopped_in > 28 and stopped_in < 38);
    try std.testing.expect(world.velocity(truck.body).linear.length() < 0.05);
}

test "a truck with a gearbox pulls hardest in first, shifts up through its gears with the clutch out between, and backs up in reverse" {
    var world = try TestWorld.init(std.testing.allocator, .{ .gravity = TestVec.init(0, -9.81, 0), .fixed_step_s = 1.0 / 120.0 });
    defer world.deinit();
    _ = try world.addCollider(.{ .shape = .{ .cuboid = .{ .half_extents = TestVec.init(50, 1, 800) } }, .local_pose = TestWorld.Pose.fromTranslation(TestVec.init(0, -1, -700)) }, null, 0);
    const truck = try addVehicle(&world, TestWorld.Pose.fromTranslation(TestVec.init(0, 1.2, 0)), .truck);
    const vehicle = world.vehicleOf(truck.vehicle).?;
    vehicle.config.engine_force_n = 6000;
    vehicle.config.gear_ratios[0..4].* = .{ 3.5, 2.2, 1.45, 1 };
    vehicle.config.shift_s = 0.5;
    try stepFor(&world, 2);
    try std.testing.expectEqual(@as(i8, 1), vehicle.gear);

    // First gear: 21 kN on 3 t is about 7 m/s² before rolling and drag.
    vehicle.input = .{ .throttle = 1 };
    try stepFor(&world, 1);
    try std.testing.expect(vehicle.speed_mps > 5.5 and vehicle.speed_mps < 7.5);

    // It reaches every gear, and over each shift, with the clutch out, it slows.
    var highest: i8 = 1;
    var shifts: usize = 0;
    var clutch_gain: f32 = -std.math.inf(f32);
    var shift_from: f32 = 0;
    var seconds: f32 = 0;
    while (seconds < 40 and vehicle.speed_mps < 27) : (seconds += 1.0 / 120.0) {
        const before = vehicle.speed_mps;
        const was_shifting = vehicle.shifting();
        try world.update(1.0 / 120.0);
        if (!was_shifting and vehicle.shifting()) {
            shifts += 1;
            shift_from = before;
        }
        if (was_shifting and !vehicle.shifting()) clutch_gain = @max(clutch_gain, vehicle.speed_mps - shift_from);
        highest = @max(highest, vehicle.gear);
    }
    try std.testing.expect(vehicle.speed_mps >= 27);
    try std.testing.expectEqual(@as(i8, 4), highest);
    try std.testing.expectEqual(@as(usize, 3), shifts);
    try std.testing.expect(clutch_gain < 0);
    try std.testing.expect(vehicle.rpm > vehicle.config.idle_rpm and vehicle.rpm <= vehicle.config.max_rpm);

    // Brake to a stop: it drops back to first. Then throttle back: it shifts into reverse.
    vehicle.input = .{ .brake = 1 };
    try stepFor(&world, 6);
    try std.testing.expectEqual(@as(i8, 1), vehicle.gear);
    vehicle.input = .{ .throttle = -1 };
    try stepFor(&world, 2);
    try std.testing.expectEqual(@as(i8, -1), vehicle.gear);
    try std.testing.expect(vehicle.speed_mps < -2);

    // Shifting by hand, it stays in the gear asked for.
    vehicle.input = .{ .brake = 1 };
    try stepFor(&world, 3);
    vehicle.config.automatic = false;
    vehicle.input = .{ .throttle = 1 };
    try stepFor(&world, 3);
    try std.testing.expectEqual(@as(i8, 1), vehicle.gear);
    vehicle.shift(1);
    try stepFor(&world, 1);
    try std.testing.expectEqual(@as(i8, 2), vehicle.gear);
}

test "a truck tows a trailer on a ball hitch through a tight turn, and both hold still on a slope with their handbrakes on" {
    for ([_]enum { turn, slope }{ .turn, .slope }) |trial| {
        // The slope is gravity tilted 15° back along +z, onto flat ground.
        const tilt: f32 = if (trial == .slope) 15.0 * std.math.pi / 180.0 else 0;
        var world = try TestWorld.init(std.testing.allocator, .{ .gravity = TestVec.init(0, -@cos(tilt), @sin(tilt)).scale(9.81), .fixed_step_s = 1.0 / 120.0 });
        defer world.deinit();
        _ = try world.addCollider(.{ .shape = .{ .half_space = .{ .normal = TestVec.init(0, 1, 0) } } }, null, 0);
        const truck = try addVehicle(&world, TestWorld.Pose.fromTranslation(TestVec.init(0, 1.2, 0)), .truck);
        const trailer = try addVehicle(&world, TestWorld.Pose.fromTranslation(TestVec.init(0, 1.2, 7)), .trailer);
        const hitch = try world.addJoint(.{
            .kind = .{ .ball = .{ .axis = TestVec.init(0, 0, 1), .cone_rad = 1.3 } },
            .anchor_a = TestVec.init(0, -0.3, 3.2),
            .anchor_b = TestVec.init(0, -0.3, -3.8),
        }, truck.body, trailer.body, 0);
        try stepFor(&world, 1);

        switch (trial) {
            .turn => {
                world.vehicleOf(truck.vehicle).?.input = .{ .throttle = 0.5, .steer = 1 };
                var steadiest: f32 = 0;
                for (0..12 * 120) |step_index| {
                    try world.update(1.0 / 120.0);
                    // No jitter: the trailer never spins up once it is following.
                    if (step_index > 4 * 120) steadiest = @max(steadiest, world.velocity(trailer.body).angular.length());
                    const gap = world.jointState(hitch).?;
                    const apart = world.pose(truck.body).transformPoint(gap.stored.joint.anchor_a).sub(world.pose(trailer.body).transformPoint(gap.stored.joint.anchor_b));
                    try std.testing.expect(apart.length() < 0.05);
                }
                try std.testing.expect(steadiest < 1.5);
                // Both upright, and the truck went round.
                try std.testing.expect(world.pose(truck.body).rotation.rotateVector(TestVec.init(0, 1, 0)).y > 0.95);
                try std.testing.expect(world.pose(trailer.body).rotation.rotateVector(TestVec.init(0, 1, 0)).y > 0.95);
                try std.testing.expect(world.pose(truck.body).translation.length() > 5);
            },
            .slope => {
                world.vehicleOf(truck.vehicle).?.input = .{ .handbrake = true };
                world.vehicleOf(trailer.vehicle).?.input = .{ .handbrake = true };
                try stepFor(&world, 1);
                const parked = world.pose(truck.body).translation;
                try stepFor(&world, 5);
                try std.testing.expect(world.pose(truck.body).translation.sub(parked).length() < 0.05);
                try std.testing.expect(world.isSleeping(truck.body));
                try std.testing.expect(world.isSleeping(trailer.body));
                // Let go, they roll back down.
                world.vehicleOf(truck.vehicle).?.input = .{};
                world.vehicleOf(trailer.vehicle).?.input = .{};
                try stepFor(&world, 2);
                try std.testing.expect(world.pose(truck.body).translation.z - parked.z > 1);
            },
        }
    }
}
