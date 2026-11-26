//! Vehicles in a clutch world: a `Vehicle` on an entity with a `RigidBody` drives on the
//! `Wheel`s on the entities below it, as humpty's raycast vehicles do. The game sets its
//! `VehicleInput` each frame (or each step, from a `FixedStep`); each wheel's `WheelState` and
//! the vehicle's `VehicleState` are read back, and each wheel's visual entity is placed where
//! the wheel is, turned by its steering and spin. 3D only.
const std = @import("std");
const clutch = @import("clutch");
const emath = @import("eggenvector");
const ecs = @import("ecs.zig");
const BodyHandle = @import("body.zig").BodyHandle;
const VehicleHandle = @import("vehicle.zig").VehicleHandle;

const Vec3 = emath.Vec3;
const log = std.log.scoped(.physics);

/// Drives its entity's body on the `Wheel`s below it. It faces -z with +y up. The defaults
/// drive a 3000 kg truck.
pub const Vehicle = struct {
    /// Marks it in the editor (nf-md-car).
    pub const icon = "\u{f010b}";
    pub const category = "Physics";
    pub const dependencies = .{ ecs.RigidBody, VehicleInput };
    engine_force_n: f32 = 12000,
    max_speed_mps: f32 = 30,
    /// Reverse's share of the engine's force and top speed.
    reverse_factor: f32 = 0.5,
    brake_force_n: f32 = 30000,
    handbrake_force_n: f32 = 20000,
    /// The share of each wheel's load holding it back as it rolls.
    rolling_resistance: f32 = 0.015,
    /// Air holding it back: this times its speed squared, in N·s²/m².
    drag_n_s2_per_m2: f32 = 2.5,
    /// At this speed steering turns the wheels half as far; 0 keeps them turning fully.
    steer_falloff_mps: f32 = 25,
    /// Forward gears, lowest first, each as many times top gear's pull as it gives: in gear
    /// `i` (from 1) the engine pushes up to `engine_force_n * gear_ratios[i - 1]`, fading out
    /// towards `max_speed_mps / gear_ratios[i - 1]`. The first zero ends the list; none at all
    /// is one gear, pushing evenly up to `max_speed_mps`. Reverse uses first gear's ratio.
    gear_ratios: [8]f32 = @splat(0),
    /// Shifts by itself; otherwise as `VehicleInput.shift` asks.
    automatic: bool = true,
    /// How long the clutch is out on each shift, with nothing driving the wheels.
    shift_s: f32 = 0.5,
    /// Shifting by itself, up past this share of a gear's top speed, and down below this.
    shift_up_share: f32 = 0.9,
    shift_down_share: f32 = 0.4,
    /// The engine's speed idling and at the top of each gear, for `VehicleState.rpm`.
    idle_rpm: f32 = 800,
    max_rpm: f32 = 2600,
};

/// What the driver asks of a `Vehicle`, set by the game.
pub const VehicleInput = struct {
    pub const category = "Physics";
    /// -1 to 1, reversing when negative.
    throttle: f32 = 0,
    /// -1 to 1, right when positive.
    steer: f32 = 0,
    /// 0 to 1.
    brake: f32 = 0,
    handbrake: bool = false,
    /// Set to 1 for the next gear up or -1 for the next down (without `Vehicle.automatic`);
    /// physics takes it and sets it back to 0.
    shift: i8 = 0,
};

/// How a `Vehicle` went in the last step, kept up to date by physics: add it to read it.
pub const VehicleState = struct {
    pub const category = "Physics";
    /// Forward speed in m/s, negative reversing.
    speed_mps: f32 = 0,
    /// 1 and up forward, -1 reverse; always 1 without gears.
    gear: i8 = 1,
    /// How many forward gears it has; 0 without a gearbox.
    gear_count: u8 = 0,
    /// The clutch is out, shifting into `shifting_to`.
    shifting: bool = false,
    shifting_to: i8 = 1,
    rpm: f32 = 0,
};

/// A wheel of the `Vehicle` above it, its suspension mounted where this entity is. It needs no
/// `Collider`: it touches the ground by a cast.
pub const Wheel = struct {
    /// Marks it in the editor (nf-md-tire).
    pub const icon = "\u{f0896}";
    pub const category = "Physics";
    radius_m: f32 = 0.5,
    /// The spring's unloaded length, from the mount to the wheel's centre.
    suspension_rest_m: f32 = 0.5,
    /// How far it squashes or stretches either side of its rest length.
    suspension_travel_m: f32 = 0.3,
    stiffness_n_per_m: f32 = 60000,
    /// N·s/m against squashing, and against stretching back.
    damping_compression: f32 = 4500,
    damping_rebound: f32 = 6500,
    max_suspension_force_n: f32 = 60000,
    /// Grip along the wheel and across it, as shares of its load.
    forward_friction: f32 = 1.2,
    side_friction: f32 = 1.0,
    steers: bool = false,
    driven: bool = false,
    brakes: bool = true,
    handbrake: bool = false,
    max_steer_deg: f32 = 35,
    /// Wheels with the same axle number and an anti-roll stiffness share a bar of the larger.
    axle: u8 = 0,
    anti_roll_n_per_m: f32 = 0,
    /// The entity drawn as the wheel, placed each frame where the wheel is, steered and spun
    /// about its axle (x) from how it was turned against the vehicle when first seen.
    visual: ecs.EntityRef = .{},
};

/// How a wheel went in the last step, kept up to date by physics: add it to read it.
pub const WheelState = struct {
    pub const category = "Physics";
    grounded: bool = false,
    contact_entity: ?clutch.EntityId = null,
    /// The spring's length, from the mount to the wheel's centre.
    suspension_m: f32 = 0,
    suspension_force_n: f32 = 0,
    /// 0 while gripping; towards 1 the more it slides.
    slip: f32 = 0,
    spin_rad_per_s: f32 = 0,
    /// Turned right when positive.
    steer_deg: f32 = 0,
};

/// Vehicles for physics plugin `P` (an `ecs.Plugin`).
pub fn Sync(comptime P: type) type {
    const World = P.World;
    const T = World.S.Real;
    const Vehicles = World.Vehicles;
    const max_wheels = Vehicles.max_wheels;
    const degrees: T = std.math.pi / 180.0;

    return struct {
        /// A vehicle's handle and body, and how each wheel's visual was turned against the
        /// vehicle when first seen, by the wheel's place in the vehicle's list.
        pub const Link = struct {
            handle: ?VehicleHandle = null,
            body: BodyHandle,
            rests: [max_wheels]emath.UnitQuat = @splat(.identity()),
        };

        pub fn install(world: *clutch.World) void {
            world.onRemove(Link, unlink);
        }

        /// Builds vehicles for new `Vehicle`s, and hands each its settings, input and wheels.
        pub fn sync(res: *P.Resource, world: *clutch.World) !void {
            const gpa = res.world.gpa;
            var fresh: std.ArrayList(struct { entity: clutch.EntityId, link: Link }) = .empty;
            defer fresh.deinit(gpa);
            var query = world.query(.{ *const Vehicle, *const P.PluginHost.Transform });
            while (query.next()) |view| {
                const entity = view.entity;
                // The body is built first; until then it waits.
                const body = (world.get(entity, P.Link) orelse continue).body orelse continue;
                const link: *Link = if (world.get(entity, Link)) |found| found else blk: {
                    try fresh.append(gpa, .{ .entity = entity, .link = .{ .body = body } });
                    break :blk &fresh.items[fresh.items.len - 1].link;
                };
                const live = if (link.handle) |handle| res.world.contains(handle) else false;
                if (!live or link.body != body) {
                    if (live) res.world.removeVehicle(link.handle.?);
                    link.* = .{ .body = body };
                    link.handle = res.world.addVehicle(.{ .body = body }, P.dataOf(entity)) catch |err| {
                        log.err("cannot add a vehicle: {t}", .{err});
                        continue;
                    };
                }
                const vehicle = res.world.vehicleOf(link.handle.?).?;
                vehicle.config = configOf(view.get(*const Vehicle).*);
                if (world.get(entity, VehicleInput)) |input| take(vehicle, input);
                gatherWheels(world, entity, view.get(*const P.PluginHost.Transform).*, vehicle, link);
            }
            for (fresh.items) |made| try world.addComponent(made.entity, made.link);

            var orphans: std.ArrayList(clutch.EntityId) = .empty;
            defer orphans.deinit(gpa);
            var undriven = world.query(.{ *const Link, clutch.Without(Vehicle) });
            while (undriven.next()) |view| try orphans.append(gpa, view.entity);
            for (orphans.items) |entity| try world.removeComponent(entity, Link);
        }

        /// The `Wheel`s whose nearest body is `chassis`'s, in its list, each keeping how it was
        /// going and its visual's rest turn.
        fn gatherWheels(world: *clutch.World, chassis: clutch.EntityId, transform: P.PluginHost.Transform, vehicle: *World.Vehicle, link: *Link) void {
            const chassis_place = P.PluginHost.place(world, chassis, transform);
            const chassis_pose = P.poseOf(chassis_place);
            var wheels: [max_wheels]Vehicles.Wheel = undefined;
            var rests: [max_wheels]emath.UnitQuat = undefined;
            var count: usize = 0;
            var query = world.query(.{ *const Wheel, *const P.PluginHost.Transform });
            while (query.next()) |view| {
                if (count == max_wheels) {
                    log.warn("a vehicle has more than {d} wheels; the rest are left off", .{max_wheels});
                    break;
                }
                const owner = P.ownerOf(world, view.entity) orelse continue;
                if (!std.meta.eql(owner, chassis)) continue;
                const setup = view.get(*const Wheel).*;
                const place = P.PluginHost.place(world, view.entity, view.get(*const P.PluginHost.Transform).*);
                const data = P.dataOf(view.entity);
                wheels[count] = .{ .config = wheelConfig(setup, chassis_pose.inverseTransformPoint(P.toVec(place.position))), .user_data = data };
                rests[count] = restOf(world, setup, chassis_place);
                for (vehicle.wheelSlice(), 0..) |old, i| if (old.user_data == data) {
                    wheels[count].state = old.state;
                    rests[count] = link.rests[i];
                };
                count += 1;
            }
            @memcpy(vehicle.wheels[0..count], wheels[0..count]);
            vehicle.wheel_count = count;
            @memcpy(link.rests[0..count], rests[0..count]);
        }

        /// How `wheel`'s visual is turned against the vehicle at `chassis`.
        fn restOf(world: *clutch.World, wheel: Wheel, chassis: ecs.Place) emath.UnitQuat {
            const visual = wheel.visual.entity orelse return .identity();
            const transform = (world.get(visual, P.PluginHost.Transform) orelse return .identity()).*;
            return chassis.rotation.inverse().mul(P.PluginHost.place(world, visual, transform).rotation);
        }

        /// Hands each vehicle its `VehicleInput`, as a `FixedStep` may have set it.
        pub fn readInput(world: *clutch.World) void {
            const res = world.getResource(P.Resource) orelse return;
            var query = world.query(.{ *const Link, *VehicleInput });
            while (query.next()) |view| {
                const vehicle = res.world.vehicleOf(view.get(*const Link).handle orelse continue) orelse continue;
                take(vehicle, view.get(*VehicleInput));
            }
        }

        /// Hands `vehicle` the driver's `input`, taking a shift it asks for.
        fn take(vehicle: *World.Vehicle, input: *VehicleInput) void {
            vehicle.input = inputOf(input.*);
            if (input.shift != 0) vehicle.shift(input.shift);
            input.shift = 0;
        }

        /// Fills `VehicleState`s and `WheelState`s, and places each wheel's visual.
        pub fn writeBack(res: *P.Resource, world: *clutch.World) void {
            var query = world.query(.{*const Link});
            while (query.next()) |view| {
                const link = view.get(*const Link);
                const vehicle = res.world.vehicleOf(link.handle orelse continue) orelse continue;
                if (world.get(view.entity, VehicleState)) |state| state.* = .{
                    .speed_mps = @floatCast(vehicle.speed_mps),
                    .gear = vehicle.gear,
                    .gear_count = vehicle.config.gearCount(),
                    .shifting = vehicle.shifting(),
                    .shifting_to = vehicle.shifting_to,
                    .rpm = @floatCast(vehicle.rpm),
                };
                if (!res.world.contains(link.body)) continue;
                const chassis = res.world.pose(link.body);
                const turn = P.fromRotation(chassis.rotation);
                for (vehicle.wheelSlice(), link.rests[0..vehicle.wheel_count]) |wheel, rest| {
                    const entity = P.entityOf(wheel.user_data);
                    const state = wheel.state;
                    if (world.get(entity, WheelState)) |out| out.* = .{
                        .grounded = state.grounded,
                        .contact_entity = if (state.contact) |collider| if (res.world.contains(collider)) P.entityOf(res.world.colliderData(collider)) else null else null,
                        .suspension_m = @floatCast(state.suspension_m),
                        .suspension_force_n = @floatCast(state.suspension_force_n),
                        .slip = @floatCast(state.slip),
                        .spin_rad_per_s = @floatCast(state.spin_rad_per_s),
                        .steer_deg = @floatCast(state.steer_rad / degrees),
                    };
                    const visual = (world.get(entity, Wheel) orelse continue).visual.entity orelse continue;
                    const transform = world.get(visual, P.PluginHost.Transform) orelse continue;
                    const hub = chassis.transformPoint(wheel.config.position.add(World.Vec.init(0, -state.suspension_m, 0)));
                    const steer = emath.UnitQuat.fromAxisAngle(Vec3.unit_y, @floatCast(-state.steer_rad));
                    // Rolling ahead (-z) turns its top forward: backwards about x.
                    const spin = emath.UnitQuat.fromAxisAngle(Vec3.unit_x, @floatCast(-state.spin_rad));
                    P.PluginHost.setPlace(world, visual, transform, P.fromVec(hub), turn.mul(steer).mul(spin).mul(rest));
                    world.markChanged(visual, P.PluginHost.Transform);
                }
            }
        }

        fn configOf(vehicle: Vehicle) Vehicles.Config {
            return .{
                .engine_force_n = vehicle.engine_force_n,
                .max_speed_mps = vehicle.max_speed_mps,
                .reverse_factor = vehicle.reverse_factor,
                .brake_force_n = vehicle.brake_force_n,
                .handbrake_force_n = vehicle.handbrake_force_n,
                .rolling_resistance = vehicle.rolling_resistance,
                .drag_n_s2_per_m2 = vehicle.drag_n_s2_per_m2,
                .steer_falloff_mps = vehicle.steer_falloff_mps,
                .gear_ratios = ratios: {
                    var ratios: [Vehicles.max_gears]T = undefined;
                    for (&ratios, vehicle.gear_ratios) |*to, from| to.* = from;
                    break :ratios ratios;
                },
                .automatic = vehicle.automatic,
                .shift_s = vehicle.shift_s,
                .shift_up_share = vehicle.shift_up_share,
                .shift_down_share = vehicle.shift_down_share,
                .idle_rpm = vehicle.idle_rpm,
                .max_rpm = vehicle.max_rpm,
            };
        }

        fn inputOf(input: VehicleInput) Vehicles.Input {
            return .{ .throttle = input.throttle, .steer = input.steer, .brake = input.brake, .handbrake = input.handbrake };
        }

        fn wheelConfig(wheel: Wheel, position: World.Vec) Vehicles.WheelConfig {
            return .{
                .position = position,
                .radius_m = wheel.radius_m,
                .suspension_rest_m = wheel.suspension_rest_m,
                .suspension_travel_m = wheel.suspension_travel_m,
                .stiffness_n_per_m = wheel.stiffness_n_per_m,
                .damping_compression = wheel.damping_compression,
                .damping_rebound = wheel.damping_rebound,
                .max_suspension_force_n = wheel.max_suspension_force_n,
                .forward_friction = wheel.forward_friction,
                .side_friction = wheel.side_friction,
                .steers = wheel.steers,
                .driven = wheel.driven,
                .brakes = wheel.brakes,
                .handbrake = wheel.handbrake,
                .max_steer_rad = wheel.max_steer_deg * degrees,
                .axle = wheel.axle,
                .anti_roll_n_per_m = wheel.anti_roll_n_per_m,
            };
        }

        fn unlink(world: *clutch.World, entity: clutch.EntityId) void {
            const res = world.getResource(P.Resource) orelse return;
            if (!res.live) return;
            if ((world.get(entity, Link) orelse return).handle) |handle| res.world.removeVehicle(handle);
        }
    };
}
