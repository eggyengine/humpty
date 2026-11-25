//! Joints in a clutch world: a `Joint` on an entity with a `RigidBody` holds its body to another
//! entity's, or to the world, through humpty's joints. Built when new, rebuilt when edited or
//! when either body is, let go when removed, so a game hitches and unhitches by adding and
//! removing one. 3D only.
const std = @import("std");
const clutch = @import("clutch");
const emath = @import("eggenvector");
const ecs = @import("ecs.zig");
const BodyHandle = @import("body.zig").BodyHandle;
const JointHandle = @import("joint.zig").JointHandle;

const Vec3 = emath.Vec3;
const log = std.log.scoped(.physics);

/// Holds this entity's body to `other`'s at an anchor on each, each in its own entity's space.
pub const Joint = struct {
    /// Marks it in the editor (nf-md-link_variant).
    pub const icon = "\u{f0339}";
    pub const category = "Physics";
    pub const dependencies = .{ecs.RigidBody};
    /// What it holds this body to; unset, the world, at `other_anchor_m` in world space. One
    /// without a body of its own is held where it is.
    other: ecs.EntityRef = .{},
    kind: Kind = .{ .ball = .{} },
    anchor_m: Vec3 = .zero,
    other_anchor_m: Vec3 = .zero,
    /// It lets go when pulled harder than this, in N; unset, never.
    break_force_n: ?f32 = null,

    pub const Kind = union(enum) {
        /// Welded together.
        fixed,
        /// Turns freely, as a trailer's hitch. With a cone limit, `axis` stays within that angle
        /// of where it pointed on the other body, so a trailer can't fold into its cab.
        ball: struct { axis: Vec3 = .init(0, 0, -1), cone_limit_deg: ?f32 = null },
        /// Turns about `axis` only, as a folding wing or ramp: limits and motor in degrees.
        hinge: struct { axis: Vec3 = .init(1, 0, 0), limits_deg: ?Limits = null, motor: Motor = .off },
        /// Slides along `axis` only, as a deck section sliding out: limits and motor in metres.
        slider: struct { axis: Vec3 = .init(0, 0, -1), limits_m: ?Limits = null, motor: Motor = .off },
    };

    /// How far it may go either way from where it was made, the other body against this one.
    pub const Limits = struct { lower: f32 = -45, upper: f32 = 45 };

    /// Drives the other body against this one, pushing with up to `max` (N·m for a hinge, N for
    /// a slider).
    pub const Motor = union(enum) {
        off,
        /// At `speed`: deg/s for a hinge, m/s for a slider.
        velocity: struct { speed: f32 = 30, max: f32 = 5000 },
        /// To `target` (deg or m from where it was made) at up to `speed`, and holds it there.
        position: struct { target: f32 = 0, speed: f32 = 30, max: f32 = 5000 },
    };
};

/// How a joint was in the last step, kept up to date by physics: add it to read it.
pub const JointState = struct {
    pub const category = "Physics";
    broken: bool = false,
    /// A hinge's angle in degrees, or a slider's offset in metres, from where it was made.
    position: f32 = 0,
    /// How hard it holds its anchors together, in N.
    force_n: f32 = 0,
};

/// Joints for physics plugin `P` (an `ecs.Plugin`).
pub fn Sync(comptime P: type) type {
    const World = P.World;
    const T = World.S.Real;
    const Joints = World.Joints;
    const degrees: T = std.math.pi / 180.0;

    return struct {
        /// A joint's handle, and what it was built from, so an edit rebuilds it.
        pub const Link = struct {
            handle: ?JointHandle = null,
            source: Joint,
            a: BodyHandle,
            b: ?BodyHandle,
        };

        pub fn install(world: *clutch.World) void {
            world.onRemove(Link, unlink);
        }

        /// Builds joints for new and edited `Joint`s, and lets go of those removed.
        pub fn sync(res: *P.Resource, world: *clutch.World) !void {
            const gpa = res.world.gpa;
            var fresh: std.ArrayList(struct { entity: clutch.EntityId, link: Link }) = .empty;
            defer fresh.deinit(gpa);
            var query = world.query(.{*const Joint});
            while (query.next()) |view| {
                const entity = view.entity;
                const joint = view.get(*const Joint).*;
                // Both bodies are built first; until then it waits.
                const a = (world.get(entity, P.Link) orelse continue).body orelse continue;
                var held = joint;
                const b: ?BodyHandle = if (joint.other.entity) |other| P.bodyOf(world, other) else null;
                // Something without a body is held to where its anchor is now.
                if (joint.other.entity) |other| if (b == null) {
                    const transform = (world.get(other, P.PluginHost.Transform) orelse continue).*;
                    const place = P.PluginHost.place(world, other, transform);
                    held.other_anchor_m = place.position.add(place.rotation.rotateVector(joint.other_anchor_m.mul(place.scale)));
                };
                if (world.get(entity, Link)) |link| {
                    const live = if (link.handle) |handle| res.world.contains(handle) else false;
                    if (live and std.meta.eql(link.source, joint) and link.a == a and std.meta.eql(link.b, b)) continue;
                    if (live) res.world.removeJoint(link.handle.?);
                    link.* = build(res, entity, joint, held, a, b);
                } else try fresh.append(gpa, .{ .entity = entity, .link = build(res, entity, joint, held, a, b) });
            }
            for (fresh.items) |made| try world.addComponent(made.entity, made.link);

            var orphans: std.ArrayList(clutch.EntityId) = .empty;
            defer orphans.deinit(gpa);
            var unheld = world.query(.{ *const Link, clutch.Without(Joint) });
            while (unheld.next()) |view| try orphans.append(gpa, view.entity);
            for (orphans.items) |entity| try world.removeComponent(entity, Link);
        }

        fn build(res: *P.Resource, entity: clutch.EntityId, source: Joint, joint: Joint, a: BodyHandle, b: ?BodyHandle) Link {
            const handle = res.world.addJoint(convert(joint), a, b, P.dataOf(entity)) catch |err| {
                log.err("cannot add a joint: {t}", .{err});
                return .{ .source = source, .a = a, .b = b };
            };
            return .{ .handle = handle, .source = source, .a = a, .b = b };
        }

        fn convert(joint: Joint) Joints.Joint {
            return .{
                .kind = switch (joint.kind) {
                    .fixed => .fixed,
                    .ball => |ball| .{ .ball = .{ .axis = P.toVec(ball.axis), .cone_rad = if (ball.cone_limit_deg) |deg| deg * degrees else null } },
                    .hinge => |hinge| .{ .hinge = .{ .axis = P.toVec(hinge.axis), .limits = limits(hinge.limits_deg, degrees), .motor = motor(hinge.motor, degrees) } },
                    .slider => |slider| .{ .slider = .{ .axis = P.toVec(slider.axis), .limits = limits(slider.limits_m, 1), .motor = motor(slider.motor, 1) } },
                },
                .anchor_a = P.toVec(joint.anchor_m),
                .anchor_b = P.toVec(joint.other_anchor_m),
                .break_force_n = if (joint.break_force_n) |n| n else null,
            };
        }

        fn limits(range: ?Joint.Limits, unit: T) ?Joints.Limits {
            const found = range orelse return null;
            return .{ .lower = found.lower * unit, .upper = found.upper * unit };
        }

        fn motor(drive: Joint.Motor, unit: T) Joints.Motor {
            return switch (drive) {
                .off => .off,
                .velocity => |v| .{ .velocity = .{ .speed = v.speed * unit, .max = v.max } },
                .position => |p| .{ .position = .{ .target = p.target * unit, .speed = p.speed * unit, .max = p.max } },
            };
        }

        /// Fills each `JointState` from its joint's last step.
        pub fn writeBack(res: *P.Resource, world: *clutch.World) void {
            var query = world.query(.{ *const Link, *JointState });
            while (query.next()) |view| {
                const link = view.get(*const Link);
                const stored = res.world.jointState(link.handle orelse continue) orelse continue;
                const unit: T = if (link.source.kind == .hinge) degrees else 1;
                view.get(*JointState).* = .{
                    .broken = stored.stored.broken,
                    .position = @floatCast(stored.stored.position / unit),
                    .force_n = @floatCast(stored.stored.force_n),
                };
            }
        }

        fn unlink(world: *clutch.World, entity: clutch.EntityId) void {
            const res = world.getResource(P.Resource) orelse return;
            if (!res.live) return;
            if ((world.get(entity, Link) orelse return).handle) |handle| res.world.removeJoint(handle);
        }
    };
}
