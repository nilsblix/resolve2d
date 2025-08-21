const std = @import("std");
const Allocator = std.mem.Allocator;
const nmath = @import("../nmath.zig");
const Vector2 = nmath.Vector2;

const RigidBody = @import("../Bodies/RigidBody.zig");

const Constraint = @This();

pub const DistanceJoint = @import("DistanceJoint.zig");
pub const OffsetDistanceJoint = @import("OffsetDistanceJoint.zig");
pub const FixedPositionJoint = @import("FixedPositionJoint.zig");
pub const MotorJoint = @import("MotorJoint.zig");

pub const Type = enum {
    distance_joint,
    offset_distance_joint,
    fixed_position_joint,
    motor_joint,
};

pub const VTable = struct {
    deinit: *const fn (ctrself: *Constraint, alloc: Allocator) void,
    solve: *const fn (ctrself: *Constraint, bodies: std.AutoArrayHashMap(RigidBody.Id, RigidBody), dt: f32, inv_dt: f32) anyerror!void,
};

pub const Parameters = struct {
    power_max: f32 = std.math.inf(f32),
    power_min: f32 = -std.math.inf(f32),
    beta: f32 = 10, // Instead of Cdot = 0 -> Cdot + beta * C = 0. This handles position correction.
};

params: Parameters,
type: Type,
vtable: VTable,
ptr: *anyopaque,

pub fn deinit(self: *Constraint, alloc: Allocator) void {
    self.vtable.deinit(self, alloc);
}

pub fn solve(self: *Constraint, bodies: std.AutoArrayHashMap(RigidBody.Id, RigidBody), dt: f32, inv_dt: f32) !void {
    try self.vtable.solve(self, bodies, dt, inv_dt);
}
