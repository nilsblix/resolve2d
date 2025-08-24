const std = @import("std");
const Allocator = std.mem.Allocator;
const nmath = @import("../nmath.zig");
const Vector2 = nmath.Vector2;
const consts = @import("../simulation_constants.zig");

const RigidBody = @import("../Bodies/RigidBody.zig");
const Constraint = @import("Constraint.zig");

const MotorJoint = @This();

id: RigidBody.Id,
target_omega: f32,

pub const VTable = Constraint.VTable{
    .deinit = MotorJoint.deinit,
    .solve = MotorJoint.solve,
};

pub fn init(alloc: Allocator, params: Constraint.Parameters, id: RigidBody.Id, target_omega: f32) !Constraint {
    var joint = try alloc.create(MotorJoint);
    joint.id = id;
    joint.target_omega = target_omega;

    return Constraint{
        .params = params,
        .type = .motor_joint,
        .vtable = MotorJoint.VTable,
        .ptr = joint,
    };
}

pub fn deinit(ctrself: *Constraint, alloc: Allocator) void {
    const self: *MotorJoint = @ptrCast(@alignCast(ctrself.ptr));
    alloc.destroy(self);
}

pub fn solve(ctrself: *Constraint, bodies: std.AutoArrayHashMap(RigidBody.Id, RigidBody), dt: f32, _: f32) anyerror!void {
    const epsilon = consts.ALLOWED_CONSTRAINT_VALUE;

    const self: *MotorJoint = @ptrCast(@alignCast(ctrself.ptr));
    const entry = bodies.getEntry(self.id) orelse return error.InvalidRigidBodyId;
    const b = entry.value_ptr;

    if (b.static) return;

    const omega = b.props.ang_momentum / b.props.inertia;
    const C = omega - self.target_omega;
    if (@abs(C) < consts.ALLOWED_CONSTRAINT_VALUE) return;

    const beta = ctrself.params.beta;
    const bias = beta * C;

    // var J = bias * b.props.inertia - b.props.torque;
    var J = b.props.torque - bias;

    const relative_velocity = b.props.torque / b.props.inertia;
    const den = 1 / (@max(@abs(relative_velocity), epsilon));
    const J_max = ctrself.params.power_max * den;
    const J_min = ctrself.params.power_min * den;
    J = std.math.clamp(J, J_min, J_max);

    b.props.ang_momentum += J * dt;
}
