const std = @import("std");
const Allocator = std.mem.Allocator;
const nmath = @import("../nmath.zig");
const Vector2 = nmath.Vector2;
const consts = @import("../simulation_constants.zig");

const RigidBody = @import("../Bodies/RigidBody.zig");
const Constraint = @import("Constraint.zig");

const Self = @This();

id1: RigidBody.Id,
id2: RigidBody.Id,
target_distance: f32,

pub const VTable = Constraint.VTable{
    .deinit = Self.deinit,
    .solve = Self.solve,
};

pub fn init(alloc: Allocator, params: Constraint.Parameters, id1: RigidBody.Id, id2: RigidBody.Id, target_distance: f32) !Constraint {
    var joint = try alloc.create(Self);
    joint.id1 = id1;
    joint.id2 = id2;
    joint.target_distance = target_distance;

    return Constraint{
        .params = params,
        .type = .distance_joint,
        .vtable = Self.VTable,
        .ptr = joint,
    };
}

pub fn deinit(ctrself: *Constraint, alloc: Allocator) void {
    const self: *Self = @ptrCast(@alignCast(ctrself.ptr));
    alloc.destroy(self);
}

pub fn solve(ctrself: *Constraint, bodies: std.AutoArrayHashMap(RigidBody.Id, RigidBody), _: f32, _: f32) anyerror!void {
    const epsilon = consts.CONSTRAINT_GRADIENT_DIVISION_LIMIT;

    const self: *Self = @ptrCast(@alignCast(ctrself.ptr));
    const entry1 = bodies.getEntry(self.id1) orelse return error.InvalidRigidBodyId;
    const b1 = entry1.value_ptr;
    const entry2 = bodies.getEntry(self.id2) orelse return error.InvalidRigidBodyId;
    const b2 = entry2.value_ptr;

    const w1 = if (b1.static) 0 else (1 / b1.props.mass);
    const w2 = if (b2.static) 0 else (1 / b2.props.mass);

    const v1 = nmath.scale2(b1.props.momentum, w1);
    const v2 = nmath.scale2(b2.props.momentum, w2);
    const relative_v = nmath.sub2(v2, v1);

    const normal = nmath.normalize2(nmath.sub2(b2.props.pos, b1.props.pos));

    const params = ctrself.params;

    const dist_error = nmath.length2(nmath.sub2(b2.props.pos, b1.props.pos)) - self.target_distance;
    const beta = params.beta;
    const position_correction = beta * dist_error / @max(self.target_distance, 0.001);

    var J = -(nmath.dot2(relative_v, normal) + position_correction) / (w1 + w2);

    const ndv = nmath.dot2(normal, relative_v);
    const den = 1 / (@max(@abs(ndv), epsilon));
    const J_max = params.power_max * den;
    const J_min = params.power_min * den;
    J = std.math.clamp(J, J_min, J_max);

    const dp = nmath.scale2(normal, J);
    b1.props.momentum.sub(dp);
    b2.props.momentum.add(dp);
}
