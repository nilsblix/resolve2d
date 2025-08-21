const std = @import("std");
const Allocator = std.mem.Allocator;
const nmath = @import("../nmath.zig");
const Vector2 = nmath.Vector2;
const consts = @import("../simulation_constants.zig");

const RigidBody = @import("../Bodies/RigidBody.zig");
const Constraint = @import("Constraint.zig");

const Self = @This();

id: RigidBody.Id,
target_position: Vector2,

pub const VTable = Constraint.VTable{
    .deinit = Self.deinit,
    .solve = Self.solve,
};

pub fn init(alloc: Allocator, params: Constraint.Parameters, id: RigidBody.Id, target_position: Vector2) !Constraint {
    var joint = try alloc.create(Self);
    joint.id = id;
    joint.target_position = target_position;

    return Constraint{
        .params = params,
        .type = .fixed_position_joint,
        .vtable = Self.VTable,
        .ptr = joint,
    };
}

pub fn deinit(ctrself: *Constraint, alloc: Allocator) void {
    const self: *Self = @ptrCast(@alignCast(ctrself.ptr));
    alloc.destroy(self);
}

pub fn solve(ctrself: *Constraint, bodies: std.AutoArrayHashMap(RigidBody.Id, RigidBody), _: f32, _: f32) anyerror!void {
    const epsilon = consts.ALLOWED_CONSTRAINT_VALUE;

    const self: *Self = @ptrCast(@alignCast(ctrself.ptr));
    const entry = bodies.getEntry(self.id) orelse return error.InvalidRigidBodyId;
    const b = entry.value_ptr;

    if (b.static) return;

    const w = 1 / b.props.mass;
    const v = nmath.scale2(b.props.momentum, w);

    const delta_pos = nmath.sub2(self.target_position, b.props.pos);
    const dist = nmath.length2(delta_pos);

    if (dist < consts.ALLOWED_CONSTRAINT_VALUE) return;

    const normal = nmath.scale2(delta_pos, 1 / dist);

    const params = ctrself.params;

    const beta = params.beta;
    const bias = beta * dist;

    const relative_velocity = nmath.dot2(v, normal);
    var J = -(relative_velocity - bias) / w;

    const den = 1 / (@max(@abs(relative_velocity), epsilon));
    const J_max = params.power_max * den;
    const J_min = params.power_min * den;
    J = std.math.clamp(J, J_min, J_max);

    const dp = nmath.scale2(normal, J);
    b.props.momentum.add(dp);
}
