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
r1: Vector2,
r2: Vector2,
target_distance: f32,

pub const VTable = Constraint.VTable{
    .deinit = Self.deinit,
    .solve = Self.solve,
};

pub fn init(alloc: Allocator, params: Constraint.Parameters, id1: RigidBody.Id, id2: RigidBody.Id, r1: Vector2, r2: Vector2, target_distance: f32) !Constraint {
    var joint = try alloc.create(Self);
    joint.id1 = id1;
    joint.id2 = id2;
    joint.r1 = r1;
    joint.r2 = r2;
    joint.target_distance = target_distance;

    return Constraint{
        .params = params,
        .type = .offset_distance_joint,
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

    const a1 = b1.localToWorld(self.r1);
    const a2 = b2.localToWorld(self.r2);

    const normal = nmath.normalize2(nmath.sub2(a2, a1));

    const inv_m1 = if (b1.static) 0 else (1 / b1.props.mass);
    const inv_m2 = if (b2.static) 0 else (1 / b2.props.mass);

    const inv_i1 = if (b1.static) 0 else (1 / b1.props.inertia);
    const inv_i2 = if (b2.static) 0 else (1 / b2.props.inertia);

    const vlinear_1 = nmath.scale2(b1.props.momentum, inv_m1);
    const omega1 = b1.props.ang_momentum * inv_i1;

    const vlinear_2 = nmath.scale2(b2.props.momentum, inv_m2);
    const omega2 = b2.props.ang_momentum * inv_i2;

    const rotated_r1 = nmath.rotate2(self.r1, b1.props.angle);
    const v_at_a1 = Vector2.init(-rotated_r1.y * omega1, rotated_r1.x * omega1);
    const v1 = nmath.add2(vlinear_1, v_at_a1);

    const rotated_r2 = nmath.rotate2(self.r2, b2.props.angle);
    const v_at_a2 = Vector2.init(-rotated_r2.y * omega2, rotated_r2.x * omega2);
    const v2 = nmath.add2(vlinear_2, v_at_a2);

    const dv = nmath.sub2(v2, v1);

    const params = ctrself.params;

    const dist_error = nmath.length2(nmath.sub2(a2, a1)) - self.target_distance;
    const beta = params.beta;
    const position_correction = beta * dist_error / @max(self.target_distance, epsilon);

    const num = nmath.dot2(normal, dv) + position_correction;
    const r1xn = nmath.cross2(rotated_r1, normal);
    const r2xn = nmath.cross2(rotated_r2, normal);
    const den = inv_m1 + inv_m2 + r1xn * r1xn * inv_i1 + r2xn * r2xn * inv_i2;
    var J = -num / den;

    const ndv = nmath.dot2(normal, dv);
    const den2 = 1 / (@max(@abs(ndv), epsilon));
    const J_max = params.power_max * den2;
    const J_min = params.power_min * den2;
    J = std.math.clamp(J, J_min, J_max);

    const dp = nmath.scale2(normal, J);

    b1.props.momentum.sub(dp);
    b1.props.ang_momentum -= J * r1xn;

    b2.props.momentum.add(dp);
    b2.props.ang_momentum += J * r2xn;
}
