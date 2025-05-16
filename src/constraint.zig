const std = @import("std");
const nmath = @import("nmath.zig");
const Vector2 = nmath.Vector2;
const Allocator = std.mem.Allocator;
const rb_mod = @import("rigidbody.zig");
const RigidBody = rb_mod.RigidBody;
const consts = @import("zigics_consts.zig");

pub const Constraints = enum {
    distance_joint,
    offset_distance_joint,
    fixed_position_joint,
    motor_joint,
};

pub const Constraint = struct {
    const VTable = struct {
        deinit: *const fn (ctrself: *Constraint, alloc: Allocator) void,
        solve: *const fn (ctrself: *Constraint, bodies: std.AutoArrayHashMap(RigidBody.Id, RigidBody), dt: f32, inv_dt: f32) anyerror!void,
    };

    pub const Parameters = struct {
        power_max: f32 = std.math.inf(f32),
        power_min: f32 = -std.math.inf(f32),
        beta: f32 = 10, // Instead of Cdot = 0 -> Cdot + beta * C = 0. This handles position correction.
    };

    params: Parameters,
    type: Constraints,
    vtable: VTable,
    ptr: *anyopaque,

    const Self = @This();

    pub fn deinit(self: *Self, alloc: Allocator) void {
        self.vtable.deinit(self, alloc);
    }

    pub fn solve(self: *Self, bodies: std.AutoArrayHashMap(RigidBody.Id, RigidBody), dt: f32, inv_dt: f32) !void {
        try self.vtable.solve(self, bodies, dt, inv_dt);
    }
};

pub const DistanceJoint = struct {
    id1: RigidBody.Id,
    id2: RigidBody.Id,
    target_distance: f32,

    pub const VTable = Constraint.VTable{
        .deinit = Self.deinit,
        .solve = Self.solve,
    };

    pub const Self = @This();

    pub fn init(alloc: Allocator, params: Constraint.Parameters, id1: RigidBody.Id, id2: RigidBody.Id, target_distance: f32) !Constraint {
        var joint = try alloc.create(DistanceJoint);
        joint.id1 = id1;
        joint.id2 = id2;
        joint.target_distance = target_distance;

        return Constraint{
            .params = params,
            .type = Constraints.distance_joint,
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
};

pub const OffsetDistanceJoint = struct {
    id1: RigidBody.Id,
    id2: RigidBody.Id,
    r1: Vector2,
    r2: Vector2,
    target_distance: f32,

    pub const VTable = Constraint.VTable{
        .deinit = Self.deinit,
        .solve = Self.solve,
    };

    pub const Self = @This();

    pub fn init(alloc: Allocator, params: Constraint.Parameters, id1: RigidBody.Id, id2: RigidBody.Id, r1: Vector2, r2: Vector2, target_distance: f32) !Constraint {
        var joint = try alloc.create(OffsetDistanceJoint);
        joint.id1 = id1;
        joint.id2 = id2;
        joint.r1 = r1;
        joint.r2 = r2;
        joint.target_distance = target_distance;

        return Constraint{
            .params = params,
            .type = Constraints.offset_distance_joint,
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
};

pub const FixedPositionJoint = struct {
    id: RigidBody.Id,
    target_position: Vector2,

    pub const VTable = Constraint.VTable{
        .deinit = Self.deinit,
        .solve = Self.solve,
    };

    const Self = @This();

    pub fn init(alloc: Allocator, params: Constraint.Parameters, id: RigidBody.Id, target_position: Vector2) !Constraint {
        var joint = try alloc.create(Self);
        joint.id = id;
        joint.target_position = target_position;

        return Constraint{
            .params = params,
            .type = Constraints.fixed_position_joint,
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
};

pub const MotorJoint = struct {
    id: RigidBody.Id,
    target_omega: f32,

    pub const VTable = Constraint.VTable{
        .deinit = Self.deinit,
        .solve = Self.solve,
    };

    pub const Self = @This();

    pub fn init(alloc: Allocator, params: Constraint.Parameters, id: RigidBody.Id, target_omega: f32) !Constraint {
        var joint = try alloc.create(Self);
        joint.id = id;
        joint.target_omega = target_omega;

        return Constraint{
            .params = params,
            .type = Constraints.motor_joint,
            .vtable = Self.VTable,
            .ptr = joint,
        };
    }

    pub fn deinit(ctrself: *Constraint, alloc: Allocator) void {
        const self: *Self = @ptrCast(@alignCast(ctrself.ptr));
        alloc.destroy(self);
    }

    pub fn solve(ctrself: *Constraint, bodies: std.AutoArrayHashMap(RigidBody.Id, RigidBody), dt: f32, _: f32) anyerror!void {
        const epsilon = consts.ALLOWED_CONSTRAINT_VALUE;

        const self: *Self = @ptrCast(@alignCast(ctrself.ptr));
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
};
