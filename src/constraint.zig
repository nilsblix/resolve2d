const std = @import("std");
const nmath = @import("nmath.zig");
const Vector2 = nmath.Vector2;
const Allocator = std.mem.Allocator;
const rb_mod = @import("rigidbody.zig");
const RigidBody = rb_mod.RigidBody;

pub const Constraints = enum {
    single_link_joint,
};

pub const Constraint = struct {
    const VTable = struct {
        deinit: *const fn (ctrself: *Constraint, alloc: Allocator) void,
        applyImpulses: *const fn (ctrself: *Constraint, dt: f32, inv_dt: f32) void,
    };

    pub const Parameters = struct {
        beta: f32 = 0.01,
        upper_lambda: f32 = std.math.inf(f32),
        lower_lambda: f32 = -std.math.inf(f32),
    };

    params: Parameters,
    type: Constraints,
    vtable: VTable,
    ptr: *anyopaque,

    const Self = @This();

    pub fn deinit(self: *Self, alloc: Allocator) void {
        self.vtable.deinit(self, alloc);
    }

    pub fn applyImpulses(self: *Self, dt: f32, inv_dt: f32) void {
        self.vtable.applyImpulses(self, dt, inv_dt);
    }
};

pub const SingleLinkJoint = struct {
    body: *RigidBody,
    local_r: Vector2,
    q: Vector2,
    distance: f32,

    const VTable = Constraint.VTable{
        .deinit = SingleLinkJoint.deinit,
        .applyImpulses = SingleLinkJoint.applyImpulses,
    };

    const Self = @This();

    pub fn init(alloc: Allocator, params: Constraint.Parameters, body: *RigidBody, local_r: Vector2, q: Vector2, distance: f32) !Constraint {
        const joint = try alloc.create(SingleLinkJoint);
        joint.* = SingleLinkJoint{
            .body = body,
            .local_r = local_r,
            .q = q,
            .distance = distance,
        };

        return Constraint{
            .type = Constraints.single_link_joint,
            .params = params,
            .ptr = joint,
            .vtable = SingleLinkJoint.VTable,
        };
    }

    pub fn deinit(ctrself: *Constraint, alloc: Allocator) void {
        const self: *Self = @ptrCast(@alignCast(ctrself.ptr));
        alloc.destroy(self);
    }

    pub fn applyImpulses(ctrself: *Constraint, dt: f32, inv_dt: f32) void {
        const self: *Self = @ptrCast(@alignCast(ctrself.ptr));

        const r = nmath.rotate2(self.local_r, self.body.props.angle);
        const k = nmath.sub2(nmath.add2(self.body.props.pos, r), self.q);
        const c = 0.5 * (nmath.dot2(k, k) - self.distance * self.distance);

        const inv_m = 1 / self.body.props.mass;
        var inv_i: f32 = undefined;
        const k_len = nmath.length2(k);
        if (k_len < 1e-2) {
            inv_i = 1 / self.body.props.inertia;
        } else {
            const n = nmath.scale2(k, 1 / k_len);
            const rn = nmath.cross2(r, n);
            inv_i = rn * rn / self.body.props.inertia;
        }
        const w = inv_i + inv_m;

        const v = nmath.scale2(self.body.props.momentum, inv_m);
        const omega = self.body.props.ang_momentum / self.body.props.inertia;

        const r_prime = nmath.rotate90counterclockwise(r);
        const r_prime_k = nmath.dot2(r_prime, k);
        const jv = nmath.dot2(k, v) + r_prime_k * omega;
        const jjT = nmath.dot2(k, k) + r_prime_k * r_prime_k;

        var lambda = inv_dt * (-ctrself.params.beta * c - jv) / (w * jjT);

        lambda = std.math.clamp(lambda, ctrself.params.lower_lambda, ctrself.params.upper_lambda);

        const fext = nmath.scale2(k, lambda);
        const torque = lambda * r_prime_k;

        const p = nmath.scale2(fext, dt);
        const p_ang = torque * dt;

        self.body.props.momentum.add(p);
        self.body.props.ang_momentum += p_ang;
    }
};
