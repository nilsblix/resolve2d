const std = @import("std");
const nmath = @import("nmath.zig");
const Vector2 = nmath.Vector2;
const rb_mod = @import("rigidbody.zig");
const RigidBody = rb_mod.RigidBody;

pub const Collision = struct {
    collides: bool,
    normal: Vector2,
    penetration: f32,
    reference_normal_id: usize,
    key: CollisionKey,
};

pub const CollisionKey = struct {
    ref_body: *RigidBody,
    inc_body: *RigidBody,
};

pub const CollisionPoint = struct {
    pn: f32 = 0.0,
    pt: f32 = 0.0,

    accumulated_pn: f32 = 0.0,
    accumulated_pt: f32 = 0.0,

    ref_r: Vector2,
    inc_r: Vector2,

    pos: Vector2,
    depth: f32,
    original_depth: f32,

    v_rel: Vector2 = .{},
};

pub const CollisionManifold = struct {
    pub const MAX_POINTS = 2;

    normal: Vector2,
    points: [CollisionManifold.MAX_POINTS]?CollisionPoint,

    prev_angle_1: f32,
    prev_angle_2: f32,

    const Self = @This();

    pub fn updateTGSDepth(self: *Self, key: CollisionKey) void {
        const b1 = key.ref_body;
        const b2 = key.inc_body;

        for (&self.points) |*point| {
            if (point.*) |*col_pt| {
                const r1 = col_pt.ref_r;
                const r2 = col_pt.inc_r;

                const r_rot_1 = nmath.rotate2(r1, b1.props.angle - self.prev_angle_1);
                const r_rot_2 = nmath.rotate2(r2, b2.props.angle - self.prev_angle_2);

                const a1 = nmath.add2(r_rot_1, b1.props.pos);
                const a2 = nmath.add2(r_rot_2, b2.props.pos);

                const depth = nmath.dot2(self.normal, nmath.sub2(a2, a1));
                col_pt.depth = depth + col_pt.original_depth;

                col_pt.ref_r = r_rot_1;
                col_pt.inc_r = r_rot_2;
            }
        }

        self.prev_angle_1 = b1.props.angle;
        self.prev_angle_2 = b2.props.angle;
    }

    pub fn calculateImpulses(self: *Self, key: CollisionKey, dt: f32, beta_bias: f32, delta_slop: f32) void {
        const b1 = key.ref_body;
        const b2 = key.inc_body;

        const inv1 = if (b1.static) 0 else (1 / b1.props.mass);
        const inv2 = if (b2.static) 0 else (1 / b2.props.mass);
        const inv_mass = inv1 + inv2;

        for (self.points, 0..) |point, idx| {
            if (point) |col_pt| {
                const r1 = col_pt.ref_r;
                const r2 = col_pt.inc_r;

                const r1xn = if (b1.static) 0.0 else nmath.cross2(r1, self.normal);
                const r2xn = if (b2.static) 0.0 else nmath.cross2(r2, self.normal);
                const kn = inv_mass + (r1xn * r1xn / b1.props.inertia) + (r2xn * r2xn / b2.props.inertia);

                const v1 = nmath.scale2(b1.props.momentum, 1 / b1.props.mass);
                const omega1 = b1.props.ang_momentum / b1.props.inertia;
                const w1_cross_r1 = Vector2.init(-r1.y * omega1, r1.x * omega1);
                const rel_1 = nmath.add2(v1, w1_cross_r1);

                const v2 = nmath.scale2(b2.props.momentum, 1 / b2.props.mass);
                const omega2 = b2.props.ang_momentum / b2.props.inertia;
                const w2_cross_r2 = Vector2.init(-r2.y * omega2, r2.x * omega2);
                const rel_2 = nmath.add2(v2, w2_cross_r2);

                const delta_v = nmath.sub2(rel_1, rel_2);

                self.points[idx].?.v_rel = delta_v;

                const v_bias = beta_bias * @max(0, -col_pt.depth - delta_slop) / dt;
                var num = nmath.dot2(delta_v, self.normal) + v_bias;
                const pn = num / kn;
                self.points[idx].?.pn = pn;

                const tangent = nmath.rotate90clockwise(self.normal);
                const r1xt = if (b1.static) 0.0 else nmath.cross2(r1, tangent);
                const r2xt = if (b2.static) 0.0 else nmath.cross2(r2, tangent);
                const kt = inv_mass + (r1xt * r1xt / b1.props.inertia) + (r2xt * r2xt / b2.props.inertia);

                num = nmath.dot2(delta_v, tangent);
                const pt = num / kt;
                // _ = pt;
                self.points[idx].?.pt = pt;
            }
        }
    }

    pub fn resetImpulses(self: *Self) void {
        for (&self.points) |*point| {
            if (point.*) |*pt| {
                pt.pn = 0.0;
                pt.pt = 0.0;
            }
        }
    }

    pub fn applyImpulses(self: *Self, key: CollisionKey, eps: f32, v_rel_static_max: f32) void {
        if (key.ref_body.static and key.inc_body.static) return;

        const b1 = key.ref_body;
        const b2 = key.inc_body;

        for (&self.points) |*point| {
            if (point.*) |*pt| {
                if (@abs(pt.pn) < eps) {
                    continue;
                }

                const r1 = pt.ref_r;
                const r2 = pt.inc_r;

                const new_accumulated_pn = @max(0.0, pt.accumulated_pn + pt.pn);
                const applied_pn = new_accumulated_pn - pt.accumulated_pn;
                pt.accumulated_pn = new_accumulated_pn;

                var mu: f32 = undefined;
                const len = nmath.length2(pt.v_rel);
                if (len < v_rel_static_max) {
                    mu = @min((b1.props.mu_s + b2.props.mu_s) / 2, 1.0);
                } else {
                    mu = @min((b1.props.mu_d + b2.props.mu_d) / 2, 1.0);
                }

                const sum_pt = pt.accumulated_pt + pt.pt;
                const new_accumulated_pt = @max(-mu * pt.accumulated_pn, @min(mu * pt.accumulated_pn, sum_pt));
                const applied_pt = new_accumulated_pt - pt.accumulated_pt;
                pt.accumulated_pt = new_accumulated_pt;

                const pn_vec = nmath.scale2(self.normal, applied_pn);

                const tangent = nmath.rotate90clockwise(self.normal);
                const pt_vec = nmath.scale2(tangent, applied_pt);

                const p = nmath.add2(pn_vec, pt_vec);
                // _ = pt_vec;
                // const p = pn_vec;

                if (!b1.static) {
                    b1.props.momentum.sub(p);
                    b1.props.ang_momentum -= nmath.cross2(r1, p);
                }

                if (!b2.static) {
                    b2.props.momentum.add(p);
                    b2.props.ang_momentum += nmath.cross2(r2, p);
                }
            }
        }
    }
};

pub fn normalShouldFlipSAT(normal: Vector2, reference: *RigidBody, incident: *RigidBody) bool {
    if (nmath.dot2(normal, nmath.sub2(incident.props.pos, reference.props.pos)) < 0.0) {
        return true;
    }
    return false;
}

pub fn overlapSAT(ret: *Collision, reference: *RigidBody, incident: *RigidBody) bool {
    const EPS: f32 = 1e-2;

    var iter = reference.normal_iter;
    while (iter.next(reference.*, incident.*)) |edge| {
        var normal = edge.dir;
        var flipped = false;

        if (normalShouldFlipSAT(normal, reference, incident)) {
            normal.negate();
            flipped = true;
        }

        const p1 = reference.projectAlongAxis(normal);
        const p2 = incident.projectAlongAxis(normal);

        // if there are any gaps ==> not colliding
        // a.high <= b.low or b.high <= a.low

        const d1 = p1[1] - p2[0];
        const d2 = p2[1] - p1[0];

        const d = @min(d1, d2);
        if (d <= 0) return false;

        if (!flipped and nmath.approxEql2(normal, ret.normal, EPS)) {
            // if they are the same
            // the reference is going to be the one that has lowest t-value wrt normal
            // the normal belongs to the body that has the lowest t-value wrt normal
            const tref = nmath.dot2(reference.props.pos, normal);
            const tinc = nmath.dot2(incident.props.pos, normal);
            // right now we're checking against current incidents normal and id
            // (which was previously reference)
            if (tref < tinc) {
                ret.penetration = d;
                ret.normal = normal;
                ret.reference_normal_id = iter.iter_performed - 1;
                ret.key = .{ .ref_body = reference, .inc_body = incident };
            }
        }

        if (!flipped and nmath.approxEql2(normal, nmath.negate2(ret.normal), EPS)) {
            const diff = nmath.sub2(incident.props.pos, reference.props.pos);

            if (nmath.dot2(diff, normal) > nmath.dot2(diff, ret.normal)) {
                ret.penetration = d;
                ret.normal = normal;
                ret.reference_normal_id = iter.iter_performed - 1;
                ret.key = .{ .ref_body = reference, .inc_body = incident };
            }
        }

        if (d < ret.penetration - EPS) {
            ret.penetration = d;
            ret.normal = normal;
            ret.reference_normal_id = iter.iter_performed - 1;
            ret.key = .{ .ref_body = reference, .inc_body = incident };
        }
    }
    return true;
}

pub fn performNarrowSAT(b1: *RigidBody, b2: *RigidBody) Collision {
    var ret: Collision = undefined;
    ret.collides = false;
    ret.penetration = std.math.inf(f32);

    // POTENTIAL: using here an arbitrary sorting thing so that when
    // objects with the same normal (ex: disc vs disc) don't swap between
    // being reference and incident
    const num1 = @intFromPtr(b1.ptr);
    const num2 = @intFromPtr(b2.ptr);
    const o1 = if (b1.props.pos.x < b2.props.pos.x or num1 < num2) b1 else b2;
    const o2 = if (o1 == b1) b2 else b1;

    if (!overlapSAT(&ret, o1, o2)) {
        return ret;
    }

    if (!overlapSAT(&ret, o2, o1)) {
        return ret;
    }

    ret.collides = true;

    return ret;
}
