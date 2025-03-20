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

    dv: Vector2 = .{},

    mass_n: f32,
    mass_t: f32,

    pub fn init(pos: Vector2, depth: f32, ref: RigidBody, inc: RigidBody) CollisionPoint {
        return CollisionPoint{
            .pn = 0,
            .pt = 0,
            .accumulated_pn = 0,
            .accumulated_pt = 0,
            .ref_r = nmath.sub2(pos, ref.props.pos),
            .inc_r = nmath.sub2(pos, inc.props.pos),
            .pos = pos,
            .depth = depth,
            .original_depth = depth,
            .dv = .{},
            .mass_n = 0,
            .mass_t = 0,
        };
    }
};

pub const CollisionManifold = struct {
    pub const MAX_POINTS = 2;

    normal: Vector2,
    tangent: Vector2 = .{},
    points: [CollisionManifold.MAX_POINTS]?CollisionPoint,

    prev_angle_1: f32,
    prev_angle_2: f32,

    friction: f32 = 0.0,

    const Self = @This();

    pub fn updateTGSDepth(self: *Self, key: CollisionKey) void {
        const b1 = key.ref_body;
        const b2 = key.inc_body;

        for (&self.points) |*null_point| {
            if (null_point.*) |*point| {
                const r1 = point.ref_r;
                const r2 = point.inc_r;

                const r_rot_1 = nmath.rotate2(r1, b1.props.angle - self.prev_angle_1);
                const r_rot_2 = nmath.rotate2(r2, b2.props.angle - self.prev_angle_2);

                const a1 = nmath.add2(r_rot_1, b1.props.pos);
                const a2 = nmath.add2(r_rot_2, b2.props.pos);

                const depth = nmath.dot2(self.normal, nmath.sub2(a2, a1));
                point.depth = depth + point.original_depth;

                point.ref_r = r_rot_1;
                point.inc_r = r_rot_2;
            }
        }

        self.prev_angle_1 = b1.props.angle;
        self.prev_angle_2 = b2.props.angle;
    }

    pub fn preStep(self: *Self, key: CollisionKey) void {
        const b1 = key.ref_body;
        const b2 = key.inc_body;

        const inv_m1 = if (b1.static) 0 else (1 / b1.props.mass);
        const inv_m2 = if (b2.static) 0 else (1 / b2.props.mass);
        const inv_mass = inv_m1 + inv_m2;

        const inv_i1 = if (b1.static) 0 else (1 / b1.props.inertia);
        const inv_i2 = if (b2.static) 0 else (1 / b2.props.inertia);

        self.tangent = nmath.rotate90clockwise(self.normal);
        // self.friction = @sqrt(b1.props.mu_d * b2.props.mu_d);
        self.friction = 0.5 * (b1.props.mu_d + b2.props.mu_d);

        for (&self.points) |*null_point| {
            if (null_point.*) |*point| {
                const r1 = point.ref_r;
                const r2 = point.inc_r;

                // const r1n = nmath.dot2(r1, self.normal);
                // const r2n = nmath.dot2(r2, self.normal);
                // const kn = inv_mass + inv_i1 * (nmath.dot2(r1, r1) - r1n * r1n) + inv_i2 * (nmath.dot2(r2, r2) - r2n * r2n);
                const r1n = nmath.cross2(r1, self.normal);
                const r2n = nmath.cross2(r2, self.normal);
                const kn = inv_mass + inv_i1 * (r1n * r1n) + inv_i2 * (r2n * r2n);

                // const r1t = nmath.dot2(r1, tangent);
                // const r2t = nmath.dot2(r2, tangent);
                // const kt = inv_mass + inv_i1 * (nmath.dot2(r1, r1) - r1t * r1t) + inv_i2 * (nmath.dot2(r2, r2) - r2t * r2t);
                const r1t = nmath.cross2(r1, self.tangent);
                const r2t = nmath.cross2(r2, self.tangent);
                const kt = inv_mass + inv_i1 * (r1t * r1t) + inv_i2 * (r2t * r2t);

                point.mass_n = if (kn > 0.0) (1 / kn) else 0.0;
                point.mass_t = if (kt > 0.0) (1 / kt) else 0.0;
            }
        }
    }

    pub fn applyImpulses(self: *Self, key: CollisionKey, dt: f32, baumgarte: f32, slop: f32) void {
        const b1 = key.ref_body;
        const b2 = key.inc_body;

        const inv_m1 = if (b1.static) 0 else (1 / b1.props.mass);
        const inv_m2 = if (b2.static) 0 else (1 / b2.props.mass);

        const inv_i1 = if (b1.static) 0 else (1 / b1.props.inertia);
        const inv_i2 = if (b2.static) 0 else (1 / b2.props.inertia);

        for (&self.points) |*null_point| {
            if (null_point.*) |*point| {
                const r1 = point.ref_r;
                const r2 = point.inc_r;

                // CALCULATION
                const vlinear_1 = nmath.scale2(b1.props.momentum, inv_m1);
                const omega1 = b1.props.ang_momentum * inv_i1;
                const vrot_1 = Vector2.init(-r1.y * omega1, r1.x * omega1);
                const v1 = nmath.add2(vlinear_1, vrot_1);

                const vlinear_2 = nmath.scale2(b2.props.momentum, inv_m2);
                const omega2 = b2.props.ang_momentum * inv_i2;
                const vrot_2 = Vector2.init(-r2.y * omega2, r2.x * omega2);
                const v2 = nmath.add2(vlinear_2, vrot_2);

                point.dv = nmath.sub2(v1, v2);

                const bias = baumgarte * @max(0, (-point.depth) - slop) / dt;
                var num = nmath.dot2(point.dv, self.normal) + bias;
                point.pn = num * point.mass_n;

                if (point.pn < 1e-5) continue;

                num = nmath.dot2(point.dv, self.tangent);
                point.pt = num * point.mass_t;

                // APPLICATION
                const new_accumulated_pn = @max(0, point.accumulated_pn + point.pn);
                const applied_pn = new_accumulated_pn - point.accumulated_pn;
                point.accumulated_pn = new_accumulated_pn;

                const max_pt = self.friction * @abs(point.accumulated_pn);

                const new_accumulated_pt = std.math.clamp(point.accumulated_pt + point.pt, -max_pt, max_pt);
                const applied_pt = new_accumulated_pt - point.accumulated_pt;
                point.accumulated_pt = new_accumulated_pt;

                const pn_vec = nmath.scale2(self.normal, applied_pn);
                const pt_vec = nmath.scale2(self.tangent, applied_pt);

                const dp = nmath.add2(pn_vec, pt_vec);
                // var dp = nmath.add2(pn_vec, pt_vec);
                // dp = .{};

                if (!b1.static) {
                    b1.props.momentum.sub(dp);
                    b1.props.ang_momentum -= nmath.cross2(r1, dp);
                }

                if (!b2.static) {
                    b2.props.momentum.add(dp);
                    b2.props.ang_momentum += nmath.cross2(r2, dp);
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
    const EPS: f32 = 1e-3;

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
        if (d <= 0.0) return false;

        if (!flipped and nmath.approxEql2(normal, ret.normal, EPS)) {
            // if they are the same
            // the reference is going to be the one that has lowest t-value wrt normal
            // the normal belongs to the body that has the lowest t-value wrt normal
            const tref = nmath.dot2(reference.props.pos, normal);
            const tinc = nmath.dot2(incident.props.pos, normal);
            // right now we're checking against current incidents normal and id
            // (which was previously reference)
            if (tref < tinc - EPS) {
                ret.penetration = d;
                ret.normal = normal;
                ret.reference_normal_id = iter.iter_performed - 1;
                ret.key = .{ .ref_body = reference, .inc_body = incident };
            }
        }

        if (!flipped and nmath.approxEql2(normal, nmath.negate2(ret.normal), EPS)) {
            const diff = nmath.sub2(incident.props.pos, reference.props.pos);

            if (nmath.dot2(diff, normal) > nmath.dot2(diff, ret.normal) + EPS) {
                ret.penetration = d;
                ret.normal = normal;
                ret.reference_normal_id = iter.iter_performed - 1;
                ret.key = .{ .ref_body = reference, .inc_body = incident };
            }
        }

        if (d + EPS < ret.penetration) {
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
    const o1 = if (num1 < num2) b1 else b2;
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
