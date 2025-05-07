const std = @import("std");
const nmath = @import("nmath.zig");
const Vector2 = nmath.Vector2;
const rb_mod = @import("rigidbody.zig");
const RigidBody = rb_mod.RigidBody;
const consts = @import("zigics_consts.zig");

// Regarding the ptrs, when things are not meant to be persistent across processes I am okay with the "unsafe" ptrs as no destorying/creating of RigidBodies will be made during the time that this will be active. If/when (FIXME) I implement a persistent storing of collisions this will have to be changed to ids.
pub const CollisionKey = struct {
    ref_body: *RigidBody,
    inc_body: *RigidBody,
};

pub const CollisionPoint = struct {
    accumulated_pn: f32 = 0.0,
    accumulated_pt: f32 = 0.0,

    ref_r: Vector2,
    inc_r: Vector2,

    pos: Vector2,
    depth: f32,
    original_depth: f32,

    mass_n: f32,
    mass_t: f32,

    const Self = @This();

    pub fn init(pos: Vector2, depth: f32, ref: RigidBody, inc: RigidBody, normal: Vector2) CollisionPoint {
        const middle = nmath.sub2(pos, nmath.scale2(normal, depth / 2));
        return CollisionPoint{
            .accumulated_pn = 0,
            .accumulated_pt = 0,
            .ref_r = nmath.sub2(middle, ref.props.pos),
            .inc_r = nmath.sub2(middle, inc.props.pos),
            .pos = middle,
            .depth = depth,
            .original_depth = depth,
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

    applied_linear_p1: Vector2 = .{},
    applied_linear_p2: Vector2 = .{},
    applied_rot_p_1: f32 = 0,
    applied_rot_p_2: f32 = 0,

    friction: f32 = 0.0,

    const Self = @This();

    pub fn updateTGSDepth(self: *Self, key: CollisionKey) void {
        const b1 = key.ref_body;
        const b2 = key.inc_body;

        for (&self.points) |*null_point| {
            var point = &(null_point.* orelse continue);

            if (nmath.approxEql(point.depth, point.original_depth, 1e-4)) continue;

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
        self.friction = @sqrt(b1.props.mu * b2.props.mu);
        // self.friction = 0.5 * (b1.props.mu_d + b2.props.mu_d);

        for (&self.points) |*null_point| {
            var point = &(null_point.* orelse continue);
            const r1 = point.ref_r;
            const r2 = point.inc_r;

            const r1n = nmath.cross2(r1, self.normal);
            const r2n = nmath.cross2(r2, self.normal);
            const kn = inv_mass + inv_i1 * (r1n * r1n) + inv_i2 * (r2n * r2n);

            const r1t = nmath.cross2(r1, self.tangent);
            const r2t = nmath.cross2(r2, self.tangent);
            const kt = inv_mass + inv_i1 * (r1t * r1t) + inv_i2 * (r2t * r2t);

            point.mass_n = if (kn > 0.0) (1 / kn) else 0.0;
            point.mass_t = if (kt > 0.0) (1 / kt) else 0.0;
        }
    }

    pub fn calculateImpulses(self: *Self, key: CollisionKey, dt: f32) void {
        const b1 = key.ref_body;
        const b2 = key.inc_body;

        const inv_m1 = if (b1.static) 0 else (1 / b1.props.mass);
        const inv_m2 = if (b2.static) 0 else (1 / b2.props.mass);

        const inv_i1 = if (b1.static) 0 else (1 / b1.props.inertia);
        const inv_i2 = if (b2.static) 0 else (1 / b2.props.inertia);

        const vlinear_1 = nmath.scale2(b1.props.momentum, inv_m1);
        const omega1 = b1.props.ang_momentum * inv_i1;

        const vlinear_2 = nmath.scale2(b2.props.momentum, inv_m2);
        const omega2 = b2.props.ang_momentum * inv_i2;

        for (&self.points) |*null_point| {
            var point = &(null_point.* orelse continue);

            if (point.depth >= 0) {
                point.accumulated_pn = 0;
                point.accumulated_pt = 0;
                continue;
            }

            const r1 = point.ref_r;
            const r2 = point.inc_r;

            // CALCULATION
            const vrot_1 = Vector2.init(-r1.y * omega1, r1.x * omega1);
            const v1 = nmath.add2(vlinear_1, vrot_1);

            const vrot_2 = Vector2.init(-r2.y * omega2, r2.x * omega2);
            const v2 = nmath.add2(vlinear_2, vrot_2);

            const dv = nmath.sub2(v1, v2);

            const bias = consts.BAUMGARTE * @max(0, (-point.depth) - consts.BAUMGARTE_SLOP) / dt;
            var num = nmath.dot2(dv, self.normal) + bias;
            const pn = num * point.mass_n;

            if (pn < consts.MIN_MANIFOLD_IMPULSE) continue;

            num = nmath.dot2(dv, self.tangent);
            const pt = num * point.mass_t;

            // APPLICATION
            const new_accumulated_pn = @max(0, point.accumulated_pn + pn);
            const applied_pn = new_accumulated_pn - point.accumulated_pn;
            point.accumulated_pn = new_accumulated_pn;

            const max_pt = self.friction * @abs(point.accumulated_pn);

            const new_accumulated_pt = std.math.clamp(point.accumulated_pt + pt, -max_pt, max_pt);
            const applied_pt = new_accumulated_pt - point.accumulated_pt;
            point.accumulated_pt = new_accumulated_pt;

            const pn_vec = nmath.scale2(self.normal, applied_pn);
            const pt_vec = nmath.scale2(self.tangent, applied_pt);

            const dp = nmath.add2(pn_vec, pt_vec);

            if (!b1.static) {
                self.applied_linear_p1.sub(dp);
                self.applied_rot_p_1 -= nmath.cross2(r1, dp);
            }

            if (!b2.static) {
                self.applied_linear_p2.add(dp);
                self.applied_rot_p_2 += nmath.cross2(r2, dp);
            }
        }

        b1.props.momentum.add(self.applied_linear_p1);
        b1.props.ang_momentum += self.applied_rot_p_1;

        b2.props.momentum.add(self.applied_linear_p2);
        b2.props.ang_momentum += self.applied_rot_p_2;

        self.applied_linear_p1 = .{};
        self.applied_rot_p_1 = 0;
        self.applied_linear_p2 = .{};
        self.applied_rot_p_2 = 0;
    }
};

pub fn normalShouldFlipSAT(normal: Vector2, reference: *RigidBody, incident: *RigidBody) bool {
    if (nmath.dot2(normal, nmath.sub2(incident.props.pos, reference.props.pos)) < 0.0) {
        return true;
    }
    return false;
}

pub fn overlapSAT(ret: *SATResult, reference: *RigidBody, incident: *RigidBody) bool {
    const EPS: f32 = consts.SAT_OVERLAP_THRESHOLD;

    // var iter = reference.normal_iter;
    var iter = reference.makeNormalIter(incident);
    while (iter.next()) |edge| {
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
        if (d <= -consts.COLLISION_MARGIN) return false;

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

pub const SATResult = struct {
    collides: bool,
    normal: Vector2,
    penetration: f32,
    reference_normal_id: usize,
    key: CollisionKey,
};

pub fn performNarrowSAT(b1: *RigidBody, b2: *RigidBody) SATResult {
    var ret: SATResult = undefined;
    ret.collides = false;
    ret.penetration = std.math.inf(f32);

    // FIXME: Allocators don't align memory the same --> Non deterministic
    // behaviour between allocators.
    const num1 = @intFromPtr(b1.ptr);
    const num2 = @intFromPtr(b2.ptr);
    const o1 = if (num1 < num2) b1 else b2;
    const o2 = if (o1 == b1) b2 else b1;

    // const o1 = b1;
    // const o2 = b2;

    if (!overlapSAT(&ret, o1, o2)) {
        return ret;
    }

    if (!overlapSAT(&ret, o2, o1)) {
        return ret;
    }

    ret.collides = true;

    return ret;
}

pub const Line = struct {
    a: Vector2,
    b: Vector2,
};

/// Line `a` is reference.
/// Line `b` is incident.
/// Line `b` is going to be clipped onto line a.
pub fn clipLineToLine(a: Line, b: Line) Line {
    const delta_a = nmath.sub2(a.b, a.a);
    const a_len = nmath.length2(delta_a);
    const tang = nmath.scale2(delta_a, 1 / a_len);

    var p1: Vector2 = b.a;
    var p2: Vector2 = b.b;

    {
        const scalar = nmath.dot2(nmath.sub2(b.a, a.a), tang);
        if (!(scalar > 0 and scalar < a_len)) {
            const clos = if (scalar < 0.5 * a_len) a.a else a.b;

            const delta_p = nmath.sub2(b.a, b.b);

            const t = -nmath.dot2(tang, nmath.sub2(b.b, clos)) / nmath.dot2(tang, delta_p);
            p1 = nmath.addmult2(b.b, delta_p, t);
        }
    }

    {
        const scalar = nmath.dot2(nmath.sub2(b.b, a.a), tang);
        if (!(scalar > 0 and scalar < a_len)) {
            const clos = if (scalar < 0.5 * a_len) a.a else a.b;

            const delta_p = nmath.sub2(b.b, b.a);

            const t = -nmath.dot2(tang, nmath.sub2(b.a, clos)) / nmath.dot2(tang, delta_p);
            p2 = nmath.addmult2(b.a, delta_p, t);
        }
    }

    return Line{ .a = p1, .b = p2 };
}
