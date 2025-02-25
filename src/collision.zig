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

    ref_r: Vector2,
    inc_r: Vector2,

    pos: Vector2,
    depth: f32,
};

pub const CollisionManifold = struct {
    pub const MAX_POINTS = 2;

    normal: Vector2,
    points: [CollisionManifold.MAX_POINTS]?CollisionPoint,
};

pub fn normalShouldFlipSAT(normal: Vector2, reference: *RigidBody, incident: *RigidBody) bool {
    if (nmath.dot2(normal, nmath.sub2(incident.props.pos, reference.props.pos)) < 0.0) {
        return true;
    }
    return false;
}

pub fn overlapSAT(ret: *Collision, reference: *RigidBody, incident: *RigidBody) bool {
    const EPS: f32 = 1e-4;

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
        if (d1 <= 0 or d2 <= 0) return false;

        const d = @min(d1, d2);

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
    // being reference and incident don't swap between
    // being reference and incident
    const o1 = if (b1.props.pos.x < b2.props.pos.x) b1 else b2;
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
