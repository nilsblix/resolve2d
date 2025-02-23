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
    b1: *RigidBody,
    b2: *RigidBody,
};

pub const CollisionPoint = struct {
    pn: f32,
    pt: f32,

    r1: Vector2,
    r2: Vector2,

    key: CollisionKey,
};

pub const CollisionManifold = struct {
    pub const MAX_POINTS = 2;

    normal: Vector2,
    points: [CollisionManifold.MAX_POINTS]?CollisionPoint,
};

pub fn overlapSAT(ret: *Collision, reference: *RigidBody, incident: *RigidBody) bool {
    const EPS: f32 = 1e-5;

    var iter = reference.normal_iter;
    while (iter.next(reference.*, incident.*)) |edge| {
        const normal = edge.dir;
        const p1 = reference.projectAlongAxis(normal);
        const p2 = incident.projectAlongAxis(normal);

        // if there are any gaps ==> not colliding
        // a.high <= b.low or b.high <= a.low

        const d1 = p1[1] - p2[0];
        const d2 = p2[1] - p1[0];
        if (d1 <= 0 or d2 <= 0) return false;

        if (nmath.approxEql2(normal, nmath.negate2(ret.normal), EPS)) {
            const diff = nmath.sub2(incident.props.pos, reference.props.pos);

            if (nmath.dot2(diff, normal) > nmath.dot2(diff, ret.normal)) {
                ret.penetration = @min(d1, d2);
                ret.normal = normal;
                ret.reference_normal_id = iter.iter_performed - 1;
            }
        }

        if (d1 < ret.penetration - EPS) {
            ret.penetration = d1;
            // ret.normal = nmath.negate2(normal);
            ret.normal = normal;
            ret.reference_normal_id = iter.iter_performed - 1;
            ret.key = .{ .b1 = reference, .b2 = incident };
        }
        if (d2 < ret.penetration - EPS) {
            ret.penetration = d2;
            // ret.normal = nmath.negate2(normal);
            ret.normal = normal;
            ret.reference_normal_id = iter.iter_performed - 1;
            ret.key = .{ .b1 = reference, .b2 = incident };
        }
    }
    return true;
}

pub fn performNarrowSAT(b1: *RigidBody, b2: *RigidBody) Collision {
    var ret: Collision = undefined;
    ret.collides = false;
    ret.penetration = std.math.inf(f32);

    if (!overlapSAT(&ret, b1, b2)) {
        return ret;
    }

    if (!overlapSAT(&ret, b2, b1)) {
        return ret;
    }

    ret.collides = true;

    return ret;
}
