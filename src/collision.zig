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

pub fn performNarrowSAT(b1: *RigidBody, b2: *RigidBody) Collision {
    const EPS = 1e-6;

    var ret: Collision = undefined;
    ret.collides = false;
    ret.penetration = -std.math.inf(f32);

    var iter = b1.normal_iter;
    while (iter.next(b1.*, b2.*)) |edge| {
        const normal = edge.dir;
        const p1 = b1.projectAlongAxis(normal);
        const p2 = b2.projectAlongAxis(normal);

        // if there are any gaps ==> not colliding
        // a.high <= b.low or b.high <= a.low

        const d1 = p1[1] - p2[0];
        if (d1 <= 0) return ret;

        const d2 = p2[1] - p1[0];
        if (d2 <= 0) return ret;

        if (d1 > ret.penetration + EPS) {
            ret.penetration = d1;
            ret.normal = normal;
            ret.reference_normal_id = iter.iter_performed;
            ret.key = .{ .b1 = b1, .b2 = b2 };
        }
        if (d2 > ret.penetration + EPS) {
            ret.penetration = d2;
            ret.normal = normal;
            ret.reference_normal_id = iter.iter_performed;
            ret.key = .{ .b1 = b1, .b2 = b2 };
        }
    }

    iter = b2.normal_iter;
    while (iter.next(b2.*, b1.*)) |edge| {
        const normal = edge.dir;
        const p1 = b1.projectAlongAxis(normal);
        const p2 = b2.projectAlongAxis(normal);

        // if there are any gaps ==> not colliding
        // a.high <= b.low or b.high <= a.low
        const d1 = p1[1] - p2[0];
        if (d1 <= 0) return ret;

        const d2 = p2[1] - p1[0];
        if (d2 <= 0) return ret;

        if (d1 > ret.penetration + EPS) {
            ret.penetration = d1;
            ret.normal = normal;
            ret.reference_normal_id = iter.iter_performed;
            ret.key = .{ .b1 = b2, .b2 = b1 };
        }
        if (d2 > ret.penetration + EPS) {
            ret.penetration = d2;
            ret.normal = normal;
            ret.reference_normal_id = iter.iter_performed;
            ret.key = .{ .b1 = b2, .b2 = b1 };
        }
    }

    ret.collides = true;

    return ret;
}
