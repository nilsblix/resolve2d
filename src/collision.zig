const std = @import("std");
const nmath = @import("nmath.zig");
const Vector2 = nmath.Vector2;
const rb_mod = @import("rigidbody.zig");
const RigidBody = rb_mod.RigidBody;

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
    const MAX_POINTS = 2;

    points: [CollisionManifold.MAX_POINTS]?CollisionPoint,
};

// FIXME: normals, points etc an dnot just bool
pub fn performNarrowSAT(b1: RigidBody, b2: RigidBody) bool {
    var iter = b1.normal_iter;
    while (iter.next(b1, b2)) |edge| {
        const normal = edge.dir;
        const p1 = b1.projectAlongAxis(normal);
        const p2 = b2.projectAlongAxis(normal);

        // if there are any gaps ==> not colliding
        // a.high <= b.low or b.high <= a.low
        if (p1[1] <= p2[0] or p2[1] <= p1[0]) {
            return false;
        }
    }

    iter = b2.normal_iter;
    while (iter.next(b2, b1)) |edge| {
        const normal = edge.dir;
        const p1 = b1.projectAlongAxis(normal);
        const p2 = b2.projectAlongAxis(normal);

        // if there are any gaps ==> not colliding
        // a.high <= b.low or b.high <= a.low
        if (p1[1] <= p2[0] or p2[1] <= p1[0]) {
            return false;
        }
    }

    return true;
}
