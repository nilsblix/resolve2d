const std = @import("std");
const nmath = @import("nmath.zig");
const Vector2 = nmath.Vector2;
const Allocator = std.mem.Allocator;
const rb_mod = @import("rigidbody.zig");
const RigidBody = rb_mod.RigidBody;
const nmath_consts = @import("nmath_constants.zig");

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

pub fn performSAT(body1: *RigidBody, body2: *RigidBody) Collision {
    const add1 = @intFromPtr(body1.ptr);
    const add2 = @intFromPtr(body2.ptr);

    const o1 = if (add1 < add2) body1 else body2;
    const o2 = if (o1 == body1) body2 else body1;

    var ret: Collision = .{
        .collides = false,
        .normal = .{},
        .penetration = std.math.inf(f32),
        .reference_normal_id = 0,
        .key = .{ .ref_body = o1, .inc_body = o2 },
    };

    if (!overlaps(&ret, o1, o2)) return ret;

    if (!overlaps(&ret, o2, o1)) return ret;

    return ret;
}

pub fn overlaps(ret: *Collision, ref: *RigidBody, inc: *RigidBody) bool {
    while (ref.normal_iter.next(ref, inc)) |edge| {
        const normal = edge.dir;

        const p1 = ref.projectAlongAxis(normal);
        const p2 = inc.projectAlongAxis(normal);

        const d1 = p1[1] - p2[0];
        const d2 = p2[1] - p1[0];

        const d = @min(d1, d2);
        if (d <= 0.0) return false;

        if (d + nmath_consts.SAT_OVERLAP_THRESHOLD < ret.penetration) {
            ret = .{
                .collides = true,
                .normal = normal,
                .penetration = d,
                .reference_normal_id = ref.normal_iter.iter_performed - 1,
                .key = .{ .ref_body = ref, .inc_body = inc },
            };
        }
    }
}

pub const Line = struct {
    a: Vector2,
    b: Vector2,
};

/// Line a is reference.
/// Line b is incident.
/// Line b is going to be clipped onto line a.
pub fn clipLineToLine(a: Line, b: Line) Line {
    const dA = nmath.sub2(a.a, a.b);
    const dB = nmath.sub2(b.a, b.b);
    const dAdB = nmath.dot2(dA, dB);

    var p1: Vector2 = undefined;
    var p2: Vector2 = undefined;

    const b1_a1 = nmath.sub2(b.a, a.a);
    const b2_a1 = nmath.sub2(b.b, a.a);
    const b1_a2 = nmath.sub2(b.a, a.b);
    const b2_a2 = nmath.sub2(b.b, a.b);

    var lt = nmath.dot2(dA, b1_a1);
    var rt = nmath.dot2(nmath.negate2(dA), b1_a2);
    if (lt > 0.0 and rt > 0.0) {
        // point is already inside
        p1 = b.a;
    } else {
        // point has to be pushed inside
        // deltaA * (b2 + t(b1 - b2) - a1) = 0
        const t = nmath.dot2(dA, b2_a1) / dAdB;
        p1 = nmath.addmult2(b.b, dB, t);
    }

    lt = nmath.dot2(dA, b2_a1);
    rt = nmath.dot2(nmath.negate2(dA), b2_a2);
    if (lt > 0.0 and rt > 0.0) {
        // point is already inside
        p2 = b.b;
    } else {
        // point has to be pushed inside
        // deltaA * (b1 + t(b2 - b1) - a2) = 0
        const t = nmath.dot2(dA, b1_a2) / dAdB;
        p2 = nmath.addmult2(b.a, nmath.negate2(dB), t);
    }
}
