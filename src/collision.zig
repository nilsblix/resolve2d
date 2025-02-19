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
