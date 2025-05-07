const std = @import("std");
const Allocator = std.mem.Allocator;
const nmath = @import("nmath.zig");
const Vector2 = nmath.Vector2;
const rb_mod = @import("rigidbody.zig");
const RigidBody = rb_mod.RigidBody;

// === RigidBody Basic Properties ===
pub export fn getRigidBodyId(ptr: *RigidBody) u64 {
    return ptr.id;
}

pub export fn isRigidBodyStatic(ptr: *RigidBody) bool {
    return ptr.static;
}

pub export fn getRigidBodyNumNormals(ptr: *RigidBody) usize {
    return ptr.num_normals;
}

// === RigidBody Dynamic Accessors ===
pub export fn getRigidBodyPosX(ptr: *RigidBody) f32 {
    return ptr.props.pos.x;
}

pub export fn getRigidBodyPosY(ptr: *RigidBody) f32 {
    return ptr.props.pos.y;
}

pub export fn getRigidBodyMomentumX(ptr: *RigidBody) f32 {
    return ptr.props.momentum.x;
}

pub export fn getRigidBodyMomentumY(ptr: *RigidBody) f32 {
    return ptr.props.momentum.y;
}

pub export fn getRigidBodyForceX(ptr: *RigidBody) f32 {
    return ptr.props.force.x;
}

pub export fn getRigidBodyForceY(ptr: *RigidBody) f32 {
    return ptr.props.force.y;
}

pub export fn getRigidBodyMass(ptr: *RigidBody) f32 {
    return ptr.props.mass;
}

pub export fn getRigidBodyAngle(ptr: *RigidBody) f32 {
    return ptr.props.angle;
}

pub export fn getRigidBodyAngularMomentum(ptr: *RigidBody) f32 {
    return ptr.props.ang_momentum;
}

pub export fn getRigidBodyTorque(ptr: *RigidBody) f32 {
    return ptr.props.torque;
}

pub export fn getRigidBodyInertia(ptr: *RigidBody) f32 {
    return ptr.props.inertia;
}

pub export fn getRigidBodyMu(ptr: *RigidBody) f32 {
    return ptr.props.mu;
}

// === RigidBody Other Properties ===
pub export fn getRigidBodyType(ptr: *RigidBody) i32 {
    return @intFromEnum(ptr.type);
}

pub export fn getRigidBodyImplementation(ptr: *RigidBody) *anyopaque {
    return ptr.ptr;
}
