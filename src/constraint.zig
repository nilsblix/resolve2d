const std = @import("std");
const nmath = @import("nmath.zig");
const Vector2 = nmath.Vector2;
const Allocator = std.mem.Allocator;
const rb_mod = @import("rigidbody.zig");
const RigidBody = rb_mod.RigidBody;

pub const Constraints = enum {
    link_joint,
};

pub const Constraint = struct {
    const VTable = struct {
        deinit: *const fn (ctrself: *Constraint, alloc: Allocator) void,
        applyImpulses: *const fn (ctrself: *Constraint) void,
    };

    type: Constraints,
    vtable: VTable,
    ptr: *anyopaque,

    const Self = @This();
};

pub const LinkJoint = struct {
    b1: *RigidBody,
    b2: *RigidBody,
    distance: f32,

    const VTable = Constraint.VTable{
        .deinit = LinkJoint.deinit,
        .applyImpulses = LinkJoint.applyImpulses,
    };

    const Self = @This();

    pub fn init(alloc: Allocator, b1: *RigidBody, b2: *RigidBody, distance: f32) !Constraint {
        const joint = try alloc.create(LinkJoint);
        joint.* = LinkJoint{
            .b1 = b1,
            .b2 = b2,
            .distance = distance,
        };

        return Constraint{
            .type = .link_joint,
            .ptr = joint,
            .vtable = LinkJoint.VTable,
        };
    }

    pub fn deinit(ctrself: *Constraint, alloc: Allocator) void {
        alloc.destroy(ctrself.ptr);
    }

    pub fn applyImpulses(ctrself: *Constraint) void {
        // TODO
        _ = ctrself;
    }
};
