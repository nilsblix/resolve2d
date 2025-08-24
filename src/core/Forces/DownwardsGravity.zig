const std = @import("std");
const Allocator = std.mem.Allocator;
const AABB = @import("../AABB.zig");
const nmath = @import("../nmath.zig");
const Vector2 = nmath.Vector2;
const RigidBody = @import("../Bodies/RigidBody.zig");
const ForceGen = @import("ForceGen.zig");

const DownwardsGravity = @This();

const forcegenerator_vtable = ForceGen.VTable{
    .deinit = DownwardsGravity.deinit,
    .apply = DownwardsGravity.apply,
};

g: f32,
g_vec: Vector2,

pub fn init(alloc: Allocator, g: f32) !ForceGen {
    const self: *DownwardsGravity = try alloc.create(DownwardsGravity);
    self.g = g;
    self.g_vec = Vector2.init(0, -g);
    return ForceGen{
        .type = .downwards_gravity,
        .ptr = self,
        .vtable = DownwardsGravity.forcegenerator_vtable,
    };
}

pub fn deinit(ptr: *anyopaque, alloc: Allocator) void {
    const self: *DownwardsGravity = @ptrCast(@alignCast(ptr));
    alloc.destroy(self);
}

pub fn apply(ptr: *anyopaque, body: *RigidBody) void {
    const self: *DownwardsGravity = @ptrCast(@alignCast(ptr));
    if (body.static) return;
    body.props.force.addmult(self.g_vec, body.props.mass);
}
