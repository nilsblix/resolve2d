const std = @import("std");
const Allocator = std.mem.Allocator;
const AABB = @import("../AABB.zig");
const nmath = @import("../nmath.zig");
const Vector2 = nmath.Vector2;
const RigidBody = @import("../Bodies/RigidBody.zig");
const ForceGen = @import("ForceGen.zig");

const Self = @This();

const forcegenerator_vtable = ForceGen.VTable{
    .deinit = Self.deinit,
    .apply = Self.apply,
};

g: f32,
g_vec: Vector2,

pub fn init(alloc: Allocator, g: f32) !ForceGen {
    const self: *Self = try alloc.create(Self);
    self.g = g;
    self.g_vec = Vector2.init(0, -g);
    return ForceGen{
        .type = .downwards_gravity,
        .ptr = self,
        .vtable = Self.forcegenerator_vtable,
    };
}

pub fn deinit(ptr: *anyopaque, alloc: Allocator) void {
    const self: *Self = @ptrCast(@alignCast(ptr));
    alloc.destroy(self);
}

pub fn apply(ptr: *anyopaque, body: *RigidBody) void {
    const self: *Self = @ptrCast(@alignCast(ptr));
    if (body.static) return;
    body.props.force.addmult(self.g_vec, body.props.mass);
}
