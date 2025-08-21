const std = @import("std");
const Allocator = std.mem.Allocator;
const AABB = @import("../AABB.zig");
const nmath = @import("../nmath.zig");
const Vector2 = nmath.Vector2;
const RigidBody = @import("../Bodies/RigidBody.zig");

const Self = @This();

pub const DownwardsGravity = @import("DownwardsGravity.zig");

pub const Type = enum {
    downwards_gravity,
};

pub const VTable = struct {
    deinit: *const fn (ptr: *anyopaque, alloc: Allocator) void,
    apply: *const fn (ptr: *anyopaque, body: *RigidBody) void,
};

type: Type,
vtable: VTable,
ptr: *anyopaque,

pub fn deinit(self: *Self, alloc: Allocator) void {
    self.vtable.deinit(self.ptr, alloc);
}

pub fn apply(self: *Self, body: *RigidBody) void {
    self.vtable.apply(self.ptr, body);
}
