const std = @import("std");
const Allocator = std.mem.Allocator;
const rg_mod = @import("rigidbody.zig");
const RigidBody = rg_mod.RigidBody;
const nmath = @import("nmath.zig");
const Vector2 = nmath.Vector2;

pub const ForceGenerator = struct {
    pub const Type = enum {
        downwards_gravity,
    };

    const VTable = struct {
        deinit: *const fn (ptr: *anyopaque, alloc: Allocator) void,
        apply: *const fn (ptr: *anyopaque, body: *RigidBody) void,
    };

    // FIXME: replace this name with kind.
    type: Type,
    vtable: VTable,
    ptr: *anyopaque,

    const Self = @This();

    pub fn deinit(self: *Self, alloc: Allocator) void {
        self.vtable.deinit(self.ptr, alloc);
    }

    pub fn apply(self: *Self, body: *RigidBody) void {
        self.vtable.apply(self.ptr, body);
    }
};

pub const DownwardsGravity = struct {
    const Self = @This();
    const forcegenerator_vtable = ForceGenerator.VTable{
        .deinit = Self.deinit,
        .apply = Self.apply,
    };

    g: f32,
    g_vec: Vector2,

    pub fn init(alloc: Allocator, g: f32) !ForceGenerator {
        const self: *Self = try alloc.create(Self);
        self.g = g;
        self.g_vec = Vector2.init(0, -g);
        return ForceGenerator{
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
        body.props.force.y -= body.props.mass * self.g;
        // body.props.force.addmult(self.g_vec, body.props.mass);
    }
};
