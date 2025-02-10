const std = @import("std");
const Allocator = std.mem.Allocator;
const rg_mod = @import("rigidbody.zig");
const RigidBody = rg_mod.RigidBody;
const nmath = @import("nmath.zig");
const Vector2 = nmath.Vector2;

pub const ForceGenerator = struct {
    const VTable = struct {
        deinit: *const fn (ptr: *anyopaque, alloc: Allocator) void,
        apply: *const fn (ptr: *anyopaque, bodies: std.ArrayList(RigidBody)) void,
        energy: *const fn (ptr: *anyopaque, bodies: std.ArrayList(RigidBody)) f32,
    };

    vtable: VTable,
    ptr: *anyopaque,

    const Self = @This();

    pub fn deinit(self: *Self, alloc: Allocator) void {
        self.vtable.deinit(self.ptr, alloc);
    }

    pub fn apply(self: *Self, bodies: std.ArrayList(RigidBody)) void {
        self.vtable.apply(self.ptr, bodies);
    }
};

pub const DownwardsGravity = struct {
    const Self = @This();
    const forcegenerator_vtable = ForceGenerator.VTable{
        .deinit = Self.deinit,
        .apply = Self.apply,
        .energy = Self.energy,
    };

    g: f32,
    g_vec: Vector2,

    pub fn init(alloc: Allocator, g: f32) !ForceGenerator {
        const self: *Self = try alloc.create(Self);
        self.g = g;
        self.g_vec = Vector2.init(0, -g);
        return ForceGenerator{
            .ptr = self,
            .vtable = Self.forcegenerator_vtable,
        };
    }

    pub fn deinit(ptr: *anyopaque, alloc: Allocator) void {
        const self: *Self = @ptrCast(@alignCast(ptr));
        alloc.destroy(self);
    }

    pub fn apply(ptr: *anyopaque, bodies: std.ArrayList(RigidBody)) void {
        const self: *Self = @ptrCast(@alignCast(ptr));
        for (bodies.items) |*body| {
            body.props.force = nmath.add2(body.props.force, self.g_vec);
        }
    }

    pub fn energy(ptr: *anyopaque, bodies: std.ArrayList(RigidBody)) f32 {
        const self: *Self = @ptrCast(@alignCast(ptr));
        var e: f32 = 0;
        for (bodies.items) |body| {
            e += body.props.pos.y * body.props.mass * self.g;
        }
        return e;
    }
};
