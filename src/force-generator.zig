const std = @import("std");
const Allocator = std.mem.Allocator;
const rg_mod = @import("rigidbody.zig");
const RigidBody = rg_mod.RigidBody;
const nmath = @import("nmath.zig");
const Vector2 = nmath.Vector2;

pub const ForceGenerators = enum {
    downwards_gravity,
    point_gravity,
};

pub const ForceGenerator = struct {
    const VTable = struct {
        deinit: *const fn (ptr: *anyopaque, alloc: Allocator) void,
        apply: *const fn (ptr: *anyopaque, bodies: std.ArrayList(RigidBody)) void,
        energy: *const fn (ptr: *anyopaque, bodies: std.ArrayList(RigidBody)) f32,
    };

    type: ForceGenerators,
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
            .type = .downwards_gravity,
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
            body.props.force.addmult(self.g_vec, body.props.mass);
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

pub const PointGravity = struct {
    const Self = @This();
    const forcegenerator_vtable = ForceGenerator.VTable{
        .deinit = Self.deinit,
        .apply = Self.apply,
        .energy = Self.energy,
    };

    G: f32,
    pos: Vector2,

    pub fn init(alloc: Allocator, G: f32, pos: Vector2) !ForceGenerator {
        const self: *Self = try alloc.create(Self);
        self.G = G;
        self.pos = pos;
        return ForceGenerator{
            .type = .point_gravity,
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
            // F = G * m1 * m2 / r^2 * r_hat
            const vec = nmath.sub2(self.pos, body.props.pos);
            const dist = nmath.length2(vec);
            if (dist < 1e-2) continue;
            const dir = nmath.scale2(vec, 1 / dist);
            const F = self.G * body.props.mass / (dist * dist);
            body.props.force.addmult(dir, F);
        }
    }

    pub fn energy(ptr: *anyopaque, bodies: std.ArrayList(RigidBody)) f32 {
        const self: *Self = @ptrCast(@alignCast(ptr));
        var e: f32 = 0;
        for (bodies.items) |body| {
            const vec = nmath.sub2(self.pos, body.props.pos);
            const dist = nmath.length2(vec);
            const W = self.G * body.props.mass / dist;
            e += W;
        }
        return e;
    }
};
