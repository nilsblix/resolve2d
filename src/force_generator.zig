const std = @import("std");
const Allocator = std.mem.Allocator;
const rg_mod = @import("rigidbody.zig");
const RigidBody = rg_mod.RigidBody;
const nmath = @import("nmath.zig");
const Vector2 = nmath.Vector2;

pub const ForceGenerators = enum {
    downwards_gravity,
    point_gravity,
    static_spring,
};

pub const ForceGenerator = struct {
    const VTable = struct {
        deinit: *const fn (ptr: *anyopaque, alloc: Allocator) void,
        apply: *const fn (ptr: *anyopaque, bodies: std.ArrayList(RigidBody)) void,
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
            if (body.static) continue;
            body.props.force.addmult(self.g_vec, body.props.mass);
        }
    }
};

pub const StaticSpring = struct {
    const Self = @This();
    const forcegenerator_vtable = ForceGenerator.VTable{
        .deinit = Self.deinit,
        .apply = Self.apply,
    };

    body: *RigidBody,
    r: Vector2,
    pos: Vector2,
    stiffness: f32,
    rest_length: f32,

    /// rest_length is calculated as the distance between pos and connected body.
    pub fn init(alloc: Allocator, connected_body: *RigidBody, pos: Vector2, r: Vector2, stiffness: f32) !ForceGenerator {
        const self: *Self = try alloc.create(Self);
        self.body = connected_body;
        self.pos = pos;
        self.r = r;
        self.stiffness = stiffness;
        // FIXME: uncomment
        // const rest_length = nmath.length2(nmath.sub2(connected_body.props.pos, pos));
        // self.rest_length = rest_length;
        self.rest_length = 0;

        return ForceGenerator{
            .type = .static_spring,
            .ptr = self,
            .vtable = Self.forcegenerator_vtable,
        };
    }

    pub fn deinit(ptr: *anyopaque, alloc: Allocator) void {
        const self: *Self = @ptrCast(@alignCast(ptr));
        alloc.destroy(self);
    }

    pub fn apply(ptr: *anyopaque, bodies: std.ArrayList(RigidBody)) void {
        _ = bodies;
        const self: *Self = @ptrCast(@alignCast(ptr));

        if (self.body.static) return;

        const rotated_r = nmath.rotate2(self.r, self.body.props.angle);
        const applied_pos = nmath.add2(rotated_r, self.body.props.pos);

        const vec = nmath.sub2(self.pos, applied_pos);
        const len = nmath.length2(vec);

        if (len < 1e-3) return;

        const F = (len - self.rest_length) * self.stiffness;

        const vec_normal = nmath.scale2(vec, 1 / len);
        const force_vec = nmath.scale2(vec_normal, F);

        self.body.props.force.add(force_vec);
        self.body.props.torque += nmath.cross2(rotated_r, force_vec);
    }
};
