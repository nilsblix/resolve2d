const std = @import("std");
const Allocator = std.mem.Allocator;
const nmath = @import("nmath.zig");
const Vector2 = nmath.Vector2;

pub const RigidBodies = enum {
    disc,
};

/// All integraton is handled in the solver ==> basically only geometric properties/functions needs
/// to be implemented by the struct.
pub const RigidBody = struct {
    pub const VTable = struct {
        deinit: *const fn (ptr: *anyopaque, alloc: Allocator) void,
        print: *const fn (ptr: *anyopaque, props: Props) void,
        isInside: *const fn (ptr: *anyopaque, props: Props, pos: Vector2) bool,
    };

    pub const Props = struct {
        // linear
        pos: Vector2,
        momentum: Vector2,
        force: Vector2,
        mass: f32,
        // rotational
        angle: f32,
        ang_momentum: f32,
        torque: f32,
        inertia: f32,
    };

    type: RigidBodies,
    props: Props,
    ptr: *anyopaque,
    vtable: VTable,

    const Self = @This();

    pub fn deinit(self: *Self, alloc: Allocator) void {
        self.vtable.deinit(self.ptr, alloc);
    }

    pub fn print(self: Self) void {
        self.vtable.print(self.ptr, self.props);
    }

    pub fn isInside(self: Self, pos: Vector2) bool {
        return self.vtable.isInside(self.ptr, self.props, pos);
    }
};

pub const DiscBody = struct {
    radius: f32,

    const rigidbody_vtable = RigidBody.VTable{
        .deinit = DiscBody.deinit,
        .print = DiscBody.print,
        .isInside = DiscBody.isInside,
    };

    pub const name: []const u8 = "DiscBody";
    const Self = @This();

    pub fn init(alloc: Allocator, pos: Vector2, angle: f32, mass: f32, radius: f32) !RigidBody {
        var self: *Self = try alloc.create(Self);
        self.radius = radius;

        return RigidBody{
            .props = .{
                .pos = pos,
                .momentum = .{},
                .force = .{},
                .angle = angle,
                .ang_momentum = 0,
                .torque = 0,
                .mass = mass,
                .inertia = 0.5 * mass * radius * radius,
            },
            .type = RigidBodies.disc,
            .ptr = self,
            .vtable = DiscBody.rigidbody_vtable,
        };
    }

    pub fn deinit(ptr: *anyopaque, alloc: Allocator) void {
        const self: *Self = @ptrCast(@alignCast(ptr));
        alloc.destroy(self);
    }

    pub fn print(ptr: *anyopaque, props: RigidBody.Props) void {
        const self: *Self = @ptrCast(@alignCast(ptr));
        std.debug.print("DiscBody =>\n", .{});
        std.debug.print("     Props = {}\n", .{props});
        std.debug.print("     Other = {}\n", .{self});
    }

    pub fn isInside(ptr: *anyopaque, props: RigidBody.Props, pos: Vector2) bool {
        const self: *Self = @ptrCast(@alignCast(ptr));

        const dist2 = nmath.length2sq(nmath.sub2(props.pos, pos));

        if (dist2 < self.radius * self.radius) {
            return true;
        }

        return false;
    }
};
