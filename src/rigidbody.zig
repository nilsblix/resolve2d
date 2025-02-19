const std = @import("std");
const Allocator = std.mem.Allocator;
const nmath = @import("nmath.zig");
const Vector2 = nmath.Vector2;

pub const RigidBodies = enum {
    disc,
    rectangle,
};

const EdgeNormalIterator = struct {
    num_iters: usize,
    iter_performed: usize = 0,

    const Self = @This();
    pub fn next(self: *Self, body: RigidBody, other: RigidBody) ?Vector2 {
        if (self.iter_performed < self.num_iters) {
            const ret = body.vtable.getNormal(body.ptr, body.props, other, self.iter_performed);
            self.iter_performed += 1;
            return ret;
        }
        return null;
    }
};

/// All integraton is handled in the solver ==> basically only geometric properties/functions needs
/// to be implemented by the struct.
pub const RigidBody = struct {
    pub const VTable = struct {
        deinit: *const fn (ptr: *anyopaque, alloc: Allocator) void,
        isInside: *const fn (ptr: *anyopaque, props: Props, pos: Vector2) bool,
        closestPoint: *const fn (ptr: *anyopaque, props: Props, pos: Vector2) Vector2,

        getNormal: *const fn (ptr: *anyopaque, props: RigidBody.Props, body: RigidBody, iter: usize) ?Vector2,
        projectAlongNormal: *const fn (ptr: *anyopaque, props: Props, normal: Vector2) [2]f32,
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

    normal_iter: EdgeNormalIterator,
    type: RigidBodies,
    props: Props,
    ptr: *anyopaque,
    vtable: VTable,

    const Self = @This();

    pub fn deinit(self: *Self, alloc: Allocator) void {
        self.vtable.deinit(self.ptr, alloc);
    }

    pub fn isInside(self: Self, pos: Vector2) bool {
        return self.vtable.isInside(self.ptr, self.props, pos);
    }

    pub fn closestPoint(self: Self, pos: Vector2) Vector2 {
        return self.vtable.closestPoint(self.ptr, self.props, pos);
    }

    pub fn projectAlongNormal(self: Self, normal: Vector2) [2]f32 {
        return self.vtable.projectAlongNormal(self.ptr, self.props, normal);
    }
};

pub const DiscBody = struct {
    radius: f32,

    const rigidbody_vtable = RigidBody.VTable{
        .deinit = DiscBody.deinit,
        .isInside = DiscBody.isInside,
        .closestPoint = DiscBody.closestPoint,
        .getNormal = DiscBody.getNormal,
        .projectAlongNormal = DiscBody.projectAlongNormal,
    };

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
            .normal_iter = EdgeNormalIterator{ .num_iters = 1 },
            .type = RigidBodies.disc,
            .ptr = self,
            .vtable = DiscBody.rigidbody_vtable,
        };
    }

    pub fn deinit(ptr: *anyopaque, alloc: Allocator) void {
        const self: *Self = @ptrCast(@alignCast(ptr));
        alloc.destroy(self);
    }

    pub fn isInside(ptr: *anyopaque, props: RigidBody.Props, pos: Vector2) bool {
        const self: *Self = @ptrCast(@alignCast(ptr));

        const dist2 = nmath.length2sq(nmath.sub2(props.pos, pos));

        if (dist2 < self.radius * self.radius) {
            return true;
        }

        return false;
    }

    pub fn closestPoint(ptr: *anyopaque, props: RigidBody.Props, pos: Vector2) Vector2 {
        const self: *Self = @ptrCast(@alignCast(ptr));
        const normal = nmath.normalize2(nmath.sub2(pos, props.pos));
        return nmath.addmult2(props.pos, normal, self.radius);
    }

    pub fn getNormal(ptr: *anyopaque, props: RigidBody.Props, body: RigidBody, iter: usize) ?Vector2 {
        _ = ptr;
        _ = iter;
        const closest = body.closestPoint(props.pos);
        const normal = nmath.normalize2(nmath.sub2(closest, props.pos));
        return normal;
    }

    pub fn projectAlongNormal(ptr: *anyopaque, props: RigidBody.Props, normal: Vector2) [2]f32 {
        const self: *Self = @ptrCast(@alignCast(ptr));
        const middle = nmath.dot2(props.pos, normal);
        const rad = self.radius;
        return [2]f32{ middle - rad, middle + rad };
    }
};

pub const RectangleBody = struct {
    width: f32,
    height: f32,

    const rigidbody_vtable = RigidBody.VTable{
        .deinit = RectangleBody.deinit,
        .isInside = RectangleBody.isInside,
        .closestPoint = RectangleBody.closestPoint,
        .getNormal = RectangleBody.getNormal,
        .projectAlongNormal = RectangleBody.projectAlongNormal,
    };

    const Self = @This();

    pub fn init(alloc: Allocator, pos: Vector2, angle: f32, mass: f32, width: f32, height: f32) !RigidBody {
        var self: *Self = try alloc.create(Self);
        self.width = width;
        self.height = height;

        return RigidBody{
            .props = .{
                .pos = pos,
                .momentum = .{},
                .force = .{},
                .angle = angle,
                .ang_momentum = 0,
                .torque = 0,
                .mass = mass,
                .inertia = (1 / 12) * mass * (width * width + height * height),
            },
            .normal_iter = EdgeNormalIterator{ .num_iters = 1 },
            .type = RigidBodies.rectangle,
            .ptr = self,
            .vtable = RectangleBody.rigidbody_vtable,
        };
    }

    pub fn deinit(ptr: *anyopaque, alloc: Allocator) void {
        const self: *Self = @ptrCast(@alignCast(ptr));
        alloc.destroy(self);
    }

    pub fn isInside(ptr: *anyopaque, props: RigidBody.Props, pos: Vector2) bool {
        const self: *Self = @ptrCast(@alignCast(ptr));

        const dist2 = nmath.length2sq(nmath.sub2(props.pos, pos));

        if (dist2 < self.radius * self.radius) {
            return true;
        }

        return false;
    }

    pub fn closestPoint(ptr: *anyopaque, props: RigidBody.Props, pos: Vector2) Vector2 {
        const self: *Self = @ptrCast(@alignCast(ptr));
        const normal = nmath.normalize2(nmath.sub2(pos, props.pos));
        return nmath.addmult2(props.pos, normal, self.radius);
    }

    pub fn getNormal(ptr: *anyopaque, props: RigidBody.Props, body: RigidBody, iter: usize) ?Vector2 {
        _ = ptr;
        _ = iter;
        const closest = body.closestPoint(props.pos);
        const normal = nmath.normalize2(nmath.sub2(closest, props.pos));
        return normal;
    }

    pub fn projectAlongNormal(ptr: *anyopaque, props: RigidBody.Props, normal: Vector2) [2]f32 {
        const self: *Self = @ptrCast(@alignCast(ptr));
        const middle = nmath.dot2(props.pos, normal);
        const rad = self.radius;
        return [2]f32{ middle - rad, middle + rad };
    }
};
