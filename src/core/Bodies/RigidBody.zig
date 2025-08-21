const std = @import("std");
const Allocator = std.mem.Allocator;
const AABB = @import("../AABB.zig");
const nmath = @import("../nmath.zig");
const Vector2 = nmath.Vector2;
const collision = @import("../collision.zig");
const MANIFOLD_MAX_PTS = collision.CollisionManifold.MAX_POINTS;
const CollisionPoint = collision.CollisionPoint;

const RigidBody = @This();
const Self = @This();

pub const Disc = @import("Disc.zig");
pub const Rectangle = @import("Rectangle.zig");

pub const Id = u16;

pub const Incident = union(enum) {
    edge: Edge.Line,
    point: Vector2,
};

pub const Edge = struct {
    pub const Line = collision.Line;
    dir: Vector2,
    edge: ?Line = null,
};

pub const Type = enum {
    disc,
    rectangle,
};

pub const VTable = struct {
    deinit: *const fn (ptr: *anyopaque, alloc: Allocator) void,
    updateAABB: *const fn (rigidself: *RigidBody) void,
    isInside: *const fn (ptr: *anyopaque, props: Props, pos: Vector2) bool,
    closestPoint: *const fn (ptr: *anyopaque, props: Props, pos: Vector2) Vector2,
    getNormal: *const fn (ptr: *anyopaque, props: RigidBody.Props, body: RigidBody, iter: usize) ?Edge,
    projectAlongAxis: *const fn (ptr: *anyopaque, props: Props, normal: Vector2) [2]f32,
    identifyCollisionPoints: *const fn (rigidself: *RigidBody, incident: *RigidBody, active_normal_iter: usize) [MANIFOLD_MAX_PTS]?CollisionPoint,
    clipAgainstEdge: *const fn (rigidself: *RigidBody, edge: Edge.Line, normal: Vector2) Incident,
};

// Use this to iterate over a bodies normals. Used with body.makeNormalIter(...)
pub const NormalIter = struct {
    b1: *RigidBody,
    b2: *RigidBody,

    num_iters: usize,
    iter_performed: usize = 0,

    const Iter = @This();
    pub fn next(self: *Iter) ?Edge {
        if (self.iter_performed < self.num_iters) {
            const ret = self.b1.vtable.getNormal(self.b1.ptr, self.b1.props, self.b2.*, self.iter_performed);
            self.iter_performed += 1;
            return ret;
        }
        self.iter_performed = 0;
        return null;
    }

    pub fn reset(self: *Iter) void {
        self.iter_performed = 0;
    }
};

/// Props are kinematic properties
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
    mu: f32,
};

id: RigidBody.Id,
aabb: AABB,
static: bool = false,
num_normals: usize,
type: Type,
/// Props are kinematic properties
props: Props,
ptr: *anyopaque,
vtable: VTable,

pub fn deinit(self: *Self, alloc: Allocator) void {
    self.vtable.deinit(self.ptr, alloc);
}

pub fn updateAABB(self: *Self) void {
    self.vtable.updateAABB(self);
}

pub fn isInside(self: Self, pos: Vector2) bool {
    return self.vtable.isInside(self.ptr, self.props, pos);
}

pub fn closestPoint(self: Self, pos: Vector2) Vector2 {
    return self.vtable.closestPoint(self.ptr, self.props, pos);
}

pub fn projectAlongAxis(self: Self, normal: Vector2) [2]f32 {
    return self.vtable.projectAlongAxis(self.ptr, self.props, normal);
}

pub fn localToWorld(self: Self, pos: Vector2) Vector2 {
    const r = nmath.rotate2(pos, self.props.angle);
    const ret = nmath.add2(r, self.props.pos);
    return ret;
}

pub fn identifyCollisionPoints(self: *Self, incident: *RigidBody, active_normal_iter: usize) [MANIFOLD_MAX_PTS]?CollisionPoint {
    return self.vtable.identifyCollisionPoints(self, incident, active_normal_iter);
}

pub fn clipAgainstEdge(self: *Self, edge: Edge) Incident {
    return self.vtable.clipAgainstEdge(self, edge.edge.?, edge.dir);
}

pub fn worldToLocal(self: Self, pos: Vector2) Vector2 {
    const r = nmath.sub2(pos, self.props.pos);
    const ret = nmath.rotate2(r, -self.props.angle);
    return ret;
}

pub fn makeNormalIter(self: *Self, other: *RigidBody) RigidBody.NormalIter {
    const ret = RigidBody.NormalIter{ .b1 = self, .b2 = other, .num_iters = self.num_normals };
    return ret;
}
