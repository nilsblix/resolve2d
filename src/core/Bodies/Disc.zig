const std = @import("std");
const Allocator = std.mem.Allocator;
const AABB = @import("../AABB.zig");
const nmath = @import("../nmath.zig");
const Vector2 = nmath.Vector2;
const collision = @import("../collision.zig");
const MANIFOLD_MAX_PTS = collision.CollisionManifold.MAX_POINTS;
const CollisionPoint = collision.CollisionPoint;

const RigidBody = @import("RigidBody.zig");
const Incident = RigidBody.Incident;
const Edge = RigidBody.Edge;

const Self = @This();

radius: f32,

const rigidbody_vtable = RigidBody.VTable{
    .deinit = Self.deinit,
    .updateAABB = Self.updateAABB,
    .isInside = Self.isInside,
    .closestPoint = Self.closestPoint,
    .getNormal = Self.getNormal,
    .projectAlongAxis = Self.projectAlongAxis,
    .identifyCollisionPoints = Self.identifyCollisionPoints,
    .clipAgainstEdge = Self.clipAgainstEdge,
};

pub fn init(alloc: Allocator, id: RigidBody.Id, pos: Vector2, angle: f32, mass: f32, mu: f32, radius: f32) !RigidBody {
    var self: *Self = try alloc.create(Self);
    self.radius = radius;

    var rigidbody = RigidBody{
        .props = .{
            .pos = pos,
            .momentum = .{},
            .force = .{},
            .angle = angle,
            .ang_momentum = 0,
            .torque = 0,
            .mass = mass,
            .inertia = 0.5 * mass * radius * radius,
            .mu = mu,
        },
        .id = id,
        .aabb = undefined,
        .num_normals = 1,
        .type = RigidBody.Type.disc,
        .ptr = self,
        .vtable = Self.rigidbody_vtable,
    };

    rigidbody.updateAABB();

    return rigidbody;
}

pub fn deinit(ptr: *anyopaque, alloc: Allocator) void {
    const self: *Self = @ptrCast(@alignCast(ptr));
    alloc.destroy(self);
}

pub fn updateAABB(rigidself: *RigidBody) void {
    const self: *Self = @ptrCast(@alignCast(rigidself.ptr));
    rigidself.aabb = .{ .pos = rigidself.props.pos, .half_width = self.radius, .half_height = self.radius };
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

pub fn getNormal(_: *anyopaque, props: RigidBody.Props, body: RigidBody, _: usize) ?Edge {
    const closest = body.closestPoint(props.pos);
    const normal = nmath.normalize2(nmath.sub2(closest, props.pos));
    return Edge{ .dir = normal };
}

pub fn projectAlongAxis(ptr: *anyopaque, props: RigidBody.Props, normal: Vector2) [2]f32 {
    const self: *Self = @ptrCast(@alignCast(ptr));
    const middle = nmath.dot2(props.pos, normal);
    const rad = self.radius;
    return [2]f32{ middle - rad, middle + rad };
}

pub fn identifyCollisionPoints(rigidself: *RigidBody, incident: *RigidBody, active_normal_iter: usize) [MANIFOLD_MAX_PTS]?CollisionPoint {
    const self: *Self = @ptrCast(@alignCast(rigidself.ptr));
    // + circle SAT: normal = other.closestToSelf - self.pos.
    // + SAT --> clip incident body to infinitesmall small edge (on the circle's radius)
    // ==> what point on incident body intersects the line from pos + normal to pos?
    // by definition it is other.closestToSelf.
    const pos = incident.closestPoint(rigidself.props.pos);
    const edge = Self.getNormal(rigidself.ptr, rigidself.props, incident.*, active_normal_iter);
    var normal = edge.?.dir;

    if (collision.normalShouldFlipSAT(normal, rigidself, incident)) {
        normal.negate();
    }

    // normalize collisionpoint against the normal => depth
    const dot = nmath.dot2(normal, nmath.sub2(pos, rigidself.props.pos));
    const depth = dot - self.radius;

    var ret: [MANIFOLD_MAX_PTS]?CollisionPoint = undefined;
    for (0..MANIFOLD_MAX_PTS) |i| {
        ret[i] = null;
    }
    ret[0] = CollisionPoint.init(pos, depth, rigidself.*, incident.*, normal);
    return ret;
}

pub fn clipAgainstEdge(rigidself: *RigidBody, edge: Edge.Line, normal: Vector2) Incident {
    const self: *Self = @ptrCast(@alignCast(rigidself.ptr));
    _ = edge;

    return Incident{ .point = nmath.addmult2(rigidself.props.pos, normal, -self.radius) };
}
