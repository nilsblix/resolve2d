const std = @import("std");
const Allocator = std.mem.Allocator;
const AABB = @import("../AABB.zig");
const nmath = @import("../nmath.zig");
const Vector2 = nmath.Vector2;
const collision = @import("../collision.zig");
const MANIFOLD_MAX_PTS = collision.CollisionManifold.MAX_POINTS;
const CollisionPoint = collision.CollisionPoint;
const consts = @import("../simulation_constants.zig");

const RigidBody = @import("RigidBody.zig");
const Incident = RigidBody.Incident;
const Edge = RigidBody.Edge;

const Self = @This();

width: f32,
height: f32,
// technically unneccesary but simplifies life.
local_vertices: [4]Vector2,

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

pub fn init(alloc: Allocator, id: RigidBody.Id, pos: Vector2, angle: f32, mass: f32, mu: f32, width: f32, height: f32) !RigidBody {
    var self: *Self = try alloc.create(Self);
    self.width = width;
    self.height = height;

    const w = width / 2;
    const h = height / 2;
    self.local_vertices = [4]Vector2{ Vector2.init(-w, -h), Vector2.init(-w, h), Vector2.init(w, h), Vector2.init(w, -h) };

    var rigidbody = RigidBody{
        .props = .{
            .pos = pos,
            .momentum = .{},
            .force = .{},
            .angle = angle,
            .ang_momentum = 0,
            .torque = 0,
            .mass = mass,
            .inertia = mass * (width * width + height * height) / 12,
            .mu = mu,
        },
        .id = id,
        .aabb = undefined,
        .num_normals = 4,
        .type = RigidBody.Type.rectangle,
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
    var width: f32 = 0;
    var height: f32 = 0;
    for (self.local_vertices) |vert| {
        const rot = nmath.rotate2(vert, rigidself.props.angle);
        if (rot.x > width) {
            width = rot.x;
        }
        if (rot.y > height) {
            height = rot.y;
        }
    }

    rigidself.aabb = .{ .pos = rigidself.props.pos, .half_width = width, .half_height = height };
}

pub fn getWorldVertices(self: *Self, props: RigidBody.Props) [4]Vector2 {
    var ret: [4]Vector2 = undefined;

    for (self.local_vertices, 0..) |vert, idx| {
        const r = nmath.rotate2(vert, props.angle);
        const world = nmath.add2(r, props.pos);
        ret[idx] = world;
    }

    return ret;
}

pub fn isInside(ptr: *anyopaque, props: RigidBody.Props, pos: Vector2) bool {
    const self: *Self = @ptrCast(@alignCast(ptr));

    const pos_local = nmath.sub2(pos, props.pos);
    const r = nmath.rotate2(pos_local, -props.angle);

    const w = self.width / 2;
    const h = self.height / 2;

    if (r.x > -w and r.x < w and r.y > -h and r.y < h) return true;

    return false;
}

pub fn closestPoint(ptr: *anyopaque, props: RigidBody.Props, pos: Vector2) Vector2 {
    const self: *Self = @ptrCast(@alignCast(ptr));

    var best_dist2: f32 = std.math.inf(f32);
    var best_pos: Vector2 = pos;

    const world_vertices = self.getWorldVertices(props);
    for (world_vertices) |vert| {
        const dist2 = nmath.length2sq(nmath.sub2(vert, pos));
        if (dist2 < best_dist2) {
            best_dist2 = dist2;
            best_pos = vert;
        }
    }

    return best_pos;
}

pub fn getNormal(ptr: *anyopaque, props: RigidBody.Props, _: RigidBody, iter: usize) ?Edge {
    const self: *Self = @ptrCast(@alignCast(ptr));

    const next_iter = if (iter == 3) 0 else iter + 1;

    const vert = self.local_vertices[iter];
    const next_vert = self.local_vertices[next_iter];

    const r1 = nmath.rotate2(vert, props.angle);
    const a1 = nmath.add2(r1, props.pos);

    const r2 = nmath.rotate2(next_vert, props.angle);
    const a2 = nmath.add2(r2, props.pos);

    // POTENTIAL: use 1/w or 1/h instead of @sqrt.
    const dir = nmath.normalize2(nmath.sub2(a2, a1));
    const normal = nmath.rotate90counterclockwise(dir);

    return Edge{ .dir = normal, .edge = .{ .a = a1, .b = a2 } };
}

pub fn projectAlongAxis(ptr: *anyopaque, props: RigidBody.Props, normal: Vector2) [2]f32 {
    const self: *Self = @ptrCast(@alignCast(ptr));

    var best_low: f32 = std.math.inf(f32);
    var best_high: f32 = -std.math.inf(f32);

    for (self.local_vertices) |vert| {
        const r = nmath.rotate2(vert, props.angle);
        const world = nmath.add2(r, props.pos);

        const dot = nmath.dot2(world, normal);
        if (dot < best_low) best_low = dot;
        if (dot > best_high) best_high = dot;
    }

    return [2]f32{ best_low, best_high };
}

pub fn identifyCollisionPoints(rigidself: *RigidBody, incident: *RigidBody, active_normal_iter: usize) [MANIFOLD_MAX_PTS]?CollisionPoint {
    const normal = Self.getNormal(rigidself.ptr, rigidself.props, incident.*, active_normal_iter);

    var ret: [MANIFOLD_MAX_PTS]?CollisionPoint = undefined;
    for (0..MANIFOLD_MAX_PTS) |i| {
        ret[i] = null;
    }

    if (normal) |n| {
        const incident_edge = incident.clipAgainstEdge(n);
        switch (incident_edge) {
            .edge => {
                const a = incident_edge.edge.a;
                const b = incident_edge.edge.b;

                var i: usize = 0;

                // check for positive penetration (non valid collision-point)
                var dot = nmath.dot2(nmath.sub2(a, n.edge.?.a), n.dir);
                if (dot < consts.COLLISION_MARGIN) {
                    const pos = incident_edge.edge.a;
                    ret[i] = CollisionPoint.init(pos, dot, rigidself.*, incident.*, n.dir);
                    i += 1;
                }

                dot = nmath.dot2(nmath.sub2(b, n.edge.?.b), n.dir);
                if (dot < consts.COLLISION_MARGIN) {
                    const pos = incident_edge.edge.b;
                    ret[i] = CollisionPoint.init(pos, dot, rigidself.*, incident.*, n.dir);
                }
            },
            .point => {
                const pos = incident_edge.point;
                const dot = nmath.dot2(nmath.sub2(pos, n.edge.?.a), n.dir);
                ret[0] = CollisionPoint.init(pos, dot, rigidself.*, incident.*, n.dir);
            },
        }
    }

    return ret;
}

pub fn clipAgainstEdge(rigidself: *RigidBody, edge: Edge.Line, normal: Vector2) Incident {
    const self: *Self = @ptrCast(@alignCast(rigidself.ptr));

    var best_edge = Edge.Line{ .a = undefined, .b = undefined };
    var best_dot = std.math.inf(f32);

    var curr_world = rigidself.localToWorld(self.local_vertices[0]);
    for (0..4) |i| {
        const next_idx = if (i == 3) 0 else i + 1;
        const next_world = rigidself.localToWorld(self.local_vertices[next_idx]);

        const tangent = nmath.normalize2(nmath.sub2(next_world, curr_world));
        const tentative_normal = nmath.rotate90counterclockwise(tangent);

        const dot = nmath.dot2(normal, tentative_normal);
        if (dot < best_dot) {
            best_edge = Edge.Line{ .a = curr_world, .b = next_world };
            best_dot = dot;
        }

        curr_world = next_world;
    }

    const clipped = collision.clipLineToLine(.{ .a = edge.a, .b = edge.b }, .{ .a = best_edge.a, .b = best_edge.b });
    return Incident{ .edge = .{ .a = clipped.a, .b = clipped.b } };
}
