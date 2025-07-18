const std = @import("std");
const Allocator = std.mem.Allocator;
const nmath = @import("nmath.zig");
const Vector2 = nmath.Vector2;
const collision = @import("collision.zig");
const manifold_max_points = collision.CollisionManifold.MAX_POINTS;
const CollisionPoint = collision.CollisionPoint;
const aabb = @import("aabb.zig");
const consts = @import("zigics_consts.zig");

pub const Incident = union(enum) {
    edge: Edge.Line,
    point: Vector2,
};

pub const Edge = struct {
    const Line = collision.Line;
    dir: Vector2,
    edge: ?Line = null,
};

/// All integraton is handled in the solver ==> basically only geometric properties/functions needs
/// to be implemented by the struct.
pub const RigidBody = struct {
    pub const Id = u64;

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
        identifyCollisionPoints: *const fn (rigidself: *RigidBody, incident: *RigidBody, active_normal_iter: usize) [manifold_max_points]?CollisionPoint,
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
    aabb: aabb.AABB,
    static: bool = false,
    num_normals: usize,
    type: Type,
    /// Props are kinematic properties
    props: Props,
    ptr: *anyopaque,
    vtable: VTable,

    const Self = @This();

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

    pub fn identifyCollisionPoints(self: *Self, incident: *RigidBody, active_normal_iter: usize) [manifold_max_points]?CollisionPoint {
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
};

test "local to world and inverses" {
    const WORLD = Vector2.init(3, 8.2);

    const rig_pos = Vector2.init(2, 3);
    const angle = 0.4;

    var r = nmath.sub2(WORLD, rig_pos);
    const local = nmath.rotate2(r, -angle);

    r = nmath.rotate2(local, angle);
    const RET = nmath.add2(r, rig_pos);

    try std.testing.expect(nmath.equals2(WORLD, RET));
}

pub const DiscBody = struct {
    radius: f32,

    const rigidbody_vtable = RigidBody.VTable{
        .deinit = DiscBody.deinit,
        .updateAABB = DiscBody.updateAABB,
        .isInside = DiscBody.isInside,
        .closestPoint = DiscBody.closestPoint,
        .getNormal = DiscBody.getNormal,
        .projectAlongAxis = DiscBody.projectAlongAxis,
        .identifyCollisionPoints = DiscBody.identifyCollisionPoints,
        .clipAgainstEdge = DiscBody.clipAgainstEdge,
    };

    const Self = @This();

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
            .vtable = DiscBody.rigidbody_vtable,
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

    pub fn identifyCollisionPoints(rigidself: *RigidBody, incident: *RigidBody, active_normal_iter: usize) [manifold_max_points]?CollisionPoint {
        const self: *Self = @ptrCast(@alignCast(rigidself.ptr));
        // + circle SAT: normal = other.closestToSelf - self.pos.
        // + SAT --> clip incident body to infinitesmall small edge (on the circle's radius)
        // ==> what point on incident body intersects the line from pos + normal to pos?
        // by definition it is other.closestToSelf.
        const pos = incident.closestPoint(rigidself.props.pos);
        const edge = DiscBody.getNormal(rigidself.ptr, rigidself.props, incident.*, active_normal_iter);
        var normal = edge.?.dir;

        if (collision.normalShouldFlipSAT(normal, rigidself, incident)) {
            normal.negate();
        }

        // normalize collisionpoint against the normal => depth
        const dot = nmath.dot2(normal, nmath.sub2(pos, rigidself.props.pos));
        const depth = dot - self.radius;

        var ret: [manifold_max_points]?CollisionPoint = undefined;
        for (0..manifold_max_points) |i| {
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
};

pub const RectangleBody = struct {
    width: f32,
    height: f32,
    // technically unneccesary but simplifies life.
    local_vertices: [4]Vector2,

    const rigidbody_vtable = RigidBody.VTable{
        .deinit = RectangleBody.deinit,
        .updateAABB = RectangleBody.updateAABB,
        .isInside = RectangleBody.isInside,
        .closestPoint = RectangleBody.closestPoint,
        .getNormal = RectangleBody.getNormal,
        .projectAlongAxis = RectangleBody.projectAlongAxis,
        .identifyCollisionPoints = RectangleBody.identifyCollisionPoints,
        .clipAgainstEdge = RectangleBody.clipAgainstEdge,
    };

    const Self = @This();

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
            .vtable = RectangleBody.rigidbody_vtable,
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

    pub fn identifyCollisionPoints(rigidself: *RigidBody, incident: *RigidBody, active_normal_iter: usize) [manifold_max_points]?CollisionPoint {
        const normal = RectangleBody.getNormal(rigidself.ptr, rigidself.props, incident.*, active_normal_iter);

        var ret: [manifold_max_points]?CollisionPoint = undefined;
        for (0..manifold_max_points) |i| {
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
        } else {
            // DEBUG FOR WASM:
            // std.debug.print("In rect identifyCollisionPoints a non-valid active_normal_id has been used. i.e. a null normal has been detected.", .{});
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
};
