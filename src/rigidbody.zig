const std = @import("std");
const Allocator = std.mem.Allocator;
const nmath = @import("nmath.zig");
const Vector2 = nmath.Vector2;
const collision = @import("collision.zig");
const manifold_max_points = collision.CollisionManifold.MAX_POINTS;

pub const RigidBodies = enum {
    disc,
    rectangle,
};

pub const Incident = union(enum) {
    edge: Edge.Points,
    point: Vector2,
};

pub const Edge = struct {
    pub const Points = struct {
        a: Vector2,
        b: Vector2,
    };
    dir: Vector2,
    // FIXME: better naming for the point att which the normal is applied?? in rect this is
    // the avg between the two lines, used primarily for rendering but may have other uses.
    middle: Vector2,
    edge: ?Points = null,
};

const EdgeNormalIterator = struct {
    num_iters: usize,
    iter_performed: usize = 0,

    const Self = @This();
    pub fn next(self: *Self, body: RigidBody, other: RigidBody) ?Edge {
        if (self.iter_performed < self.num_iters) {
            const ret = body.vtable.getNormal(body.ptr, body.props, other, self.iter_performed);
            self.iter_performed += 1;
            return ret;
        }
        self.iter_performed = 0;
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
        getNormal: *const fn (ptr: *anyopaque, props: RigidBody.Props, body: RigidBody, iter: usize) ?Edge,
        projectAlongAxis: *const fn (ptr: *anyopaque, props: Props, normal: Vector2) [2]f32,
        identifyCollisionPoints: *const fn (rigidself: *RigidBody, incident: *RigidBody, active_normal_iter: usize) [manifold_max_points]?Vector2,
        clipAgainstEdge: *const fn (rigidself: *RigidBody, edge: Edge.Points, normal: Vector2) Incident,
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
    };

    normal_iter: EdgeNormalIterator,
    static: bool = false,
    type: RigidBodies,
    /// Props are kinematic properties
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

    pub fn projectAlongAxis(self: Self, normal: Vector2) [2]f32 {
        return self.vtable.projectAlongAxis(self.ptr, self.props, normal);
    }

    pub fn localToWorld(self: Self, pos: Vector2) Vector2 {
        const r = nmath.rotate2(pos, self.props.angle);
        const ret = nmath.add2(r, self.props.pos);
        return ret;
    }

    pub fn identifyCollisionPoints(self: *Self, incident: *RigidBody, active_normal_iter: usize) [manifold_max_points]?Vector2 {
        return self.vtable.identifyCollisionPoints(self, incident, active_normal_iter);
    }

    pub fn clipAgainstEdge(self: *Self, edge: Edge.Points, normal: Vector2) Incident {
        return self.vtable.clipAgainstEdge(self, edge, normal);
    }

    pub fn worldToLocal(self: Self, pos: Vector2) Vector2 {
        const r = nmath.sub2(pos, self.props.pos);
        const ret = nmath.rotate2(r, -self.props.angle);
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
        .isInside = DiscBody.isInside,
        .closestPoint = DiscBody.closestPoint,
        .getNormal = DiscBody.getNormal,
        .projectAlongAxis = DiscBody.projectAlongAxis,
        .identifyCollisionPoints = DiscBody.identifyCollisionPoints,
        .clipAgainstEdge = DiscBody.clipAgainstEdge,
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
        // _ = ptr;
        // _ = pos;
        // return props.pos;

        const self: *Self = @ptrCast(@alignCast(ptr));
        const normal = nmath.normalize2(nmath.sub2(pos, props.pos));
        return nmath.addmult2(props.pos, normal, self.radius);
    }

    pub fn getNormal(ptr: *anyopaque, props: RigidBody.Props, body: RigidBody, iter: usize) ?Edge {
        _ = iter;
        const self: *Self = @ptrCast(@alignCast(ptr));

        const closest = body.closestPoint(props.pos);
        const normal = nmath.normalize2(nmath.sub2(closest, props.pos));
        return Edge{ .dir = normal, .middle = nmath.addmult2(props.pos, normal, self.radius) };
    }

    pub fn projectAlongAxis(ptr: *anyopaque, props: RigidBody.Props, normal: Vector2) [2]f32 {
        const self: *Self = @ptrCast(@alignCast(ptr));
        const middle = nmath.dot2(props.pos, normal);
        const rad = self.radius;
        return [2]f32{ middle - rad, middle + rad };
    }

    pub fn identifyCollisionPoints(rigidself: *RigidBody, incident: *RigidBody, active_normal_iter: usize) [manifold_max_points]?Vector2 {
        _ = active_normal_iter;
        // + circle SAT: normal = other.closestToSelf - self.pos.
        // + SAT --> clip incident body to infinitesmall small edge (on the circle's radius)
        // ==> what point on incident body intersects the line from pos + normal to pos?
        // by definition it is other.closestToSelf.
        const point = incident.closestPoint(rigidself.props.pos);

        var ret: [manifold_max_points]?Vector2 = undefined;
        for (0..manifold_max_points) |i| {
            ret[i] = null;
        }
        ret[0] = point;
        return ret;
    }

    pub fn clipAgainstEdge(rigidself: *RigidBody, edge: Edge.Points, normal: Vector2) Incident {
        const self: *Self = @ptrCast(@alignCast(rigidself.ptr));

        const dist = nmath.length2(nmath.sub2(edge.b, edge.a));
        // a -- (along tangent) --> b
        const tangent = nmath.rotate90clockwise(normal);

        // how far from a is self.pos? < 0 ==> left of A, > 0 ==> right of A
        const t = nmath.dot2(nmath.sub2(rigidself.props.pos, edge.a), tangent);

        if (t > 0.0 and t < dist) {
            return Incident{ .point = nmath.addmult2(rigidself.props.pos, normal, -self.radius) };
        }

        std.debug.print("circle clipAgainstSelf is outside of the middle!!", .{});

        const dx = if (t < 0.0) -t else dist - t;
        const dy = @sqrt(self.radius * self.radius - dx * dx);

        const t_y = nmath.dot2(nmath.sub2(rigidself.props.pos, edge.a), normal);
        const dy_mult: f32 = if (t_y < 0.0) 1 else -1;

        const pos = rigidself.props.pos;
        return Incident{ .point = Vector2.init(pos.x + dx, pos.y + dy_mult * dy) };
    }
};

pub const RectangleBody = struct {
    width: f32,
    height: f32,
    // technically unneccesary but simplifies life.
    local_vertices: [4]Vector2,

    const rigidbody_vtable = RigidBody.VTable{
        .deinit = RectangleBody.deinit,
        .isInside = RectangleBody.isInside,
        .closestPoint = RectangleBody.closestPoint,
        .getNormal = RectangleBody.getNormal,
        .projectAlongAxis = RectangleBody.projectAlongAxis,
        .identifyCollisionPoints = RectangleBody.identifyCollisionPoints,
        .clipAgainstEdge = RectangleBody.clipAgainstEdge,
    };

    const Self = @This();

    pub fn init(alloc: Allocator, pos: Vector2, angle: f32, mass: f32, width: f32, height: f32) !RigidBody {
        var self: *Self = try alloc.create(Self);
        self.width = width;
        self.height = height;

        const w = width / 2;
        const h = height / 2;
        self.local_vertices = [4]Vector2{ Vector2.init(-w, -h), Vector2.init(-w, h), Vector2.init(w, h), Vector2.init(w, -h) };

        return RigidBody{
            .props = .{
                .pos = pos,
                .momentum = .{},
                .force = .{},
                .angle = angle,
                .ang_momentum = 0,
                .torque = 0,
                .mass = mass,
                .inertia = mass * (width * width + height * height) / 12,
            },
            .normal_iter = EdgeNormalIterator{ .num_iters = 4 },
            .type = RigidBodies.rectangle,
            .ptr = self,
            .vtable = RectangleBody.rigidbody_vtable,
        };
    }

    pub fn deinit(ptr: *anyopaque, alloc: Allocator) void {
        const self: *Self = @ptrCast(@alignCast(ptr));
        alloc.destroy(self);
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

    pub fn getNormal(ptr: *anyopaque, props: RigidBody.Props, body: RigidBody, iter: usize) ?Edge {
        _ = body;
        const self: *Self = @ptrCast(@alignCast(ptr));

        const next_iter = if (iter == 3) 0 else iter + 1;

        const vert = self.local_vertices[iter];
        const next_vert = self.local_vertices[next_iter];

        const r1 = nmath.rotate2(vert, props.angle);
        const a1 = nmath.add2(r1, props.pos);

        const r2 = nmath.rotate2(next_vert, props.angle);
        const a2 = nmath.add2(r2, props.pos);

        // FIXME: use 1/width or 1/height or something based on iter
        const dir = nmath.normalize2(nmath.sub2(a2, a1));
        const normal = nmath.rotate90counterclockwise(dir);

        const avg = nmath.scale2(nmath.add2(a1, a2), 0.5);

        return Edge{ .dir = normal, .middle = avg, .edge = .{ .a = a1, .b = a2 } };
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

    pub fn identifyCollisionPoints(rigidself: *RigidBody, incident: *RigidBody, active_normal_iter: usize) [manifold_max_points]?Vector2 {
        const normal = RectangleBody.getNormal(rigidself.ptr, rigidself.props, incident.*, active_normal_iter);
        var ret: [manifold_max_points]?Vector2 = undefined;
        for (0..manifold_max_points) |i| {
            ret[i] = null;
        }

        if (normal) |n| {
            const incident_edge = incident.clipAgainstEdge(n.edge.?, n.dir);
            switch (incident_edge) {
                .edge => {
                    // ret[0] = incident_edge.edge.a;
                    // ret[1] = incident_edge.edge.b;

                    const a = incident_edge.edge.a;
                    const b = incident_edge.edge.b;

                    var i: usize = 0;

                    // check for positive penetration (non valid collision-point)
                    if (nmath.dot2(nmath.sub2(a, n.edge.?.a), n.dir) < 0.0) {
                        ret[i] = incident_edge.edge.a;
                        i += 1;
                    }
                    if (nmath.dot2(nmath.sub2(b, n.edge.?.b), n.dir) < 0.0) {
                        ret[i] = incident_edge.edge.b;
                    }
                },
                .point => {
                    ret[0] = incident_edge.point;
                },
            }
        } else {
            std.debug.print("In rect identifyCollisionPoints a non-valid active_normal_id has been used. i.e. a null normal has been detected.", .{});
        }

        return ret;
    }

    pub fn clipAgainstEdge(rigidself: *RigidBody, edge: Edge.Points, normal: Vector2) Incident {
        const self: *Self = @ptrCast(@alignCast(rigidself.ptr));

        var best_edge = Edge.Points{ .a = undefined, .b = undefined };
        var best_dot = std.math.inf(f32);

        var curr_world = rigidself.localToWorld(self.local_vertices[0]);
        for (0..4) |i| {
            const next_idx = if (i == 3) 0 else i + 1;
            const next_world = rigidself.localToWorld(self.local_vertices[next_idx]);

            const tangent = nmath.normalize2(nmath.sub2(next_world, curr_world));
            const tentative_normal = nmath.rotate90counterclockwise(tangent);

            const dot = nmath.dot2(normal, tentative_normal);
            if (dot < best_dot) {
                best_edge = Edge.Points{ .a = curr_world, .b = next_world };
                best_dot = dot;
            }

            curr_world = next_world;
        }

        const T = nmath.rotate90clockwise(normal);

        const A = edge.a;
        const B = edge.b;
        const C = best_edge.a;
        const D = best_edge.b;

        var Cp = C;
        var Dp = D;

        const line_dist = nmath.length2(nmath.sub2(A, B));

        const scalar_C = nmath.dot2(nmath.sub2(C, A), T);
        const scalar_D = nmath.dot2(nmath.sub2(D, A), T);

        const c_between = scalar_C > 0 and scalar_C < line_dist;
        const d_between = scalar_D > 0 and scalar_D < line_dist;

        if (!c_between) {
            const clos = if (scalar_C < line_dist / 2) A else B;

            const delta_L = nmath.sub2(C, D);
            const t = -nmath.dot2(T, nmath.sub2(D, clos)) / nmath.dot2(delta_L, T);
            Cp = nmath.addmult2(D, delta_L, t);
        }

        if (!d_between) {
            const clos = if (scalar_D < line_dist / 2) A else B;

            const delta_L = nmath.sub2(D, C);
            const t = -nmath.dot2(T, nmath.sub2(C, clos)) / nmath.dot2(delta_L, T);
            Dp = nmath.addmult2(C, delta_L, t);
        }

        return Incident{ .edge = .{ .a = Cp, .b = Dp } };
    }
};
