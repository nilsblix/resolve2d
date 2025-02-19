const std = @import("std");
const Allocator = std.mem.Allocator;
const nmath = @import("nmath.zig");
const Vector2 = nmath.Vector2;

pub const RigidBodies = enum {
    disc,
    rectangle,
};

const Edge = struct {
    const Points = struct {
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

        var best_dist2: f32 = undefined;
        var best_pos: Vector2 = undefined;

        const world_vertices = self.getWorldVertices(props);
        for (world_vertices) |vert| {
            const dist2 = nmath.length2sq(nmath.sub2(vert, pos));
            if (dist2 < best_dist2 or best_dist2 == undefined) {
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

        const avg = nmath.scale2(nmath.add2(a1, a2), 0.5);

        return Edge{ .dir = nmath.rotate90counterclockwise(dir), .middle = avg, .edge = .{ .a = a1, .b = a2 } };
    }

    pub fn projectAlongAxis(ptr: *anyopaque, props: RigidBody.Props, normal: Vector2) [2]f32 {
        const self: *Self = @ptrCast(@alignCast(ptr));

        var best_low: f32 = undefined;
        var best_high: f32 = undefined;

        for (self.local_vertices) |vert| {
            const r = nmath.rotate2(vert, props.angle);
            const world = nmath.add2(r, props.pos);

            const dot = nmath.dot2(world, normal);
            if (dot < best_low or best_low == undefined) best_low = dot;
            if (dot > best_high or best_high == undefined) best_high = dot;
        }

        return [2]f32{ best_low, best_high };
    }
};
