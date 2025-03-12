const std = @import("std");
const Allocator = std.mem.Allocator;
const nmath = @import("nmath.zig");
const Vector2 = nmath.Vector2;
const rb_mod = @import("rigidbody.zig");
const RigidBody = rb_mod.RigidBody;
const fg_mod = @import("force_generator.zig");
const ForceGenerator = fg_mod.ForceGenerator;
const rl = @import("raylib");
const def_rend = @import("default_renderer.zig");
pub const Units = def_rend.Units;
const Renderer = def_rend.Renderer;
const clsn = @import("collision.zig");
const qtree = @import("quadtree.zig");
const QuadTree = qtree.QuadTree;

pub const EntityFactory = struct {
    pub const BodyOptions = struct {
        pos: Vector2,
        vel: Vector2 = .{},
        angle: f32 = 0.0,
        omega: f32 = 0.0,
        mu_s: f32 = 0.5,
        mu_d: f32 = 0.4,
        mass_prop: union(enum) {
            /// kg / m^2
            density: f32,
            mass: f32,
        },
    };

    pub const DiscOptions = struct {
        radius: f32 = 1.0,
    };

    pub const RectangleOptions = struct {
        width: f32 = 1.0,
        height: f32 = 0.5,
    };

    pub const GeometryOptions = union(enum) { disc: DiscOptions, rectangle: RectangleOptions };

    solver: *Solver,

    const Self = @This();

    pub fn makeDiscBody(self: *Self, rigid_opt: BodyOptions, geometry_opt: DiscOptions) !*RigidBody {
        const mass = switch (rigid_opt.mass_prop) {
            .mass => |m| m,
            .density => |density| @as(f32, std.math.pi) * geometry_opt.radius * geometry_opt.radius * density,
        };

        var body = try rb_mod.DiscBody.init(self.solver.alloc, rigid_opt.pos, rigid_opt.angle, mass, rigid_opt.mu_s, rigid_opt.mu_d, geometry_opt.radius);

        body.props.momentum = nmath.scale2(rigid_opt.vel, mass);
        body.props.ang_momentum = rigid_opt.omega * body.props.inertia;

        try self.solver.bodies.append(body);
        return &self.solver.bodies.items[self.solver.bodies.items.len - 1];
    }

    pub fn makeRectangleBody(self: *Self, rigid_opt: BodyOptions, geometry_opt: RectangleOptions) !*RigidBody {
        const mass = switch (rigid_opt.mass_prop) {
            .mass => |m| m,
            .density => |density| geometry_opt.width * geometry_opt.height * density,
        };

        var body = try rb_mod.RectangleBody.init(self.solver.alloc, rigid_opt.pos, rigid_opt.angle, mass, rigid_opt.mu_s, rigid_opt.mu_d, geometry_opt.width, geometry_opt.height);

        body.props.momentum = nmath.scale2(rigid_opt.vel, mass);
        body.props.ang_momentum = rigid_opt.omega * body.props.inertia;

        try self.solver.bodies.append(body);
        return &self.solver.bodies.items[self.solver.bodies.items.len - 1];
    }

    pub fn makeDownwardsGravity(self: *Self, g: f32) !void {
        try self.solver.force_generators.append(try fg_mod.DownwardsGravity.init(self.solver.alloc, g));
    }
};

// pub fn Solver(comptime qtree_node_threshold: usize, comptime qtree_max_depth: usize) type {
pub const Solver = struct {
    alloc: Allocator,
    bodies: std.ArrayList(RigidBody),
    force_generators: std.ArrayList(ForceGenerator),
    manifolds: std.AutoArrayHashMap(clsn.CollisionKey, clsn.CollisionManifold),
    quadtree: QuadTree,

    const Self = @This();
    pub fn init(alloc: Allocator, comptime quadtree_node_threshold: usize, comptime quadtree_max_depth: usize) !Self {
        return .{
            .alloc = alloc,
            .bodies = std.ArrayList(RigidBody).init(alloc),
            .force_generators = std.ArrayList(ForceGenerator).init(alloc),
            .manifolds = std.AutoArrayHashMap(clsn.CollisionKey, clsn.CollisionManifold).init(alloc),
            .quadtree = try QuadTree.init(alloc, quadtree_node_threshold, quadtree_max_depth),
        };
    }

    pub fn deinit(self: *Self, alloc: Allocator) void {
        self.manifolds.deinit();
        for (self.force_generators.items) |*gen| {
            gen.deinit(self.alloc);
        }
        self.force_generators.deinit();
        for (self.bodies.items) |*body| {
            body.deinit(self.alloc);
        }
        self.bodies.deinit();
        self.quadtree.deinit(alloc);
    }

    pub fn clear(self: *Self, alloc: Allocator) !void {
        self.deinit(alloc);
        self.bodies = std.ArrayList(RigidBody).init(alloc);
        self.force_generators = std.ArrayList(ForceGenerator).init(alloc);
        self.manifolds = std.AutoArrayHashMap(clsn.CollisionKey, clsn.CollisionManifold).init(alloc);
        // self.quadtree.clear(alloc);
        self.quadtree.initRoot(alloc);
    }

    pub fn process(self: *Self, alloc: Allocator, dt: f32, sub_steps: usize, collision_iters: usize) !void {
        const f32_sub: f32 = @floatFromInt(sub_steps);
        const sub_dt = dt / f32_sub;

        var time_spent_updating_manifolds: i64 = 0;
        var time_spent_solving_collisions: i64 = 0;

        for (0..sub_steps) |_| {
            for (self.force_generators.items) |*gen| {
                gen.apply(self.bodies);
            }

            for (self.bodies.items) |*body| {
                if (body.static) continue;
                var props: *RigidBody.Props = &body.props;

                props.momentum.addmult(props.force, sub_dt);
                props.ang_momentum += props.torque * sub_dt;

                body.updateAABB();
            }

            var st = std.time.microTimestamp();
            try self.updateManifolds(alloc);
            var et = std.time.microTimestamp();
            time_spent_updating_manifolds += et - st;

            st = std.time.microTimestamp();
            for (0..collision_iters) |_| {
                var iter = self.manifolds.iterator();
                while (iter.next()) |entry| {
                    const manifold = entry.value_ptr;
                    const key = entry.key_ptr.*;
                    manifold.resetImpulses();
                    manifold.calculateImpulses(key, sub_dt, 0.05, 0.00);
                    manifold.applyImpulses(key, 1e-4, 4e-1);
                }
            }
            et = std.time.microTimestamp();
            time_spent_solving_collisions += et - st;

            for (self.bodies.items) |*body| {
                if (body.static) continue;
                var props: *RigidBody.Props = &body.props;

                props.pos.addmult(props.momentum, sub_dt / props.mass);
                props.angle += props.ang_momentum * sub_dt / props.inertia;

                props.force = .{};
                props.torque = 0;
            }
        }
        // std.debug.print("time spent colliding = {d} ms\n", .{@as(f32, @floatFromInt(time_spent_updating_manifolds)) * 1e-3});
        // std.debug.print("time spent solving colls = {d} ms\n", .{@as(f32, @floatFromInt(time_spent_solving_collisions)) * 1e-3});
    }

    fn updateManifolds(self: *Self, alloc: Allocator) !void {
        var remove_keys = std.ArrayList(clsn.CollisionKey).init(alloc);
        defer remove_keys.deinit();

        var iter = self.manifolds.iterator();
        while (iter.next()) |entry| {
            const key = entry.key_ptr;
            const manifold = entry.value_ptr;

            const sat = clsn.performNarrowSAT(key.ref_body, key.inc_body);
            if (!sat.collides) {
                try remove_keys.append(key.*);
                continue;
            }

            key.ref_body = sat.key.ref_body;
            key.inc_body = sat.key.inc_body;

            manifold.normal = sat.normal;
            manifold.points = sat.key.ref_body.identifyCollisionPoints(sat.key.inc_body, sat.reference_normal_id);
        }

        for (remove_keys.items) |key| {
            _ = self.manifolds.swapRemove(key);
        }

        try self.manifolds.reIndex();

        var st = std.time.microTimestamp();
        self.quadtree.clear(alloc);
        var et = std.time.microTimestamp();
        std.debug.print("time to clear qtree = {d}\n", .{@as(f32, @floatFromInt((et - st))) * 1e-3});

        st = std.time.microTimestamp();
        try self.quadtree.insertValues(alloc, self.bodies.items);
        et = std.time.microTimestamp();
        std.debug.print("time to insert values = {d}\n", .{@as(f32, @floatFromInt((et - st))) * 1e-3});

        var queries = std.ArrayList(*RigidBody).init(alloc);
        defer queries.deinit();

        st = std.time.microTimestamp();
        for (self.bodies.items) |*body1| {
            queries.clearRetainingCapacity();
            try self.quadtree.queryAABB(body1.aabb, &queries);

            for (queries.items) |body2| {
                if (body1.static and body2.static) continue;
                if (body1.ptr == body2.ptr) continue;

                var tmp = clsn.CollisionKey{ .ref_body = body1, .inc_body = body2 };
                if (!self.manifolds.contains(tmp)) {
                    tmp = clsn.CollisionKey{ .ref_body = body2, .inc_body = body1 };
                    if (!self.manifolds.contains(tmp)) {
                        if (!body1.aabb.intersects(body2.aabb)) continue;

                        const sat = clsn.performNarrowSAT(body1, body2);
                        if (!sat.collides) continue;

                        const manifold = clsn.CollisionManifold{
                            .normal = sat.normal,
                            .points = sat.key.ref_body.identifyCollisionPoints(sat.key.inc_body, sat.reference_normal_id),
                            .prev_angle_1 = sat.key.ref_body.props.angle,
                            .prev_angle_2 = sat.key.inc_body.props.angle,
                        };

                        try self.manifolds.put(sat.key, manifold);
                    }
                }
            }
        }
        et = std.time.microTimestamp();
        std.debug.print("time to update narrowly (query + sat) = {}", .{@as(f32, @floatFromInt((et - st))) * 1e-3});
    }

    pub fn entityFactory(self: *Self) EntityFactory {
        return EntityFactory{ .solver = self };
    }
};

pub const World = struct {
    solver: Solver,
    renderer: ?Renderer,

    const Self = @This();
    pub fn init(alloc: Allocator, screen_size: Units.Size, default_world_width: f32, init_renderer: bool, comptime quadtree_node_threshold: usize, comptime quadtree_max_depth: usize) !Self {
        return .{
            .solver = try Solver.init(alloc, quadtree_node_threshold, quadtree_max_depth),
            .renderer = if (init_renderer) Renderer.init(screen_size, default_world_width) else null,
        };
    }

    pub fn deinit(self: *Self, alloc: Allocator) void {
        self.solver.deinit(alloc);
    }

    pub fn render(self: *Self, show_collisions: bool, show_qtree: bool) void {
        if (self.renderer) |*rend| {
            rend.render(self.solver, show_collisions, show_qtree);
        }
    }
};
