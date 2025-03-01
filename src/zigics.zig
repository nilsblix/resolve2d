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

pub const Solver = struct {
    alloc: Allocator,
    bodies: std.ArrayList(RigidBody),
    force_generators: std.ArrayList(ForceGenerator),
    manifolds: std.AutoArrayHashMap(clsn.CollisionKey, clsn.CollisionManifold),

    const Self = @This();
    pub fn init(alloc: Allocator) Self {
        return .{
            .alloc = alloc,
            .bodies = std.ArrayList(RigidBody).init(alloc),
            .force_generators = std.ArrayList(ForceGenerator).init(alloc),
            .manifolds = std.AutoArrayHashMap(clsn.CollisionKey, clsn.CollisionManifold).init(alloc),
        };
    }

    pub fn deinit(self: *Self) void {
        self.manifolds.deinit();
        for (self.force_generators.items) |*gen| {
            gen.deinit(self.alloc);
        }
        self.force_generators.deinit();
        for (self.bodies.items) |*body| {
            body.deinit(self.alloc);
        }
        self.bodies.deinit();
    }

    pub fn clear(self: *Self, alloc: Allocator) void {
        self.deinit();
        self.* = Self.init(alloc);
    }

    pub fn process(self: *Self, alloc: Allocator, dt: f32, collision_iters: usize) !void {
        for (self.force_generators.items) |*gen| {
            gen.apply(self.bodies);
        }

        for (self.bodies.items) |*body| {
            if (body.static) continue;
            var props: *RigidBody.Props = &body.props;

            props.momentum.addmult(props.force, dt);
            props.ang_momentum += props.torque * dt;
        }

        try self.updateManifolds(alloc);

        for (0..collision_iters) |_| {
            var iter = self.manifolds.iterator();
            while (iter.next()) |entry| {
                const manifold = entry.value_ptr;
                manifold.applyImpulses(entry.key_ptr.*, dt, 0.1, 0.01, 1e-7);
            }
        }

        for (self.bodies.items) |*body| {
            if (body.static) continue;
            var props: *RigidBody.Props = &body.props;

            props.pos.addmult(props.momentum, dt / props.mass);
            props.angle += props.ang_momentum * dt / props.inertia;

            props.force = .{};
            props.torque = 0;
        }
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

        for (0..self.bodies.items.len) |id1| {
            const body1 = &self.bodies.items[id1];
            for (id1 + 1..self.bodies.items.len) |id2| {
                const body2 = &self.bodies.items[id2];

                if (body1.static and body2.static) continue;

                var tmp = clsn.CollisionKey{ .ref_body = body1, .inc_body = body2 };
                if (!self.manifolds.contains(tmp)) {
                    tmp = clsn.CollisionKey{ .ref_body = body2, .inc_body = body1 };
                    if (!self.manifolds.contains(tmp)) {
                        const sat = clsn.performNarrowSAT(body1, body2);
                        if (!sat.collides) continue;

                        const manifold = clsn.CollisionManifold{
                            .normal = sat.normal,
                            .points = sat.key.ref_body.identifyCollisionPoints(sat.key.inc_body, sat.reference_normal_id),
                        };

                        try self.manifolds.put(sat.key, manifold);
                    }
                }
            }
        }
    }

    pub fn entityFactory(self: *Self) EntityFactory {
        return EntityFactory{ .solver = self };
    }
};

pub const World = struct {
    solver: Solver,
    renderer: ?Renderer,

    const Self = @This();
    pub fn init(alloc: Allocator, screen_size: Units.Size, default_world_width: f32, init_renderer: bool) World {
        return .{
            .solver = Solver.init(alloc),
            .renderer = if (init_renderer) Renderer.init(screen_size, default_world_width) else null,
        };
    }

    pub fn deinit(self: *Self) void {
        self.solver.deinit();
    }

    pub fn render(self: *Self, show_collisions: bool) void {
        if (self.renderer) |*rend| {
            rend.render(self.solver, show_collisions);
        }
    }
};
