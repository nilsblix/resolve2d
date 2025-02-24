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

    pub fn process(self: *Self, alloc: Allocator, dt: f32) !void {
        for (self.force_generators.items) |*gen| {
            gen.apply(self.bodies);
        }

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

        for (self.bodies.items) |*body| {
            if (body.static) continue;
            var props: *RigidBody.Props = &body.props;

            props.momentum.addmult(props.force, dt);
            props.pos.addmult(props.momentum, dt / props.mass);

            props.ang_momentum += props.torque * dt;
            props.angle += props.ang_momentum * dt / props.inertia;

            props.force = .{};
            props.torque = 0;
        }
    }

    pub fn getEnergy(self: Self) f32 {
        var E: f32 = 0;
        for (self.force_generators.items) |*gen| {
            E += gen.energy(self.bodies);
        }
        for (self.bodies.items) |*body| {
            const len2 = nmath.length2sq(nmath.scale2(body.props.momentum, 1 / body.props.mass));
            E += 1 / 2 * body.props.mass * len2;
            const ang_vel = body.props.ang_momentum / body.props.inertia;
            E += 1 / 2 * body.props.inertia * ang_vel * ang_vel;
        }
        return E;
    }

    pub fn makeDiscBody(self: *Self, pos: Vector2, mass: f32, radius: f32) !void {
        const body: rb_mod.RigidBody = try rb_mod.DiscBody.init(self.alloc, pos, 0, mass, radius);
        try self.bodies.append(body);
    }

    pub fn makeRectangleBody(self: *Self, pos: Vector2, mass: f32, width: f32, height: f32) !void {
        const body: rb_mod.RigidBody = try rb_mod.RectangleBody.init(self.alloc, pos, 0, mass, width, height);
        try self.bodies.append(body);
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

    pub fn process(self: *Self, alloc: Allocator, dt: f32) !void {
        try self.solver.process(alloc, dt);
    }

    pub fn render(self: *Self, show_collisions: bool) void {
        if (self.renderer) |*rend| {
            const ext = 0.5;
            const left = rend.units.w2s(Vector2.init(-ext, 0));
            const top = rend.units.w2s(Vector2.init(0, ext));
            const bottom = rend.units.w2s(Vector2.init(0, -ext));
            const right = rend.units.w2s(Vector2.init(ext, 0));

            const rl_left = rl.Vector2.init(left.x, left.y);
            const rl_top = rl.Vector2.init(top.x, top.y);
            const rl_bottom = rl.Vector2.init(bottom.x, bottom.y);
            const rl_right = rl.Vector2.init(right.x, right.y);

            const width = rend.units.mult.w2s * 0.02;

            rl.drawLineEx(rl_left, rl_right, width, rl.Color.red);
            rl.drawLineEx(rl_bottom, rl_top, width, rl.Color.red);

            rend.render(self.solver, show_collisions);
        }
    }
};
