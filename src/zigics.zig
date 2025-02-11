const std = @import("std");
const Allocator = std.mem.Allocator;
const nmath = @import("nmath.zig");
const Vector2 = nmath.Vector2;
const rb_mod = @import("rigidbody.zig");
const RigidBody = rb_mod.RigidBody;
const fg_mod = @import("force-generator.zig");
const ForceGenerator = fg_mod.ForceGenerator;
const rl = @import("raylib");

// FIXME: transform camera / and always update properties like mult and size and viewport and fuck all else
pub const Units = struct {
    pub const Size = struct {
        width: f32,
        height: f32,
    };

    const TransformMult = struct {
        w2s: f32,
        s2w: f32,
    };

    const Camera = struct {
        pos: Vector2,
        zoom: f32,
        viewport: Size,
    };

    camera: Camera,
    mult: TransformMult,
    default_world_size: Size,
    screen_size: Size,

    const Self = @This();
    /// Assumes that screen_size and world_size have the same proportions.
    /// Assumes that in screen units top left is (0,0).
    pub fn init(screen_size: Size, default_world_width: f32) Self {
        const aspect_ratio = screen_size.height / screen_size.width;

        const default_world_size = Size{ .width = default_world_width, .height = default_world_width * aspect_ratio };

        // const x = default_world_size.width / 2;
        // const y = default_world_size.height / 2;
        const camera = Camera{
            .pos = .{},
            .zoom = 1.0,
            .viewport = default_world_size,
        };

        const mult = TransformMult{
            .w2s = screen_size.width / default_world_width,
            .s2w = default_world_width / screen_size.width,
        };

        return .{
            .camera = camera,
            .mult = mult,
            .default_world_size = default_world_size,
            .screen_size = screen_size,
        };
    }

    /// Will flip y as world bottom-left is (0,0)
    pub fn w2s(self: Self, pos: Vector2) Vector2 {
        const x = self.mult.w2s * (pos.x - self.camera.pos.x) / self.camera.zoom;
        const y = self.screen_size.height - self.mult.w2s * (pos.y - self.camera.pos.y) / self.camera.zoom;

        return Vector2.init(x, y);
    }

    /// Will flip y as world bottom-left is (0,0)
    pub fn s2w(self: Self, pos: Vector2) Vector2 {
        const x = self.camera.zoom * pos.x * self.mult.s2w + self.camera.pos.x;
        const y = self.camera.zoom * (self.screen_size.height - pos.y) * self.mult.s2w + self.camera.pos.y;
        return Vector2.init(x, y);
    }

    /// Max x from [oldmin -> oldmax] ==> [newmin -> newmax]
    pub fn map(x: f32, omin: f32, omax: f32, nmin: f32, nmax: f32) f32 {
        return nmin + ((x - omin) * (nmin - nmax)) / (omin - omax);
    }
};

test "s2w and w2s should be inverses" {
    var units = Units.init(.{ .width = 1000, .height = 500 }, 20);
    const vec = Vector2.init(23, 34);
    const ret = units.s2w(units.w2s(vec));
    try std.testing.expect(nmath.equals2(ret, vec));
}

test "unit mapping" {
    const x = 0.8;
    const y = Units.map(x, 0, 1, 10, 20);
    try std.testing.expect(y == 18);
}

pub const Renderer = struct {
    units: Units,

    const thickness: f32 = 0.01;
    const segment_resolution: i32 = 30;

    const Self = @This();
    pub fn init(screen_size: Units.Size, default_world_width: f32) Renderer {
        return .{
            .units = Units.init(screen_size, default_world_width),
        };
    }

    pub fn render(self: *const Self, physics: Physics, debug_mode: bool) void {
        for (physics.bodies.items) |body| {
            const screen_pos = self.units.w2s(body.props.pos);
            const int_pos = nmath.toInt2(screen_pos);
            _ = int_pos;

            switch (body.type) {
                .disc => {
                    const b: *rb_mod.DiscBody = @ptrCast(@alignCast(body.ptr));
                    const rad = b.radius;
                    if (debug_mode) {
                        const inner = self.units.mult.w2s * (rad - 0.5 * Self.thickness);
                        const outer = self.units.mult.w2s * (rad + 0.5 * Self.thickness);
                        rl.drawRing(.{ .x = screen_pos.x, .y = screen_pos.y }, inner, outer, 0, 360, Self.segment_resolution, rl.Color.green);
                        break;
                    }

                    rl.drawCircleV(.{ .x = screen_pos.x, .y = screen_pos.y }, self.units.mult.w2s * rad, rl.Color.orange);
                },
            }
        }
    }
};

pub const Physics = struct {
    bodies: std.ArrayList(RigidBody),
    force_generators: std.ArrayList(ForceGenerator),
    // joints: std.ArrayList(Joint),

    const Self = @This();
    pub fn init(alloc: Allocator) Self {
        return .{
            .bodies = std.ArrayList(RigidBody).init(alloc),
            .force_generators = std.ArrayList(ForceGenerator).init(alloc),
            // .joints = std.ArrayList(Joint).init(alloc),
        };
    }

    pub fn deinit(self: *Self, alloc: Allocator) void {
        for (self.force_generators.items) |*gen| {
            gen.deinit(alloc);
        }
        self.force_generators.deinit();
        // self.joints.deinit();
        for (self.bodies.items) |*body| {
            body.deinit(alloc);
        }
        self.bodies.deinit();
    }

    pub fn process(self: *Self, dt: f32) void {
        for (self.force_generators.items) |*gen| {
            gen.apply(self.bodies);
        }

        for (self.bodies.items) |*body| {
            var props: *RigidBody.Props = &body.props;

            props.momentum.addmult(props.force, dt);
            props.pos.addmult(props.momentum, dt / props.mass);

            props.ang_momentum += props.torque * dt;
            props.angle += props.ang_momentum * dt / props.inertia;

            props.force = .{};
            props.torque = 0;
        }
    }
};

pub const World = struct {
    physics: Physics,
    renderer: ?Renderer,

    const Self = @This();
    pub fn init(alloc: Allocator, screen_size: Units.Size, default_world_width: f32, init_renderer: bool) World {
        return .{
            .physics = Physics.init(alloc),
            .renderer = if (init_renderer) Renderer.init(screen_size, default_world_width) else null,
        };
    }

    pub fn deinit(self: *Self, alloc: Allocator) void {
        self.physics.deinit(alloc);
    }

    pub fn process(self: *Self, dt: f32) void {
        self.physics.process(dt);
    }

    pub fn render(self: *Self, debug_mode: bool) void {
        if (self.renderer) |rend| {
            rend.render(self.physics, debug_mode);
        }
    }
};
