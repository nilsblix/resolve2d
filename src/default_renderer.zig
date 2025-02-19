const std = @import("std");
const nmath = @import("nmath.zig");
const Vector2 = nmath.Vector2;
const zigics = @import("zigics.zig");
const Physics = zigics.Physics;
const fg_mod = @import("force_generator.zig");
const rb_mod = @import("rigidbody.zig");
const rl = @import("raylib");
const RigidBody = rb_mod.RigidBody;

pub const Units = struct {
    pub const Size = struct {
        width: f32,
        height: f32,
    };

    const TransformMult = struct {
        w2s: f32,
        s2w: f32,
    };

    /// pos is bottom left of the viewport
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

    fn updateViewPort(self: *Self) void {
        const dims = Vector2.init(self.screen_size.width, self.screen_size.height);
        const world_view = nmath.scale2(dims, self.mult.s2w);
        self.camera.viewport = Size{ .width = world_view.x, .height = world_view.y };
    }

    pub fn adjustCameraZoom(self: *Self, factor: f32, screen_pos: Vector2) void {
        const old_world_pos = self.s2w(screen_pos);

        self.camera.zoom /= factor;

        self.mult.w2s = (self.screen_size.width / self.default_world_size.width) / self.camera.zoom;
        self.mult.s2w = (self.default_world_size.width / self.screen_size.width) * self.camera.zoom;

        const new_world_pos = self.s2w(screen_pos);
        const delta_world = nmath.sub2(new_world_pos, old_world_pos);
        self.camera.pos.sub(delta_world);

        self.updateViewPort();
    }

    pub fn adjustCameraPos(self: *Self, delta: Vector2) void {
        // self.camera.pos.sub(nmath.scale2(delta, self.camera.zoom));
        self.camera.pos.sub(delta);
    }

    /// Will flip y as world bottom-left is (0,0)
    pub fn w2s(self: Self, pos: Vector2) Vector2 {
        const x = self.mult.w2s * (pos.x - self.camera.pos.x);
        const y = self.screen_size.height - self.mult.w2s * (pos.y - self.camera.pos.y);

        return Vector2.init(x, y);
    }

    /// Will flip y as world bottom-left is (0,0)
    pub fn s2w(self: Self, pos: Vector2) Vector2 {
        const x = pos.x * self.mult.s2w + self.camera.pos.x;
        const y = (self.screen_size.height - pos.y) * self.mult.s2w + self.camera.pos.y;
        return Vector2.init(x, y);
    }

    /// Max x from [oldmin -> oldmax] ==> [newmin -> newmax]
    pub fn map(x: f32, omin: f32, omax: f32, nmin: f32, nmax: f32) f32 {
        return nmin + ((x - omin) * (nmin - nmax)) / (omin - omax);
    }
};

test "mults should be inverses" {
    var units = Units.init(.{ .width = 1000, .height = 500 }, 20);

    // units.adjustCameraPos(Vector2.init(1, 0.5));
    units.adjustCameraZoom(1.3, Vector2.init(-1, 2));

    const val = 3.1415;
    const ret = units.mult.s2w * units.mult.w2s * val;

    std.debug.print("mults should be inverses \n", .{});
    std.debug.print("     val = {}\n", .{val});
    std.debug.print("     ret = {}\n", .{ret});

    try std.testing.expect(ret == val);
}

test "s2w and w2s should be inverses" {
    var units = Units.init(.{ .width = 1000, .height = 500 }, 20);

    // units.adjustCameraPos(Vector2.init(1, 0.5));
    units.adjustCameraZoom(1.3, Vector2.init(-1, 2));

    const vec = Vector2.init(23, 34);
    const ret = units.s2w(units.w2s(vec));

    std.debug.print("transformations should be inverses \n", .{});
    std.debug.print("     vec = {}\n", .{vec});
    std.debug.print("     ret = {}\n", .{ret});

    try std.testing.expect(nmath.equals2(ret, vec));
}

test "unit mapping" {
    const x = 0.8;
    const y = Units.map(x, 0, 1, 10, 20);
    try std.testing.expect(y == 18);
}

const OptPair = struct {
    world: f32,
    scr: f32,

    pub fn init(units: Units, value: f32) OptPair {
        return .{
            .world = value,
            .scr = units.mult.w2s * value,
        };
    }
};

const SpringRenderOptions = struct {
    segment_width: OptPair,
    segment_color: rl.Color = rl.Color.white,
};

const BodyOptions = struct {
    color: rl.Color = rl.Color.green,
    inner_color: rl.Color,
    edge_thickness: OptPair,

    pub fn init(units: Units, color: rl.Color, edge_thickness: f32) BodyOptions {
        const n: u8 = 1;
        const n2: u8 = 3;
        return .{
            .color = rl.Color.init(color.r / n, color.g / n, color.b / n, color.a),
            .inner_color = rl.Color.init(color.r, color.g, color.b, color.a / n2),
            .edge_thickness = OptPair.init(units, edge_thickness),
        };
    }
};

pub const Renderer = struct {
    units: Units,
    spring_options: SpringRenderOptions,
    body_options: BodyOptions,

    const RING_RES: i32 = 30;

    const Self = @This();
    pub fn init(screen_size: Units.Size, default_world_width: f32) Renderer {
        const units = Units.init(screen_size, default_world_width);
        return .{
            .units = units,
            .spring_options = .{
                .segment_width = OptPair.init(units, 0.03),
                .segment_color = rl.Color.white,
            },
            .body_options = BodyOptions.init(units, rl.Color.red, 0.02),
        };
    }

    pub fn adjustCameraZoom(self: *Self, factor: f32, screen_pos: Vector2) void {
        self.units.adjustCameraZoom(factor, screen_pos);

        self.spring_options.segment_width.scr = self.units.mult.w2s * self.spring_options.segment_width.world;
        self.body_options.edge_thickness.scr = self.units.mult.w2s * self.body_options.edge_thickness.world;
    }

    pub fn adjustCameraPos(self: *Self, delta: Vector2) void {
        self.units.adjustCameraPos(delta);
    }

    pub fn render(self: *Self, physics: Physics) void {
        for (physics.force_generators.items) |*gen| {
            switch (gen.type) {
                .downwards_gravity => continue,
                .point_gravity => continue,
                .static_spring => {
                    const g: *fg_mod.StaticSpring = @ptrCast(@alignCast(gen.ptr));
                    const start = nmath.add2(g.body.props.pos, nmath.rotate2(g.r, g.body.props.angle));
                    self.spring(g.pos, start);
                },
            }
        }
        for (physics.bodies.items) |*body| {
            const nmath_screen_pos = self.units.w2s(body.props.pos);
            const screen_pos = rl.Vector2.init(nmath_screen_pos.x, nmath_screen_pos.y);

            switch (body.type) {
                .disc => {
                    self.discbody(screen_pos, body);
                },
                .rectangle => {
                    self.rectanglebody(body);
                },
            }
        }
    }

    fn spring(self: Self, start_pos: Vector2, end_pos: Vector2) void {
        const nmath_sp1 = self.units.w2s(start_pos);
        const nmath_sp2 = self.units.w2s(end_pos);
        const sp1 = rl.Vector2.init(nmath_sp1.x, nmath_sp1.y);
        const sp2 = rl.Vector2.init(nmath_sp2.x, nmath_sp2.y);

        rl.drawLineEx(sp1, sp2, self.spring_options.segment_width.scr, self.spring_options.segment_color);
    }

    fn discbody(self: Self, screen_pos: rl.Vector2, body: *rb_mod.RigidBody) void {
        const disc: *rb_mod.DiscBody = @ptrCast(@alignCast(body.ptr));

        const rad = disc.radius;
        const vec = rl.Vector2.init(screen_pos.x, screen_pos.y);
        const rot_vec = Vector2.init(std.math.cos(body.props.angle), std.math.sin(body.props.angle));
        const rot_world = nmath.add2(nmath.scale2(rot_vec, rad), body.props.pos);
        const rot_screen = self.units.w2s(rot_world);
        const rl_rot_screen = rl.Vector2.init(rot_screen.x, rot_screen.y);

        const inner = self.units.mult.w2s * (rad - 0.5 * self.body_options.edge_thickness.world);
        const outer = self.units.mult.w2s * (rad + 0.5 * self.body_options.edge_thickness.world);
        rl.drawRing(vec, 0, inner, 0, 360, Self.RING_RES, self.body_options.inner_color);
        rl.drawRing(vec, inner, outer, 0, 360, Self.RING_RES, self.body_options.color);
        rl.drawLineEx(vec, rl_rot_screen, self.body_options.edge_thickness.scr, self.body_options.color);
    }

    pub fn rectanglebody(self: *Self, body: *rb_mod.RigidBody) void {
        const rect: *rb_mod.RectangleBody = @ptrCast(@alignCast(body.ptr));

        const verts = rect.getWorldVertices(body.props);

        var rlv: [4]rl.Vector2 = undefined;
        for (verts, 0..) |vert, i| {
            const screen = self.units.w2s(vert);
            const rls = rlv2(screen);
            rlv[i] = rls;
        }

        for (rlv, 0..) |v, i| {
            const next = rlv[if (i == 3) 0 else i + 1];
            rl.drawLineEx(v, next, self.body_options.edge_thickness.scr, self.body_options.color);
        }

        const edge = self.body_options.edge_thickness.world;
        for (rect.local_vertices, 0..) |vert, i| {
            var v = vert;
            v.x += if (v.x > 0) -edge else edge;
            v.y += if (v.y > 0) -edge else edge;
            const r = nmath.rotate2(v, body.props.angle);
            const world = nmath.add2(r, body.props.pos);
            const screen = self.units.w2s(world);
            rlv[i] = rlv2(screen);
        }

        rl.drawTriangle(rlv[0], rlv[2], rlv[1], self.body_options.inner_color);
        rl.drawTriangle(rlv[0], rlv[3], rlv[2], self.body_options.inner_color);
    }
};

pub fn rlv2(a: Vector2) rl.Vector2 {
    return rl.Vector2.init(a.x, a.y);
}
