const std = @import("std");
const rl = @import("raylib");
const r2d = @import("resolve2d");
const nmath = r2d.nmath;
const Vector2 = nmath.Vector2;
const Units = @import("Units.zig");

const Renderer = @This();

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

const ColorPair = struct {
    inner: rl.Color,
    edge: rl.Color,
};

const SpringRenderOptions = struct {
    segment_width: OptPair,
    segment_color: rl.Color = rl.Color.white,
};

const BodyOptions = struct {
    color: ColorPair,
    static_color: ColorPair,
    edge_thickness: OptPair,

    pub fn init(units: Units, color: rl.Color, static_color: rl.Color, edge_thickness: f32) BodyOptions {
        const n: u8 = 1;
        const n2: u8 = 3;
        return .{
            .color = ColorPair{
                .inner = rl.Color.init(color.r, color.g, color.b, color.a / n2),
                .edge = rl.Color.init(color.r / n, color.g / n, color.b / n, color.a),
            },
            .static_color = ColorPair{
                .inner = rl.Color.init(static_color.r, static_color.g, static_color.b, static_color.a / n2),
                .edge = rl.Color.init(static_color.r / n, static_color.g / n, static_color.b / n, static_color.a),
            },
            .edge_thickness = OptPair.init(units, edge_thickness),
        };
    }
};

units: Units,
spring_options: SpringRenderOptions,
body_options: BodyOptions,

const RING_RES: i32 = 30;

pub fn init(screen_size: Units.Size, default_world_width: f32) Renderer {
    var units = Units.init(screen_size, default_world_width);
    units.updateDimensions(screen_size.toVector2());
    return .{
        .units = units,
        .spring_options = .{
            .segment_width = OptPair.init(units, 0.03),
            .segment_color = rl.Color.white,
        },
        .body_options = BodyOptions.init(units, rl.Color.red, rl.Color.pink, 0.03),
    };
}

pub fn adjustCameraZoom(self: *Renderer, factor: f32, screen_pos: Vector2) void {
    self.units.adjustCameraZoom(factor, screen_pos);

    self.spring_options.segment_width.scr = self.units.mult.w2s * self.spring_options.segment_width.world;
    self.body_options.edge_thickness.scr = self.units.mult.w2s * self.body_options.edge_thickness.world;
}

pub fn adjustCameraPos(self: *Renderer, delta: Vector2) void {
    self.units.adjustCameraPos(delta);
}

pub fn render(self: *Renderer, solver: r2d.Solver, show_collisions: bool, show_aabbs: bool) void {
    for (solver.force_generators.items) |*gen| {
        switch (gen.type) {
            .downwards_gravity => continue,
        }
    }
    var body_iter = solver.bodies.iterator();
    while (body_iter.next()) |*entry| {
        const body = entry.value_ptr;
        const nmath_screen_pos = self.units.w2s(body.props.pos);
        const screen_pos = rl.Vector2.init(nmath_screen_pos.x, nmath_screen_pos.y);

        if (show_aabbs) {
            const screen = self.units.w2s(nmath.sub2(body.props.pos, Vector2.init(body.aabb.half_width, -body.aabb.half_height)));
            const rls = rlv2(screen);
            const mult = self.units.mult.w2s;
            rl.drawRectangleV(rls, rlv2(Vector2.init(body.aabb.half_width * 2 * mult, body.aabb.half_height * 2 * mult)), rl.Color.red);
        }

        switch (body.type) {
            .disc => {
                self.discbody(screen_pos, body);
            },
            .rectangle => {
                self.rectanglebody(body);
            },
        }
    }

    if (show_collisions) {
        var iter = solver.manifolds.iterator();
        while (iter.next()) |entry| {
            const manifold = entry.value_ptr;

            const normal = manifold.normal;
            for (manifold.points) |point| {
                if (point) |pt| {
                    const screen = self.units.w2s(pt.pos);
                    const rls = rl.Vector2.init(screen.x, screen.y);
                    rl.drawCircleV(rls, 0.05 * self.units.mult.w2s, rl.Color.lime);

                    const pn_vec = nmath.scale2(normal, pt.accumulated_pn);

                    const tangent = nmath.rotate90clockwise(normal);
                    const pt_vec = nmath.scale2(tangent, pt.accumulated_pt);
                    const p = nmath.add2(pn_vec, pt_vec);
                    // const p = pn_vec;

                    const screen2 = self.units.w2s(nmath.add2(pt.pos, p));
                    const rls2 = rl.Vector2.init(screen2.x, screen2.y);
                    // const teal = rl.Color.init(66, 182, 245, 255);
                    rl.drawLineEx(rls, rls2, self.units.mult.w2s * 0.02, rl.Color.white);

                    // const screen3 = self.units.w2s(nmath.add2(pt.pos, pt_vec));
                    // const rls3 = rl.Vector2.init(screen3.x, screen3.y);
                    // // const teal = rl.Color.init(66, 182, 245, 255);
                    // rl.drawLineEx(rls, rls3, self.units.mult.w2s * 0.02, rl.Color.white);

                    var scr = self.units.w2s(nmath.add2(pt.pos, nmath.negate2(pt.ref_r)));
                    var rla = rl.Vector2.init(scr.x, scr.y);
                    rl.drawLineEx(rla, rls, self.units.mult.w2s * 0.015, rl.Color.red);

                    scr = self.units.w2s(nmath.add2(pt.pos, nmath.negate2(pt.inc_r)));
                    rla = rl.Vector2.init(scr.x, scr.y);
                    rl.drawLineEx(rla, rls, self.units.mult.w2s * 0.015, rl.Color.orange);

                    scr = self.units.w2s(nmath.addmult2(pt.pos, nmath.negate2(normal), pt.depth));
                    rla = rlv2(scr);
                    rl.drawLineEx(rla, rls, self.units.mult.w2s * 0.03, rl.Color.sky_blue);
                    rl.drawCircleV(rla, self.units.mult.w2s * 0.02, rl.Color.blue);
                }
            }
        }
    }

    for (solver.constraints.items) |constraint| {
        switch (constraint.type) {
            .distance_joint => {},
            .offset_distance_joint => {},
            .fixed_position_joint => {},
            .motor_joint => {},
        }
    }
}

fn spring(self: Renderer, start_pos: Vector2, end_pos: Vector2) void {
    const nmath_sp1 = self.units.w2s(start_pos);
    const nmath_sp2 = self.units.w2s(end_pos);
    const sp1 = rl.Vector2.init(nmath_sp1.x, nmath_sp1.y);
    const sp2 = rl.Vector2.init(nmath_sp2.x, nmath_sp2.y);

    rl.drawLineEx(sp1, sp2, self.spring_options.segment_width.scr, self.spring_options.segment_color);
}

fn discbody(self: Renderer, screen_pos: rl.Vector2, body: *r2d.RigidBody) void {
    const disc: *r2d.RigidBody.Disc = @ptrCast(@alignCast(body.ptr));
    var color = if (body.static) self.body_options.static_color else self.body_options.color;

    if (!body.static) {
        const n = 1;
        const n2 = 3;
        const temp_col = rl.Color.init(255, 240, 28, 255);
        color.inner = rl.Color.init(temp_col.r, temp_col.g, temp_col.b, temp_col.a / n2);
        color.edge = rl.Color.init(temp_col.r / n, temp_col.g / n, temp_col.b / n, temp_col.a);
    }

    const rad = disc.radius;
    const vec = rl.Vector2.init(screen_pos.x, screen_pos.y);
    const rot_vec = Vector2.init(std.math.cos(body.props.angle), std.math.sin(body.props.angle));
    const rot_world = nmath.add2(nmath.scale2(rot_vec, rad), body.props.pos);
    const rot_screen = self.units.w2s(rot_world);
    const rl_rot_screen = rl.Vector2.init(rot_screen.x, rot_screen.y);

    const inner = self.units.mult.w2s * (rad - self.body_options.edge_thickness.world);
    const outer = self.units.mult.w2s * rad;
    rl.drawRing(vec, 0, inner, 0, 360, Renderer.RING_RES, color.inner);
    rl.drawRing(vec, inner, outer, 0, 360, Renderer.RING_RES, color.edge);
    rl.drawLineEx(vec, rl_rot_screen, self.body_options.edge_thickness.scr, color.edge);
}

pub fn rectanglebody(self: *Renderer, body: *r2d.RigidBody) void {
    const rect: *r2d.RigidBody.Rectangle = @ptrCast(@alignCast(body.ptr));
    var color = if (body.static) self.body_options.static_color else self.body_options.color;

    if (!body.static) {
        const n = 1;
        const n2 = 3;
        const temp_col = rl.Color.init(164, 255, 28, 255);
        color.inner = rl.Color.init(temp_col.r, temp_col.g, temp_col.b, temp_col.a / n2);
        color.edge = rl.Color.init(temp_col.r / n, temp_col.g / n, temp_col.b / n, temp_col.a);
    }

    const edge = self.body_options.edge_thickness.world;
    var outer_verts: [4]Vector2 = undefined;
    var inner_verts: [4]rl.Vector2 = undefined;

    const half_edge = edge / 2;

    for (rect.local_vertices, 0..) |vert, idx| {
        var v1 = vert;
        v1.x += if (v1.x > 0) -half_edge else half_edge;
        v1.y += if (v1.y > 0) -half_edge else half_edge;

        var vi = v1;
        vi.x += if (vi.x > 0) -half_edge else half_edge;
        vi.y += if (vi.y > 0) -half_edge else half_edge;

        outer_verts[idx] = v1;

        const r = nmath.rotate2(vi, body.props.angle);
        const a = nmath.add2(r, body.props.pos);
        const rlscreen = rlv2(self.units.w2s(a));
        inner_verts[idx] = rlscreen;
    }

    for (0..4) |idx| {
        var v1 = outer_verts[idx];
        var v2 = outer_verts[if (idx == 3) 0 else idx + 1];

        switch (idx) {
            0 => {
                v1.y += half_edge;
                v2.y -= half_edge;
            },
            1 => {
                v1.x -= half_edge;
                v2.x += half_edge;
            },
            2 => {
                v1.y -= half_edge;
                v2.y += half_edge;
            },
            3 => {
                v1.x += half_edge;
                v2.x -= half_edge;
            },
            else => unreachable,
        }

        var r = nmath.rotate2(v1, body.props.angle);
        var a = nmath.add2(r, body.props.pos);
        const rlscreen1 = rlv2(self.units.w2s(a));

        r = nmath.rotate2(v2, body.props.angle);
        a = nmath.add2(r, body.props.pos);
        const rlscreen2 = rlv2(self.units.w2s(a));

        rl.drawLineEx(rlscreen1, rlscreen2, self.body_options.edge_thickness.scr, color.edge);
    }

    rl.drawTriangle(inner_verts[0], inner_verts[2], inner_verts[1], color.inner);
    rl.drawTriangle(inner_verts[0], inner_verts[3], inner_verts[2], color.inner);
}

fn rlv2(a: Vector2) rl.Vector2 {
    return rl.Vector2.init(a.x, a.y);
}
