const std = @import("std");
const rl = @import("raylib");
const zigics = @import("zigics.zig");
const rigidbody = @import("rigidbody.zig");
const forcegenerator = @import("force_generator.zig");
const nmath = @import("nmath.zig");
const Vector2 = nmath.Vector2;
const Allocator = std.mem.Allocator;
const collision = @import("collision.zig");

fn setupScene(solver: *zigics.Solver) !void {
    var factory = solver.entityFactory();

    var opt: zigics.EntityFactory.BodyOptions = .{ .pos = .{}, .mass_prop = .{ .mass = 1.0 } };

    opt.mass_prop = .{ .density = 5 };
    opt.mu_d = 0.5;
    opt.mu_s = 0.7;

    opt.pos = Vector2.init(5, -1);
    var ground = try factory.makeRectangleBody(opt, .{ .width = 20, .height = 1 });
    ground.static = true;

    opt.pos = Vector2.init(-4.5, 3.5);
    var wall1 = try factory.makeRectangleBody(opt, .{ .width = 1, .height = 10 });
    wall1.static = true;

    opt.pos = Vector2.init(14.5, 3.5);
    var wall2 = try factory.makeRectangleBody(opt, .{ .width = 1, .height = 10 });
    wall2.static = true;

    opt.pos = Vector2.init(5, 5);
    _ = try factory.makeDiscBody(opt, .{ .radius = 1.0 });

    opt.pos = Vector2.init(5, 3);
    var floating = try factory.makeRectangleBody(opt, .{ .width = 5, .height = 0.5 });
    floating.props.angle = 0.8;
    floating.static = true;

    opt.pos = Vector2.init(2, 3);
    floating = try factory.makeRectangleBody(opt, .{ .width = 5, .height = 0.5 });
    floating.props.angle = -0.2;
    floating.static = true;

    opt.pos = Vector2.init(9, 4);
    floating = try factory.makeDiscBody(opt, .{ .radius = 1.5 });
    floating.static = true;

    const width: f32 = 1.0;
    for (0..2) |y| {
        for (0..15) |i| {
            const idx: f32 = @floatFromInt(i);
            const x: f32 = idx * width - 2;

            const height = 0.4 + idx / 20;

            const yf: f32 = @floatFromInt(y);
            opt.pos = Vector2.init(x, yf + 8.0);

            if (@mod(i, 2) == 0) {
                _ = try factory.makeRectangleBody(opt, .{ .width = width, .height = height });
            } else {
                _ = try factory.makeDiscBody(opt, .{ .radius = width / 2 });
            }
        }
    }

    try factory.makeDownwardsGravity(9.82);
}

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();

    const alloc = gpa.allocator();

    // const screen_width = 1536;
    // const screen_height = 864;
    const screen_width = 1280;
    const screen_height = 720;

    var world = zigics.World.init(alloc, .{ .width = screen_width, .height = screen_height }, 10, true);
    defer world.deinit();

    try setupScene(&world.solver);

    var mouse_spring = MouseSpring{};

    rl.initWindow(screen_width, screen_height, "zigics");
    defer rl.closeWindow();

    const HZ: i32 = 60;
    const STANDARD_DT: f32 = 1 / @as(f32, HZ);
    const SUB_STEPS = 16;
    rl.setTargetFPS(HZ);

    var simulating: bool = false;
    var show_collisions: bool = true;
    var slow_motion = false;
    var steps: u32 = 0;

    var sim_dt: f32 = -1;
    var render_dt: f32 = -1;

    var screen_prev_mouse_pos: Vector2 = .{};
    var screen_mouse_pos: Vector2 = .{};
    var prev_mouse_pos: Vector2 = .{};
    var mouse_pos: Vector2 = .{};

    while (!rl.windowShouldClose()) {
        screen_prev_mouse_pos = screen_mouse_pos;
        prev_mouse_pos = mouse_pos;

        const rl_pos = rl.getMousePosition();
        screen_mouse_pos = Vector2.init(rl_pos.x, rl_pos.y);
        mouse_pos = world.renderer.?.units.s2w(Vector2.init(screen_mouse_pos.x, screen_mouse_pos.y));

        const delta_screen = nmath.sub2(screen_mouse_pos, screen_prev_mouse_pos);
        var world_delta_mouse_pos = nmath.scale2(delta_screen, world.renderer.?.units.mult.s2w);
        world_delta_mouse_pos.y *= -1;

        rl.beginDrawing();
        defer rl.endDrawing();

        if (world.renderer) |rend| {
            try mouse_spring.update(alloc, rend.units, &world.solver);
        }

        const delta_wheel = rl.getMouseWheelMove();
        if (delta_wheel != 0) {
            world.renderer.?.adjustCameraZoom(std.math.exp(delta_wheel / 100), screen_mouse_pos);
        }

        if (rl.isMouseButtonDown(.left)) {
            // const mouse_delta = nmath.sub2(mouse_pos, prev_mouse_pos);
            world.renderer.?.adjustCameraPos(world_delta_mouse_pos);
        }

        if (rl.isKeyPressed(.space)) {
            simulating = !simulating;
        }

        if (rl.isKeyPressed(.d)) {
            show_collisions = !show_collisions;
        }

        if (rl.isKeyPressed(.k)) {
            slow_motion = !slow_motion;
        }

        if (rl.isKeyPressed(.r)) {
            world.solver.clear(alloc);
            try setupScene(&world.solver);
        }

        if (simulating) {
            steps += 1;

            if (rl.isKeyDown(.m)) {
                world.solver.bodies.items[1].props.angle += 0.02;
            }

            if (rl.isKeyDown(.n)) {
                world.solver.bodies.items[1].props.angle -= 0.02;
            }

            const start = std.time.nanoTimestamp();
            if (slow_motion) {
                const sub_dt = STANDARD_DT / SUB_STEPS;
                try world.process(alloc, sub_dt);
            } else {
                for (0..SUB_STEPS) |s| {
                    _ = s;
                    const sub_dt = STANDARD_DT / SUB_STEPS;
                    try world.process(alloc, sub_dt);
                }
            }
            const end = std.time.nanoTimestamp();
            sim_dt = @floatFromInt(end - start);
        } else {
            rl.drawText("paused", 5, 0, 64, rl.Color.white);
        }

        rl.clearBackground(.{ .r = 18, .g = 18, .b = 18, .a = 1 });
        const start = std.time.nanoTimestamp();
        world.render(show_collisions);
        const end = std.time.nanoTimestamp();
        render_dt = @floatFromInt(end - start);

        const font_size = 16;

        rl.drawText(rl.textFormat("bodies = %d : manifolds = %d", .{ world.solver.bodies.items.len, world.solver.manifolds.count() }), 5, screen_height - 3 * font_size, font_size, rl.Color.white);

        rl.drawText(rl.textFormat("sr = %.3f ms : render = %.3f ms", .{ sim_dt / 1e6, render_dt / 1e6 }), 5, screen_height - 2 * font_size, font_size, rl.Color.white);

        rl.drawText(rl.textFormat("%.3f ms : steps = %d", .{ STANDARD_DT * 1e3, steps }), 5, screen_height - font_size, font_size, rl.Color.white);
    }
}

const MouseSpring = struct {
    active: bool = false,

    const Self = @This();

    pub fn update(self: *Self, alloc: Allocator, units: zigics.Units, physics: *zigics.Solver) !void {
        const rl_pos = rl.getMousePosition();
        const mouse_pos = units.s2w(Vector2.init(rl_pos.x, rl_pos.y));

        if (!self.active and rl.isKeyPressed(.v)) {
            for (physics.bodies.items) |*body| {
                if (body.static) continue;
                if (body.isInside(mouse_pos)) {
                    const r_rotated = nmath.sub2(mouse_pos, body.props.pos);
                    const r = nmath.rotate2(r_rotated, -body.props.angle);
                    const spring = try forcegenerator.StaticSpring.init(alloc, body, mouse_pos, r, 80.0);
                    try physics.force_generators.append(spring);
                    self.active = true;
                    return;
                }
            }
        }

        if (self.active and rl.isKeyDown(.v)) {
            const gen = physics.force_generators.items[physics.force_generators.items.len - 1];
            const spring: *forcegenerator.StaticSpring = @ptrCast(@alignCast(gen.ptr));
            spring.pos = mouse_pos;
        }

        if (self.active and rl.isKeyReleased(.v)) {
            var spring = physics.force_generators.popOrNull();
            spring.?.deinit(alloc);
            self.active = false;
        }
    }
};
