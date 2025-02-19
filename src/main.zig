const std = @import("std");
const rl = @import("raylib");
const zigics = @import("zigics.zig");
const rigidbody = @import("rigidbody.zig");
const forcegenerator = @import("force_generator.zig");
const nmath = @import("nmath.zig");
const Vector2 = nmath.Vector2;
const Allocator = std.mem.Allocator;

const MouseSpring = struct {
    active: bool = false,

    const Self = @This();

    pub fn update(self: *Self, alloc: Allocator, units: zigics.Units, physics: *zigics.Physics) !void {
        const rl_pos = rl.getMousePosition();
        const mouse_pos = units.s2w(Vector2.init(rl_pos.x, rl_pos.y));

        if (!self.active and rl.isKeyPressed(.v)) {
            for (physics.bodies.items) |*body| {
                if (body.isInside(mouse_pos)) {
                    const r_rotated = nmath.sub2(mouse_pos, body.props.pos);
                    const r = nmath.rotate2(r_rotated, -body.props.angle);
                    const spring = try forcegenerator.StaticSpring.init(alloc, body, mouse_pos, r, 20.0);
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

    // world.renderer.?.units.camera.zoom = 5.0;

    try world.physics.makeDiscBody(Vector2.init(5, 5), 2.0, 0.6);
    // _ = try world.physics.makeDiscBody(Vector2.init(3, 3), 2.0, 0.4);
    try world.physics.makeRectangleBody(Vector2.init(3, 3), 2.0, 1.0, 0.5);
    world.physics.bodies.items[1].props.ang_momentum = 1;

    var mouse_spring = MouseSpring{};

    const static_spring = try forcegenerator.StaticSpring.init(alloc, &world.physics.bodies.items[0], Vector2.init(3, 5), .{}, 20.0);
    try world.physics.force_generators.append(static_spring);

    const static_spring3 = try forcegenerator.StaticSpring.init(alloc, &world.physics.bodies.items[1], Vector2.init(3, 5), Vector2.init(-0.25, 0), 20.0);
    try world.physics.force_generators.append(static_spring3);

    const static_spring2 = try forcegenerator.StaticSpring.init(alloc, &world.physics.bodies.items[0], Vector2.init(8, 5), .{}, 20.0);
    try world.physics.force_generators.append(static_spring2);

    const gravity = try forcegenerator.DownwardsGravity.init(alloc, 20);
    try world.physics.force_generators.append(gravity);

    // const point_gravity = try forcegenerator.PointGravity.init(alloc, 1.0, .{ .x = 5, .y = 3 });
    // try world.physics.force_generators.append(point_gravity);
    //
    // const point2_gravity = try forcegenerator.PointGravity.init(alloc, 4.0, .{ .x = 8, .y = 4 });
    // try world.physics.force_generators.append(point2_gravity);

    rl.initWindow(screen_width, screen_height, "zigics");
    defer rl.closeWindow();

    const HZ: i32 = 60;
    const DT: f32 = 1 / @as(f32, HZ);
    rl.setTargetFPS(HZ);

    var simulating: bool = false;
    var steps: u32 = 0;

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
            try mouse_spring.update(alloc, rend.units, &world.physics);
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

        if (simulating) {
            steps += 1;
            world.process(DT);
        }

        rl.clearBackground(.{ .r = 18, .g = 18, .b = 18, .a = 1 });
        world.render();

        const b1 = world.physics.bodies.items[0];
        const b2 = world.physics.bodies.items[1];
        var iter = b1.normal_iter;
        while (iter.next(b1, b2)) |normal| {
            const start = world.renderer.?.units.w2s(b1.props.pos);
            const end = world.renderer.?.units.w2s(nmath.add2(b1.props.pos, normal));
            const rl_s = rl.Vector2.init(start.x, start.y);
            const rl_e = rl.Vector2.init(end.x, end.y);
            const m = world.renderer.?.units.mult.w2s;
            rl.drawLineEx(rl_s, rl_e, 0.04 * m, rl.Color.green);
        }

        if (!simulating) {
            rl.drawText("paused", 5, 0, 64, rl.Color.white);
        }

        const font_size = 16;

        rl.drawText(rl.textFormat("%.3f ms : steps = %d", .{ DT * 1e3, steps }), 5, screen_height - font_size, font_size, rl.Color.white);
    }
}
