const std = @import("std");
const rl = @import("raylib");
const zigics = @import("zigics");
const demos = zigics.demos;
const Renderer = @import("Renderer.zig");
const nmath = zigics.nmath;
const Vector2 = nmath.Vector2;

pub fn main() !void {
    const alloc = std.heap.page_allocator;

    const SCREEN_WIDTH = 1280;
    const SCREEN_HEIGHT = 900;

    const TARGET_FPS = 60;
    const SUB_STEPS = 4;
    const CONSTR_ITERS = 4;

    const DT = 1 / TARGET_FPS;

    rl.setConfigFlags(.{ .msaa_4x_hint = true });
    rl.initWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "zigics");
    defer rl.closeWindow();

    rl.setTargetFPS(TARGET_FPS);

    var solver = try zigics.Solver.init(alloc, 2, 4);
    std.debug.print("solver: {}\n", .{solver});

    var fac = solver.entityFactory();
    _ = try fac.makeDownwardsGravity(9.82);

    var opt = zigics.EntityFactory.BodyOptions{ .pos = .{}, .mass_prop = .{ .density = 1.0 } };
    opt.pos = zigics.nmath.Vector2.init(1, 0);
    var rect = try fac.makeRectangleBody(opt, .{ .width = 1.0, .height = 1.0 });
    rect.props.momentum.x = 1;
    rect.props.force.x = 1;

    opt.pos = zigics.nmath.Vector2.init(-1, 0);
    var rect2 = try fac.makeRectangleBody(opt, .{ .width = 1.0, .height = 1.0 });
    rect2.props.momentum.x = 1;
    rect2.props.force.x = 1;

    opt.pos = zigics.nmath.Vector2.init(-3, 0);
    var rect3 = try fac.makeRectangleBody(opt, .{ .width = 1.0, .height = 1.0 });
    rect3.props.momentum.x = 1;
    rect3.props.force.x = 1;
    rect3.static = true;

    opt.pos = zigics.nmath.Vector2.init(-3, 2);
    var disc = try fac.makeDiscBody(opt, .{ .radius = 1.0 });
    disc.props.momentum.x = 1;
    disc.props.force.x = 1;

    try fac.excludeCollisionPair(0, 1);
    _ = try fac.makeDistanceJoint(.{}, 0, 1, 2.0);
    _ = try fac.makeOffsetDistanceJoint(.{}, 1, 2, .{}, .{}, 2.0);
    _ = try fac.makeFixedPositionJoint(.{}, 3, disc.props.pos);
    _ = try fac.makeMotorJoint(.{}, 3, 1.0);

    // try demos.setupCarScene(&solver);
    // try demos.setupBridgeStressTestScene(&solver);

    var renderer = Renderer.init(.{
        .width = SCREEN_WIDTH,
        .height = SCREEN_HEIGHT,
    }, 20);

    var frame: usize = 0;

    var screen_prev_mouse_pos: Vector2 = .{};
    var screen_mouse_pos: Vector2 = .{};
    var prev_mouse_pos: Vector2 = .{};
    var mouse_pos: Vector2 = .{};

    while (!rl.windowShouldClose()) {
        screen_prev_mouse_pos = screen_mouse_pos;
        prev_mouse_pos = mouse_pos;

        const rl_pos = rl.getMousePosition();
        screen_mouse_pos = Vector2.init(rl_pos.x, rl_pos.y);
        mouse_pos = renderer.units.s2w(Vector2.init(screen_mouse_pos.x, screen_mouse_pos.y));

        const delta_screen = nmath.sub2(screen_mouse_pos, screen_prev_mouse_pos);
        var world_delta_mouse_pos = nmath.scale2(delta_screen, renderer.units.mult.s2w);
        world_delta_mouse_pos.y *= -1;

        const delta_wheel = rl.getMouseWheelMove();
        if (delta_wheel != 0) {
            renderer.adjustCameraZoom(std.math.exp(delta_wheel / 100), screen_mouse_pos);
        }

        if (rl.isMouseButtonDown(.left)) {
            renderer.adjustCameraPos(world_delta_mouse_pos);
        }

        std.debug.print("MOUSE POS WORLD: x: {d}, y: {d}\n", .{ mouse_pos.x, mouse_pos.y });

        rl.beginDrawing();
        defer rl.endDrawing();
        defer rl.clearBackground(.{ .r = 18, .g = 18, .b = 18, .a = 1 });

        try solver.process(alloc, DT, SUB_STEPS, CONSTR_ITERS);
        try renderer.render(solver, false, false);

        frame += 1;

        std.debug.print("frame: {}\n", .{frame});
    }
}
