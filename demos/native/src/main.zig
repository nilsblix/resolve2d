const std = @import("std");
const rl = @import("raylib");
const zigics = @import("zigics");
const examples = zigics.examples;
const Renderer = @import("Renderer.zig");
const Units = @import("Units.zig");
const nmath = zigics.nmath;
const Vector2 = nmath.Vector2;

pub fn main() !void {
    const alloc = std.heap.page_allocator;

    // const SCREEN_WIDTH = 1280;
    // const SCREEN_HEIGHT = 900;
    var screen_dims = Vector2.init(900, 560);

    const TARGET_FPS: f32 = 60;
    const SUB_STEPS = 4;
    const CONSTR_ITERS = 4;

    const DT = 1 / TARGET_FPS;

    rl.setConfigFlags(.{
        .msaa_4x_hint = true,
        .window_resizable = true,
    });
    rl.initWindow(@intFromFloat(screen_dims.x), @intFromFloat(screen_dims.y), "zigics");
    defer rl.closeWindow();

    rl.setTargetFPS(TARGET_FPS);

    var solver = try zigics.Solver.init(alloc, 2, 4);
    std.debug.print("solver: {}\n", .{solver});

    // try examples.setup_0_1_car_platformer(&solver);
    // try examples.setup_0_2_bridge_stress(&solver);
    try examples.setup_0_3_many_boxes(&solver);
    // try examples.setup_0_4_also_many_boxes(&solver);
    std.debug.print("\n", .{});
    std.debug.print("solver.bodies.count() = {d}\n", .{solver.bodies.count()});
    // std.debug.print("solver.bodies.values() = {any}\n", .{solver.bodies.values()});

    var renderer = Renderer.init(.{
        .width = screen_dims.x,
        .height = screen_dims.y,
    }, 70);

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
        mouse_pos = renderer.units.s2w(screen_mouse_pos);

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

        handleCar(&solver, &renderer);

        if (rl.isWindowResized()) {
            screen_dims = Vector2.init(
                @floatFromInt(rl.getScreenWidth()),
                @floatFromInt(rl.getScreenHeight()),
            );
            renderer.units.updateDimensions(screen_dims);
        }

        rl.beginDrawing();
        defer rl.endDrawing();
        defer rl.clearBackground(.{ .r = 18, .g = 18, .b = 18, .a = 1 });

        if (rl.isKeyPressed(.right) or rl.isKeyDown(.l)) {
            try solver.process(alloc, DT, SUB_STEPS, CONSTR_ITERS);
        }

        std.debug.print("mouse pos world = {}\n", .{mouse_pos});

        {
            const der_scr = renderer.units.w2s(mouse_pos);
            rl.drawCircle(@intFromFloat(der_scr.x), @intFromFloat(der_scr.y), 5.0, rl.Color.red);
            const zero = renderer.units.w2s(Vector2.zero);
            rl.drawCircle(@intFromFloat(zero.x), @intFromFloat(zero.y), renderer.units.mult.w2s * 0.25, rl.Color.red);
            const bottom_right = renderer.units.s2w(nmath.scale2(screen_dims, 1.0));
            std.debug.print("bottom_right (world pos) = {}\n", .{bottom_right});
        }

        renderer.render(solver, false, false);

        frame += 1;
    }
}

fn handleCar(solver: *zigics.Solver, renderer: *Renderer) void {
    _ = solver;
    _ = renderer;
    // const car_handle = solver.bodyHandle(3);
    // const wl = solver.bodyHandle(4);
    // const wr = solver.bodyHandle(5);
    //
    // const wheel_mom = 20.0;
    // const body_acc = 10.0;
    //
    // if (rl.isKeyDown(.d)) {
    //     wl.body_unwrap().props.ang_momentum = -wheel_mom;
    //     wr.body_unwrap().props.ang_momentum = -wheel_mom;
    //     car_handle.body_unwrap().props.torque = if (car_handle.body_unwrap().props.momentum.length() > 8) 0 else body_acc;
    // }
    //
    // if (rl.isKeyDown(.a)) {
    //     wl.body_unwrap().props.ang_momentum = wheel_mom;
    //     wr.body_unwrap().props.ang_momentum = wheel_mom;
    //     car_handle.body_unwrap().props.torque = if (car_handle.body_unwrap().props.momentum.length() > 8) 0 else -body_acc;
    // }
    //
    // const half_view = nmath.scale2(renderer.units.camera.viewport.toVector2(), 0.25);
    // renderer.units.camera.pos = nmath.sub2(car_handle.body_unwrap().props.pos, half_view);
}
