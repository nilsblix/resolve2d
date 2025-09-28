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

    const SCREEN_WIDTH = 800;
    const SCREEN_HEIGHT = 500;

    const TARGET_FPS: f32 = 60;
    const SUB_STEPS = 4;
    const CONSTR_ITERS = 4;

    const DT = 1 / TARGET_FPS;

    rl.setConfigFlags(.{
        .msaa_4x_hint = true,
        .window_resizable = true,
        .window_maximized = true,
    });

    rl.initWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "zigics");
    defer rl.closeWindow();

    rl.setTargetFPS(TARGET_FPS);

    var solver = try zigics.Solver.init(alloc, 2, 4);

    try examples.setup_0_1_car_platformer(&solver);
    // try examples.setup_0_2_bridge_stress(&solver);
    // try examples.setup_0_3_many_boxes(&solver);
    // try examples.setup_0_4_also_many_boxes(&solver);

    var screen_dims = Vector2.init(
        @floatFromInt(rl.getScreenWidth()),
        @floatFromInt(rl.getScreenHeight()),
    );

    var renderer = Renderer.init(.{
        .width = screen_dims.x,
        .height = screen_dims.y,
    }, 70);

    var frame: usize = 0;

    var simulating = false;

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

        screen_dims = Vector2.init(
            @floatFromInt(rl.getScreenWidth()),
            @floatFromInt(rl.getScreenHeight()),
        );

        if (!nmath.equals2(screen_dims, renderer.units.screen_size.toVector2())) {
            renderer.units.updateDimensions(screen_dims);
        }

        rl.beginDrawing();
        defer rl.endDrawing();
        defer rl.clearBackground(.{ .r = 18, .g = 18, .b = 18, .a = 1 });

        if (rl.isKeyPressed(.right) or rl.isKeyDown(.l) or simulating) {
            try solver.process(alloc, DT, SUB_STEPS, CONSTR_ITERS);
        }

        if (rl.isKeyPressed(.space)) {
            simulating = !simulating;
        }

        {
            const der_scr = renderer.units.w2s(mouse_pos);
            rl.drawCircle(@intFromFloat(der_scr.x), @intFromFloat(der_scr.y), 5.0, rl.Color.red);
        }

        renderer.render(solver, false, false);

        frame += 1;
    }
}

fn handleCar(solver: *zigics.Solver, renderer: *Renderer) void {
    const car_handle = solver.bodyHandle(3);
    const wl = solver.bodyHandle(4);
    const wr = solver.bodyHandle(5);

    const wheel_mom = 20.0;
    const body_acc = 400.0;
    // const limit = 100;

    if (rl.isKeyDown(.d)) {
        wl.body_unwrap().props.ang_momentum = -wheel_mom;
        wr.body_unwrap().props.ang_momentum = -wheel_mom;
        car_handle.body_unwrap().props.torque = body_acc;
        // car_handle.body_unwrap().props.torque = if (car_handle.body_unwrap().props.momentum.length() > limit) 0 else body_acc;
    }

    if (rl.isKeyDown(.a)) {
        wl.body_unwrap().props.ang_momentum = wheel_mom;
        wr.body_unwrap().props.ang_momentum = wheel_mom;
        car_handle.body_unwrap().props.torque = -body_acc;
        // car_handle.body_unwrap().props.torque = if (car_handle.body_unwrap().props.momentum.length() > limit) 0 else -body_acc;
    }

    const half_view = nmath.scale2(renderer.units.camera.viewport.toVector2(), 0.25);
    renderer.units.camera.pos = nmath.sub2(car_handle.body_unwrap().props.pos, half_view);
}
