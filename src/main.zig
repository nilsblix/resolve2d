const std = @import("std");
const rl = @import("raylib");
const zigics = @import("zigics.zig");
const rigidbody = @import("rigidbody.zig");
const forcegenerator = @import("force-generator.zig");
const nmath = @import("nmath.zig");
const Vector2 = nmath.Vector2;

// hello world

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();

    const allocator = gpa.allocator();

    const screen_width = 1280;
    const screen_height = 720;

    var world = zigics.World.init(allocator, .{ .width = screen_width, .height = screen_height }, 10, true);
    defer world.deinit(allocator);

    var disc = try rigidbody.DiscBody.init(allocator, Vector2.init(3, 5), 3.14, 2.0, 3.0);
    disc.print();
    try world.physics.bodies.append(disc);

    // const gravity = try forcegenerator.DownwardsGravity.init(allocator, 1);
    // try world.physics.force_generators.append(gravity);
    const point_gravity = try forcegenerator.PointGravity.init(allocator, 1.0, .{ .x = 5, .y = 3 });
    try world.physics.force_generators.append(point_gravity);

    const point2_gravity = try forcegenerator.PointGravity.init(allocator, 1.0, .{ .x = 8, .y = 4 });
    try world.physics.force_generators.append(point2_gravity);

    rl.initWindow(screen_width, screen_height, "hello world");
    defer rl.closeWindow();

    const HZ: i32 = 120;
    const DT: f32 = 1 / @as(f32, HZ);
    rl.setTargetFPS(HZ);

    var pos = rl.Vector2.init(0, 0);

    while (!rl.windowShouldClose()) {
        rl.beginDrawing();
        defer rl.endDrawing();

        world.process(DT);

        rl.clearBackground(.{ .r = 18, .g = 18, .b = 18, .a = 1 });

        pos = rl.getMousePosition();

        circle(Vector2.init(pos.x, pos.y), 50, rl.Color.green);

        world.render(false);
        // for (physics.bodies.items) |body| {
        //     var inv_disc_pos = body.props.pos;
        //     inv_disc_pos.y = screen_height - inv_disc_pos.y;
        //     circle(inv_disc_pos, 30, rl.Color.lime);
        // }
    }
}

pub fn circle(pos: Vector2, rad: f32, color: rl.Color) void {
    rl.drawCircle(@intFromFloat(pos.x), @intFromFloat(pos.y), rad, color);
}
