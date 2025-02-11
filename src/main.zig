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

    const alloc = gpa.allocator();

    const screen_width = 1280;
    const screen_height = 720;

    var world = zigics.World.init(alloc, .{ .width = screen_width, .height = screen_height }, 10, true);
    defer world.deinit(alloc);

    const disc = try rigidbody.DiscBody.init(alloc, Vector2.init(3, 5), 3.14, 2.0, 0.5);
    try world.physics.bodies.append(disc);

    try world.physics.bodies.append(try rigidbody.DiscBody.init(alloc, Vector2.init(8, 1), 0, 2.0, 1));

    // const gravity = try forcegenerator.DownwardsGravity.init(allocator, 1);
    // try world.physics.force_generators.append(gravity);
    const point_gravity = try forcegenerator.PointGravity.init(alloc, 1.0, .{ .x = 5, .y = 3 });
    try world.physics.force_generators.append(point_gravity);

    const point2_gravity = try forcegenerator.PointGravity.init(alloc, 1.0, .{ .x = 8, .y = 4 });
    try world.physics.force_generators.append(point2_gravity);

    rl.initWindow(screen_width, screen_height, "zigics");
    defer rl.closeWindow();

    const HZ: i32 = 120;
    const DT: f32 = 1 / @as(f32, HZ);
    rl.setTargetFPS(HZ);

    while (!rl.windowShouldClose()) {
        rl.beginDrawing();
        defer rl.endDrawing();

        world.process(DT);

        rl.clearBackground(.{ .r = 18, .g = 18, .b = 18, .a = 1 });

        world.render(true);
    }
}
