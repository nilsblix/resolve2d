const std = @import("std");
const rl = @import("raylib");
const zigics = @import("world.zig");

// hello world

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();

    const allocator = gpa.allocator();

    const screen_width = 1280;
    const screen_height = 720;

    const world = zigics.World.init(allocator, zigics.Units.Size{ .width = screen_width, .height = screen_height }, 10);
    // world.deinit();
    _ = world;

    rl.initWindow(screen_width, screen_height, "hello world");
    defer rl.closeWindow();
    rl.setTargetFPS(120);

    var pos = rl.Vector2.init(0, 0);

    while (!rl.windowShouldClose()) {
        rl.beginDrawing();
        defer rl.endDrawing();

        rl.clearBackground(.{ .r = 60, .g = 18, .b = 18, .a = 1 });

        pos = rl.getMousePosition();

        rl.drawRectangle(screen_width / 2, screen_height / 2, screen_width / 2, screen_height / 2, rl.Color.sky_blue);

        rl.drawCircle(@intFromFloat(pos.x), @intFromFloat(pos.y), 100, rl.Color.green);
    }
}
