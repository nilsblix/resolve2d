const std = @import("std");
const rl = @import("raylib");
const zigics = @import("zigics.zig");
const rigidbody = @import("rigidbody.zig");
const nmath = @import("nmath.zig");
const Vector2 = nmath.Vector2;

// hello world

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();

    const allocator = gpa.allocator();

    const screen_width = 1280;
    const screen_height = 720;

    var disc = try rigidbody.DiscBody.init(allocator, Vector2.init(3, 4), 3.14, 2.0, 3.0);
    defer disc.deinit(allocator);
    disc.print();

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
