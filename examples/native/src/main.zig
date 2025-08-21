const std = @import("std");
const rl = @import("raylib");
const zigics = @import("zigics");

const WINDOW_WIDTH = 1280;
const WINDOW_HEIGHT = 900;

pub fn main() !void {
    rl.initWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "zigics");
    defer rl.closeWindow();

    // Access zigics types to verify package integration
    const Vector2 = zigics.nmath.Vector2;
    var _origin = Vector2{ .x = 0, .y = 0 };

    while (!rl.windowShouldClose()) {
        rl.beginDrawing();
        defer rl.endDrawing();

        rl.drawRectangle(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT, rl.Color.sky_blue);
    }
}
