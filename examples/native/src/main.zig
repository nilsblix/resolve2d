const std = @import("std");
const rl = @import("raylib");

const WINDOW_WIDTH = 1280;
const WINDOW_HEIGHT = 900;

pub fn main() !void {
    rl.initWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "zigics");
    defer rl.closeWindow();

    while (!rl.windowShouldClose()) {
        rl.beginDrawing();
        defer rl.endDrawing();

        rl.drawRectangle(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT, rl.Color.sky_blue);
    }
}
