const std = @import("std");
const r2d = @import("resolve2d");
const nmath = r2d.nmath;
const Vector2 = nmath.Vector2;
const Units = @import("Units.zig");

test "mults should be inverses" {
    var units = Units.init(.{ .width = 1000, .height = 500 }, 20);

    // units.adjustCameraPos(Vector2.init(1, 0.5));
    units.adjustCameraZoom(1.3, Vector2.init(-1, 2));

    const val = 3.1415;
    const ret = units.mult.s2w * units.mult.w2s * val;

    std.debug.print("mults should be inverses \n", .{});
    std.debug.print("     val = {}\n", .{val});
    std.debug.print("     ret = {}\n", .{ret});

    try std.testing.expect(ret == val);
}

test "s2w and w2s should be inverses" {
    var units = Units.init(.{ .width = 1000, .height = 500 }, 20);

    // units.adjustCameraPos(Vector2.init(1, 0.5));
    units.adjustCameraZoom(1.3, Vector2.init(-1, 2));

    const vec = Vector2.init(23, 34);
    const ret = units.s2w(units.w2s(vec));

    std.debug.print("transformations should be inverses \n", .{});
    std.debug.print("     vec = {}\n", .{vec});
    std.debug.print("     ret = {}\n", .{ret});

    try std.testing.expect(nmath.equals2(ret, vec));
}

test "units mapping makes sense" {
    const width: f32 = 1234;
    const height: f32 = 513;
    const default_width: f32 = 20;
    var units = Units.init(.{ .width = width, .height = height }, default_width);

    const screen = Vector2.init(width, height);
    const world = units.s2w(screen);

    const expected = Vector2.init(default_width, 0);

    std.debug.print("units mapping makes sense\n", .{});
    std.debug.print("     screen (input) = {}\n", .{screen});
    std.debug.print("     world (output) = {}\n", .{world});
    std.debug.print("     expected       = {}\n", .{expected});

    try std.testing.expect(nmath.approxEql2(world, expected, 0.001));
}

test "unit mapping" {
    const x: f32 = 0.8;
    const y: f32 = Units.map2(f32, x, 0, 1, 10, 20);
    const expected: f32 = 18;
    try std.testing.expect(y == expected);

    std.debug.print("unit mapping\n", .{});
    std.debug.print("     0.8 (0 -> 1) => (10, 20) should equal 18. output = {}\n", .{y});
    std.debug.print("     input = {}\n", .{x});
    std.debug.print("     output = {}\n", .{y});
    std.debug.print("     expected = {}\n", .{expected});

    try std.testing.expect(y == expected);
}
