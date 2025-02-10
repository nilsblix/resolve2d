const std = @import("std");
const expect = std.testing.expect;

pub const Vector2 = struct {
    x: f32 = 0.0,
    y: f32 = 0.0,

    pub const zero = Vector2.init(0, 0);

    const Self = @This();
    pub fn init(x: f32, y: f32) Self {
        return .{
            .x = x,
            .y = y,
        };
    }
};

pub fn add2(a: Vector2, b: Vector2) Vector2 {
    return .{ .x = a.x + b.x, .y = a.y + b.y };
}

pub fn sub2(a: Vector2, b: Vector2) Vector2 {
    return .{ .x = a.x - b.x, .y = a.y - b.y };
}

pub fn scale2(a: Vector2, s: f32) Vector2 {
    return Vector2.init(a.x * s, a.y * s);
}

pub fn multelem2(a: Vector2, b: Vector2) Vector2 {
    return .{ .x = a.x * b.x, .y = a.y * b.y };
}

pub fn divelem2(a: Vector2, b: Vector2) Vector2 {
    std.debug.assert(b.x != 0 or b.y != 0);
    return .{ .x = a.x / b.x, .y = a.y / b.y };
}

pub fn dot2(a: Vector2, b: Vector2) f32 {
    return a.x * b.x + a.y * b.y;
}

pub fn cross2(a: Vector2, b: Vector2) f32 {
    return a.x * b.y - a.y * b.x;
}

pub fn length2sq(a: Vector2) f32 {
    return dot2(a, a);
}

pub fn length2(a: Vector2) f32 {
    return @sqrt(length2sq(a));
}

pub fn normalize2(a: Vector2) Vector2 {
    const len = length2(a);
    if (len < 1e-6) std.debug.print("nmath.normalize2 has calculated a low len = {}", .{len});
    return scale2(a, 1 / len);
}

pub fn addmult2(a: Vector2, b: Vector2, s: f32) Vector2 {
    return add2(a, scale2(b, s));
}

fn equals2(a: Vector2, b: Vector2) bool {
    return a.x == b.x and a.y == b.y;
}

test "vector2" {
    const a = Vector2.init(2, 3);
    const b = Vector2.init(4, 5);

    try expect(equals2(a, b) == false and equals2(a, Vector2.init(2, 3)) == true);

    try expect(equals2(add2(a, b), Vector2.init(6, 8)));
    try expect(equals2(sub2(a, b), Vector2.init(-2, -2)));
    try expect(equals2(multelem2(a, b), Vector2.init(8, 15)));
    try expect(equals2(divelem2(a, b), Vector2.init(0.5, 0.6)));
    try expect(dot2(a, b) == @as(f32, 23));
    try expect(cross2(a, b) == @as(f32, -2));

    const @"13": f32 = 13;
    const @"41": f32 = 41;
    try expect(length2sq(a) == @"13");
    try expect(length2sq(b) == @"41");
    try expect(length2(a) == @sqrt(@"13"));
    try expect(length2(b) == @sqrt(@"41"));
}
