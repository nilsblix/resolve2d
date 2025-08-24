const std = @import("std");
const nmath = @import("nmath.zig");
const Vector2 = nmath.Vector2;
const consts = @import("simulation_constants.zig");

pos: Vector2,
half_width: f32,
half_height: f32,

const AABB = @This();

pub fn intersects(self: AABB, other: AABB) bool {
    const EPS = consts.AABB_EPS_OVERLAP;
    const dx = @abs(other.pos.x - self.pos.x);
    const dy = @abs(other.pos.y - self.pos.y);

    return (dx <= (self.half_width + other.half_width) + EPS) and
        (dy <= (self.half_height + other.half_height) + EPS);
}

pub fn isInside(self: AABB, pos: Vector2) bool {
    return pos.x >= self.pos.x - self.half_width and pos.x <= self.pos.x + self.half_width and pos.y >= self.pos.y - self.half_height and pos.y <= self.pos.y + self.half_height;
}

pub fn getVertices(self: AABB) [4]Vector2 {
    return .{
        Vector2.init(self.pos.x - self.half_width, self.pos.y - self.half_height),
        Vector2.init(self.pos.x + self.half_width, self.pos.y - self.half_height),
        Vector2.init(self.pos.x + self.half_width, self.pos.y + self.half_height),
        Vector2.init(self.pos.x - self.half_width, self.pos.y + self.half_height),
    };
}

test "sanity check aabb overlapping" {
    var s1 = AABB{ .pos = Vector2.init(1, 1), .half_width = 1.0, .half_height = 0.5 };
    var s2 = AABB{ .pos = Vector2.init(2, 3), .half_width = 1.5, .half_height = 3.0 };

    try std.testing.expect(s1.intersects(s2));

    s2.pos.y = 10;
    try std.testing.expect(!s1.intersects(s2));

    s1.pos.y = 10;
    try std.testing.expect(s1.intersects(s2));
}
