const std = @import("std");
const nmath = @import("nmath.zig");
const Vector2 = nmath.Vector2;

pub const AABB = struct {
    pos: Vector2,
    half_width: f32,
    half_height: f32,

    const Self = @This();

    pub fn intersects(self: Self, other: AABB) bool {
        // const p = nmath.sub2(other.pos, self.pos);
        //
        // return (self.half_width > p.x - other.half_width or p.x + other.half_width > -self.half_width) and (self.half_height > p.y - other.half_height or p.y + other.half_height > -self.half_height);

        const dx = @abs(other.pos.x - self.pos.x);
        const dy = @abs(other.pos.y - self.pos.y);

        return (dx <= (self.half_width + other.half_width)) and
            (dy <= (self.half_height + other.half_height));
    }
};

test "sanity check aabb overlapping" {
    var s1 = AABB{ .pos = Vector2.init(1, 1), .half_width = 1.0, .half_height = 0.5 };
    var s2 = AABB{ .pos = Vector2.init(2, 3), .half_width = 1.5, .half_height = 3.0 };

    try std.testing.expect(s1.intersects(s2));

    s2.pos.y = 10;
    try std.testing.expect(!s1.intersects(s2));

    s1.pos.y = 10;
    try std.testing.expect(s1.intersects(s2));
}
