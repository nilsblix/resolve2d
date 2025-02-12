const std = @import("std");
const rl = @import("raylib");
const zigics = @import("zigics.zig");
const rigidbody = @import("rigidbody.zig");
const forcegenerator = @import("force-generator.zig");
const nmath = @import("nmath.zig");
const Vector2 = nmath.Vector2;
const Allocator = std.mem.Allocator;

const MouseSpring = struct {
    active: bool = false,

    const Self = @This();

    pub fn update(self: *Self, alloc: Allocator, units: zigics.Units, physics: *zigics.Physics) !void {
        const rl_pos = rl.getMousePosition();
        const mouse_pos = units.s2w(Vector2.init(rl_pos.x, rl_pos.y));

        if (!self.active and rl.isMouseButtonPressed(.left)) {
            for (physics.bodies.items) |*body| {
                if (body.type == .disc) {
                    const disc: *rigidbody.DiscBody = @ptrCast(@alignCast(body.ptr));
                    const len2 = nmath.length2sq(nmath.sub2(body.props.pos, mouse_pos));
                    if (len2 < disc.radius * disc.radius) {
                        const spring = try forcegenerator.StaticSpring.init(alloc, body, mouse_pos, 20.0);
                        try physics.force_generators.append(spring);
                        self.active = true;
                        return;
                    }
                }
            }
        }

        if (self.active and rl.isMouseButtonDown(.left)) {
            const gen = physics.force_generators.items[physics.force_generators.items.len - 1];
            const spring: *forcegenerator.StaticSpring = @ptrCast(@alignCast(gen.ptr));
            spring.pos = mouse_pos;
        }

        if (self.active and rl.isMouseButtonReleased(.left)) {
            var spring = physics.force_generators.popOrNull();
            spring.?.deinit(alloc);
            self.active = false;
        }
    }
};

// const MouseSpring = struct {
//     spring: ?*forcegenerator.ForceGenerator,
//     active: bool = false,
//
//     const stiffness: f32 = 15.0;
//     const Self = @This();
//
//     pub fn update(self: *Self, units: zigics.Units, physics: zigics.Physics) void {
//         const rl_mouse_pos = rl.getMousePosition();
//         const mouse_pos = units.s2w(Vector2.init(rl_mouse_pos.x, rl_mouse_pos.y));
//         const spring: *forcegenerator.StaticSpring = @ptrCast(@alignCast(self.spring.?.ptr));
//
//         if (rl.isMouseButtonPressed(.left)) {
//             for (physics.bodies.items) |*body| {
//                 if (body.type == .disc) {
//                     const disc: *rigidbody.DiscBody = @ptrCast(@alignCast(body.ptr));
//                     const len2 = nmath.length2sq(nmath.sub2(body.props.pos, mouse_pos));
//                     if (len2 < disc.radius * disc.radius) {
//                         // spring.body = body;
//                         // spring.pos = mouse_pos;
//                         // self.active = true;
//                         return;
//                     }
//                 }
//             }
//
//             self.active = false;
//         }
//
//         spring.pos = mouse_pos;
//
//         if (!rl.isMouseButtonDown(.left)) {
//             self.active = false;
//         }
//
//         if (rl.isMouseButtonReleased(.left)) {
//             self.active = false;
//         }
//     }
// };

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();

    const alloc = gpa.allocator();

    const screen_width = 1280;
    const screen_height = 720;

    var world = zigics.World.init(alloc, .{ .width = screen_width, .height = screen_height }, 10, true);
    defer world.deinit();

    _ = try world.physics.makeDiscBody(Vector2.init(5, 5), 2.0, 0.2);
    _ = try world.physics.makeDiscBody(Vector2.init(3, 3), 2.0, 0.2);

    var mouse_spring = MouseSpring{};

    world.physics.bodies.items[0].props.ang_momentum = 1;

    const static_spring = try forcegenerator.StaticSpring.init(alloc, &world.physics.bodies.items[0], Vector2.init(3, 5), 20.0);
    try world.physics.force_generators.append(static_spring);

    const static_spring3 = try forcegenerator.StaticSpring.init(alloc, &world.physics.bodies.items[1], Vector2.init(3, 5), 20.0);
    try world.physics.force_generators.append(static_spring3);

    const static_spring2 = try forcegenerator.StaticSpring.init(alloc, &world.physics.bodies.items[0], Vector2.init(8, 5), 20.0);
    try world.physics.force_generators.append(static_spring2);

    const gravity = try forcegenerator.DownwardsGravity.init(alloc, 20);
    try world.physics.force_generators.append(gravity);

    const point_gravity = try forcegenerator.PointGravity.init(alloc, 1.0, .{ .x = 5, .y = 3 });
    try world.physics.force_generators.append(point_gravity);

    const point2_gravity = try forcegenerator.PointGravity.init(alloc, 4.0, .{ .x = 8, .y = 4 });
    try world.physics.force_generators.append(point2_gravity);

    rl.initWindow(screen_width, screen_height, "zigics");
    defer rl.closeWindow();

    const HZ: i32 = 120;
    const DT: f32 = 1 / @as(f32, HZ);
    rl.setTargetFPS(HZ);

    var simulating: bool = false;
    var steps: u32 = 0;

    while (!rl.windowShouldClose()) {
        rl.beginDrawing();
        defer rl.endDrawing();

        if (world.renderer) |rend| {
            try mouse_spring.update(alloc, rend.units, &world.physics);
        }

        if (rl.isKeyPressed(.space)) {
            simulating = !simulating;
        }

        if (simulating) {
            steps += 1;
            world.process(DT);
        }

        rl.clearBackground(.{ .r = 18, .g = 18, .b = 18, .a = 1 });
        world.render(true);

        if (!simulating) {
            rl.drawText("paused", 5, 0, 64, rl.Color.white);
        }

        // var buf: [256]u8 = undefined;
        // const str = try std.fmt.bufPrint(&buf, "{}", .{steps});
        // const str_many_ptr: [*]const u8 = &str;
        // // rl.drawText(@as([]const u8, str), 5, 64, 32, rl.Color.white);
        // rl.drawText(str_many_ptr, 5, 64, 32, rl.Color.white);
    }
}
