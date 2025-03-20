const std = @import("std");
const rl = @import("raylib");
const zigics = @import("zigics.zig");
const rigidbody = @import("rigidbody.zig");
const forcegenerator = @import("force_generator.zig");
const nmath = @import("nmath.zig");
const Vector2 = nmath.Vector2;
const Allocator = std.mem.Allocator;
const collision = @import("collision.zig");
const renderer = @import("default_renderer.zig");
const ctrs = @import("constraint.zig");

pub fn setupConstraints(solver: *zigics.Solver) !void {
    var factory = solver.entityFactory();

    var opt: zigics.EntityFactory.BodyOptions = .{ .pos = .{}, .mass_prop = .{ .density = 5 } };
    // opt.mu_d = 0.2;
    // opt.mu_s = 0.3;
    opt.mu_d = 0.3;
    opt.mu_s = 0.4;
    var body: *rigidbody.RigidBody = undefined;

    opt.pos = Vector2.init(0, 0);
    body = try factory.makeRectangleBody(opt, .{ .width = 10.0, .height = 1.0 });
    body.static = true;

    opt.pos = Vector2.init(-9, -2);
    opt.angle = 0.45;
    body = try factory.makeRectangleBody(opt, .{ .width = 9.0, .height = 1.0 });
    body.static = true;
    opt.angle = 0;

    opt.pos = Vector2.init(-18, -4);
    body = try factory.makeRectangleBody(opt, .{ .width = 10.0, .height = 1.0 });
    body.static = true;

    opt.pos = Vector2.init(9, 2);
    opt.angle = 0.45;
    body = try factory.makeRectangleBody(opt, .{ .width = 9.0, .height = 1.0 });
    body.static = true;
    opt.angle = 0;

    opt.pos = Vector2.init(18, 4);
    body = try factory.makeRectangleBody(opt, .{ .width = 10.0, .height = 1.0 });
    body.static = true;

    opt.pos = Vector2.init(-22, 6);
    body = try factory.makeRectangleBody(opt, .{ .width = 1.0, .height = 20 });
    body.static = true;

    opt.pos = Vector2.init(22, 14);
    body = try factory.makeRectangleBody(opt, .{ .width = 1.0, .height = 20 });
    body.static = true;

    try factory.makeDownwardsGravity(9.82);

    opt.pos = Vector2.init(-1, 5);
    body = try factory.makeRectangleBody(opt, .{ .width = 2.0, .height = 1.0 });

    opt.pos = Vector2.init(1, 5);
    body = try factory.makeRectangleBody(opt, .{ .width = 2.0, .height = 1.0 });

    _ = try factory.makeSingleLinkJoint(.{ .beta = 5 }, body, .{}, Vector2.init(0, 5.001), 0.0);

    opt.pos = Vector2.init(3, 5);
    body = try factory.makeRectangleBody(opt, .{ .width = 2.0, .height = 1.0 });
}

pub fn setupStacking(solver: *zigics.Solver) !void {
    var factory = solver.entityFactory();

    var opt: zigics.EntityFactory.BodyOptions = .{ .pos = .{}, .mass_prop = .{ .density = 5 } };
    // opt.mu_d = 0.2;
    // opt.mu_s = 0.3;
    opt.mu_d = 0.1;
    opt.mu_s = 0.2;
    var body: *rigidbody.RigidBody = undefined;

    opt.pos = Vector2.init(0, 0);
    body = try factory.makeRectangleBody(opt, .{ .width = 10.0, .height = 1.0 });
    body.static = true;

    opt.pos = Vector2.init(-9, -2);
    opt.angle = 0.45;
    body = try factory.makeRectangleBody(opt, .{ .width = 9.0, .height = 1.0 });
    body.static = true;
    opt.angle = 0;

    opt.pos = Vector2.init(-18, -4);
    body = try factory.makeRectangleBody(opt, .{ .width = 10.0, .height = 1.0 });
    body.static = true;

    opt.pos = Vector2.init(9, 2);
    opt.angle = 0.45;
    body = try factory.makeRectangleBody(opt, .{ .width = 9.0, .height = 1.0 });
    body.static = true;
    opt.angle = 0;

    opt.pos = Vector2.init(18, 4);
    body = try factory.makeRectangleBody(opt, .{ .width = 10.0, .height = 1.0 });
    body.static = true;

    opt.pos = Vector2.init(-22, 6);
    body = try factory.makeRectangleBody(opt, .{ .width = 1.0, .height = 20 });
    body.static = true;

    opt.pos = Vector2.init(22, 14);
    body = try factory.makeRectangleBody(opt, .{ .width = 1.0, .height = 20 });
    body.static = true;

    try factory.makeDownwardsGravity(9.82);

    opt.mass_prop = .{ .density = 5 };

    opt.pos.x = 0.0;
    var width: f32 = 1.0;
    var height: f32 = 0.7;
    for (0..10) |y| {
        opt.pos.y = @as(f32, @floatFromInt(y)) * height + 1.0;
        _ = try factory.makeRectangleBody(opt, .{ .width = width, .height = height });
    }

    width = 0.8;
    height = 1.0;
    opt.pos.x = 15.0;
    for (0..10) |y| {
        opt.pos.y = @as(f32, @floatFromInt(y)) * height + 5.1;
        _ = try factory.makeRectangleBody(opt, .{ .width = width, .height = height });
    }

    // opt.pos = Vector2.init(18, 10);
    // opt.vel = Vector2.init(-15, 0);
    // opt.angle = -0.5;
    // _ = try factory.makeRectangleBody(opt, .{ .width = 1.0, .height = 0.8 });
    // opt.angle = 0.0;
    // opt.vel = .{};

    opt.pos = Vector2.init(-20, -3.25);
    _ = try factory.makeRectangleBody(opt, .{ .width = 1.0, .height = 0.7 });

    opt.pos = Vector2.init(-18, -3.25);
    _ = try factory.makeRectangleBody(opt, .{ .width = 1.0, .height = 0.7 });

    opt.pos = Vector2.init(-16, -3.25);
    opt.angle = 0.1;
    _ = try factory.makeRectangleBody(opt, .{ .width = 1.0, .height = 0.7 });
    opt.angle = 0.0;
}

pub fn setupPrimary(solver: *zigics.Solver) !void {
    var factory = solver.entityFactory();

    var opt: zigics.EntityFactory.BodyOptions = .{ .pos = .{}, .mass_prop = .{ .density = 1 } };
    // opt.mu_d = 0.2;
    // opt.mu_s = 0.3;
    opt.mu_d = 0.1;
    opt.mu_s = 0.2;
    var body: *rigidbody.RigidBody = undefined;

    opt.pos = Vector2.init(0, 0);
    body = try factory.makeRectangleBody(opt, .{ .width = 10.0, .height = 1.0 });
    body.static = true;

    opt.pos = Vector2.init(-9, -2);
    opt.angle = 0.45;
    body = try factory.makeRectangleBody(opt, .{ .width = 9.0, .height = 1.0 });
    body.static = true;
    opt.angle = 0;

    opt.pos = Vector2.init(-18, -4);
    body = try factory.makeRectangleBody(opt, .{ .width = 10.0, .height = 1.0 });
    body.static = true;

    opt.pos = Vector2.init(9, 2);
    opt.angle = 0.45;
    body = try factory.makeRectangleBody(opt, .{ .width = 9.0, .height = 1.0 });
    body.static = true;
    opt.angle = 0;

    opt.pos = Vector2.init(18, 4);
    body = try factory.makeRectangleBody(opt, .{ .width = 10.0, .height = 1.0 });
    body.static = true;

    opt.pos = Vector2.init(-22, 6);
    body = try factory.makeRectangleBody(opt, .{ .width = 1.0, .height = 20 });
    body.static = true;

    opt.pos = Vector2.init(22, 14);
    body = try factory.makeRectangleBody(opt, .{ .width = 1.0, .height = 20 });
    body.static = true;

    opt.pos = Vector2.init(0, 8);
    body = try factory.makeDiscBody(opt, .{ .radius = 2.0 });
    body.static = true;

    opt.pos = Vector2.init(-12, 5);
    opt.angle = -0.4;
    body = try factory.makeRectangleBody(opt, .{ .width = 10.0, .height = 0.3 });
    body.static = true;
    opt.angle = 0;

    opt.mass_prop = .{ .density = 5.0 };

    opt.pos = Vector2.init(-10, 15);
    _ = try factory.makeRectangleBody(opt, .{ .width = 4.0, .height = 2.0 });

    for (0..10) |x| {
        for (0..35) |y| {
            const xf = @as(f32, @floatFromInt(x)) + 10;
            const yf = @as(f32, @floatFromInt(y)) + 10;

            opt.pos = Vector2.init(xf, yf);

            if (@mod(x, 2) == 0 and @mod(y, 2) == 0) {
                _ = try factory.makeDiscBody(opt, .{ .radius = 0.5 });
            } else {
                _ = try factory.makeRectangleBody(opt, .{ .width = 1.0, .height = 0.8 });
            }
        }
    }

    opt.pos.x = 15;
    const height = 0.7;
    for (0..15) |y| {
        opt.pos.y = 5 + height * @as(f32, @floatFromInt(y));

        _ = try factory.makeRectangleBody(opt, .{ .width = 1.0, .height = height });
    }

    opt.pos = Vector2.init(-10, 10);
    body = try factory.makeRectangleBody(opt, .{ .width = 8.0, .height = 0.6 });

    const q = nmath.add2(body.props.pos, Vector2.init(0.01, 0));
    _ = try factory.makeSingleLinkJoint(.{ .beta = 5 }, body, .{}, q, 0.0);

    try factory.makeDownwardsGravity(9.82);
}

pub fn setupDominos(solver: *zigics.Solver) !void {
    var factory = solver.entityFactory();

    var opt: zigics.EntityFactory.BodyOptions = .{ .pos = .{}, .mass_prop = .{ .density = 5 } };
    opt.mu_d = 0.5;
    opt.mu_s = opt.mu_d;

    const AREA_WIDTH: f32 = 30;
    const AREA_HEIGHT: f32 = 10;

    const AREA_POS = Vector2.init(0, -0.5);

    _ = try factory.makeDownwardsGravity(9.82);

    opt.pos = Vector2.init(AREA_POS.x, AREA_POS.y);
    var ground = try factory.makeRectangleBody(opt, .{ .width = AREA_WIDTH, .height = 1 });
    ground.static = true;

    _ = AREA_HEIGHT;
    opt.mass_prop = .{ .density = 20.0 };

    const height: f32 = 2.5;
    const width: f32 = 0.6;
    opt.pos.y = height / 2 - 0.15;
    opt.angle = 0.02;
    const rect_opt: zigics.EntityFactory.RectangleOptions = .{ .width = width, .height = height };
    for (1..13) |xi| {
        const x_offset: f32 = 2.51 * (@as(f32, @floatFromInt(xi)) - 7);
        opt.pos.x = x_offset;
        if (xi == 1) {
            // var saved = opt;
            // saved.pos.x += 1.5;
            // var body = try factory.makeRectangleBody(saved, rect_opt);
            // body.props.angle = -0.9;
        } else {
            _ = try factory.makeRectangleBody(opt, rect_opt);
        }
    }
}

pub fn setupCollisionPointTestScene(solver: *zigics.Solver) !void {
    var factory = solver.entityFactory();

    var opt: zigics.EntityFactory.BodyOptions = .{ .pos = .{}, .mass_prop = .{ .density = 5 } };
    opt.mu_d = 0.2;
    opt.mu_s = opt.mu_d;

    const AREA_WIDTH: f32 = 60;
    const AREA_POS = Vector2.init(0, -0.5);

    _ = try factory.makeDownwardsGravity(9.82);

    opt.pos = Vector2.init(AREA_POS.x, AREA_POS.y);
    var ground = try factory.makeRectangleBody(opt, .{ .width = AREA_WIDTH, .height = 1 });
    ground.static = true;

    const width = 30;
    const x_block = width / 3;
    const angle = 0.25;

    opt.mu_d = 0.0;
    opt.mass_prop = .{ .mass = 1 };

    opt.pos = Vector2.init(-x_block, 20);
    opt.angle = -angle;
    var body = try factory.makeRectangleBody(opt, .{ .width = width, .height = 0.5 });
    body.static = true;
    opt.angle = 0;

    opt.pos = Vector2.init(x_block, 15);
    opt.angle = angle;
    body = try factory.makeRectangleBody(opt, .{ .width = width, .height = 0.5 });
    body.static = true;
    opt.angle = 0;

    opt.pos = Vector2.init(-x_block, 10);
    opt.angle = -angle;
    body = try factory.makeRectangleBody(opt, .{ .width = width, .height = 0.5 });
    body.static = true;
    opt.angle = 0;

    opt.pos = Vector2.init(x_block, 5);
    opt.angle = angle;
    body = try factory.makeRectangleBody(opt, .{ .width = width, .height = 0.5 });
    body.static = true;
    opt.angle = 0;

    opt.mass_prop = .{ .mass = 1.0 };

    const x: f32 = -x_block - width * 0.4;
    opt.angle = -angle;
    for (0..10) |xi| {
        opt.pos = Vector2.init(x + 1.8 * @as(f32, @floatFromInt(xi)), 28);
        opt.mu_d = 0.6 - 0.05 * @as(f32, @floatFromInt(xi));
        _ = try factory.makeRectangleBody(opt, .{ .width = 1.0, .height = 0.8 });
    }

    _ = try factory.makeDownwardsGravity(9.82);
}

pub fn setupArchScene(solver: *zigics.Solver) !void {
    var factory = solver.entityFactory();

    var opt: zigics.EntityFactory.BodyOptions = .{ .pos = .{}, .mass_prop = .{ .density = 5 } };
    opt.mu_d = 0.6; // Dynamic friction
    opt.mu_s = 0.8; // Static friction

    const GROUND_WIDTH: f32 = 20;
    const GROUND_HEIGHT: f32 = 1;
    const ARCH_CENTER = Vector2.init(5, 3);
    const ARCH_RADIUS: f32 = 5.0;
    const NUM_BLOCKS: f32 = 10;
    const BLOCK_WIDTH: f32 = 1.5;
    const BLOCK_HEIGHT: f32 = 0.5;
    const ANGLE_STEP: f32 = @as(f32, std.math.pi) / (NUM_BLOCKS + 1);

    // Ground
    opt.pos = Vector2.init(ARCH_CENTER.x, ARCH_CENTER.y - ARCH_RADIUS - 1);
    var ground = try factory.makeRectangleBody(opt, .{ .width = GROUND_WIDTH, .height = GROUND_HEIGHT });
    ground.static = true;

    // Arch blocks
    for (0..NUM_BLOCKS) |i| {
        const if32: f32 = @floatFromInt(i);
        const angle = (if32 + 1) * ANGLE_STEP - @as(f32, std.math.pi) / 2;
        opt.pos.x = ARCH_CENTER.x + ARCH_RADIUS * @cos(angle);
        opt.pos.y = ARCH_CENTER.y + ARCH_RADIUS * @sin(angle);
        opt.mass_prop = .{ .density = 5 };
        var block = try factory.makeRectangleBody(opt, .{ .width = BLOCK_WIDTH, .height = BLOCK_HEIGHT });
        block.props.angle = angle;
    }

    // Keystone at the top
    opt.pos = Vector2.init(ARCH_CENTER.x, ARCH_CENTER.y + ARCH_RADIUS);
    opt.mass_prop = .{ .mass = 10 };
    var keystone = try factory.makeRectangleBody(opt, .{ .width = BLOCK_WIDTH, .height = BLOCK_HEIGHT });
    keystone.props.angle = 0;

    try factory.makeDownwardsGravity(9.82);
}

pub fn setupScene(solver: *zigics.Solver) !void {
    var factory = solver.entityFactory();

    var opt: zigics.EntityFactory.BodyOptions = .{ .pos = .{}, .mass_prop = .{ .mass = 1.0 } };

    opt.mass_prop = .{ .density = 5 };
    opt.mu_d = 0.5;
    opt.mu_s = 0.7;

    const AREA_WIDTH: f32 = 30;
    const AREA_HEIGHT: f32 = 10;

    const AREA_POS = Vector2.init(0, -0.5);

    opt.pos = Vector2.init(AREA_POS.x, AREA_POS.y);
    var ground = try factory.makeRectangleBody(opt, .{ .width = AREA_WIDTH, .height = 1 });
    ground.static = true;

    opt.pos = Vector2.init(AREA_POS.x + 0.5 - AREA_WIDTH / 2, AREA_POS.y - 0.5 + AREA_HEIGHT / 2);
    var wall1 = try factory.makeRectangleBody(opt, .{ .width = 1, .height = 10 });
    wall1.static = true;

    opt.pos = Vector2.init(AREA_POS.x - 0.5 + AREA_WIDTH / 2, AREA_POS.y - 0.5 + AREA_HEIGHT / 2);
    var wall2 = try factory.makeRectangleBody(opt, .{ .width = 1, .height = 10 });
    wall2.static = true;

    opt.pos = Vector2.init(5, 5);
    opt.angle = 0.5;
    var float1 = try factory.makeRectangleBody(opt, .{ .width = 3.0, .height = 1.0 });
    float1.static = true;
    opt.angle = 0;

    opt.pos = Vector2.init(2, 4.5);
    var float2 = try factory.makeRectangleBody(opt, .{ .width = 4.0, .height = 1.0 });
    float2.static = true;

    opt.pos = Vector2.init(-2, 4.5);
    var float3 = try factory.makeRectangleBody(opt, .{ .width = 4.0, .height = 1.0 });
    float3.static = true;

    // opt.pos = Vector2.init(-5, 4);
    // var float4 = try factory.makeDiscBody(opt, .{ .radius = 1.5 });
    // float4.static = true;

    opt.pos = Vector2.init(-6, 3.5);
    opt.angle = -0.1;
    var float5 = try factory.makeRectangleBody(opt, .{ .width = 4.0, .height = 1.0 });
    opt.angle = 0;
    float5.static = true;

    opt.pos = Vector2.init(-3, 14);
    opt.mass_prop = .{ .density = 10.0 };
    _ = try factory.makeRectangleBody(opt, .{ .width = 3.0, .height = 1.0 });

    opt.pos.x = 6;
    opt.mass_prop = .{ .density = 4.0 };
    for (1..20) |y| {
        var yf: f32 = @floatFromInt(y);
        yf /= 2;
        opt.pos.y = yf;
        _ = try factory.makeRectangleBody(opt, .{ .width = 1.0, .height = 0.5 });
    }

    for (0..3) |y| {
        for (0..17) |x| {
            const yf: f32 = @floatFromInt(y);
            const xf: f32 = @as(f32, @floatFromInt(x)) - 5;

            opt.pos = Vector2.init(xf, yf + 6);

            if (@mod(x, 2) == 1 or @mod(y, 2) == 0) {
                _ = try factory.makeRectangleBody(opt, .{ .width = 1.0, .height = 0.5 });
            } else {
                _ = try factory.makeDiscBody(opt, .{ .radius = 0.5 });
            }
        }
    }

    try factory.makeDownwardsGravity(9.82);
}
