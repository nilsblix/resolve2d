const zigics = @import("zigics.zig");
const rigidbody = @import("rigidbody.zig");
const forcegenerator = @import("force_generator.zig");
const nmath = @import("nmath.zig");
const Vector2 = nmath.Vector2;
const collision = @import("collision.zig");
const ctrs = @import("constraint.zig");
const Solver = zigics.Solver;

pub fn setupCarScene(solver: *Solver) !void {
    var fac = solver.entityFactory();

    var opt: zigics.EntityFactory.BodyOptions = .{ .pos = .{}, .mass_prop = .{ .density = 1 } };
    opt.mu = 0.3;
    var body: *rigidbody.RigidBody = undefined;

    try fac.makeDownwardsGravity(9.82);

    // === PLATFORM 1 ===
    opt.pos = Vector2.init(0, -100);
    body = try fac.makeRectangleBody(opt, .{ .width = 1000, .height = 20 });
    body.static = true;

    opt.pos = Vector2.init(0, 0);
    body = try fac.makeRectangleBody(opt, .{ .width = 40, .height = 10 });
    body.static = true;

    opt.pos = Vector2.init(-19, 15);
    body = try fac.makeRectangleBody(opt, .{ .width = 2, .height = 20 });
    body.static = true;

    // CAR
    opt.pos = Vector2.init(5, 10);
    body = try fac.makeRectangleBody(opt, .{ .width = 3, .height = 1.2 });
    const rad: f32 = 0.55;
    opt.pos = Vector2.init(4.1, 9.8 - rad);
    const wheel_l = try fac.makeDiscBody(opt, .{ .radius = rad });
    opt.pos.x = 5.9;
    const wheel_r = try fac.makeDiscBody(opt, .{ .radius = rad });

    try fac.excludeCollisionPair(body.id, wheel_l.id);
    try fac.excludeCollisionPair(body.id, wheel_r.id);

    // Obstacles
    opt.pos = Vector2.init(-15, 8);
    var rect_opt = zigics.EntityFactory.RectangleOptions {
        .width = 0.6,
        .height = 0.4,
    };
    var x: f32 = -15;
    opt.pos = Vector2.init(x, 9);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 10);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 11);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 12);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 13);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    x = -14;
    opt.pos = Vector2.init(x, 9);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 10);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 11);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 12);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 13);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    x = -13;
    opt.pos = Vector2.init(x, 9);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 10);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 11);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 12);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 13);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    x = -12;
    opt.pos = Vector2.init(x, 9);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 10);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 11);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 12);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 13);
    _ = try fac.makeRectangleBody(opt, rect_opt);

    // Rocks
    opt.pos = Vector2.init(0, 5);
    opt.angle = 1.0;
    rect_opt = .{ .width = 0.8, .height = 0.4 };
    body = try fac.makeRectangleBody(opt, rect_opt);
    body.static = true;

    opt.pos = Vector2.init(-3, 5);
    opt.angle = 3.0;
    rect_opt = .{ .width = 0.8, .height = 0.7 };
    body = try fac.makeRectangleBody(opt, rect_opt);
    body.static = true;

    opt.pos = Vector2.init(1, 5);
    opt.angle = 0.5;
    rect_opt = .{ .width = 0.8, .height = 0.4 };
    body = try fac.makeRectangleBody(opt, rect_opt);
    body.static = true;

    opt.pos = Vector2.init(5, 5);
    opt.angle = 3.0;
    rect_opt = .{ .width = 0.3, .height = 0.4 };
    body = try fac.makeRectangleBody(opt, rect_opt);
    body.static = true;

    opt.pos = Vector2.init(-9, 5);
    opt.angle = -1.0;
    rect_opt = .{ .width = 0.9, .height = 0.3 };
    body = try fac.makeRectangleBody(opt, rect_opt);
    body.static = true;

    opt.pos = Vector2.init(-10, 5);
    opt.angle = -2.0;
    rect_opt = .{ .width = 0.8, .height = 0.6 };
    body = try fac.makeRectangleBody(opt, rect_opt);
    body.static = true;

    opt.angle = 0;

    opt.pos = Vector2.init(20, 12);
    body = try fac.makeRectangleBody(opt, .{ .width = 15, .height = 0.5 });
    body.static = true;

    opt.pos = Vector2.init(7, 14);
    opt.angle = -0.3;
    body = try fac.makeRectangleBody(opt, .{ .width = 14, .height = 0.5 });
    body.static = true;

    // === CONNECTING BRIDGE ===
    opt.pos = Vector2.init(30, 7);
    opt.angle = 0.25;
    body = try fac.makeRectangleBody(opt, .{ .width = 20, .height = 1.0 });
    body.static = true;
    opt.angle = 0;

    // Rotating thing
    opt.pos = Vector2.init(35, 13);
    body = try fac.makeRectangleBody(opt, .{ .width = 10.5, .height = 0.5 });
    var q = nmath.add2(body.props.pos, Vector2.init(0, 0.01));
    _ = try fac.makeSingleLinkJoint(.{}, body.id, .{}, q, 0.0);

    opt.pos = Vector2.init(38, 19);
    _ = try fac.makeDiscBody(opt, .{ .radius = 1.0 });

    // === PLATFORM 2 ===
    opt.pos = Vector2.init(48, 9.5);
    body = try fac.makeRectangleBody(opt, .{ .width = 15, .height = 1 });
    body.static = true;

    opt.mass_prop = .{ .mass = 100 };
    opt.pos = Vector2.init(52, 12);
    _ = try fac.makeRectangleBody(opt, .{ .width = 3, .height = 3 });
    opt.mass_prop = .{ .density = 1 };

    rect_opt = zigics.EntityFactory.RectangleOptions {
        .width = 0.6,
        .height = 0.4,
    };
    x = 45;
    opt.pos = Vector2.init(x, 13);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 14);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 15);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 16);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 17);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    x = 46;
    opt.pos = Vector2.init(x, 13);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 14);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 15);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 16);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 17);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    x = 47;
    opt.pos = Vector2.init(x, 13);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 14);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 15);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 16);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 17);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    x = 48;
    opt.pos = Vector2.init(x, 13);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 14);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 15);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 16);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 17);
    _ = try fac.makeRectangleBody(opt, rect_opt);

    // === CONNECTING THING 2 ===

    opt.pos = Vector2.init(76.2, 3.3);
    opt.angle = -0.3;
    body = try fac.makeRectangleBody(opt, .{ .width = 40, .height = 1.0 });
    body.static = true;
    opt.angle = 0;

    rect_opt = zigics.EntityFactory.RectangleOptions {
        .width = 0.9,
        .height = 0.4,
    };
    opt.mu = 0.7;
    x = 65;
    opt.pos = Vector2.init(x, 9);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 10);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 11);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 12);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 13);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    x = 66;
    opt.pos = Vector2.init(x, 9);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 10);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 11);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 12);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 13);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    x = 67;
    opt.pos = Vector2.init(x, 9);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 10);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 11);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 12);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 13);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    x = 68;
    opt.pos = Vector2.init(x, 9);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 10);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 11);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 12);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.pos = Vector2.init(x, 13);
    _ = try fac.makeRectangleBody(opt, rect_opt);
    opt.mu = 0.5;

    // === PLATFORM 3 ===
    opt.pos = Vector2.init(110, -2);
    body = try fac.makeRectangleBody(opt, .{ .width = 35, .height = 1.0 });
    body.static = true;

    opt.pos = Vector2.init(110, -4);
    body = try fac.makeDiscBody(opt, .{ .radius = 4.0 });
    body.static = true;

    opt.pos = Vector2.init(100, 5);
    body = try fac.makeRectangleBody(opt, .{ .width = 13, .height = 0.5 });
    q = nmath.add2(body.props.pos, Vector2.init(0, 0.01));
    _ = try fac.makeSingleLinkJoint(.{}, body.id, .{}, q, 0);
    _ = try fac.makeMotorJoint(.{}, body.id, 3.14);
}

pub fn setup2(solver: *Solver) !void {
    var fac = solver.entityFactory();

    var opt: zigics.EntityFactory.BodyOptions = .{ .pos = .{}, .mass_prop = .{ .density = 5 } };
    opt.mu = 0.3;
    var body: *rigidbody.RigidBody = undefined;

    try fac.makeDownwardsGravity(9.82);

    const MID = Vector2.init(20, 0);

    opt.pos = MID;
    opt.pos.y = -5;
    body = try fac.makeRectangleBody(opt, .{ .width = 1000, .height = 10 });
    body.static = true;

    opt.vel.x = 70;
    opt.vel.y = 10;
    opt.omega = -6;
    opt.mass_prop = .{ .mass = 100 };
    opt.pos = Vector2.init(-40, 10);
    _ = try fac.makeRectangleBody(opt, .{ .width = 8.0, .height = 8.0 });
    opt.vel = .{};
    opt.omega = 0;
    opt.mass_prop = .{ .density = 5 };

    opt.pos = Vector2.init(20, 5);
    body = try fac.makeRectangleBody(opt, .{ .width = 4.0, .height = 1.0 });

    const max: f32 = 100.0;
    const params = ctrs.Constraint.Parameters{ .beta = 100, .upper_lambda = max, .lower_lambda = -max };
    _ = try fac.makeSingleLinkJoint(params, body.id, .{}, nmath.add2(body.props.pos, Vector2.init(0.001, 0)), 0.0);

    for (10..30) |x| {
        const xf = 2.0 * @as(f32, @floatFromInt(x));
        for (10..36) |y| {
            const yf = @as(f32, @floatFromInt(y));
            opt.pos = Vector2.init(xf, yf);

            if (@mod(y, 2) == 0) {
                body = try fac.makeRectangleBody(opt, .{ .width = 1.0, .height = 1.0 });
                if (@mod(x, 2) == 0) {
                    _ = try fac.makeSingleLinkJoint(params, body.id, .{}, nmath.add2(body.props.pos, Vector2.init(0.001, 0)), 0.0);
                }
            } else {
                _ = try fac.makeDiscBody(opt, .{ .radius = 0.5 });
            }
        }
    }
}

pub fn setup1(solver: *Solver) !void {
    var fac = solver.entityFactory();

    var opt: zigics.EntityFactory.BodyOptions = .{ .pos = .{}, .mass_prop = .{ .density = 5 } };
    opt.mu = 0.3;
    var body: *rigidbody.RigidBody = undefined;

    try fac.makeDownwardsGravity(9.82);

    const MID = Vector2.init(20, 0);

    opt.pos = MID;
    opt.pos.y = -5;
    body = try fac.makeRectangleBody(opt, .{ .width = 1000, .height = 10 });
    body.static = true;

    opt.pos = Vector2.init(80, 20);
    body = try fac.makeRectangleBody(opt, .{ .width = 3.0, .height  = 60 });
    body.static = true;

    opt.vel.x = 70;
    opt.vel.y = 30;
    opt.omega = -6;
    opt.mass_prop = .{ .mass = 800 };
    opt.pos = Vector2.init(-70, 10);
    _ = try fac.makeRectangleBody(opt, .{ .width = 8.0, .height = 8.0 });
    opt.vel = .{};
    opt.omega = 0;
    opt.mass_prop = .{ .density = 5 };
    //
    // opt.vel.x = 70;
    // opt.vel.y = 10;
    // opt.omega = -6;
    // opt.mass_prop = .{ .mass = 800 };
    // opt.pos = Vector2.init(-40, 10);
    // _ = try fac.makeRectangleBody(opt, .{ .width = 8.0, .height = 8.0 });
    // opt.vel = .{};
    // opt.omega = 0;
    // opt.mass_prop = .{ .density = 5 };

    for (10..30) |x| {
        const xf = 2.0 * @as(f32, @floatFromInt(x));
        for (1..96) |y| {
            const yf = @as(f32, @floatFromInt(y));

            opt.pos = Vector2.init(xf, yf);

            if (@mod(y, 2) == 0) {
                _ = try fac.makeRectangleBody(opt, .{ .width = 1.0, .height = 1.0 });
            } else {
                _ = try fac.makeDiscBody(opt, .{ .radius = 0.5 });
            }
        }
    }
}
