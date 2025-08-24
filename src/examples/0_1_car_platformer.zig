const root = @import("../root.zig");

const nmath = root.nmath;
const Vector2 = nmath.Vector2;
const collision = root.collision;
const RigidBody = root.RigidBody;
const Constraint = root.Constraint;
const Solver = root.Solver;

const utils = @import("utils.zig");

pub fn setup(solver: *Solver) !void {
    var fac = solver.entityFactory();

    var opt: root.EntityFactory.BodyOptions = .{ .pos = .{}, .mass_prop = .{ .density = 1 } };
    opt.mu = 0.3;

    try fac.makeDownwardsGravity(9.82);

    try utils.car(&fac, .{});

    // // === PLATFORM 1 ===
    // opt.pos = Vector2.init(0, -100);
    // body = try fac.makeRectangleBody(opt, .{ .width = 1000, .height = 20 });
    // body.static = true;
    //
    // opt.pos = Vector2.init(0, 0);
    // body = try fac.makeRectangleBody(opt, .{ .width = 40, .height = 10 });
    // body.static = true;
    //
    // opt.pos = Vector2.init(-19, 15);
    // body = try fac.makeRectangleBody(opt, .{ .width = 2, .height = 20 });
    // body.static = true;
    //
    // // CAR
    // try utils.car(&fac, Vector2.init(5, 10));
    // opt.mu = 0.5;
    //
    // // Obstacles
    // opt.pos = Vector2.init(-15, 8);
    // var rect_opt = root.EntityFactory.RectangleOptions{
    //     .width = 0.6,
    //     .height = 0.4,
    // };
    // var x: f32 = -15;
    // opt.pos = Vector2.init(x, 9);
    // _ = try fac.makeRectangleBody(opt, rect_opt);
    // opt.pos = Vector2.init(x, 10);
    // _ = try fac.makeRectangleBody(opt, rect_opt);
    // opt.pos = Vector2.init(x, 11);
    // _ = try fac.makeRectangleBody(opt, rect_opt);
    // opt.pos = Vector2.init(x, 12);
    // _ = try fac.makeRectangleBody(opt, rect_opt);
    // opt.pos = Vector2.init(x, 13);
    // _ = try fac.makeRectangleBody(opt, rect_opt);
    // x = -14;
    // opt.pos = Vector2.init(x, 9);
    // _ = try fac.makeRectangleBody(opt, rect_opt);
    // opt.pos = Vector2.init(x, 10);
    // _ = try fac.makeRectangleBody(opt, rect_opt);
    // opt.pos = Vector2.init(x, 11);
    // _ = try fac.makeRectangleBody(opt, rect_opt);
    // opt.pos = Vector2.init(x, 12);
    // _ = try fac.makeRectangleBody(opt, rect_opt);
    // opt.pos = Vector2.init(x, 13);
    // _ = try fac.makeRectangleBody(opt, rect_opt);
    // x = -13;
    // opt.pos = Vector2.init(x, 9);
    // _ = try fac.makeRectangleBody(opt, rect_opt);
    // opt.pos = Vector2.init(x, 10);
    // _ = try fac.makeRectangleBody(opt, rect_opt);
    // opt.pos = Vector2.init(x, 11);
    // _ = try fac.makeRectangleBody(opt, rect_opt);
    // opt.pos = Vector2.init(x, 12);
    // _ = try fac.makeRectangleBody(opt, rect_opt);
    // opt.pos = Vector2.init(x, 13);
    // _ = try fac.makeRectangleBody(opt, rect_opt);
    // x = -12;
    // opt.pos = Vector2.init(x, 9);
    // _ = try fac.makeRectangleBody(opt, rect_opt);
    // opt.pos = Vector2.init(x, 10);
    // _ = try fac.makeRectangleBody(opt, rect_opt);
    // opt.pos = Vector2.init(x, 11);
    // _ = try fac.makeRectangleBody(opt, rect_opt);
    // opt.pos = Vector2.init(x, 12);
    // _ = try fac.makeRectangleBody(opt, rect_opt);
    // opt.pos = Vector2.init(x, 13);
    // _ = try fac.makeRectangleBody(opt, rect_opt);
    //
    // // Rocks
    // opt.pos = Vector2.init(0, 5);
    // opt.angle = 1.0;
    // rect_opt = .{ .width = 0.8, .height = 0.4 };
    // body = try fac.makeRectangleBody(opt, rect_opt);
    // body.static = true;
    //
    // opt.pos = Vector2.init(4.1, 5);
    // opt.angle = 3.0;
    // rect_opt = .{ .width = 0.8, .height = 0.7 };
    // body = try fac.makeRectangleBody(opt, rect_opt);
    // body.static = true;
    //
    // opt.pos = Vector2.init(1, 5);
    // opt.angle = 0.5;
    // rect_opt = .{ .width = 0.8, .height = 0.4 };
    // body = try fac.makeRectangleBody(opt, rect_opt);
    // body.static = true;
    //
    // opt.pos = Vector2.init(15, 5);
    // opt.angle = 3.0;
    // rect_opt = .{ .width = 0.3, .height = 0.4 };
    // body = try fac.makeRectangleBody(opt, rect_opt);
    // body.static = true;
    //
    // opt.pos = Vector2.init(-9, 5);
    // opt.angle = -1.0;
    // rect_opt = .{ .width = 0.9, .height = 0.3 };
    // body = try fac.makeRectangleBody(opt, rect_opt);
    // body.static = true;
    //
    // opt.pos = Vector2.init(-10, 5);
    // opt.angle = -2.0;
    // rect_opt = .{ .width = 0.8, .height = 0.6 };
    // body = try fac.makeRectangleBody(opt, rect_opt);
    // body.static = true;
    //
    // opt.angle = 0;
    //
    // opt.pos = Vector2.init(20, 12);
    // body = try fac.makeRectangleBody(opt, .{ .width = 15, .height = 0.5 });
    // body.static = true;
    //
    // opt.pos = Vector2.init(7, 14);
    // opt.angle = -0.3;
    // body = try fac.makeRectangleBody(opt, .{ .width = 14, .height = 0.5 });
    // body.static = true;
    //
    // // === CONNECTING BRIDGE ===
    // opt.pos = Vector2.init(30, 7);
    // opt.angle = 0.25;
    // body = try fac.makeRectangleBody(opt, .{ .width = 20, .height = 1.0 });
    // body.static = true;
    // opt.angle = 0;
    //
    // // Rotating thing
    // opt.pos = Vector2.init(35, 13);
    // body = try fac.makeRectangleBody(opt, .{ .width = 10.5, .height = 0.5 });
    // _ = try fac.makeFixedPositionJoint(.{}, body.id, body.props.pos);
    //
    // opt.pos = Vector2.init(38, 19);
    // _ = try fac.makeDiscBody(opt, .{ .radius = 1.0 });
    //
    // opt.pos = Vector2.init(48, 9.5);
    // body = try fac.makeRectangleBody(opt, .{ .width = 15, .height = 1 });
    // body.static = true;
    //
    // opt.mass_prop = .{ .mass = 10 };
    // opt.pos = Vector2.init(52, 12);
    // _ = try fac.makeRectangleBody(opt, .{ .width = 3, .height = 3 });
    // opt.mass_prop = .{ .density = 1 };
    //
    // // rect_opt = zigics.EntityFactory.RectangleOptions {
    // //     .width = 0.6,
    // //     .height = 0.4,
    // // };
    // // x = 45;
    // // opt.pos = Vector2.init(x, 13);
    // // _ = try fac.makeRectangleBody(opt, rect_opt);
    // // opt.pos = Vector2.init(x, 14);
    // // _ = try fac.makeRectangleBody(opt, rect_opt);
    // // opt.pos = Vector2.init(x, 15);
    // // _ = try fac.makeRectangleBody(opt, rect_opt);
    // // opt.pos = Vector2.init(x, 16);
    // // _ = try fac.makeRectangleBody(opt, rect_opt);
    // // opt.pos = Vector2.init(x, 17);
    // // _ = try fac.makeRectangleBody(opt, rect_opt);
    // // x = 46;
    // // opt.pos = Vector2.init(x, 13);
    // // _ = try fac.makeRectangleBody(opt, rect_opt);
    // // opt.pos = Vector2.init(x, 14);
    // // _ = try fac.makeRectangleBody(opt, rect_opt);
    // // opt.pos = Vector2.init(x, 15);
    // // _ = try fac.makeRectangleBody(opt, rect_opt);
    // // opt.pos = Vector2.init(x, 16);
    // // _ = try fac.makeRectangleBody(opt, rect_opt);
    // // opt.pos = Vector2.init(x, 17);
    // // _ = try fac.makeRectangleBody(opt, rect_opt);
    // // x = 47;
    // // opt.pos = Vector2.init(x, 13);
    // // _ = try fac.makeRectangleBody(opt, rect_opt);
    // // opt.pos = Vector2.init(x, 14);
    // // _ = try fac.makeRectangleBody(opt, rect_opt);
    // // opt.pos = Vector2.init(x, 15);
    // // _ = try fac.makeRectangleBody(opt, rect_opt);
    // // opt.pos = Vector2.init(x, 16);
    // // _ = try fac.makeRectangleBody(opt, rect_opt);
    // // opt.pos = Vector2.init(x, 17);
    // // _ = try fac.makeRectangleBody(opt, rect_opt);
    // // x = 48;
    // // opt.pos = Vector2.init(x, 13);
    // // _ = try fac.makeRectangleBody(opt, rect_opt);
    // // opt.pos = Vector2.init(x, 14);
    // // _ = try fac.makeRectangleBody(opt, rect_opt);
    // // opt.pos = Vector2.init(x, 15);
    // // _ = try fac.makeRectangleBody(opt, rect_opt);
    // // opt.pos = Vector2.init(x, 16);
    // // _ = try fac.makeRectangleBody(opt, rect_opt);
    // // opt.pos = Vector2.init(x, 17);
    // // _ = try fac.makeRectangleBody(opt, rect_opt);
    //
    // // === CONNECTING THING 2 ===
    //
    // opt.pos = Vector2.init(76.2, 3.3);
    // opt.angle = -0.3;
    // body = try fac.makeRectangleBody(opt, .{ .width = 40, .height = 1.0 });
    // body.static = true;
    // opt.angle = 0;
    //
    // rect_opt = root.EntityFactory.RectangleOptions{
    //     .width = 0.9,
    //     .height = 0.4,
    // };
    // opt.mu = 0.7;
    // x = 65;
    // opt.pos = Vector2.init(x, 9);
    // _ = try fac.makeRectangleBody(opt, rect_opt);
    // opt.pos = Vector2.init(x, 10);
    // _ = try fac.makeRectangleBody(opt, rect_opt);
    // opt.pos = Vector2.init(x, 11);
    // _ = try fac.makeRectangleBody(opt, rect_opt);
    // opt.pos = Vector2.init(x, 12);
    // _ = try fac.makeRectangleBody(opt, rect_opt);
    // opt.pos = Vector2.init(x, 13);
    // _ = try fac.makeRectangleBody(opt, rect_opt);
    // x = 66;
    // opt.pos = Vector2.init(x, 9);
    // _ = try fac.makeRectangleBody(opt, rect_opt);
    // opt.pos = Vector2.init(x, 10);
    // _ = try fac.makeRectangleBody(opt, rect_opt);
    // opt.pos = Vector2.init(x, 11);
    // _ = try fac.makeRectangleBody(opt, rect_opt);
    // opt.pos = Vector2.init(x, 12);
    // _ = try fac.makeRectangleBody(opt, rect_opt);
    // opt.pos = Vector2.init(x, 13);
    // _ = try fac.makeRectangleBody(opt, rect_opt);
    // x = 67;
    // opt.pos = Vector2.init(x, 9);
    // _ = try fac.makeRectangleBody(opt, rect_opt);
    // opt.pos = Vector2.init(x, 10);
    // _ = try fac.makeRectangleBody(opt, rect_opt);
    // opt.pos = Vector2.init(x, 11);
    // _ = try fac.makeRectangleBody(opt, rect_opt);
    // opt.pos = Vector2.init(x, 12);
    // _ = try fac.makeRectangleBody(opt, rect_opt);
    // opt.pos = Vector2.init(x, 13);
    // _ = try fac.makeRectangleBody(opt, rect_opt);
    // x = 68;
    // opt.pos = Vector2.init(x, 9);
    // _ = try fac.makeRectangleBody(opt, rect_opt);
    // opt.pos = Vector2.init(x, 10);
    // _ = try fac.makeRectangleBody(opt, rect_opt);
    // opt.pos = Vector2.init(x, 11);
    // _ = try fac.makeRectangleBody(opt, rect_opt);
    // opt.pos = Vector2.init(x, 12);
    // _ = try fac.makeRectangleBody(opt, rect_opt);
    // opt.pos = Vector2.init(x, 13);
    // _ = try fac.makeRectangleBody(opt, rect_opt);
    // opt.mu = 0.5;
    //
    // // === PLATFORM 3 ===
    // opt.pos = Vector2.init(110, -2);
    // body = try fac.makeRectangleBody(opt, .{ .width = 35, .height = 1.0 });
    // body.static = true;
    //
    // opt.pos = Vector2.init(110, -4);
    // body = try fac.makeDiscBody(opt, .{ .radius = 4.0 });
    // body.static = true;
    //
    // opt.pos = Vector2.init(100, 5);
    // body = try fac.makeRectangleBody(opt, .{ .width = 12.9, .height = 0.5 });
    // // const q = nmath.add2(body.props.pos, Vector2.init(0, 0.01));
    // _ = try fac.makeFixedPositionJoint(.{}, body.id, body.props.pos);
    // const pow = 100;
    // const m2 = Constraint.Parameters{
    //     .beta = 100,
    //     .power_max = pow,
    //     .power_min = -pow,
    // };
    // _ = try fac.makeMotorJoint(m2, body.id, 3.14);
    //
    // opt.pos = Vector2.init(104, 8);
    // _ = try fac.makeDiscBody(opt, .{ .radius = 2.0 });
}
