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

    var opt: root.EntityFactory.BodyOptions = .{ .pos = .{}, .mass_prop = .{ .density = 5 } };
    opt.mu = 0.3;
    var body: *RigidBody = undefined;

    try fac.makeDownwardsGravity(9.82);

    const MID = Vector2.init(20, 0);

    opt.pos = MID;
    opt.pos.y = -5;
    body = try fac.makeRectangleBody(opt, .{ .width = 1000, .height = 10 });
    body.static = true;

    opt.pos = Vector2.init(80, 20);
    body = try fac.makeRectangleBody(opt, .{ .width = 3.0, .height = 60 });
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
