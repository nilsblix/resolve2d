const root = @import("../root.zig");

const nmath = root.nmath;
const Vector2 = nmath.Vector2;
const collision = root.collision;
const RigidBody = root.RigidBody;
const Constraint = root.Constraint;
const Solver = root.Solver;

const utils = @import("utils.zig");

pub fn setup(solver: *root.Solver) !void {
    _ = solver;

    // var fac = solver.entityFactory();
    //
    // var opt: root.EntityFactory.BodyOptions = .{ .pos = .{}, .mass_prop = .{ .density = 1 } };
    // opt.mu = 0.4;
    // var body: *RigidBody = undefined;
    //
    // try fac.makeDownwardsGravity(9.82);
    //
    // const rec_opt: root.EntityFactory.RectangleOptions = .{ .width = 0.3, .height = 0.4 };
    // opt.pos = Vector2.init(7, 4);
    // _ = try fac.makeRectangleBody(opt, rec_opt);
    // opt.pos = Vector2.init(2, 4);
    // _ = try fac.makeRectangleBody(opt, rec_opt);
    // opt.pos = Vector2.init(3, 3);
    // _ = try fac.makeRectangleBody(opt, rec_opt);
    //
    // // === CAR ===
    // try utils.car(&fac, Vector2.init(2, 5));
    //
    // // === BRIDGE ===
    // opt = .{ .pos = .{}, .mass_prop = .{ .density = 1 } };
    // opt.mu = 0.4;
    //
    // opt.pos = .{};
    // body = try fac.makeRectangleBody(opt, .{ .width = 2.0, .height = 2.0 });
    // body.static = true;
    //
    // opt.pos.y = 0.75;
    // const width: f32 = 2.0;
    //
    // const power = 0.5;
    // const params = Constraint.Parameters{
    //     .beta = 10,
    //     .power_max = power,
    //     .power_min = -power,
    // };
    //
    // var prev_body = body.*;
    //
    // for (0..20) |idx| {
    //     opt.pos.x = 2.0 + width * @as(f32, @floatFromInt(idx));
    //     body = try fac.makeRectangleBody(opt, .{ .width = width, .height = 0.3 });
    //     if (idx == 0) {
    //         const r1 = Vector2.init(1, 0.75);
    //         const r2 = Vector2.init(-width / 4, 0);
    //         const dist = nmath.dist2(nmath.add2(prev_body.props.pos, r1), nmath.add2(body.props.pos, r2));
    //         _ = try fac.makeOffsetDistanceJoint(params, prev_body.id, body.id, r1, r2, dist);
    //     } else {
    //         const r1 = Vector2.init(width / 4, 0);
    //         const r2 = Vector2.init(-width / 4, 0);
    //         const dist = 0.01 + nmath.dist2(nmath.add2(prev_body.props.pos, r1), nmath.add2(body.props.pos, r2));
    //         _ = try fac.makeOffsetDistanceJoint(params, prev_body.id, body.id, r1, r2, dist);
    //     }
    //     // _ = try fac.excludeCollisionPair(prev_body.id, body.id);
    //     prev_body = body.*;
    // }
    //
    // opt.pos = Vector2.init(1.5 + width * 20, 0);
    // body = try fac.makeRectangleBody(opt, .{ .width = 1.0, .height = 1.5 });
    // body.static = true;
    //
    // const r1 = Vector2.init(width / 4, 0);
    // const r2 = Vector2.init(1, 0.75);
    // const dist = nmath.dist2(nmath.add2(prev_body.props.pos, r1), nmath.add2(body.props.pos, r2));
    // _ = try fac.makeOffsetDistanceJoint(params, prev_body.id, body.id, r1, r2, dist);
}
