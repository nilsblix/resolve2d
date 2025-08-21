const root = @import("../root.zig");

const nmath = root.nmath;
const Vector2 = nmath.Vector2;

pub fn car(fac: *root.EntityFactory, pos: Vector2) !void {
    var opt: root.EntityFactory.BodyOptions = .{ .pos = .{}, .mass_prop = .{ .density = 1 } };
    opt.mu = 0.4;

    const t_pos = nmath.sub2(pos, Vector2.init(5, 10));

    opt.pos = pos;
    const body_ptr = try fac.makeRectangleBody(opt, .{ .width = 5, .height = 0.9 });
    const body = body_ptr.*;

    opt.mu = 1.5;
    const rad: f32 = 1.0;
    opt.pos = nmath.add2(t_pos, Vector2.init(3.5, 9.8 - rad));
    const wheel_l = try fac.makeDiscBody(opt, .{ .radius = rad });
    opt.pos.x = t_pos.x + 6.5;
    const wheel_r = try fac.makeDiscBody(opt, .{ .radius = rad });
    opt.mu = 0.5;

    const dist_car = nmath.dist2(body.props.pos, wheel_l.props.pos);

    opt.pos = nmath.add2(t_pos, Vector2.init(5.25, 10.8));
    const body2 = try fac.makeRectangleBody(opt, .{ .width = 1.2, .height = 0.5 });

    const power_limit = 0.2;
    const params = root.Constraint.Parameters{
        .beta = 10,
        .power_min = -power_limit,
        .power_max = power_limit,
    };

    _ = try fac.makeOffsetDistanceJoint(params, wheel_l.id, body.id, .{}, Vector2.init(-1.5, 0), rad + 0.2);
    _ = try fac.makeOffsetDistanceJoint(params, wheel_r.id, body.id, .{}, Vector2.init(1.5, 0), rad + 0.2);
    _ = try fac.makeDistanceJoint(params, wheel_l.id, wheel_r.id, 3);

    _ = try fac.makeDistanceJoint(params, wheel_l.id, body.id, dist_car);
    _ = try fac.makeDistanceJoint(params, wheel_r.id, body.id, dist_car);

    try fac.excludeCollisionPair(body.id, wheel_l.id);
    try fac.excludeCollisionPair(body.id, wheel_r.id);
    try fac.excludeCollisionPair(body.id, body2.id);

    const dist21 = nmath.dist2(nmath.add2(body.props.pos, Vector2.init(0.25, 0)), nmath.add2(body2.props.pos, Vector2.init(-1, 1)));
    const dist22 = nmath.dist2(nmath.add2(body.props.pos, Vector2.init(0.25, 0)), nmath.add2(body2.props.pos, Vector2.init(1, 1)));
    _ = try fac.makeOffsetDistanceJoint(.{}, body.id, body2.id, Vector2.init(0.25, 0), Vector2.init(-1, 1), dist21);
    _ = try fac.makeOffsetDistanceJoint(.{}, body.id, body2.id, Vector2.init(0.25, 0), Vector2.init(1, 1), dist22);
    const dist23 = nmath.dist2(body.props.pos, body2.props.pos);
    _ = try fac.makeDistanceJoint(.{}, body.id, body2.id, dist23);
}
