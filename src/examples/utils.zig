const root = @import("../root.zig");

const nmath = root.nmath;
const Vector2 = nmath.Vector2;

pub fn car(fac: *root.EntityFactory, pos: Vector2) !void {
    var opt: root.EntityFactory.BodyOptions = .{ .pos = .{}, .mass_prop = .{ .density = 1 } };
    opt.mu = 0.4;

    const t_pos = nmath.sub2(pos, Vector2.init(5, 10));

    opt.pos = pos;
    const bh = try fac.makeRectangleBody(opt, .{ .width = 5, .height = 0.9 });

    opt.mu = 1.5;
    const rad: f32 = 1.0;
    opt.pos = nmath.add2(t_pos, Vector2.init(3.5, 9.8 - rad));
    const wl = try fac.makeDiscBody(opt, .{ .radius = rad });
    opt.pos.x = t_pos.x + 6.5;
    const wr = try fac.makeDiscBody(opt, .{ .radius = rad });
    opt.mu = 0.5;

    wr.body_unwrap().props.ang_momentum = -100;

    const dist_car = nmath.dist2(bh.body_unwrap().props.pos, wl.body_unwrap().props.pos);

    opt.pos = nmath.add2(t_pos, Vector2.init(5.25, 10.8));
    const bh2 = try fac.makeRectangleBody(opt, .{ .width = 1.2, .height = 0.5 });

    const power_limit = 2;
    const params = root.Constraint.Parameters{
        .beta = 14,
        .power_min = -power_limit,
        .power_max = power_limit,
    };

    _ = try fac.makeOffsetDistanceJoint(params, wl, bh, .{}, Vector2.init(-1.5, 0), rad + 0.2);
    _ = try fac.makeOffsetDistanceJoint(params, wr, bh, .{}, Vector2.init(1.5, 0), rad + 0.2);
    _ = try fac.makeDistanceJoint(params, wl, wr, 3);

    _ = try fac.makeDistanceJoint(params, wl, bh, dist_car);
    _ = try fac.makeDistanceJoint(params, wr, bh, dist_car);

    try fac.excludeCollisionPair(bh, wl);
    try fac.excludeCollisionPair(bh, wr);
    try fac.excludeCollisionPair(bh, bh2);

    const dist21 = nmath.dist2(nmath.add2(bh.body_unwrap().props.pos, Vector2.init(0.25, 0)), nmath.add2(bh2.body_unwrap().props.pos, Vector2.init(-1, 1)));
    const dist22 = nmath.dist2(nmath.add2(bh.body_unwrap().props.pos, Vector2.init(0.25, 0)), nmath.add2(bh2.body_unwrap().props.pos, Vector2.init(1, 1)));
    _ = try fac.makeOffsetDistanceJoint(.{}, bh, bh2, Vector2.init(0.25, 0), Vector2.init(-1, 1), dist21);
    _ = try fac.makeOffsetDistanceJoint(.{}, bh, bh2, Vector2.init(0.25, 0), Vector2.init(1, 1), dist22);
    const dist23 = nmath.dist2(bh.body_unwrap().props.pos, bh2.body_unwrap().props.pos);
    _ = try fac.makeDistanceJoint(.{ .beta = 100 }, bh, bh2, dist23);
}
