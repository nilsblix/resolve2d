const std = @import("std");
const zigics = @import("core/zigics.zig");

pub const nmath = @import("core/nmath.zig");
pub const Solver = zigics.Solver;
pub const EntityFactory = zigics.EntityFactory;
pub const RigidBody = @import("core/Bodies/RigidBody.zig");
pub const Constraint = @import("core/Constraints/Constraint.zig");
pub const ForceGen = @import("core/Forces/ForceGen.zig");
pub const collision = @import("core/collision.zig");

const Examples = struct {
    const setup = *const fn(solver: *Solver) anyerror!void;

    setup_0_1_car_platformer: setup,
    setup_0_2_bridge_stress: setup,
    setup_0_3_many_boxes: setup,
    setup_0_4_also_many_boxes: setup,
};

pub const examples = Examples {
    .setup_0_1_car_platformer = @import("examples/0_1_car_platformer.zig").setup,
    .setup_0_2_bridge_stress = @import("examples/0_2_bridge_stress.zig").setup,
    .setup_0_3_many_boxes = @import("examples/0_3_many_boxes.zig").setup,
    .setup_0_4_also_many_boxes = @import("examples/0_4_also_many_boxes.zig").setup,
};
