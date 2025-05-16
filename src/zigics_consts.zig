/// The collision-solver (manifold) stops trying to solve the collision when
/// the applied impulse goes below this value
pub const MIN_MANIFOLD_IMPULSE: f32 = 1e-4;
/// The baumgarte factor applied to collisions
pub const BAUMGARTE: f32 = 0.02;
/// The baumgarte slop allowed by the collision-solver
pub const BAUMGARTE_SLOP: f32 = 0.005;
/// When calculating best normals in SAT, the new normal has to be the below
/// value better than the last as to eliminate "false" positives.
pub const SAT_OVERLAP_THRESHOLD: f32 = 1e-4;
pub const COLLISION_MARGIN: f32 = 0.01;
/// When nmath is dividing by something, it warns in the console
/// (std.debug.print) when trying to divide a value smaller than this. This
/// value is primarily used for debugging purposes.
pub const NMATH_WARN_DIVIDING_BELOW: f32 = 1e-3;
/// The padding of intersection that gets applied when checking between aabb
/// intersection.consts
pub const AABB_EPS_OVERLAP: f32 = 0.01;
/// Position correction constant with constraints. Similar to baumgarte in collisions.
pub const CONSTRAINT_POSITION_CORRECTION: f32 = 0.04;
