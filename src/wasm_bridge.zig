const std = @import("std");
const Allocator = std.mem.Allocator;
const nmath = @import("core/nmath.zig");
const Vector2 = nmath.Vector2;
const rb_mod = @import("core/rigidbody.zig");
const RigidBody = rb_mod.RigidBody;
const zigics = @import("core/zigics.zig");
const Solver = zigics.Solver; // What should I do with this?
const demos = @import("demos.zig");

// Most allocators seem to work. They have to be posix and thread independant though.
var gpa = std.heap.GeneralPurposeAllocator(.{}){};
const alloc = gpa.allocator();
// const alloc = std.heap.wasm_allocator;

var solver: ?*Solver = null;

// === Solver functions ===
pub export fn solverInit(spatialhash_cell_width: f32, spatialhash_table_size_mult: usize) bool {
    if (solver != null) return true;

    const solver_ptr = alloc.create(Solver) catch return false;
    solver_ptr.* = Solver.init(alloc, spatialhash_cell_width, spatialhash_table_size_mult) catch {
        alloc.destroy(solver_ptr);
        return false;
    };

    solver = solver_ptr;

    return true;
}

pub export fn solverDeinit() void {
    var solv = solver orelse return;
    solv.deinit();
    alloc.destroy(solv);
    solver = null;
}

pub export fn solverProcess(dt: f32, sub_steps: usize, collision_iters: usize) bool {
    var solv = solver orelse return false;
    solv.process(alloc, dt, sub_steps, collision_iters) catch return false;
    return true;
}

pub export fn solverGetRigidbodyPtrById(id: u64) *RigidBody {
    var solv = solver orelse unreachable;
    const entry = solv.bodies.getEntry(id) orelse unreachable;
    return entry.value_ptr;
}

pub export fn solverGetNumBodies() usize {
    return solver.?.bodies.count();
}

pub export fn solverGetBodyIdBasedOnIter(iter_idx: usize) u64 {
    const keys = solver.?.bodies.keys();
    return keys[iter_idx];
}

pub export fn solverRemoveBodyById(id: u64) bool {
    solver.?.removeRigidBody(id) catch return false;
    return true;
}

// === Setup demos ===

pub export fn setupBridgeStressTestScene() bool {
    demos.setupBridgeStressTestScene(solver.?) catch return false;
    return true;
}

pub export fn setupCarScene() bool {
    demos.setupCarScene(solver.?) catch return false;
    return true;
}

pub export fn setupDemo1() bool {
    const solv = solver orelse return false;
    demos.setup1(solv) catch return false;
    return true;
}

pub export fn setupDemo2() bool {
    const solv = solver orelse return false;
    demos.setup2(solv) catch return false;
    return true;
}


// === RigidBody Basic Properties ===
pub export fn getRigidBodyPtrFromId(id: u64) usize {
    const entry = solver.?.bodies.getEntry(@as(RigidBody.Id, id));
    return @intFromPtr(entry.?.value_ptr);
}

pub export fn getRigidBodyIdFromPtr(ptr: usize) u64 {
    const body: *RigidBody = @ptrFromInt(ptr);
    return body.id;
}

pub export fn isRigidBodyStatic(ptr: usize) bool {
    const body: *RigidBody = @ptrFromInt(ptr);
    return body.static;
}

pub export fn getRigidBodyNumNormals(ptr: usize) usize {
    const body: *RigidBody = @ptrFromInt(ptr);
    return body.num_normals;
}

/// Returns the integer version of the enum.
pub export fn getRigidBodyType(ptr: usize) usize {
    const body: *RigidBody = @ptrFromInt(ptr);
    return @intFromEnum(body.type);
}

pub export fn getRigidBodyImplementation(ptr: usize) u32 {
    const body: *RigidBody = @ptrFromInt(ptr);
    return @intFromPtr(body.ptr);
}

// === RigidBody AABB things ===
pub export fn getRigidBodyAABBPosX(ptr: usize) f32 {
    const body: *RigidBody = @ptrFromInt(ptr);
    return body.aabb.pos.x;
}

pub export fn getRigidBodyAABBPosY(ptr: usize) f32 {
    const body: *RigidBody = @ptrFromInt(ptr);
    return body.aabb.pos.y;
}

pub export fn getRigidBodyAABBHalfWidth(ptr: usize) f32 {
    const body: *RigidBody = @ptrFromInt(ptr);
    return body.aabb.half_width;
}

pub export fn getRigidBodyAABBHalfHeight(ptr: usize) f32 {
    const body: *RigidBody = @ptrFromInt(ptr);
    return body.aabb.half_height;
}

// === RigidBody Kinematic Properties (used as body.props...) === 
pub export fn getRigidBodyPosX(ptr: usize) f32 {
    const body: *RigidBody = @ptrFromInt(ptr);
    return body.props.pos.x;
}

pub export fn getRigidBodyPosY(ptr: usize) f32 {
    const body: *RigidBody = @ptrFromInt(ptr);
    return body.props.pos.y;
}

pub export fn getRigidBodyMomentumX(ptr: usize) f32 {
    const body: *RigidBody = @ptrFromInt(ptr);
    return body.props.momentum.x;
}

pub export fn setRigidBodyMomentumX(ptr: usize, value: f32) void {
    const body: *RigidBody = @ptrFromInt(ptr);
    body.props.momentum.x = value;
}

pub export fn getRigidBodyMomentumY(ptr: usize) f32 {
    const body: *RigidBody = @ptrFromInt(ptr);
    return body.props.momentum.y;
}

pub export fn setRigidBodyMomentumY(ptr: usize, value: f32) void {
    const body: *RigidBody = @ptrFromInt(ptr);
    body.props.momentum.y = value;
}

pub export fn getRigidBodyAngularVelocity(ptr: usize) f32 {
    const body: *RigidBody = @ptrFromInt(ptr);
    return body.props.ang_momentum / body.props.inertia;
}

pub export fn getRigidBodyForceX(ptr: usize) f32 {
    const body: *RigidBody = @ptrFromInt(ptr);
    return body.props.force.x;
}

pub export fn setRigidBodyForceX(ptr: usize, value: f32) void {
    const body: *RigidBody = @ptrFromInt(ptr);
    body.props.force.x = value;
}

pub export fn getRigidBodyForceY(ptr: usize) f32 {
    const body: *RigidBody = @ptrFromInt(ptr);
    return body.props.force.y;
}

pub export fn setRigidBodyForceY(ptr: usize, value: f32) void {
    const body: *RigidBody = @ptrFromInt(ptr);
    body.props.force.y = value;
}

pub export fn getRigidBodyMass(ptr: usize) f32 {
    const body: *RigidBody = @ptrFromInt(ptr);
    return body.props.mass;
}

pub export fn getRigidBodyAngle(ptr: usize) f32 {
    const body: *RigidBody = @ptrFromInt(ptr);
    return body.props.angle;
}

pub export fn getRigidBodyAngularMomentum(ptr: usize) f32 {
    const body: *RigidBody = @ptrFromInt(ptr);
    return body.props.ang_momentum;
}

pub export fn setRigidBodyAngularMomentum(ptr: usize, value: f32) void {
    const body: *RigidBody = @ptrFromInt(ptr);
    body.props.ang_momentum = value;
}

pub export fn getRigidBodyTorque(ptr: usize) f32 {
    const body: *RigidBody = @ptrFromInt(ptr);
    return body.props.torque;
}

pub export fn setRigidBodyTorque(ptr: usize, value: f32) void {
    const body: *RigidBody = @ptrFromInt(ptr);
    body.props.torque = value;
}

pub export fn getRigidBodyInertia(ptr: usize) f32 {
    const body: *RigidBody = @ptrFromInt(ptr);
    return body.props.inertia;
}

pub export fn getRigidBodyFrictionCoeff(ptr: usize) f32 {
    const body: *RigidBody = @ptrFromInt(ptr);
    return body.props.mu;
}

// === DiscBody specific properties ===
pub export fn getDiscBodyRadiusAssumeType(implementation_ptr: usize) f32 {
    const disc: *rb_mod.DiscBody = @ptrFromInt(implementation_ptr);
    return disc.radius;
}

// === RectangleBody specific properties ===
pub export fn getRectangleBodyWidthAssumeType(implementation_ptr: usize) f32 {
    const rect: *rb_mod.RectangleBody = @ptrFromInt(implementation_ptr);
    return rect.width;
}

pub export fn getRectangleBodyHeightAssumeType(implementation_ptr: usize) f32 {
    const rect: *rb_mod.RectangleBody = @ptrFromInt(implementation_ptr);
    return rect.height;
}
