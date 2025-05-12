const std = @import("std");
const Allocator = std.mem.Allocator;
const nmath = @import("nmath.zig");
const Vector2 = nmath.Vector2;
const rb_mod = @import("rigidbody.zig");
const RigidBody = rb_mod.RigidBody;
const zigics = @import("zigics.zig");
const Solver = zigics.Solver; // What should I do with this?
const demos = @import("demos.zig");

// Yeppers it seems like GPA is not wasm-compatible due to threading?..
// var gpa = std.heap.GeneralPurposeAllocator(.{}){};
// const alloc = gpa.allocator();
const alloc = std.heap.wasm_allocator;

var solver: ?*Solver = null;

// === Solver functions ===
pub export fn solverInit() bool {
    if (solver != null) return true;

    const solver_ptr = alloc.create(Solver) catch return false;
    solver_ptr.* = Solver.init(alloc) catch {
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

// === Setup demos ===

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

pub export fn getRigidImplementation(ptr: usize) u32 {
    const body: *RigidBody = @ptrFromInt(ptr);
    return @intFromPtr(body.ptr);
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

pub export fn getRigidBodyMomemtumX(ptr: usize) f32 {
    const body: *RigidBody = @ptrFromInt(ptr);
    return body.props.momentum.x;
}

pub export fn getRigidBodyMomemtumY(ptr: usize) f32 {
    const body: *RigidBody = @ptrFromInt(ptr);
    return body.props.momentum.y;
}

pub export fn getRigidBodyForceX(ptr: usize) f32 {
    const body: *RigidBody = @ptrFromInt(ptr);
    return body.props.force.x;
}

pub export fn getRigidBodyForceY(ptr: usize) f32 {
    const body: *RigidBody = @ptrFromInt(ptr);
    return body.props.force.y;
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

pub export fn getRigidBodyTorque(ptr: usize) f32 {
    const body: *RigidBody = @ptrFromInt(ptr);
    return body.props.torque;
}

pub export fn getRigidBodyInertia(ptr: usize) f32 {
    const body: *RigidBody = @ptrFromInt(ptr);
    return body.props.inertia;
}

pub export fn getRigidBodyFrictionCoeff(ptr: usize) f32 {
    const body: *RigidBody = @ptrFromInt(ptr);
    return body.props.mu;
}
