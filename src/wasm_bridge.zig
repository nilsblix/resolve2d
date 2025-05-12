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

pub export fn getRigidBodyPosX(ptr: usize) f32 {
    const body: *RigidBody = @ptrFromInt(ptr);
    return body.props.pos.x;
}

// pub export fn getRigidBodyId(ptr: *RigidBody) u64 {
//     return ptr.id;
// }
//
// pub export fn isRigidBodyStatic(ptr: *RigidBody) bool {
//     return ptr.static;
// }
//
// pub export fn getRigidBodyNumNormals(ptr: *RigidBody) usize {
//     return ptr.num_normals;
// }

// === RigidBody Dynamic Accessors ===
// pub export fn getRigidBodyPosX(ptr: *RigidBody) f32 {
//     return ptr.props.pos.x;
// }

// pub export fn getRigidBodyPosY(ptr: *RigidBody) f32 {
//     return ptr.props.pos.y;
// }
//
// pub export fn getRigidBodyMomentumX(ptr: *RigidBody) f32 {
//     return ptr.props.momentum.x;
// }
//
// pub export fn getRigidBodyMomentumY(ptr: *RigidBody) f32 {
//     return ptr.props.momentum.y;
// }
//
// pub export fn getRigidBodyForceX(ptr: *RigidBody) f32 {
//     return ptr.props.force.x;
// }
//
// pub export fn getRigidBodyForceY(ptr: *RigidBody) f32 {
//     return ptr.props.force.y;
// }
//
// pub export fn getRigidBodyMass(ptr: *RigidBody) f32 {
//     return ptr.props.mass;
// }
//
// pub export fn getRigidBodyAngle(ptr: *RigidBody) f32 {
//     return ptr.props.angle;
// }
//
// pub export fn getRigidBodyAngularMomentum(ptr: *RigidBody) f32 {
//     return ptr.props.ang_momentum;
// }
//
// pub export fn getRigidBodyTorque(ptr: *RigidBody) f32 {
//     return ptr.props.torque;
// }
//
// pub export fn getRigidBodyInertia(ptr: *RigidBody) f32 {
//     return ptr.props.inertia;
// }
//
// pub export fn getRigidBodyMu(ptr: *RigidBody) f32 {
//     return ptr.props.mu;
// }
//
// // === RigidBody Other Properties ===
// pub export fn getRigidBodyType(ptr: *RigidBody) i32 {
//     return @intFromEnum(ptr.type);
// }
//
// pub export fn getRigidBodyImplementation(ptr: *RigidBody) *anyopaque {
//     return ptr.ptr;
// }
