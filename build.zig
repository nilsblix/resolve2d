const std = @import("std");

pub fn build(b: *std.Build) void {
    // Accept standard target/optimize options when this package is used as a
    // dependency. We don't need to use them here, but registering them
    // prevents `invalid option: -Dcpu/-Dtarget/-Doptimize` errors when
    // dependents pass these through via `b.dependency`.
    _ = b.standardTargetOptions(.{});
    _ = b.standardOptimizeOption(.{});

    _ = b.addModule("zigics", .{
        .root_source_file = b.path("src/root.zig"),
    });

    const target = b.resolveTargetQuery(.{
        .cpu_arch = .wasm32,
        .os_tag = .freestanding,
        .abi = .none,
    });

    const exe = b.addExecutable(.{
        .name = "zigics",
        .root_source_file = b.path("src/wasm_root.zig"),
        .target = target,
        .optimize = .ReleaseFast,
    });
    exe.entry = .disabled;
    exe.rdynamic = true;
    b.installArtifact(exe);
}
