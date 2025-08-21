const std = @import("std");

pub fn build(b: *std.Build) void {
    _ = b.addModule("zigics", .{
        .root_source_file = b.path("src/core/zigics.zig"),
    });

    const target = b.resolveTargetQuery(.{
        .cpu_arch = .wasm32,
        .os_tag = .freestanding,
        .abi = .none,
    });

    const exe = b.addExecutable(.{
        .name = "zigics",
        .root_source_file = b.path("src/wasm_bridge.zig"),
        .target = target,
        .optimize = .ReleaseFast,
    });
    exe.entry = .disabled;
    exe.rdynamic = true;
    b.installArtifact(exe);
}
