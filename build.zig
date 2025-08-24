const std = @import("std");

pub fn build(b: *std.Build) void {
    _ = b.addModule("zigics", .{
        .root_source_file = b.path("src/root.zig"),
    });

    // FIXME: Uncomment.
    // const target = b.resolveTargetQuery(.{
    //     .cpu_arch = .wasm32,
    //     .os_tag = .freestanding,
    //     .abi = .none,
    // });
    //
    // const exe = b.addExecutable(.{
    //     .name = "zigics",
    //     .root_source_file = b.path("src/wasm_root.zig"),
    //     .target = target,
    //     .optimize = .ReleaseFast,
    // });
    // exe.entry = .disabled;
    // exe.rdynamic = true;
    // b.installArtifact(exe);
}
