const std = @import("std");

pub fn build(b: *std.Build) void {
    const platform = b.option([]const u8, "platform", "Target platform: 'native' or 'wasm'") orelse "native";
    const optimize = b.standardOptimizeOption(.{});

    var target: std.Build.ResolvedTarget = undefined;
    const is_wasm = std.mem.eql(u8, platform, "wasm");

    if (is_wasm) {
        target = b.resolveTargetQuery(.{
            .cpu_arch = .wasm32,
            .os_tag = .freestanding,
            .abi = .none,
        });
    } else {
        target = b.standardTargetOptions(.{});
    }

    const exe = b.addExecutable(.{
        .name = "zigics",
        .root_source_file = if (is_wasm)
            b.path("src/wasm_bridge.zig")
        else
            b.path("src/demos/native/main.zig"),
        .target = target,
        .optimize = optimize,
    });

    const zigics_mod = b.addModule("zigics", .{
        .root_source_file = b.path("src/core/zigics.zig"),
    });

    exe.root_module.addImport("zigics", zigics_mod);

    if (is_wasm) {
        exe.entry = .disabled;
        exe.rdynamic = true;
    } else {
        const raylib_dep = b.dependency("raylib_zig", .{
            .target = target,
            .optimize = optimize,
        });
        const raylib = raylib_dep.module("raylib");
        const raygui = raylib_dep.module("raygui"); // raygui module
        const raylib_artifact = raylib_dep.artifact("raylib"); // raylib C library

        exe.linkLibrary(raylib_artifact);
        exe.root_module.addImport("raylib", raylib);
        exe.root_module.addImport("raygui", raygui);
    }

    b.installArtifact(exe);

    if (!is_wasm) { // And is therefore native.
        const run_cmd = b.addRunArtifact(exe);
        run_cmd.step.dependOn(b.getInstallStep());

        if (b.args) |args| {
            run_cmd.addArgs(args);
        }

        const run_step = b.step("run", "Run the app");
        run_step.dependOn(&run_cmd.step);
    }
}
