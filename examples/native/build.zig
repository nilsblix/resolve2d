const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = .ReleaseFast;

    const exe = b.addExecutable(.{
        .name = "zigics",
        .root_source_file = b.path("src/main.zig"),
        .target = target,
        .optimize = optimize,
    });

    const raylib_dep = b.dependency("raylib_zig", .{
        .target = target,
        .optimize = optimize,
    });

    const raylib = raylib_dep.module("raylib"); // main raylib module
    const raygui = raylib_dep.module("raygui"); // raygui module
    const raylib_artifact = raylib_dep.artifact("raylib"); // raylib C library

    exe.linkLibrary(raylib_artifact);
    exe.root_module.addImport("raylib", raylib);
    exe.root_module.addImport("raygui", raygui);

    const zigics_dep = b.dependency("zigics", .{
        .target = target,
        .optimize = optimize,
    });
    const zigics_mod = zigics_dep.module("zigics");

    exe.root_module.addImport("zigics", zigics_mod);

    b.installArtifact(exe);
}
