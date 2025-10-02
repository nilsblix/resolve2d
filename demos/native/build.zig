const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = .Debug;

    const exe = b.addExecutable(.{
        .name = "resolve2d",
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/main.zig"),
            .target = target,
            .optimize = optimize,
        }),
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

    const resolve2d_dep = b.dependency("resolve2d", .{
        .target = target,
        .optimize = optimize,
    });

    const resolve2d_mod = resolve2d_dep.module("resolve2d");

    exe.root_module.addImport("resolve2d", resolve2d_mod);

    b.installArtifact(exe);

    // -------------------------
    // Test step
    // -------------------------
    const unit_tests = b.addTest(.{
        .root_source_file = b.path("src/tests.zig"),
        .target = target,
        .optimize = optimize,
    });
    unit_tests.root_module.addImport("resolve2d", resolve2d_mod);

    const test_step = b.step("test", "Run all tests");
    test_step.dependOn(&b.addRunArtifact(unit_tests).step);
}
