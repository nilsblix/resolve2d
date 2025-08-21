# Native zigics example

This example shows how to integrate the `zigics` physics package in a native
Zig application alongside the [raylib](https://www.raylib.com/) bindings.

## Building

Ensure `zig` is installed on your system. From this directory run:

```
zig build run
```

## Using zigics in your own project

1. Add `zigics` to `build.zig.zon`:

```zig
.dependencies = .{
    .zigics = .{ .path = "../.." },
};
```

2. In `build.zig` obtain the module from the dependency and add it to your
   executable:

```zig
const zigics_dep = b.dependency("zigics", .{ .target = target, .optimize = optimize });
const zigics_mod = zigics_dep.module("zigics");
exe.root_module.addImport("zigics", zigics_mod);
```

3. Import `zigics` in your source files:

```zig
const zigics = @import("zigics");
```

See `src/main.zig` for a minimal example that verifies the package linkage.
