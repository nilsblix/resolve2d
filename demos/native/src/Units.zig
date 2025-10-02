const std = @import("std");
const r2d = @import("resolve2d");
const nmath = r2d.nmath;
const Vector2 = nmath.Vector2;

pub const Size = struct {
    width: f32,
    height: f32,

    pub fn toVector2(self: Size) Vector2 {
        return Vector2{ .x = self.width, .y = self.height };
    }

    pub fn fromVector2(vec: Size) Size {
        return Size{ .width = vec.x, .heigth = vec.y };
    }
};

pub const TransformMult = struct {
    w2s: f32,
    s2w: f32,
};

/// pos is bottom left of the viewport
pub const Camera = struct {
    pos: Vector2,
    zoom: f32,
    viewport: Size,
};

camera: Camera,
mult: TransformMult,
default_world_size: Size,
screen_size: Size,

const Units = @This();
/// Assumes that screen_size and world_size have the same proportions.
/// Assumes that in screen units top left is (0,0).
pub fn init(screen_size: Size, default_world_width: f32) Units {
    const aspect_ratio = screen_size.height / screen_size.width;

    const default_world_size = Size{ .width = default_world_width, .height = default_world_width * aspect_ratio };

    const camera = Camera{
        .pos = Vector2.zero,
        .zoom = 1.0,
        .viewport = default_world_size,
    };

    const mult = TransformMult{
        .w2s = screen_size.width / default_world_width,
        .s2w = default_world_width / screen_size.width,
    };

    return .{
        .camera = camera,
        .mult = mult,
        .default_world_size = default_world_size,
        .screen_size = screen_size,
    };
}

pub fn updateDimensions(self: *Units, new_screen_size: Vector2) void {
    const current_size = Vector2.init(self.screen_size.width, self.screen_size.height);
    const middle_world = self.s2w(nmath.scale2(current_size, 0.5));

    const prev_units = self.*;

    const size = Size{ .width = new_screen_size.x, .height = new_screen_size.y };
    self.* = Units.init(size, prev_units.camera.viewport.width);
    self.camera.zoom = prev_units.camera.zoom;

    // Whats the new middle of the world? Get the delta, and move there.
    const new_middle_world = self.s2w(nmath.scale2(new_screen_size, 0.5));
    const delta = nmath.sub2(middle_world, new_middle_world);
    self.camera.pos.add(delta);
}

fn updateViewPort(self: *Units) void {
    const dims = Vector2.init(self.screen_size.width, self.screen_size.height);
    const world_view = nmath.scale2(dims, self.mult.s2w);
    self.camera.viewport = Size{ .width = world_view.x, .height = world_view.y };
}

pub fn adjustCameraZoom(self: *Units, factor: f32, screen_pos: Vector2) void {
    const old_world_pos = self.s2w(screen_pos);

    self.camera.zoom /= factor;

    self.mult.w2s = (self.screen_size.width / self.default_world_size.width) / self.camera.zoom;
    self.mult.s2w = (self.default_world_size.width / self.screen_size.width) * self.camera.zoom;

    const new_world_pos = self.s2w(screen_pos);
    const delta_world = nmath.sub2(new_world_pos, old_world_pos);
    self.camera.pos.sub(delta_world);

    self.updateViewPort();
}

pub fn adjustCameraPos(self: *Units, delta: Vector2) void {
    // self.camera.pos.sub(nmath.scale2(delta, self.camera.zoom));
    self.camera.pos.sub(delta);
}

/// Will flip y as world bottom-left is (0,0)
pub fn w2s(self: Units, pos: Vector2) Vector2 {
    const x = self.mult.w2s * (pos.x - self.camera.pos.x);
    const y = self.screen_size.height - self.mult.w2s * (pos.y - self.camera.pos.y);

    return Vector2.init(x, y);
}

/// Will flip y as world bottom-left is (0,0)
pub fn s2w(self: Units, pos: Vector2) Vector2 {
    const x = pos.x * self.mult.s2w + self.camera.pos.x;
    const y = (self.screen_size.height - pos.y) * self.mult.s2w + self.camera.pos.y;
    return Vector2.init(x, y);
}

/// Max x from [oldmin -> oldmax] ==> [newmin -> newmax]
pub fn map(x: f32, omin: f32, omax: f32, nmin: f32, nmax: f32) f32 {
    return nmin + ((x - omin) * (nmax - nmin)) / (omax - omin);
}

pub fn map2(comptime T: type, x: T, omin: T, omax: T, nmin: T, nmax: T) T {
    return nmin + ((x - omin) * (nmax - nmin)) / (omax - omin);
}
