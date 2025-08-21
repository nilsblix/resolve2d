/// spatial_table - incremented array where if C != C + 1 then C is start of body_array and C + 1 is end of body_array
/// body_array - stores ids of bodies where the ones close to eachother are in the same cell
const std = @import("std");
const nmath = @import("nmath.zig");
const Vector2 = nmath.Vector2;
const Allocator = std.mem.Allocator;
const RigidBody = @import("Bodies/RigidBody.zig");
const AABB = @import("aabb.zig").AABB;

// This is a dense implementation of spatial-hashing.
table: std.ArrayList(usize),
// As said in collision.zig, when things aren't persistent, this "unsafe" ptr business is okay.
body_indices: std.ArrayList(*RigidBody),
table_size: usize,
cell_size: f32,

const Self = @This();

pub fn init(alloc: Allocator, cell_size: f32, table_size: usize, bodies: *std.AutoArrayHashMap(RigidBody.Id, RigidBody)) !Self {
    var self = Self{
        .table = try std.ArrayList(usize).initCapacity(alloc, table_size + 1),
        .body_indices = std.ArrayList(*RigidBody).init(alloc),
        .table_size = table_size,
        .cell_size = cell_size,
    };

    try self.table.ensureTotalCapacity(table_size + 1);
    self.table.appendNTimesAssumeCapacity(0, table_size + 1);

    const T = struct {
        pub fn inc(spat: *Self, id: usize, body_ptr: *RigidBody) void {
            _ = body_ptr;
            const val = &spat.table.items[id];
            val.* += 1;
        }
        pub fn putIntoIndices(spat: *Self, id: usize, body_ptr: *RigidBody) void {
            const val = &spat.table.items[id];
            val.* -= 1;
            spat.body_indices.items[val.*] = body_ptr;
        }
    };

    var body_iter = bodies.iterator();

    // Increment each id by 1.
    while (body_iter.next()) |*entry| {
        const body_ptr = entry.value_ptr;
        self.iterateAABBHashes(body_ptr, T.inc);
    }

    // Flood/create partial sums.
    var start: usize = 0;
    for (0..table_size) |id| {
        start += self.table.items[id];
        self.table.items[id] = start;
    }
    self.table.items[table_size] = start;

    // Populate body_indices.
    // POTENTIAL: Before this part, using body_indices will result in a
    // seg-fault as the memory has not been written yet.
    try self.body_indices.ensureTotalCapacity(start);
    self.body_indices.appendNTimesAssumeCapacity(undefined, start);
    body_iter.reset();
    while (body_iter.next()) |*entry| {
        const body_ptr = entry.value_ptr;
        self.iterateAABBHashes(body_ptr, T.putIntoIndices);
    }

    return self;
}

pub fn deinit(self: *Self) void {
    self.table.deinit();
    self.body_indices.deinit();
}

pub fn hash(table_size: usize, xi: i64, yi: i64) usize {
    const h = @as(u64, @bitCast(xi * 92837111 ^ yi * 689287499));
    return @intCast(h % table_size);
}

fn iterateAABBHashes(self: *Self, body_ptr: *RigidBody, onCell: *const fn (spat: *Self, id: usize, body_ptr: *RigidBody) void) void {
    const aabb = body_ptr.aabb;
    const verts = aabb.getVertices();

    const minx = verts[0].x;
    const miny = verts[0].y;

    const maxx = verts[2].x;
    const maxy = verts[2].y;

    const min_xi: i64 = @intFromFloat(@floor(minx / self.cell_size));
    const min_yi: i64 = @intFromFloat(@floor(miny / self.cell_size));
    const max_xi: i64 = @intFromFloat(@floor(maxx / self.cell_size));
    const max_yi: i64 = @intFromFloat(@floor(maxy / self.cell_size));

    var yi: i64 = min_yi;
    while (yi <= max_yi) : (yi += 1) {
        var xi: i64 = min_xi;
        while (xi <= max_xi) : (xi += 1) {
            const id = Self.hash(self.table_size, xi, yi);
            onCell(self, id, body_ptr);
        }
    }
}

pub fn query(self: Self, target: *RigidBody, res: *std.ArrayList(*RigidBody)) !void {
    const aabb = target.aabb;
    const verts = aabb.getVertices();

    const minx = verts[0].x;
    const miny = verts[0].y;

    const maxx = verts[2].x;
    const maxy = verts[2].y;

    const min_xi: i64 = @intFromFloat(@floor(minx / self.cell_size));
    const min_yi: i64 = @intFromFloat(@floor(miny / self.cell_size));
    const max_xi: i64 = @intFromFloat(@floor(maxx / self.cell_size));
    const max_yi: i64 = @intFromFloat(@floor(maxy / self.cell_size));

    var yi: i64 = min_yi;
    while (yi <= max_yi) : (yi += 1) {
        var xi: i64 = min_xi;
        while (xi <= max_xi) : (xi += 1) {
            const id = Self.hash(self.table_size, xi, yi);
            const curr = self.table.items[id];
            const next = self.table.items[id + 1];
            if (curr != next) {
                for (curr..next) |idx| {
                    try res.append(self.body_indices.items[idx]);
                }
            }
        }
    }
}
