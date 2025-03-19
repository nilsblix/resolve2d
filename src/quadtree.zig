const std = @import("std");
const Allocator = std.mem.Allocator;
const nmath = @import("nmath.zig");
const Vector2 = nmath.Vector2;
const rb_mod = @import("rigidbody.zig");
const RigidBody = rb_mod.RigidBody;
const AABB = @import("aabb.zig").AABB;

fn getEntireAABB(bodies: []RigidBody) AABB {
    var max = Vector2.init(-std.math.inf(f32), -std.math.inf(f32));
    var min = Vector2.init(std.math.inf(f32), std.math.inf(f32));

    for (bodies) |body| {
        const a = body.aabb;
        if (a.pos.x + a.half_width > max.x) max.x = a.pos.x + a.half_width;
        if (a.pos.x - a.half_width < min.x) min.x = a.pos.x - a.half_width;
        if (a.pos.y + a.half_height > max.y) max.y = a.pos.y + a.half_height;
        if (a.pos.y - a.half_height < min.y) min.y = a.pos.y - a.half_height;
    }

    const pos = Vector2.init((max.x + min.x) / 2, (max.y + min.y) / 2);

    const w2 = (max.x - min.x) / 2;
    const h2 = (max.y - min.y) / 2;

    return AABB{ .pos = pos, .half_width = w2, .half_height = h2 };
}

pub const Quadrant = enum {
    bottom_left,
    bottom_right,
    top_right,
    top_left,
};

pub fn computeNextAABB(aabb: AABB, quadrant: Quadrant) AABB {
    const half_dims = nmath.scale2(Vector2.init(aabb.half_width, aabb.half_height), 0.5);
    const a = aabb.pos;

    const pos = switch (quadrant) {
        .bottom_left => Vector2.init(a.x - half_dims.x, a.y - half_dims.y),
        .bottom_right => Vector2.init(a.x + half_dims.x, a.y - half_dims.y),
        .top_right => Vector2.init(a.x + half_dims.x, a.y + half_dims.y),
        .top_left => Vector2.init(a.x - half_dims.x, a.y + half_dims.y),
    };

    return AABB{ .pos = pos, .half_width = aabb.half_width / 2, .half_height = aabb.half_height / 2 };
}

pub const QuadTree = struct {
    pub const Node = struct {
        children: [4]?*Node = .{ null, null, null, null },
        values: std.ArrayListUnmanaged(*RigidBody),
        // values: std.ArrayList(*RigidBody),
        aabb: AABB,

        pub fn init(alloc: Allocator, aabb: AABB, threshold: usize) !Node {
            return Node{
                .values = try std.ArrayListUnmanaged(*RigidBody).initCapacity(alloc, threshold),
                // .values = std.ArrayList(*RigidBody).init(alloc),
                .aabb = aabb,
            };
        }

        pub fn deinit(self: *Node, alloc: Allocator) void {
            self.values.deinit(alloc);
            for (self.children) |child| {
                if (child) |node| {
                    node.deinit(alloc);
                    alloc.destroy(node);
                }
            }
        }

        pub fn isLeaf(self: *Node) bool {
            return self.children[0] == null;
        }

        pub fn getChild(self: *Node, quad: Quadrant) ?*Node {
            return switch (quad) {
                .bottom_left => self.children[0],
                .bottom_right => self.children[1],
                .top_right => self.children[2],
                .top_left => self.children[3],
            };
        }

        pub fn append(node: *Node, alloc: Allocator, value: *RigidBody, depth: usize, threshold: usize, max_depth: usize) anyerror!void {
            if (node.isLeaf()) {
                if (depth >= max_depth or node.values.items.len < threshold) {
                    try node.values.append(alloc, value);
                } else {
                    // we split this thing
                    try node.split(alloc, depth, threshold, max_depth);
                    try node.append(alloc, value, depth, threshold, max_depth);
                }
            } else {
                for (node.children) |child| {
                    if (child) |kid| {
                        if (kid.aabb.intersects(value.aabb)) {
                            try kid.append(alloc, value, depth + 1, threshold, max_depth);
                        }
                    }
                }
            }
        }

        pub fn split(node: *Node, alloc: Allocator, depth: usize, threshold: usize, max_depth: usize) !void {
            const aabb = node.aabb;

            for (0..4) |q| {
                if (node.children[q]) |kid| {
                    std.debug.print("should never be here. deinit while split", .{});
                    kid.deinit(alloc);
                    alloc.destroy(kid);
                }
                node.children[q] = try alloc.create(Node);
                node.children[q].?.* = try Node.init(alloc, computeNextAABB(aabb, @enumFromInt(q)), threshold);
            }

            for (node.values.items) |value| {
                for (node.children) |child| {
                    if (child) |kid| {
                        if (kid.aabb.intersects(value.aabb)) {
                            try kid.append(alloc, value, depth + 1, threshold, max_depth);
                        }
                    }
                }
            }
        }

        pub fn queryAABB(node: *Node, query_aabb: AABB, results: *std.ArrayList(*RigidBody)) !void {
            if (!node.aabb.intersects(query_aabb)) return;

            if (node.isLeaf()) {
                for (node.values.items) |value| {
                    try results.append(value);
                }
                return;
            }

            for (node.children) |child| {
                if (child) |kid| {
                    try kid.queryAABB(query_aabb, results);
                }
            }
        }
    };

    root: Node = undefined,
    node_threshold: usize = 1,
    max_depth: usize = 0,

    const Self = @This();
    pub fn init(alloc: Allocator, comptime node_threshold: usize, comptime max_depth: usize) !QuadTree {
        var qtree = QuadTree{};
        qtree.node_threshold = node_threshold;
        qtree.max_depth = max_depth;

        const aabb = AABB{ .pos = .{}, .half_height = 0.0, .half_width = 0.0 };
        qtree.root = try Node.init(alloc, aabb, node_threshold);

        return qtree;
    }

    pub fn deinit(self: *Self, alloc: Allocator) void {
        self.root.deinit(alloc);
    }

    pub fn queryAABB(self: *Self, query_aabb: AABB, results: *std.ArrayList(*RigidBody)) !void {
        try self.root.queryAABB(query_aabb, results);
    }

    pub fn insertValues(self: *Self, alloc: Allocator, bodies: []RigidBody) !void {
        const aabb = getEntireAABB(bodies);
        self.root.aabb = aabb;

        for (bodies) |*body| {
            try self.root.append(alloc, body, 0, self.node_threshold, self.max_depth);
        }
    }

    pub fn clear(self: *Self, alloc: Allocator) void {
        self.root.deinit(alloc);
        self.root = Node.init(alloc, AABB{ .pos = .{}, .half_width = 0.0, .half_height = 0.0 }, self.node_threshold) catch unreachable;
    }

    pub fn initRoot(self: *Self, alloc: Allocator) void {
        self.root = Node.init(alloc, AABB{ .pos = .{}, .half_width = 0.0, .half_height = 0.0 }, self.node_threshold) catch unreachable;
    }

    pub fn calculateNumNodes(self: *Self) usize {
        const T = struct {
            pub fn count(node: Node) usize {
                if (node.children[0] == null) {
                    return 1;
                } else {
                    var num: usize = 0;
                    for (node.children) |child| {
                        if (child) |kid| {
                            num += count(kid.*);
                        }
                    }
                    return num;
                }
                return 0;
            }
        };

        return T.count(self.root);
    }
};
