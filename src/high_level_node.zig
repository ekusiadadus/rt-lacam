//! HighLevelNode: search node in LaCAM's configuration-space graph.
//!
//! Each HLN holds a complete agent configuration, the low-level
//! constraint DFS tree, cost metrics, and neighbor links.

const std = @import("std");
const Allocator = std.mem.Allocator;
const Coord = @import("config.zig").Coord;
const Config = @import("config.zig").Config;
const Deque = @import("deque.zig").Deque;
const LowLevelNode = @import("low_level_node.zig").LowLevelNode;

pub const HighLevelNode = struct {
    config: Config, // agent positions (owned clone)
    order: []u32, // agent priority ordering
    parent: ?*HighLevelNode,
    tree: Deque(*LowLevelNode), // low-level constraint DFS
    g: i32, // path cost from root
    h: i32, // heuristic (sum of distances to goals)
    f: i32, // g + h
    neighbors: std.AutoArrayHashMap(*HighLevelNode, void),
    allocator: Allocator,

    pub fn init(
        allocator: Allocator,
        cfg: Config,
        order: []u32,
        parent: ?*HighLevelNode,
        g: i32,
        h: i32,
    ) !*HighLevelNode {
        const self = try allocator.create(HighLevelNode);

        // Clone config and order
        const owned_cfg = try cfg.clone(allocator);
        const owned_order = try allocator.alloc(u32, order.len);
        @memcpy(owned_order, order);

        // Initialize tree with root LLN
        var tree = Deque(*LowLevelNode).init(allocator);
        const root_lln = try LowLevelNode.initRoot(allocator);
        try tree.pushBack(root_lln);

        self.* = .{
            .config = owned_cfg,
            .order = owned_order,
            .parent = parent,
            .tree = tree,
            .g = g,
            .h = h,
            .f = g +| h,
            .neighbors = std.AutoArrayHashMap(*HighLevelNode, void).init(allocator),
            .allocator = allocator,
        };
        return self;
    }

    pub fn addNeighbor(self: *HighLevelNode, other: *HighLevelNode) !void {
        try self.neighbors.put(other, {});
    }

    pub fn deinit(self: *HighLevelNode) void {
        self.config.free(self.allocator);
        self.allocator.free(self.order);
        // Drain LLN tree
        while (self.tree.popFront()) |lln| {
            lln.deinit();
        }
        self.tree.deinit();
        self.neighbors.deinit();
        self.allocator.destroy(self);
    }
};

// --- Tests ---

const testing = std.testing;

test "hln: create and destroy" {
    var pos = [_]Coord{ .{ .y = 0, .x = 0 }, .{ .y = 1, .x = 1 } };
    const cfg = Config{ .positions = &pos };
    var order = [_]u32{ 0, 1 };

    const node = try HighLevelNode.init(testing.allocator, cfg, &order, null, 0, 4);
    defer node.deinit();

    try testing.expectEqual(0, node.g);
    try testing.expectEqual(4, node.h);
    try testing.expectEqual(4, node.f);
    try testing.expectEqual(null, node.parent);
    try testing.expect(!node.tree.isEmpty()); // has root LLN
}

test "hln: neighbor tracking" {
    var pos_a = [_]Coord{.{ .y = 0, .x = 0 }};
    var pos_b = [_]Coord{.{ .y = 1, .x = 1 }};
    var order = [_]u32{0};

    const a = try HighLevelNode.init(testing.allocator, .{ .positions = &pos_a }, &order, null, 0, 0);
    defer a.deinit();
    const b = try HighLevelNode.init(testing.allocator, .{ .positions = &pos_b }, &order, a, 1, 1);
    defer b.deinit();

    try a.addNeighbor(b);
    try testing.expectEqual(1, a.neighbors.count());
}

// --- Edge case tests ---

test "hln: saturated f value" {
    var pos = [_]Coord{.{ .y = 0, .x = 0 }};
    const cfg = Config{ .positions = &pos };
    var order = [_]u32{0};

    // g + h would overflow without saturation
    const node = try HighLevelNode.init(
        testing.allocator,
        cfg,
        &order,
        null,
        std.math.maxInt(i32) - 1,
        10,
    );
    defer node.deinit();

    // Should saturate instead of overflow
    try testing.expectEqual(std.math.maxInt(i32), node.f);
}

test "hln: many neighbors" {
    var pos = [_]Coord{.{ .y = 0, .x = 0 }};
    const cfg = Config{ .positions = &pos };
    var order = [_]u32{0};

    const center = try HighLevelNode.init(testing.allocator, cfg, &order, null, 0, 0);
    defer center.deinit();

    // Add 20 neighbors
    var neighbors: [20]*HighLevelNode = undefined;
    for (0..20) |i| {
        var npos = [_]Coord{.{ .y = @intCast(i), .x = 0 }};
        neighbors[i] = try HighLevelNode.init(
            testing.allocator,
            Config{ .positions = &npos },
            &order,
            center,
            @intCast(i),
            0,
        );
        try center.addNeighbor(neighbors[i]);
    }
    defer for (0..20) |i| {
        neighbors[i].deinit();
    };

    try testing.expectEqual(20, center.neighbors.count());
}

test "hln: config and order are deep copies" {
    var pos = [_]Coord{.{ .y = 5, .x = 5 }};
    const cfg = Config{ .positions = &pos };
    var order = [_]u32{0};

    const node = try HighLevelNode.init(testing.allocator, cfg, &order, null, 0, 0);
    defer node.deinit();

    // Mutate originals
    pos[0] = .{ .y = 99, .x = 99 };
    order[0] = 99;

    // Node should be unaffected (deep copy)
    try testing.expect(node.config.get(0).eql(.{ .y = 5, .x = 5 }));
    try testing.expectEqual(0, node.order[0]);
}

test "hln: tree initialized with root LLN" {
    var pos = [_]Coord{.{ .y = 0, .x = 0 }};
    const cfg = Config{ .positions = &pos };
    var order = [_]u32{0};

    const node = try HighLevelNode.init(testing.allocator, cfg, &order, null, 0, 0);
    defer node.deinit();

    // Tree should have exactly one root LLN
    try testing.expect(!node.tree.isEmpty());
    try testing.expectEqual(1, node.tree.len);
}
