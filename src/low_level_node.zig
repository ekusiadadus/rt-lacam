//! LowLevelNode: constraint node in LaCAM's lazy DFS.
//!
//! Each LLN specifies partial agent assignments.
//! getChild() creates a deeper constraint by appending one more assignment.

const std = @import("std");
const Allocator = std.mem.Allocator;
const Coord = @import("config.zig").Coord;

pub const LowLevelNode = struct {
    who: []u32,
    where: []Coord,
    depth: u32,
    allocator: Allocator,

    /// Create root LLN (no constraints).
    pub fn initRoot(allocator: Allocator) !*LowLevelNode {
        const self = try allocator.create(LowLevelNode);
        self.* = .{
            .who = &[_]u32{},
            .where = &[_]Coord{},
            .depth = 0,
            .allocator = allocator,
        };
        return self;
    }

    /// Create child LLN by appending agent `who_val` at position `where_val`.
    pub fn getChild(self: *const LowLevelNode, allocator: Allocator, who_val: u32, where_val: Coord) !*LowLevelNode {
        const new_len = self.depth + 1;

        const new_who = try allocator.alloc(u32, new_len);
        if (self.depth > 0) {
            @memcpy(new_who[0..self.depth], self.who[0..self.depth]);
        }
        new_who[self.depth] = who_val;

        const new_where = try allocator.alloc(Coord, new_len);
        if (self.depth > 0) {
            @memcpy(new_where[0..self.depth], self.where[0..self.depth]);
        }
        new_where[self.depth] = where_val;

        const child = try allocator.create(LowLevelNode);
        child.* = .{
            .who = new_who,
            .where = new_where,
            .depth = new_len,
            .allocator = allocator,
        };
        return child;
    }

    pub fn deinit(self: *LowLevelNode) void {
        if (self.depth > 0) {
            self.allocator.free(self.who);
            self.allocator.free(self.where);
        }
        self.allocator.destroy(self);
    }
};

// --- Tests ---

const testing = std.testing;

test "lln: root has depth 0" {
    const root = try LowLevelNode.initRoot(testing.allocator);
    defer root.deinit();

    try testing.expectEqual(0, root.depth);
}

test "lln: getChild increments depth" {
    const root = try LowLevelNode.initRoot(testing.allocator);
    defer root.deinit();

    const child = try root.getChild(testing.allocator, 0, .{ .y = 1, .x = 2 });
    defer child.deinit();

    try testing.expectEqual(1, child.depth);
    try testing.expectEqual(0, child.who[0]);
    try testing.expect(child.where[0].eql(.{ .y = 1, .x = 2 }));

    const grandchild = try child.getChild(testing.allocator, 1, .{ .y = 3, .x = 4 });
    defer grandchild.deinit();

    try testing.expectEqual(2, grandchild.depth);
    try testing.expectEqual(0, grandchild.who[0]);
    try testing.expectEqual(1, grandchild.who[1]);
}

// --- Edge case tests ---

test "lln: deep chain (10 levels)" {
    var nodes: [11]*LowLevelNode = undefined;
    nodes[0] = try LowLevelNode.initRoot(testing.allocator);

    for (1..11) |i| {
        nodes[i] = try nodes[i - 1].getChild(
            testing.allocator,
            @intCast(i),
            .{ .y = @intCast(i), .x = @intCast(i) },
        );
    }

    // Verify chain integrity
    try testing.expectEqual(10, nodes[10].depth);
    try testing.expectEqual(1, nodes[10].who[0]);
    try testing.expectEqual(10, nodes[10].who[9]);
    try testing.expect(nodes[10].where[9].eql(.{ .y = 10, .x = 10 }));

    // Cleanup in reverse order
    var i: usize = 10;
    while (i > 0) : (i -= 1) {
        nodes[i].deinit();
    }
    nodes[0].deinit();
}

test "lln: multiple siblings from same parent" {
    const root = try LowLevelNode.initRoot(testing.allocator);
    defer root.deinit();

    const child_a = try root.getChild(testing.allocator, 0, .{ .y = 0, .x = 1 });
    defer child_a.deinit();
    const child_b = try root.getChild(testing.allocator, 1, .{ .y = 1, .x = 0 });
    defer child_b.deinit();
    const child_c = try root.getChild(testing.allocator, 2, .{ .y = 2, .x = 2 });
    defer child_c.deinit();

    // All siblings have depth 1
    try testing.expectEqual(1, child_a.depth);
    try testing.expectEqual(1, child_b.depth);
    try testing.expectEqual(1, child_c.depth);

    // Each has its own assignment
    try testing.expectEqual(0, child_a.who[0]);
    try testing.expectEqual(1, child_b.who[0]);
    try testing.expectEqual(2, child_c.who[0]);
}

test "lln: child preserves parent constraints" {
    const root = try LowLevelNode.initRoot(testing.allocator);
    defer root.deinit();

    const child = try root.getChild(testing.allocator, 5, .{ .y = 3, .x = 4 });
    defer child.deinit();

    const grandchild = try child.getChild(testing.allocator, 7, .{ .y = 1, .x = 2 });
    defer grandchild.deinit();

    // Grandchild should have both parent's and its own assignment
    try testing.expectEqual(5, grandchild.who[0]);
    try testing.expect(grandchild.where[0].eql(.{ .y = 3, .x = 4 }));
    try testing.expectEqual(7, grandchild.who[1]);
    try testing.expect(grandchild.where[1].eql(.{ .y = 1, .x = 2 }));
}
