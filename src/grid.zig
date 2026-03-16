//! Grid type for MAPF.
//!
//! 2D boolean grid where true = passable, false = obstacle.
//! Provides neighbor enumeration for 4-connected grids.

const std = @import("std");
const Allocator = std.mem.Allocator;
const Coord = @import("config.zig").Coord;

pub const MAX_NEIGHBORS = 4;

pub const Grid = struct {
    data: []bool,
    width: u32,
    height: u32,
    allocator: Allocator,

    /// Initialize grid from a flat u8 array (1 = passable, 0 = obstacle).
    pub fn init(allocator: Allocator, raw: []const u8, width: u32, height: u32) !Grid {
        const size = @as(usize, width) * @as(usize, height);
        const data = try allocator.alloc(bool, size);
        for (0..size) |i| {
            data[i] = raw[i] != 0;
        }
        return Grid{
            .data = data,
            .width = width,
            .height = height,
            .allocator = allocator,
        };
    }

    /// Initialize grid where all cells are passable.
    pub fn initAllPassable(allocator: Allocator, width: u32, height: u32) !Grid {
        const size = @as(usize, width) * @as(usize, height);
        const data = try allocator.alloc(bool, size);
        @memset(data, true);
        return Grid{
            .data = data,
            .width = width,
            .height = height,
            .allocator = allocator,
        };
    }

    pub fn deinit(self: *Grid) void {
        self.allocator.free(self.data);
        self.* = undefined;
    }

    pub fn isValid(self: *const Grid, coord: Coord) bool {
        return coord.y >= 0 and coord.x >= 0 and
            coord.y < @as(i32, @intCast(self.height)) and
            coord.x < @as(i32, @intCast(self.width));
    }

    pub fn isPassable(self: *const Grid, coord: Coord) bool {
        if (!self.isValid(coord)) return false;
        return self.data[self.indexOf(coord)];
    }

    pub fn setPassable(self: *Grid, coord: Coord, passable: bool) void {
        if (self.isValid(coord)) {
            self.data[self.indexOf(coord)] = passable;
        }
    }

    /// Get up to 4 cardinal neighbors (N, S, E, W) that are passable.
    /// Returns count of valid neighbors written to `out`.
    pub fn neighbors(self: *const Grid, coord: Coord, out: *[MAX_NEIGHBORS]Coord) u32 {
        var count: u32 = 0;
        const deltas = [_][2]i32{ .{ -1, 0 }, .{ 1, 0 }, .{ 0, -1 }, .{ 0, 1 } };

        for (deltas) |d| {
            const n = Coord{ .y = coord.y + d[0], .x = coord.x + d[1] };
            if (self.isPassable(n)) {
                out[count] = n;
                count += 1;
            }
        }
        return count;
    }

    fn indexOf(self: *const Grid, coord: Coord) usize {
        return @as(usize, @intCast(coord.y)) * @as(usize, self.width) + @as(usize, @intCast(coord.x));
    }
};

// --- Tests ---

const testing = std.testing;

test "grid: all passable" {
    var g = try Grid.initAllPassable(testing.allocator, 3, 3);
    defer g.deinit();

    try testing.expect(g.isPassable(.{ .y = 0, .x = 0 }));
    try testing.expect(g.isPassable(.{ .y = 2, .x = 2 }));
    try testing.expect(!g.isPassable(.{ .y = -1, .x = 0 }));
    try testing.expect(!g.isPassable(.{ .y = 0, .x = 3 }));
}

test "grid: from raw data with obstacle" {
    const raw = [_]u8{
        1, 1, 1,
        1, 0, 1, // center is obstacle
        1, 1, 1,
    };
    var g = try Grid.init(testing.allocator, &raw, 3, 3);
    defer g.deinit();

    try testing.expect(g.isPassable(.{ .y = 0, .x = 0 }));
    try testing.expect(!g.isPassable(.{ .y = 1, .x = 1 })); // obstacle
    try testing.expect(g.isPassable(.{ .y = 2, .x = 2 }));
}

test "grid: neighbors in open area" {
    var g = try Grid.initAllPassable(testing.allocator, 5, 5);
    defer g.deinit();

    var buf: [MAX_NEIGHBORS]Coord = undefined;

    // Center: 4 neighbors
    const count_center = g.neighbors(.{ .y = 2, .x = 2 }, &buf);
    try testing.expectEqual(4, count_center);

    // Corner: 2 neighbors
    const count_corner = g.neighbors(.{ .y = 0, .x = 0 }, &buf);
    try testing.expectEqual(2, count_corner);

    // Edge: 3 neighbors
    const count_edge = g.neighbors(.{ .y = 0, .x = 1 }, &buf);
    try testing.expectEqual(3, count_edge);
}

test "grid: neighbors with obstacle" {
    const raw = [_]u8{
        1, 0, 1,
        1, 1, 1,
        1, 1, 1,
    };
    var g = try Grid.init(testing.allocator, &raw, 3, 3);
    defer g.deinit();

    var buf: [MAX_NEIGHBORS]Coord = undefined;

    // (0,0) has right blocked by obstacle
    const count = g.neighbors(.{ .y = 0, .x = 0 }, &buf);
    try testing.expectEqual(1, count); // only down
    try testing.expect(buf[0].eql(.{ .y = 1, .x = 0 }));
}

test "grid: setPassable" {
    var g = try Grid.initAllPassable(testing.allocator, 3, 3);
    defer g.deinit();

    try testing.expect(g.isPassable(.{ .y = 1, .x = 1 }));
    g.setPassable(.{ .y = 1, .x = 1 }, false);
    try testing.expect(!g.isPassable(.{ .y = 1, .x = 1 }));
}

// --- Edge case tests ---

test "grid: 1x1 single cell" {
    var g = try Grid.initAllPassable(testing.allocator, 1, 1);
    defer g.deinit();

    try testing.expect(g.isPassable(.{ .y = 0, .x = 0 }));
    try testing.expect(!g.isPassable(.{ .y = 0, .x = 1 }));
    try testing.expect(!g.isPassable(.{ .y = 1, .x = 0 }));
    try testing.expect(!g.isPassable(.{ .y = -1, .x = 0 }));

    var buf: [MAX_NEIGHBORS]Coord = undefined;
    const count = g.neighbors(.{ .y = 0, .x = 0 }, &buf);
    try testing.expectEqual(0, count); // No neighbors in 1x1
}

test "grid: 1xN line grid" {
    var g = try Grid.initAllPassable(testing.allocator, 5, 1);
    defer g.deinit();

    var buf: [MAX_NEIGHBORS]Coord = undefined;

    // Left end: only right neighbor
    const left = g.neighbors(.{ .y = 0, .x = 0 }, &buf);
    try testing.expectEqual(1, left);

    // Middle: two neighbors (left and right)
    const mid = g.neighbors(.{ .y = 0, .x = 2 }, &buf);
    try testing.expectEqual(2, mid);

    // Right end: only left neighbor
    const right = g.neighbors(.{ .y = 0, .x = 4 }, &buf);
    try testing.expectEqual(1, right);
}

test "grid: Nx1 column grid" {
    var g = try Grid.initAllPassable(testing.allocator, 1, 5);
    defer g.deinit();

    var buf: [MAX_NEIGHBORS]Coord = undefined;

    // Top: only down
    const top = g.neighbors(.{ .y = 0, .x = 0 }, &buf);
    try testing.expectEqual(1, top);

    // Middle: up and down
    const mid = g.neighbors(.{ .y = 2, .x = 0 }, &buf);
    try testing.expectEqual(2, mid);
}

test "grid: all obstacles" {
    const raw = [_]u8{ 0, 0, 0, 0 };
    var g = try Grid.init(testing.allocator, &raw, 2, 2);
    defer g.deinit();

    try testing.expect(!g.isPassable(.{ .y = 0, .x = 0 }));
    try testing.expect(!g.isPassable(.{ .y = 1, .x = 1 }));

    var buf: [MAX_NEIGHBORS]Coord = undefined;
    const count = g.neighbors(.{ .y = 0, .x = 0 }, &buf);
    try testing.expectEqual(0, count);
}

test "grid: all four corners of large grid" {
    var g = try Grid.initAllPassable(testing.allocator, 10, 10);
    defer g.deinit();

    var buf: [MAX_NEIGHBORS]Coord = undefined;

    // All corners should have exactly 2 neighbors
    try testing.expectEqual(2, g.neighbors(.{ .y = 0, .x = 0 }, &buf));
    try testing.expectEqual(2, g.neighbors(.{ .y = 0, .x = 9 }, &buf));
    try testing.expectEqual(2, g.neighbors(.{ .y = 9, .x = 0 }, &buf));
    try testing.expectEqual(2, g.neighbors(.{ .y = 9, .x = 9 }, &buf));
}

test "grid: isValid boundary" {
    var g = try Grid.initAllPassable(testing.allocator, 3, 3);
    defer g.deinit();

    // Valid
    try testing.expect(g.isValid(.{ .y = 0, .x = 0 }));
    try testing.expect(g.isValid(.{ .y = 2, .x = 2 }));

    // Invalid: just outside
    try testing.expect(!g.isValid(.{ .y = -1, .x = 0 }));
    try testing.expect(!g.isValid(.{ .y = 0, .x = -1 }));
    try testing.expect(!g.isValid(.{ .y = 3, .x = 0 }));
    try testing.expect(!g.isValid(.{ .y = 0, .x = 3 }));
    try testing.expect(!g.isValid(.{ .y = -1, .x = -1 }));

    // Invalid: far outside
    try testing.expect(!g.isValid(.{ .y = 1000, .x = 0 }));
    try testing.expect(!g.isValid(.{ .y = 0, .x = -1000 }));
}

test "grid: setPassable on invalid coord is no-op" {
    var g = try Grid.initAllPassable(testing.allocator, 2, 2);
    defer g.deinit();

    // Should not crash
    g.setPassable(.{ .y = -1, .x = 0 }, false);
    g.setPassable(.{ .y = 0, .x = 100 }, false);

    // Original cells unchanged
    try testing.expect(g.isPassable(.{ .y = 0, .x = 0 }));
}
