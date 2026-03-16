//! Lazy BFS distance table.
//!
//! Computes shortest distances from a goal coordinate on demand.
//! BFS expands only as far as needed to answer get() queries.

const std = @import("std");
const Allocator = std.mem.Allocator;
const Coord = @import("config.zig").Coord;
const Grid = @import("grid.zig").Grid;
const MAX_NEIGHBORS = @import("grid.zig").MAX_NEIGHBORS;
const Deque = @import("deque.zig").Deque;

pub const NIL_DIST: i32 = std.math.maxInt(i32);

pub const DistTable = struct {
    grid: *const Grid,
    goal: Coord,
    table: []i32,
    queue: Deque(Coord),
    allocator: Allocator,

    pub fn init(allocator: Allocator, grid: *const Grid, goal: Coord) !DistTable {
        const size = @as(usize, grid.width) * @as(usize, grid.height);
        const table = try allocator.alloc(i32, size);
        @memset(table, NIL_DIST);

        var dt = DistTable{
            .grid = grid,
            .goal = goal,
            .table = table,
            .queue = Deque(Coord).init(allocator),
            .allocator = allocator,
        };

        // Seed BFS with goal (distance 0)
        if (grid.isPassable(goal)) {
            dt.setDist(goal, 0);
            try dt.queue.pushBack(goal);
        }

        return dt;
    }

    pub fn deinit(self: *DistTable) void {
        self.allocator.free(self.table);
        self.queue.deinit();
        self.* = undefined;
    }

    /// Get distance from `target` to goal. Expands BFS lazily.
    pub fn get(self: *DistTable, target: Coord) i32 {
        if (!self.grid.isPassable(target)) return NIL_DIST;

        // Already computed
        const d = self.getDist(target);
        if (d < NIL_DIST) return d;

        // Lazy BFS expansion
        while (!self.queue.isEmpty()) {
            const u = self.queue.popFront().?;
            const u_dist = self.getDist(u);

            var nbuf: [MAX_NEIGHBORS]Coord = undefined;
            const n_count = self.grid.neighbors(u, &nbuf);

            for (0..n_count) |i| {
                const v = nbuf[i];
                if (u_dist + 1 < self.getDist(v)) {
                    self.setDist(v, u_dist + 1);
                    self.queue.pushBack(v) catch return NIL_DIST;
                }
            }

            if (u.eql(target)) return u_dist;
        }

        return NIL_DIST;
    }

    fn getDist(self: *const DistTable, coord: Coord) i32 {
        if (!self.grid.isValid(coord)) return NIL_DIST;
        return self.table[indexOf(self.grid, coord)];
    }

    fn setDist(self: *DistTable, coord: Coord, dist: i32) void {
        self.table[indexOf(self.grid, coord)] = dist;
    }

    fn indexOf(grid: *const Grid, coord: Coord) usize {
        return @as(usize, @intCast(coord.y)) * @as(usize, grid.width) + @as(usize, @intCast(coord.x));
    }
};

// --- Tests ---

const testing = std.testing;

test "dist_table: distance to self is 0" {
    var g = try Grid.initAllPassable(testing.allocator, 5, 5);
    defer g.deinit();

    const goal = Coord{ .y = 2, .x = 2 };
    var dt = try DistTable.init(testing.allocator, &g, goal);
    defer dt.deinit();

    try testing.expectEqual(0, dt.get(goal));
}

test "dist_table: manhattan distance on open grid" {
    var g = try Grid.initAllPassable(testing.allocator, 5, 5);
    defer g.deinit();

    const goal = Coord{ .y = 0, .x = 0 };
    var dt = try DistTable.init(testing.allocator, &g, goal);
    defer dt.deinit();

    try testing.expectEqual(0, dt.get(.{ .y = 0, .x = 0 }));
    try testing.expectEqual(1, dt.get(.{ .y = 0, .x = 1 }));
    try testing.expectEqual(1, dt.get(.{ .y = 1, .x = 0 }));
    try testing.expectEqual(4, dt.get(.{ .y = 2, .x = 2 }));
    try testing.expectEqual(8, dt.get(.{ .y = 4, .x = 4 }));
}

test "dist_table: obstacle forces detour" {
    // Grid:
    //  . . .
    //  # # .
    //  . . .
    const raw = [_]u8{
        1, 1, 1,
        0, 0, 1,
        1, 1, 1,
    };
    var g = try Grid.init(testing.allocator, &raw, 3, 3);
    defer g.deinit();

    const goal = Coord{ .y = 2, .x = 0 };
    var dt = try DistTable.init(testing.allocator, &g, goal);
    defer dt.deinit();

    // Direct path blocked; must go around: (0,0) -> (0,1) -> (0,2) -> (1,2) -> (2,2) -> (2,1) -> (2,0)
    try testing.expectEqual(6, dt.get(.{ .y = 0, .x = 0 }));
}

test "dist_table: unreachable returns NIL" {
    // Grid:
    //  . #
    //  # .
    const raw = [_]u8{
        1, 0,
        0, 1,
    };
    var g = try Grid.init(testing.allocator, &raw, 2, 2);
    defer g.deinit();

    const goal = Coord{ .y = 0, .x = 0 };
    var dt = try DistTable.init(testing.allocator, &g, goal);
    defer dt.deinit();

    try testing.expectEqual(0, dt.get(.{ .y = 0, .x = 0 }));
    try testing.expectEqual(NIL_DIST, dt.get(.{ .y = 1, .x = 1 })); // unreachable
}

test "dist_table: lazy evaluation (only expands as needed)" {
    var g = try Grid.initAllPassable(testing.allocator, 10, 10);
    defer g.deinit();

    const goal = Coord{ .y = 0, .x = 0 };
    var dt = try DistTable.init(testing.allocator, &g, goal);
    defer dt.deinit();

    // Query a near cell first
    try testing.expectEqual(1, dt.get(.{ .y = 0, .x = 1 }));

    // Queue should still have items (not fully expanded)
    try testing.expect(!dt.queue.isEmpty());

    // Query far cell
    try testing.expectEqual(18, dt.get(.{ .y = 9, .x = 9 }));
}

test "dist_table: obstacle cell returns NIL" {
    const raw = [_]u8{
        1, 0,
        1, 1,
    };
    var g = try Grid.init(testing.allocator, &raw, 2, 2);
    defer g.deinit();

    const goal = Coord{ .y = 0, .x = 0 };
    var dt = try DistTable.init(testing.allocator, &g, goal);
    defer dt.deinit();

    try testing.expectEqual(NIL_DIST, dt.get(.{ .y = 0, .x = 1 })); // obstacle
}

// --- Edge case tests ---

test "dist_table: 1x1 grid goal at only cell" {
    var g = try Grid.initAllPassable(testing.allocator, 1, 1);
    defer g.deinit();

    var dt = try DistTable.init(testing.allocator, &g, .{ .y = 0, .x = 0 });
    defer dt.deinit();

    try testing.expectEqual(0, dt.get(.{ .y = 0, .x = 0 }));
    try testing.expectEqual(NIL_DIST, dt.get(.{ .y = 1, .x = 0 })); // out of bounds
}

test "dist_table: goal on obstacle returns NIL for all" {
    const raw = [_]u8{ 0, 1, 1, 1 };
    var g = try Grid.init(testing.allocator, &raw, 2, 2);
    defer g.deinit();

    var dt = try DistTable.init(testing.allocator, &g, .{ .y = 0, .x = 0 }); // goal is obstacle
    defer dt.deinit();

    try testing.expectEqual(NIL_DIST, dt.get(.{ .y = 0, .x = 0 }));
    try testing.expectEqual(NIL_DIST, dt.get(.{ .y = 0, .x = 1 }));
    try testing.expectEqual(NIL_DIST, dt.get(.{ .y = 1, .x = 0 }));
    try testing.expectEqual(NIL_DIST, dt.get(.{ .y = 1, .x = 1 }));
}

test "dist_table: repeated queries return same result" {
    var g = try Grid.initAllPassable(testing.allocator, 5, 5);
    defer g.deinit();

    var dt = try DistTable.init(testing.allocator, &g, .{ .y = 0, .x = 0 });
    defer dt.deinit();

    const d1 = dt.get(.{ .y = 3, .x = 3 });
    const d2 = dt.get(.{ .y = 3, .x = 3 });
    const d3 = dt.get(.{ .y = 3, .x = 3 });

    try testing.expectEqual(d1, d2);
    try testing.expectEqual(d2, d3);
    try testing.expectEqual(6, d1);
}

test "dist_table: out of bounds coord returns NIL" {
    var g = try Grid.initAllPassable(testing.allocator, 3, 3);
    defer g.deinit();

    var dt = try DistTable.init(testing.allocator, &g, .{ .y = 0, .x = 0 });
    defer dt.deinit();

    try testing.expectEqual(NIL_DIST, dt.get(.{ .y = -1, .x = 0 }));
    try testing.expectEqual(NIL_DIST, dt.get(.{ .y = 0, .x = -1 }));
    try testing.expectEqual(NIL_DIST, dt.get(.{ .y = 3, .x = 0 }));
    try testing.expectEqual(NIL_DIST, dt.get(.{ .y = 0, .x = 3 }));
}

test "dist_table: long corridor distance" {
    // 1x10 corridor
    var g = try Grid.initAllPassable(testing.allocator, 10, 1);
    defer g.deinit();

    var dt = try DistTable.init(testing.allocator, &g, .{ .y = 0, .x = 0 });
    defer dt.deinit();

    try testing.expectEqual(9, dt.get(.{ .y = 0, .x = 9 }));
    try testing.expectEqual(5, dt.get(.{ .y = 0, .x = 5 }));
}
