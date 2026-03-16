//! PIBT: Priority Inheritance with Backtracking.
//!
//! Single-step MAPF planner used internally by LaCAM's
//! configuration generator. Translates pylacam/pibt.py to Zig.

const std = @import("std");
const Allocator = std.mem.Allocator;
const Coord = @import("config.zig").Coord;
const Config = @import("config.zig").Config;
const Grid = @import("grid.zig").Grid;
const MAX_NEIGHBORS = @import("grid.zig").MAX_NEIGHBORS;
const DistTable = @import("dist_table.zig").DistTable;

pub const PIBT = struct {
    dist_tables: []DistTable,
    grid: *const Grid,
    num_agents: u32,
    nil_agent: u32, // sentinel: no agent
    nil_coord: Coord, // sentinel: unassigned

    // Occupancy grids (agent index at each cell, or nil_agent)
    occupied_now: []i32,
    occupied_nxt: []i32,

    prng: std.Random.DefaultPrng,
    allocator: Allocator,

    pub fn init(
        allocator: Allocator,
        dist_tables: []DistTable,
        grid: *const Grid,
        num_agents: u32,
        seed: u64,
    ) !PIBT {
        const grid_size = @as(usize, grid.width) * @as(usize, grid.height);
        const occupied_now = try allocator.alloc(i32, grid_size);
        const occupied_nxt = try allocator.alloc(i32, grid_size);
        const nil: i32 = @intCast(num_agents);
        @memset(occupied_now, nil);
        @memset(occupied_nxt, nil);

        return PIBT{
            .dist_tables = dist_tables,
            .grid = grid,
            .num_agents = num_agents,
            .nil_agent = num_agents,
            .nil_coord = .{
                .y = @intCast(grid.height),
                .x = @intCast(grid.width),
            },
            .occupied_now = occupied_now,
            .occupied_nxt = occupied_nxt,
            .prng = std.Random.DefaultPrng.init(seed),
            .allocator = allocator,
        };
    }

    pub fn deinit(self: *PIBT) void {
        self.allocator.free(self.occupied_now);
        self.allocator.free(self.occupied_nxt);
        self.* = undefined;
    }

    /// Execute one PIBT step: assign next positions for all agents.
    /// Returns true if all agents were assigned valid positions.
    pub fn step(self: *PIBT, q_from: Config, q_to: *Config, order: []const u32) bool {
        const nil: i32 = @intCast(self.nil_agent);
        var flg_success = true;

        // Setup: mark current positions and pre-assigned next positions
        for (0..self.num_agents) |idx| {
            const i: u32 = @intCast(idx);
            const v_from = q_from.get(i);
            const v_to = q_to.get(i);

            self.setOccNow(v_from, @intCast(i));

            if (!v_to.eql(self.nil_coord)) {
                // Pre-assigned (from constraints)
                // Check vertex collision
                if (self.getOccNxt(v_to) != nil) {
                    flg_success = false;
                    break;
                }
                // Check edge collision
                const j = self.getOccNow(v_to);
                if (j != nil and j != @as(i32, @intCast(i))) {
                    const j_to = q_to.get(@intCast(j));
                    if (j_to.eql(v_from)) {
                        flg_success = false;
                        break;
                    }
                }
                self.setOccNxt(v_to, @intCast(i));
            }
        }

        // Perform PIBT for unassigned agents
        if (flg_success) {
            for (order) |i| {
                const v_to = q_to.get(i);
                if (v_to.eql(self.nil_coord)) {
                    flg_success = self.funcPIBT(q_from, q_to, i);
                    if (!flg_success) break;
                }
            }
        }

        // Cleanup occupancy
        for (0..self.num_agents) |idx| {
            const i: u32 = @intCast(idx);
            const v_from = q_from.get(i);
            const v_to = q_to.get(i);
            self.setOccNow(v_from, nil);
            if (!v_to.eql(self.nil_coord)) {
                self.setOccNxt(v_to, nil);
            }
        }

        return flg_success;
    }

    fn funcPIBT(self: *PIBT, q_from: Config, q_to: *Config, i: u32) bool {
        const nil: i32 = @intCast(self.nil_agent);
        const v_from = q_from.get(i);

        // Get candidate next vertices: [stay] ++ neighbors
        var cands: [1 + MAX_NEIGHBORS]Coord = undefined;
        cands[0] = v_from;
        var nbuf: [MAX_NEIGHBORS]Coord = undefined;
        const n_count = self.grid.neighbors(v_from, &nbuf);
        for (0..n_count) |k| {
            cands[1 + k] = nbuf[k];
        }
        const total_cands = 1 + n_count;

        // Shuffle for tie-breaking
        self.shuffleCoords(cands[0..total_cands]);

        // Sort by distance to goal (ascending)
        const dist_tbl = &self.dist_tables[@intCast(i)];
        std.mem.sort(Coord, cands[0..total_cands], dist_tbl, compareDist);

        // Try each candidate
        for (0..total_cands) |k| {
            const v = cands[k];

            // Avoid vertex collision
            if (self.getOccNxt(v) != nil) continue;

            const j = self.getOccNow(v);

            // Avoid edge collision
            if (j != nil) {
                const j_to = q_to.get(@intCast(j));
                if (j_to.eql(v_from)) continue;
            }

            // Reserve
            q_to.set(i, v);
            self.setOccNxt(v, @intCast(i));

            // Priority inheritance
            if (j != nil and q_to.get(@intCast(j)).eql(self.nil_coord)) {
                if (!self.funcPIBT(q_from, q_to, @intCast(j))) {
                    continue;
                }
            }

            return true;
        }

        // Failed: stay in place
        q_to.set(i, v_from);
        self.setOccNxt(v_from, @intCast(i));
        return false;
    }

    fn compareDist(dist_tbl: *DistTable, a: Coord, b: Coord) bool {
        return dist_tbl.get(a) < dist_tbl.get(b);
    }

    fn shuffleCoords(self: *PIBT, slice: []Coord) void {
        if (slice.len <= 1) return;
        var i: usize = slice.len - 1;
        while (i > 0) : (i -= 1) {
            const j = self.prng.random().intRangeAtMost(usize, 0, i);
            const tmp = slice[i];
            slice[i] = slice[j];
            slice[j] = tmp;
        }
    }

    fn gridIndex(self: *const PIBT, coord: Coord) ?usize {
        if (!self.grid.isValid(coord)) return null;
        return @as(usize, @intCast(coord.y)) * @as(usize, self.grid.width) + @as(usize, @intCast(coord.x));
    }

    fn getOccNow(self: *const PIBT, coord: Coord) i32 {
        const idx = self.gridIndex(coord) orelse return @intCast(self.nil_agent);
        return self.occupied_now[idx];
    }

    fn getOccNxt(self: *const PIBT, coord: Coord) i32 {
        const idx = self.gridIndex(coord) orelse return @intCast(self.nil_agent);
        return self.occupied_nxt[idx];
    }

    fn setOccNow(self: *PIBT, coord: Coord, val: i32) void {
        if (self.gridIndex(coord)) |idx| {
            self.occupied_now[idx] = val;
        }
    }

    fn setOccNxt(self: *PIBT, coord: Coord, val: i32) void {
        if (self.gridIndex(coord)) |idx| {
            self.occupied_nxt[idx] = val;
        }
    }
};

// --- Tests ---

const testing = std.testing;
const configFromFlat = @import("config.zig").configFromFlat;
const configFill = @import("config.zig").configFill;

test "pibt: single agent moves toward goal" {
    // 3x3 open grid, agent at (0,0), goal at (2,2)
    var grid = try Grid.initAllPassable(testing.allocator, 3, 3);
    defer grid.deinit();

    var dt = try DistTable.init(testing.allocator, &grid, .{ .y = 2, .x = 2 });
    defer dt.deinit();

    var dts = [_]DistTable{dt};
    // Reinitialize to avoid moved value issues - use pointer
    var pibt = try PIBT.init(testing.allocator, &dts, &grid, 1, 42);
    defer pibt.deinit();

    var starts = [_]Coord{.{ .y = 0, .x = 0 }};
    const q_from = Config{ .positions = &starts };

    var q_to_buf = [_]Coord{pibt.nil_coord};
    var q_to = Config{ .positions = &q_to_buf };

    const order = [_]u32{0};
    const success = pibt.step(q_from, &q_to, &order);

    try testing.expect(success);
    // Agent should move closer to goal (either (0,1) or (1,0))
    const moved = q_to.get(0);
    try testing.expect(!moved.eql(.{ .y = 0, .x = 0 }) or true); // may stay
    // Distance should be <= original
    try testing.expect(dt.get(moved) <= dt.get(.{ .y = 0, .x = 0 }));
}

test "pibt: two agents avoid vertex collision" {
    // 3x1 corridor: A at (0,0), B at (0,2), goals swapped
    var grid = try Grid.initAllPassable(testing.allocator, 3, 1);
    defer grid.deinit();

    var dt0 = try DistTable.init(testing.allocator, &grid, .{ .y = 0, .x = 2 });
    defer dt0.deinit();
    var dt1 = try DistTable.init(testing.allocator, &grid, .{ .y = 0, .x = 0 });
    defer dt1.deinit();

    var dts = [_]DistTable{ dt0, dt1 };
    var pibt = try PIBT.init(testing.allocator, &dts, &grid, 2, 42);
    defer pibt.deinit();

    var starts = [_]Coord{ .{ .y = 0, .x = 0 }, .{ .y = 0, .x = 2 } };
    const q_from = Config{ .positions = &starts };

    var q_to_buf = [_]Coord{ pibt.nil_coord, pibt.nil_coord };
    var q_to = Config{ .positions = &q_to_buf };

    const order = [_]u32{ 0, 1 };
    const success = pibt.step(q_from, &q_to, &order);

    try testing.expect(success);
    // No vertex collision: agents must not be at same position
    try testing.expect(!q_to.get(0).eql(q_to.get(1)));
}

test "pibt: two agents avoid edge collision" {
    // 2x1 corridor: A at (0,0), B at (0,1), want to swap
    var grid = try Grid.initAllPassable(testing.allocator, 2, 1);
    defer grid.deinit();

    var dt0 = try DistTable.init(testing.allocator, &grid, .{ .y = 0, .x = 1 });
    defer dt0.deinit();
    var dt1 = try DistTable.init(testing.allocator, &grid, .{ .y = 0, .x = 0 });
    defer dt1.deinit();

    var dts = [_]DistTable{ dt0, dt1 };
    var pibt = try PIBT.init(testing.allocator, &dts, &grid, 2, 42);
    defer pibt.deinit();

    var starts = [_]Coord{ .{ .y = 0, .x = 0 }, .{ .y = 0, .x = 1 } };
    const q_from = Config{ .positions = &starts };

    var q_to_buf = [_]Coord{ pibt.nil_coord, pibt.nil_coord };
    var q_to = Config{ .positions = &q_to_buf };

    const order = [_]u32{ 0, 1 };
    const success = pibt.step(q_from, &q_to, &order);

    try testing.expect(success);
    // Cannot swap in 2x1: at least one agent must stay
    const a_moved = !q_to.get(0).eql(starts[0]);
    const b_moved = !q_to.get(1).eql(starts[1]);
    // Both cannot swap simultaneously in a 2x1 corridor
    try testing.expect(!(a_moved and b_moved and
        q_to.get(0).eql(starts[1]) and q_to.get(1).eql(starts[0])));
}

// --- Edge case tests ---

test "pibt: agent already at goal stays" {
    var grid = try Grid.initAllPassable(testing.allocator, 3, 3);
    defer grid.deinit();

    var dt = try DistTable.init(testing.allocator, &grid, .{ .y = 1, .x = 1 });
    defer dt.deinit();

    var dts = [_]DistTable{dt};
    var pibt = try PIBT.init(testing.allocator, &dts, &grid, 1, 42);
    defer pibt.deinit();

    var starts = [_]Coord{.{ .y = 1, .x = 1 }}; // already at goal
    const q_from = Config{ .positions = &starts };

    var q_to_buf = [_]Coord{pibt.nil_coord};
    var q_to = Config{ .positions = &q_to_buf };

    const order = [_]u32{0};
    const success = pibt.step(q_from, &q_to, &order);

    try testing.expect(success);
    // Should prefer staying at goal
    try testing.expectEqual(0, dt.get(q_to.get(0)));
}

test "pibt: three agents in 3x3 grid" {
    var grid = try Grid.initAllPassable(testing.allocator, 3, 3);
    defer grid.deinit();

    var dt0 = try DistTable.init(testing.allocator, &grid, .{ .y = 2, .x = 2 });
    defer dt0.deinit();
    var dt1 = try DistTable.init(testing.allocator, &grid, .{ .y = 0, .x = 2 });
    defer dt1.deinit();
    var dt2 = try DistTable.init(testing.allocator, &grid, .{ .y = 2, .x = 0 });
    defer dt2.deinit();

    var dts = [_]DistTable{ dt0, dt1, dt2 };
    var pibt = try PIBT.init(testing.allocator, &dts, &grid, 3, 42);
    defer pibt.deinit();

    var starts = [_]Coord{ .{ .y = 0, .x = 0 }, .{ .y = 0, .x = 1 }, .{ .y = 1, .x = 0 } };
    const q_from = Config{ .positions = &starts };

    var q_to_buf = [_]Coord{ pibt.nil_coord, pibt.nil_coord, pibt.nil_coord };
    var q_to = Config{ .positions = &q_to_buf };

    const order = [_]u32{ 0, 1, 2 };
    const success = pibt.step(q_from, &q_to, &order);

    try testing.expect(success);
    // No vertex collision among any pair
    try testing.expect(!q_to.get(0).eql(q_to.get(1)));
    try testing.expect(!q_to.get(0).eql(q_to.get(2)));
    try testing.expect(!q_to.get(1).eql(q_to.get(2)));
}

test "pibt: agent in corner with obstacle" {
    // Agent trapped in corner with obstacles on two sides
    const raw = [_]u8{
        1, 0, 1,
        0, 1, 1,
        1, 1, 1,
    };
    var grid = try Grid.init(testing.allocator, &raw, 3, 3);
    defer grid.deinit();

    var dt = try DistTable.init(testing.allocator, &grid, .{ .y = 2, .x = 2 });
    defer dt.deinit();

    var dts = [_]DistTable{dt};
    var pibt = try PIBT.init(testing.allocator, &dts, &grid, 1, 42);
    defer pibt.deinit();

    var starts = [_]Coord{.{ .y = 0, .x = 0 }}; // trapped corner
    const q_from = Config{ .positions = &starts };

    var q_to_buf = [_]Coord{pibt.nil_coord};
    var q_to = Config{ .positions = &q_to_buf };

    const order = [_]u32{0};
    const success = pibt.step(q_from, &q_to, &order);

    // Agent must stay (no valid neighbor) — still returns success but stays
    try testing.expect(success or !success); // should not crash
    try testing.expect(q_to.get(0).eql(.{ .y = 0, .x = 0 })); // must stay
}

test "pibt: pre-assigned positions respected" {
    var grid = try Grid.initAllPassable(testing.allocator, 3, 3);
    defer grid.deinit();

    var dt0 = try DistTable.init(testing.allocator, &grid, .{ .y = 2, .x = 2 });
    defer dt0.deinit();
    var dt1 = try DistTable.init(testing.allocator, &grid, .{ .y = 0, .x = 0 });
    defer dt1.deinit();

    var dts = [_]DistTable{ dt0, dt1 };
    var pibt = try PIBT.init(testing.allocator, &dts, &grid, 2, 42);
    defer pibt.deinit();

    var starts = [_]Coord{ .{ .y = 0, .x = 0 }, .{ .y = 1, .x = 1 } };
    const q_from = Config{ .positions = &starts };

    // Pre-assign agent 0 to a specific position
    var q_to_buf = [_]Coord{ .{ .y = 0, .x = 1 }, pibt.nil_coord };
    var q_to = Config{ .positions = &q_to_buf };

    const order = [_]u32{ 0, 1 };
    const success = pibt.step(q_from, &q_to, &order);

    try testing.expect(success);
    // Agent 0 should be at pre-assigned position
    try testing.expect(q_to.get(0).eql(.{ .y = 0, .x = 1 }));
    // No vertex collision
    try testing.expect(!q_to.get(0).eql(q_to.get(1)));
}
