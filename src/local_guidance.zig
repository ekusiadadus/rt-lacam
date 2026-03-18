//! Local Guidance for configuration-based MAPF.
//!
//! Computes per-agent directional hints that bias PIBT's action
//! selection. Resolves conflicts between agents' preferred next
//! steps using sequential reservation.
//!
//! Simplified implementation of: Arita & Okumura, "Local Guidance
//! for Configuration-Based Multi-Agent Pathfinding"
//! (arXiv:2510.19072, 2025)

const std = @import("std");
const Allocator = std.mem.Allocator;
const Coord = @import("config.zig").Coord;
const Config = @import("config.zig").Config;
const Grid = @import("grid.zig").Grid;
const MAX_NEIGHBORS = @import("grid.zig").MAX_NEIGHBORS;
const DistTable = @import("dist_table.zig").DistTable;

pub const LocalGuidance = struct {
    /// Per-agent guidance hint: preferred next coordinate.
    hints: []?Coord,
    num_agents: u32,
    /// Cell reservation bitmap for conflict resolution.
    reserved: []bool,
    width: u32,
    height: u32,
    allocator: Allocator,

    pub fn init(allocator: Allocator, num_agents: u32, width: u32, height: u32) !LocalGuidance {
        const hints = try allocator.alloc(?Coord, num_agents);
        @memset(hints, null);
        const grid_size = @as(usize, width) * @as(usize, height);
        const reserved = try allocator.alloc(bool, grid_size);
        @memset(reserved, false);
        return LocalGuidance{
            .hints = hints,
            .num_agents = num_agents,
            .reserved = reserved,
            .width = width,
            .height = height,
            .allocator = allocator,
        };
    }

    pub fn deinit(self: *LocalGuidance) void {
        self.allocator.free(self.hints);
        self.allocator.free(self.reserved);
        self.* = undefined;
    }

    /// Compute guidance hints for all agents.
    ///
    /// Plans agents in priority order (farthest from goal first).
    /// Each agent gets the best unreserved neighbor that reduces
    /// distance to its goal. Reserved cells are skipped to avoid
    /// directing multiple agents to the same cell.
    pub fn compute(
        self: *LocalGuidance,
        config: Config,
        order: []const u32,
        dist_tables: []DistTable,
        grid: *const Grid,
    ) void {
        @memset(self.hints, null);
        @memset(self.reserved, false);

        // Reserve current positions
        for (0..self.num_agents) |idx| {
            const pos = config.get(@intCast(idx));
            self.setReserved(pos, true);
        }

        // Plan in priority order
        for (order) |i| {
            const current = config.get(i);
            const dist_tbl = &dist_tables[i];
            const current_dist = dist_tbl.get(current);
            if (current_dist == 0) continue; // already at goal

            // Find best unreserved neighbor that reduces distance
            var best: ?Coord = null;
            var best_dist: i32 = current_dist;
            var nbuf: [MAX_NEIGHBORS]Coord = undefined;
            const n_count = grid.neighbors(current, &nbuf);

            for (0..n_count) |k| {
                const v = nbuf[k];
                const d = dist_tbl.get(v);
                if (d < best_dist and !self.isReserved(v)) {
                    best_dist = d;
                    best = v;
                }
            }

            // Fallback: best neighbor even if reserved
            if (best == null) {
                best_dist = current_dist;
                for (0..n_count) |k| {
                    const v = nbuf[k];
                    const d = dist_tbl.get(v);
                    if (d < best_dist) {
                        best_dist = d;
                        best = v;
                    }
                }
            }

            if (best) |next| {
                self.hints[i] = next;
                self.setReserved(current, false);
                self.setReserved(next, true);
            }
        }
    }

    fn gridIdx(self: *const LocalGuidance, coord: Coord) ?usize {
        if (coord.y < 0 or coord.x < 0) return null;
        if (coord.y >= @as(i32, @intCast(self.height))) return null;
        if (coord.x >= @as(i32, @intCast(self.width))) return null;
        return @as(usize, @intCast(coord.y)) * @as(usize, self.width) +
            @as(usize, @intCast(coord.x));
    }

    fn isReserved(self: *const LocalGuidance, coord: Coord) bool {
        const idx = self.gridIdx(coord) orelse return true;
        return self.reserved[idx];
    }

    fn setReserved(self: *LocalGuidance, coord: Coord, val: bool) void {
        if (self.gridIdx(coord)) |idx| {
            self.reserved[idx] = val;
        }
    }
};

// --- Tests ---

const testing = std.testing;

test "local_guidance: init and deinit" {
    var lg = try LocalGuidance.init(testing.allocator, 3, 5, 5);
    defer lg.deinit();
    try testing.expectEqual(null, lg.hints[0]);
    try testing.expectEqual(null, lg.hints[1]);
    try testing.expectEqual(null, lg.hints[2]);
}

test "local_guidance: single agent gets hint toward goal" {
    var grid = try Grid.initAllPassable(testing.allocator, 5, 5);
    defer grid.deinit();

    var dt = try DistTable.init(testing.allocator, &grid, .{ .y = 4, .x = 4 });
    defer dt.deinit();
    var dts = [_]DistTable{dt};

    var lg = try LocalGuidance.init(testing.allocator, 1, 5, 5);
    defer lg.deinit();

    var pos = [_]Coord{.{ .y = 0, .x = 0 }};
    const config = Config{ .positions = &pos };
    const order = [_]u32{0};

    lg.compute(config, &order, &dts, &grid);

    try testing.expect(lg.hints[0] != null);
    const hint = lg.hints[0].?;
    try testing.expect(dt.get(hint) < dt.get(.{ .y = 0, .x = 0 }));
}

test "local_guidance: conflict resolution avoids same cell" {
    var grid = try Grid.initAllPassable(testing.allocator, 3, 3);
    defer grid.deinit();

    var dt0 = try DistTable.init(testing.allocator, &grid, .{ .y = 2, .x = 2 });
    defer dt0.deinit();
    var dt1 = try DistTable.init(testing.allocator, &grid, .{ .y = 2, .x = 2 });
    defer dt1.deinit();
    var dts = [_]DistTable{ dt0, dt1 };

    var lg = try LocalGuidance.init(testing.allocator, 2, 3, 3);
    defer lg.deinit();

    var pos = [_]Coord{ .{ .y = 0, .x = 1 }, .{ .y = 1, .x = 0 } };
    const config = Config{ .positions = &pos };
    const order = [_]u32{ 0, 1 };

    lg.compute(config, &order, &dts, &grid);

    if (lg.hints[0] != null and lg.hints[1] != null) {
        try testing.expect(!lg.hints[0].?.eql(lg.hints[1].?));
    }
}

test "local_guidance: agent at goal gets no hint" {
    var grid = try Grid.initAllPassable(testing.allocator, 3, 3);
    defer grid.deinit();

    var dt = try DistTable.init(testing.allocator, &grid, .{ .y = 1, .x = 1 });
    defer dt.deinit();
    var dts = [_]DistTable{dt};

    var lg = try LocalGuidance.init(testing.allocator, 1, 3, 3);
    defer lg.deinit();

    var pos = [_]Coord{.{ .y = 1, .x = 1 }};
    const config = Config{ .positions = &pos };
    const order = [_]u32{0};

    lg.compute(config, &order, &dts, &grid);

    try testing.expectEqual(null, lg.hints[0]);
}
