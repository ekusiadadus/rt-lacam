//! Lightweight Traffic Map for congestion-aware MAPF.
//!
//! Tracks directed edge usage during PIBT execution and provides
//! normalized congestion penalties to bias action selection away
//! from frequently-traversed edges.
//!
//! Reference: Shen et al., "A Lightweight Traffic Map for Efficient
//! Anytime LaCAM*" (arXiv:2603.07891, 2026)

const std = @import("std");
const Allocator = std.mem.Allocator;
const Coord = @import("config.zig").Coord;

pub const TrafficMap = struct {
    /// Per-cell directed edge counts: 4 directions per cell.
    /// Layout: (y * width + x) * 4 + dir
    /// Direction: N=0(-1,0) S=1(1,0) W=2(0,-1) E=3(0,1)
    counts: []u32,
    width: u32,
    height: u32,
    max_count: u32,
    /// Upper bound for normalized penalty (integer).
    w_ub: i32,
    allocator: Allocator,

    pub fn init(allocator: Allocator, width: u32, height: u32, w_ub: i32) !TrafficMap {
        const size = @as(usize, width) * @as(usize, height) * 4;
        const counts = try allocator.alloc(u32, size);
        @memset(counts, 0);
        return TrafficMap{
            .counts = counts,
            .width = width,
            .height = height,
            .max_count = 0,
            .w_ub = w_ub,
            .allocator = allocator,
        };
    }

    pub fn deinit(self: *TrafficMap) void {
        self.allocator.free(self.counts);
        self.* = undefined;
    }

    /// Direction index for adjacent cells.
    pub fn dirIndex(from: Coord, to: Coord) ?u2 {
        const dy = to.y - from.y;
        const dx = to.x - from.x;
        if (dy == -1 and dx == 0) return 0; // N
        if (dy == 1 and dx == 0) return 1; // S
        if (dy == 0 and dx == -1) return 2; // W
        if (dy == 0 and dx == 1) return 3; // E
        return null;
    }

    fn edgeIdx(self: *const TrafficMap, from: Coord, dir: u2) usize {
        const cell = @as(usize, @intCast(from.y)) * @as(usize, self.width) +
            @as(usize, @intCast(from.x));
        return cell * 4 + @as(usize, dir);
    }

    fn increment(self: *TrafficMap, from: Coord, dir: u2) void {
        const idx = self.edgeIdx(from, dir);
        self.counts[idx] += 1;
        if (self.counts[idx] > self.max_count) {
            self.max_count = self.counts[idx];
        }
    }

    /// Record a committed action (agent moved from -> to).
    /// Wait actions (from == to) increment all outgoing edges.
    pub fn recordCommitted(self: *TrafficMap, from: Coord, to: Coord) void {
        if (from.eql(to)) {
            self.recordWait(from);
            return;
        }
        if (dirIndex(from, to)) |dir| {
            self.increment(from, dir);
        }
    }

    /// Record a blocked action (agent wanted from -> to but was blocked).
    pub fn recordBlocked(self: *TrafficMap, from: Coord, to: Coord) void {
        if (dirIndex(from, to)) |dir| {
            self.increment(from, dir);
        }
    }

    /// Wait action: increment all outgoing edges from this cell.
    fn recordWait(self: *TrafficMap, at: Coord) void {
        for (0..4) |d| {
            self.increment(at, @intCast(d));
        }
    }

    /// Get traffic penalty for traversing edge from -> to.
    /// Returns integer cost in [0, w_ub], normalized by max count.
    pub fn getPenalty(self: *const TrafficMap, from: Coord, to: Coord) i32 {
        if (from.eql(to)) return 0;
        if (self.max_count == 0) return 0;
        const dir = dirIndex(from, to) orelse return 0;
        const raw = self.counts[self.edgeIdx(from, dir)];
        return @intCast(
            (@as(u64, raw) * @as(u64, @intCast(self.w_ub))) /
                @as(u64, self.max_count),
        );
    }

    /// Reset all counts.
    pub fn reset(self: *TrafficMap) void {
        @memset(self.counts, 0);
        self.max_count = 0;
    }
};

// --- Tests ---

const testing = std.testing;

test "traffic_map: init and deinit" {
    var tm = try TrafficMap.init(testing.allocator, 5, 5, 10);
    defer tm.deinit();
    try testing.expectEqual(0, tm.max_count);
    try testing.expectEqual(0, tm.getPenalty(.{ .y = 0, .x = 0 }, .{ .y = 0, .x = 1 }));
}

test "traffic_map: record committed increments count" {
    var tm = try TrafficMap.init(testing.allocator, 3, 3, 10);
    defer tm.deinit();
    tm.recordCommitted(.{ .y = 0, .x = 0 }, .{ .y = 0, .x = 1 });
    try testing.expectEqual(1, tm.max_count);
    // Penalty = w_ub (only edge with traffic, so max = this edge)
    try testing.expectEqual(10, tm.getPenalty(.{ .y = 0, .x = 0 }, .{ .y = 0, .x = 1 }));
    // Other edges have zero penalty
    try testing.expectEqual(0, tm.getPenalty(.{ .y = 0, .x = 0 }, .{ .y = 1, .x = 0 }));
}

test "traffic_map: wait increments all outgoing" {
    var tm = try TrafficMap.init(testing.allocator, 3, 3, 10);
    defer tm.deinit();
    tm.recordCommitted(.{ .y = 1, .x = 1 }, .{ .y = 1, .x = 1 }); // wait
    try testing.expect(tm.getPenalty(.{ .y = 1, .x = 1 }, .{ .y = 0, .x = 1 }) > 0);
    try testing.expect(tm.getPenalty(.{ .y = 1, .x = 1 }, .{ .y = 2, .x = 1 }) > 0);
    try testing.expect(tm.getPenalty(.{ .y = 1, .x = 1 }, .{ .y = 1, .x = 0 }) > 0);
    try testing.expect(tm.getPenalty(.{ .y = 1, .x = 1 }, .{ .y = 1, .x = 2 }) > 0);
}

test "traffic_map: normalization scales to w_ub" {
    var tm = try TrafficMap.init(testing.allocator, 3, 3, 10);
    defer tm.deinit();
    for (0..10) |_| {
        tm.recordCommitted(.{ .y = 0, .x = 0 }, .{ .y = 0, .x = 1 });
    }
    for (0..5) |_| {
        tm.recordCommitted(.{ .y = 0, .x = 0 }, .{ .y = 1, .x = 0 });
    }
    try testing.expectEqual(10, tm.getPenalty(.{ .y = 0, .x = 0 }, .{ .y = 0, .x = 1 }));
    try testing.expectEqual(5, tm.getPenalty(.{ .y = 0, .x = 0 }, .{ .y = 1, .x = 0 }));
}

test "traffic_map: reset clears all" {
    var tm = try TrafficMap.init(testing.allocator, 3, 3, 10);
    defer tm.deinit();
    tm.recordCommitted(.{ .y = 0, .x = 0 }, .{ .y = 0, .x = 1 });
    try testing.expect(tm.max_count > 0);
    tm.reset();
    try testing.expectEqual(0, tm.max_count);
    try testing.expectEqual(0, tm.getPenalty(.{ .y = 0, .x = 0 }, .{ .y = 0, .x = 1 }));
}

test "traffic_map: same cell returns zero penalty" {
    var tm = try TrafficMap.init(testing.allocator, 3, 3, 10);
    defer tm.deinit();
    try testing.expectEqual(0, tm.getPenalty(.{ .y = 1, .x = 1 }, .{ .y = 1, .x = 1 }));
}

test "traffic_map: non-adjacent returns zero penalty" {
    var tm = try TrafficMap.init(testing.allocator, 5, 5, 10);
    defer tm.deinit();
    try testing.expectEqual(0, tm.getPenalty(.{ .y = 0, .x = 0 }, .{ .y = 2, .x = 2 }));
}

test "traffic_map: blocked action recorded" {
    var tm = try TrafficMap.init(testing.allocator, 3, 3, 10);
    defer tm.deinit();
    tm.recordBlocked(.{ .y = 0, .x = 0 }, .{ .y = 0, .x = 1 });
    try testing.expectEqual(1, tm.max_count);
    try testing.expectEqual(10, tm.getPenalty(.{ .y = 0, .x = 0 }, .{ .y = 0, .x = 1 }));
}
