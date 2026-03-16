//! Coord and Config types for MAPF.
//!
//! Config represents a set of agent positions and supports
//! hashing for use as HashMap keys (EXPLORED table).

const std = @import("std");
const Allocator = std.mem.Allocator;

pub const Coord = struct {
    y: i32,
    x: i32,

    pub fn eql(a: Coord, b: Coord) bool {
        return a.y == b.y and a.x == b.x;
    }

    pub fn hash(self: Coord) u64 {
        const yy: u64 = @bitCast(@as(i64, self.y));
        const xx: u64 = @bitCast(@as(i64, self.x));
        return yy *% 0x517cc1b727220a95 +% xx;
    }
};

/// Configuration: array of agent positions.
pub const Config = struct {
    positions: []Coord,

    pub fn eql(a: Config, b: Config) bool {
        if (a.positions.len != b.positions.len) return false;
        for (a.positions, b.positions) |pa, pb| {
            if (!pa.eql(pb)) return false;
        }
        return true;
    }

    pub fn hash(self: Config) u64 {
        var h: u64 = 0xcbf29ce484222325; // FNV offset basis
        for (self.positions) |pos| {
            h ^= pos.hash();
            h *%= 0x100000001b3; // FNV prime
        }
        return h;
    }

    pub fn clone(self: Config, allocator: Allocator) !Config {
        const new_positions = try allocator.alloc(Coord, self.positions.len);
        @memcpy(new_positions, self.positions);
        return Config{ .positions = new_positions };
    }

    pub fn free(self: Config, allocator: Allocator) void {
        allocator.free(self.positions);
    }

    pub fn get(self: Config, idx: usize) Coord {
        return self.positions[idx];
    }

    pub fn set(self: *Config, idx: usize, coord: Coord) void {
        self.positions[idx] = coord;
    }

    pub fn len(self: Config) usize {
        return self.positions.len;
    }
};

/// HashMap context for Config keys.
pub const ConfigContext = struct {
    pub fn hash(_: ConfigContext, key: Config) u64 {
        return key.hash();
    }

    pub fn eql(_: ConfigContext, a: Config, b: Config) bool {
        return a.eql(b);
    }
};

/// Create a Config from a flat i32 array [y0, x0, y1, x1, ...].
/// Returns error.InvalidArgument if flat array is too short.
pub fn configFromFlat(allocator: Allocator, flat: []const i32, n_agents: u32) !Config {
    const required_len = @as(usize, n_agents) * 2;
    if (flat.len < required_len) return error.InvalidArgument;
    const positions = try allocator.alloc(Coord, n_agents);
    for (0..n_agents) |i| {
        positions[i] = .{
            .y = flat[i * 2],
            .x = flat[i * 2 + 1],
        };
    }
    return Config{ .positions = positions };
}

/// Create a Config with all positions set to a fill value.
pub fn configFill(allocator: Allocator, n_agents: u32, fill: Coord) !Config {
    const positions = try allocator.alloc(Coord, n_agents);
    @memset(positions, fill);
    return Config{ .positions = positions };
}

// --- Tests ---

const testing = std.testing;

test "coord: equality" {
    const a = Coord{ .y = 1, .x = 2 };
    const b = Coord{ .y = 1, .x = 2 };
    const c = Coord{ .y = 1, .x = 3 };

    try testing.expect(a.eql(b));
    try testing.expect(!a.eql(c));
}

test "coord: hash consistency" {
    const a = Coord{ .y = 5, .x = 10 };
    const b = Coord{ .y = 5, .x = 10 };

    try testing.expectEqual(a.hash(), b.hash());
}

test "coord: hash differs for different coords" {
    const a = Coord{ .y = 0, .x = 0 };
    const b = Coord{ .y = 0, .x = 1 };
    const c = Coord{ .y = 1, .x = 0 };

    try testing.expect(a.hash() != b.hash());
    try testing.expect(a.hash() != c.hash());
    try testing.expect(b.hash() != c.hash());
}

test "config: equality" {
    const allocator = testing.allocator;

    var pos_a = [_]Coord{ .{ .y = 0, .x = 0 }, .{ .y = 1, .x = 1 } };
    var pos_b = [_]Coord{ .{ .y = 0, .x = 0 }, .{ .y = 1, .x = 1 } };
    var pos_c = [_]Coord{ .{ .y = 0, .x = 0 }, .{ .y = 2, .x = 2 } };

    const a = Config{ .positions = &pos_a };
    const b = Config{ .positions = &pos_b };
    const c = Config{ .positions = &pos_c };

    try testing.expect(a.eql(b));
    try testing.expect(!a.eql(c));

    // Hash consistency
    try testing.expectEqual(a.hash(), b.hash());
    try testing.expect(a.hash() != c.hash());

    _ = allocator;
}

test "config: clone" {
    const allocator = testing.allocator;

    var pos = [_]Coord{ .{ .y = 3, .x = 4 }, .{ .y = 5, .x = 6 } };
    const original = Config{ .positions = &pos };

    const cloned = try original.clone(allocator);
    defer cloned.free(allocator);

    try testing.expect(original.eql(cloned));
    // Ensure deep copy (different memory)
    try testing.expect(original.positions.ptr != cloned.positions.ptr);
}

test "config: from flat array" {
    const allocator = testing.allocator;

    const flat = [_]i32{ 1, 2, 3, 4, 5, 6 };
    const cfg = try configFromFlat(allocator, &flat, 3);
    defer cfg.free(allocator);

    try testing.expectEqual(3, cfg.len());
    try testing.expect(cfg.get(0).eql(.{ .y = 1, .x = 2 }));
    try testing.expect(cfg.get(1).eql(.{ .y = 3, .x = 4 }));
    try testing.expect(cfg.get(2).eql(.{ .y = 5, .x = 6 }));
}

test "config: HashMap context" {
    const allocator = testing.allocator;

    var pos_a = [_]Coord{ .{ .y = 0, .x = 0 } };
    var pos_b = [_]Coord{ .{ .y = 1, .x = 1 } };

    const a = Config{ .positions = &pos_a };
    const b = Config{ .positions = &pos_b };

    var map = std.HashMap(Config, i32, ConfigContext, 80).init(allocator);
    defer map.deinit();

    try map.put(a, 10);
    try map.put(b, 20);

    try testing.expectEqual(10, map.get(a).?);
    try testing.expectEqual(20, map.get(b).?);
}

// --- Edge case tests ---

test "config: zero agents" {
    const allocator = testing.allocator;

    var empty_pos = [_]Coord{};
    const a = Config{ .positions = &empty_pos };
    const b = Config{ .positions = &empty_pos };

    try testing.expect(a.eql(b));
    try testing.expectEqual(a.hash(), b.hash());
    try testing.expectEqual(0, a.len());

    // Clone of empty
    const cloned = try a.clone(allocator);
    defer cloned.free(allocator);
    try testing.expect(a.eql(cloned));
    try testing.expectEqual(0, cloned.len());
}

test "config: single agent" {
    var pos = [_]Coord{.{ .y = 3, .x = 7 }};
    const cfg = Config{ .positions = &pos };

    try testing.expectEqual(1, cfg.len());
    try testing.expect(cfg.get(0).eql(.{ .y = 3, .x = 7 }));
}

test "coord: negative coordinates" {
    const a = Coord{ .y = -10, .x = -20 };
    const b = Coord{ .y = -10, .x = -20 };
    const c = Coord{ .y = -10, .x = -21 };

    try testing.expect(a.eql(b));
    try testing.expect(!a.eql(c));
    try testing.expectEqual(a.hash(), b.hash());
    try testing.expect(a.hash() != c.hash());
}

test "coord: extreme values" {
    const max_coord = Coord{ .y = std.math.maxInt(i32), .x = std.math.maxInt(i32) };
    const min_coord = Coord{ .y = std.math.minInt(i32), .x = std.math.minInt(i32) };
    const zero = Coord{ .y = 0, .x = 0 };

    try testing.expect(!max_coord.eql(min_coord));
    try testing.expect(!max_coord.eql(zero));
    try testing.expect(max_coord.hash() != min_coord.hash());
}

test "config: different lengths not equal" {
    var pos1 = [_]Coord{.{ .y = 0, .x = 0 }};
    var pos2 = [_]Coord{ .{ .y = 0, .x = 0 }, .{ .y = 1, .x = 1 } };

    const a = Config{ .positions = &pos1 };
    const b = Config{ .positions = &pos2 };

    try testing.expect(!a.eql(b));
}

test "config: configFill" {
    const allocator = testing.allocator;
    const fill = Coord{ .y = 99, .x = 99 };
    const cfg = try configFill(allocator, 5, fill);
    defer cfg.free(allocator);

    try testing.expectEqual(5, cfg.len());
    for (0..5) |i| {
        try testing.expect(cfg.get(i).eql(fill));
    }
}

test "config: set modifies position" {
    const allocator = testing.allocator;
    var cfg = try configFill(allocator, 3, .{ .y = 0, .x = 0 });
    defer cfg.free(allocator);

    cfg.set(1, .{ .y = 5, .x = 5 });
    try testing.expect(cfg.get(1).eql(.{ .y = 5, .x = 5 }));
    try testing.expect(cfg.get(0).eql(.{ .y = 0, .x = 0 })); // unchanged
}
