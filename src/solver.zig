//! Real-Time LaCAM Solver.
//!
//! Implements the incremental DFS with state persistence across
//! step() calls and re-rooting after agent execution.
//!
//! Reference: "Real-Time LaCAM for Real-Time MAPF" (Liang et al., SOCS 2025)

const std = @import("std");
const Allocator = std.mem.Allocator;
const Coord = @import("config.zig").Coord;
const Config = @import("config.zig").Config;
const ConfigContext = @import("config.zig").ConfigContext;
const configFill = @import("config.zig").configFill;
const Grid = @import("grid.zig").Grid;
const MAX_NEIGHBORS = @import("grid.zig").MAX_NEIGHBORS;
const DistTable = @import("dist_table.zig").DistTable;
const NIL_DIST = @import("dist_table.zig").NIL_DIST;
const Deque = @import("deque.zig").Deque;
const PIBT = @import("pibt.zig").PIBT;
const LowLevelNode = @import("low_level_node.zig").LowLevelNode;
const HighLevelNode = @import("high_level_node.zig").HighLevelNode;

pub const Solver = struct {
    // Problem
    grid: Grid,
    goals: Config,
    num_agents: u32,
    flg_star: bool,

    // Persistent search state
    open: Deque(*HighLevelNode),
    explored: std.HashMap(Config, *HighLevelNode, ConfigContext, 80),
    dist_tables: []DistTable,
    pibt: PIBT,
    current_node: *HighLevelNode,
    goal_node: ?*HighLevelNode,
    prng: std.Random.DefaultPrng,

    allocator: Allocator,

    /// Initialize solver from C-compatible flat arrays.
    pub fn initFromFlat(
        allocator: Allocator,
        grid_ptr: [*]const u8,
        width: u32,
        height: u32,
        starts_ptr: [*]const i32,
        goals_ptr: [*]const i32,
        n_agents: u32,
        flg_star: bool,
        seed: u64,
    ) !*Solver {
        if (width == 0 or height == 0 or n_agents == 0) return error.InvalidArgument;
        if (n_agents > std.math.maxInt(u32) / 2) return error.InvalidArgument;

        const grid_size = @as(usize, width) * @as(usize, height);
        const grid_raw = grid_ptr[0..grid_size];
        const flat_len: usize = @as(usize, n_agents) * 2;
        const starts_flat = starts_ptr[0..flat_len];
        const goals_flat = goals_ptr[0..flat_len];

        var grid = try Grid.init(allocator, grid_raw, width, height);
        errdefer grid.deinit();

        // Build starts and goals Config
        const starts_pos = try allocator.alloc(Coord, n_agents);
        const goals_pos = try allocator.alloc(Coord, n_agents);
        for (0..n_agents) |i| {
            starts_pos[i] = .{ .y = starts_flat[i * 2], .x = starts_flat[i * 2 + 1] };
            goals_pos[i] = .{ .y = goals_flat[i * 2], .x = goals_flat[i * 2 + 1] };
        }
        const starts = Config{ .positions = starts_pos };
        const goals = Config{ .positions = goals_pos };

        return try initInternal(allocator, grid, starts, goals, n_agents, flg_star, seed);
    }

    /// Initialize solver from Zig types (for testing).
    pub fn initFromConfigs(
        allocator: Allocator,
        grid: Grid,
        starts: Config,
        goals: Config,
        flg_star: bool,
        seed: u64,
    ) !*Solver {
        const n: u32 = @intCast(starts.len());
        return try initInternal(allocator, grid, starts, goals, n, flg_star, seed);
    }

    fn initInternal(
        allocator: Allocator,
        grid: Grid,
        starts: Config,
        goals: Config,
        n_agents: u32,
        flg_star: bool,
        seed: u64,
    ) !*Solver {
        const self = try allocator.create(Solver);
        errdefer allocator.destroy(self);

        // Build distance tables
        const dist_tables = try allocator.alloc(DistTable, n_agents);
        for (0..n_agents) |i| {
            dist_tables[i] = try DistTable.init(allocator, &grid, goals.get(@intCast(i)));
        }

        // Initialize PIBT
        const pibt = try PIBT.init(allocator, dist_tables, &grid, n_agents, seed);

        // Initialize search
        var open = Deque(*HighLevelNode).init(allocator);
        var explored = std.HashMap(Config, *HighLevelNode, ConfigContext, 80).init(allocator);
        var prng = std.Random.DefaultPrng.init(seed);

        // Create initial HLN
        const h_init = getHValue(starts, dist_tables, n_agents);
        const order_init = try getOrder(allocator, starts, dist_tables, n_agents, &prng);
        const n_init = try HighLevelNode.init(allocator, starts, order_init, null, 0, h_init);
        allocator.free(order_init); // HLN clones it

        try open.pushFront(n_init);
        const starts_clone = try starts.clone(allocator);
        try explored.put(starts_clone, n_init);

        const goals_owned = try goals.clone(allocator);

        self.* = .{
            .grid = grid,
            .goals = goals_owned,
            .num_agents = n_agents,
            .flg_star = flg_star,
            .open = open,
            .explored = explored,
            .dist_tables = dist_tables,
            .pibt = pibt,
            .current_node = n_init,
            .goal_node = null,
            .prng = prng,
            .allocator = allocator,
        };

        // Fix grid pointers (they were pointing to stack local)
        self.pibt.grid = &self.grid;
        for (0..n_agents) |i| {
            self.dist_tables[i].grid = &self.grid;
        }

        return self;
    }

    pub fn deinit(self: *Solver) void {
        // Free all HLN nodes via explored map
        var it = self.explored.iterator();
        while (it.next()) |entry| {
            // Free the key (cloned Config)
            entry.key_ptr.*.free(self.allocator);
            // Free the node
            entry.value_ptr.*.deinit();
        }
        self.explored.deinit();

        // Drain open (nodes already freed above)
        self.open.deinit();

        // Free dist tables
        for (self.dist_tables) |*dt| {
            dt.deinit();
        }
        self.allocator.free(self.dist_tables);

        self.pibt.deinit();
        self.goals.free(self.allocator);
        self.grid.deinit();
        self.allocator.destroy(self);
    }

    /// Run incremental DFS for up to deadline_ms milliseconds.
    /// Writes next positions to out_positions (flat [y0,x0,y1,x1,...]).
    /// Returns number of agents that moved, 0 if no progress, -1 on error.
    pub fn stepFlat(self: *Solver, deadline_ms: u32, out_ptr: [*]i32) i32 {
        const next_config = self.stepInternal(deadline_ms) orelse return 0;

        var moved: i32 = 0;
        for (0..self.num_agents) |i| {
            const pos = next_config.get(@intCast(i));
            out_ptr[i * 2] = pos.y;
            out_ptr[i * 2 + 1] = pos.x;
            if (!pos.eql(self.current_node.config.get(@intCast(i)))) {
                moved += 1;
            }
        }
        return moved;
    }

    /// Run incremental DFS, return next Config or null.
    pub fn stepInternal(self: *Solver, deadline_ms: u32) ?Config {
        const start_ns = std.time.nanoTimestamp();
        const deadline_ns = start_ns + @as(i128, deadline_ms) * 1_000_000;

        while (!self.open.isEmpty()) {
            // Check deadline
            if (std.time.nanoTimestamp() >= deadline_ns) break;

            const n = self.open.peekFront() orelse break;

            // Goal check
            if (self.goal_node == null and n.config.eql(self.goals)) {
                self.goal_node = n;
                if (!self.flg_star) break;
            }

            // Lower bound pruning
            if (self.goal_node) |gn| {
                if (gn.g <= n.f) {
                    _ = self.open.popFront();
                    continue;
                }
            }

            // Exhausted low-level constraints
            if (n.tree.isEmpty()) {
                _ = self.open.popFront();
                continue;
            }

            // Low-level DFS expansion
            const c = n.tree.popFront() orelse continue;
            defer c.deinit();

            if (c.depth < self.num_agents) {
                const i = n.order[c.depth];
                const v = n.config.get(i);

                // Candidates: stay + neighbors
                var cands: [1 + MAX_NEIGHBORS]Coord = undefined;
                cands[0] = v;
                var nbuf: [MAX_NEIGHBORS]Coord = undefined;
                const n_count = self.grid.neighbors(v, &nbuf);
                for (0..n_count) |k| {
                    cands[1 + k] = nbuf[k];
                }
                const total = 1 + n_count;

                // Shuffle
                self.shuffleCoords(cands[0..total]);

                // Add children to tree
                for (0..total) |k| {
                    const child = c.getChild(self.allocator, i, cands[k]) catch continue;
                    n.tree.pushBack(child) catch {
                        child.deinit();
                        continue;
                    };
                }
            }

            // Generate next configuration via PIBT
            const q_to = self.configurationGenerator(n, c) orelse continue;

            // Check if already explored
            if (self.explored.get(q_to)) |n_known| {
                q_to.free(self.allocator);
                n.addNeighbor(n_known) catch {};
                self.open.pushFront(n_known) catch {};
                // Dijkstra update (LaCAM*)
                if (self.flg_star) {
                    self.dijkstraUpdate(n);
                }
            } else {
                // New configuration
                const h_new = getHValue(q_to, self.dist_tables, self.num_agents);
                if (h_new == NIL_DIST) continue; // unreachable config
                const g_new = n.g +| self.getEdgeCost(n.config, q_to);
                const order_new = getOrder(self.allocator, q_to, self.dist_tables, self.num_agents, &self.prng) catch continue;
                const n_new = HighLevelNode.init(self.allocator, q_to, order_new, n, g_new, h_new) catch {
                    self.allocator.free(order_new);
                    q_to.free(self.allocator);
                    continue;
                };
                self.allocator.free(order_new);

                n.addNeighbor(n_new) catch {};
                self.open.pushFront(n_new) catch {};

                // Store with cloned key
                const key = q_to.clone(self.allocator) catch {
                    q_to.free(self.allocator);
                    continue;
                };
                self.explored.put(key, n_new) catch {
                    key.free(self.allocator);
                };
                q_to.free(self.allocator);
            }
        }

        return self.extractNextConfig();
    }

    /// Re-root DFS tree after agents move to new positions.
    pub fn rerootFlat(self: *Solver, current_ptr: [*]const i32) void {
        const positions = self.allocator.alloc(Coord, self.num_agents) catch return;
        defer self.allocator.free(positions);
        for (0..self.num_agents) |i| {
            positions[i] = .{ .y = current_ptr[i * 2], .x = current_ptr[i * 2 + 1] };
        }
        const cfg = Config{ .positions = positions };
        self.reroot(cfg);
    }

    pub fn reroot(self: *Solver, current: Config) void {
        // Find or create node for current config
        const new_node = if (self.explored.get(current)) |existing|
            existing
        else blk: {
            const h = getHValue(current, self.dist_tables, self.num_agents);
            const order = getOrder(self.allocator, current, self.dist_tables, self.num_agents, &self.prng) catch return;
            const node = HighLevelNode.init(self.allocator, current, order, null, self.current_node.g + 1, h) catch {
                self.allocator.free(order);
                return;
            };
            self.allocator.free(order);

            self.current_node.addNeighbor(node) catch {};
            node.addNeighbor(self.current_node) catch {};

            const key = current.clone(self.allocator) catch return;
            self.explored.put(key, node) catch {
                key.free(self.allocator);
            };
            break :blk node;
        };

        // Swap parent pointer (re-root)
        if (new_node.parent == self.current_node) {
            new_node.parent = null;
            self.current_node.parent = new_node;
        }

        self.current_node = new_node;

        // Ensure continued exploration
        self.open.pushFront(new_node) catch {};
    }

    pub fn exploredSize(self: *const Solver) u32 {
        return @intCast(self.explored.count());
    }

    // --- Private helpers ---

    fn configurationGenerator(self: *Solver, n: *HighLevelNode, c: *const LowLevelNode) ?Config {
        // Setup partial config from constraints
        var q_to = configFill(self.allocator, self.num_agents, self.pibt.nil_coord) catch return null;

        for (0..c.depth) |k| {
            q_to.set(c.who[k], c.where[k]);
        }

        // Apply PIBT for remaining agents
        const success = self.pibt.step(n.config, &q_to, n.order);
        if (!success) {
            q_to.free(self.allocator);
            return null;
        }
        return q_to;
    }

    fn dijkstraUpdate(self: *Solver, n_from: *HighLevelNode) void {
        // Simple BFS-style Dijkstra propagation
        var queue = Deque(*HighLevelNode).init(self.allocator);
        defer queue.deinit();
        queue.pushBack(n_from) catch return;

        while (!queue.isEmpty()) {
            if (!self.flg_star) break;
            const from = queue.popFront() orelse break;

            var nit = from.neighbors.iterator();
            while (nit.next()) |entry| {
                const to = entry.key_ptr.*;
                const g = from.g + self.getEdgeCost(from.config, to.config);
                if (g < to.g) {
                    to.g = g;
                    to.f = to.g + to.h;
                    to.parent = from;
                    queue.pushBack(to) catch {};
                    if (self.goal_node) |gn| {
                        if (to.f < gn.g) {
                            self.open.pushFront(to) catch {};
                        }
                    }
                }
            }
        }
    }

    fn extractNextConfig(self: *Solver) ?Config {
        const gn = self.goal_node orelse return null;

        // Backtrack from goal to find path
        var path_len: u32 = 0;
        var n: ?*HighLevelNode = gn;
        while (n != null) : (n = n.?.parent) {
            path_len += 1;
        }

        // Find current_node in path, return next
        n = gn;
        var prev: ?*HighLevelNode = null;
        while (n != null) {
            if (n.? == self.current_node) {
                // Return the config after current_node toward goal
                if (prev) |p| return p.config;
                return null; // Already at goal
            }
            prev = n;
            n = n.?.parent;
        }

        // current_node not on best path — try direct neighbor toward goal
        return self.greedyNextStep();
    }

    fn greedyNextStep(self: *Solver) ?Config {
        // Find neighbor with lowest h value
        var nit = self.current_node.neighbors.iterator();
        var best: ?*HighLevelNode = null;
        var best_f: i32 = std.math.maxInt(i32);

        while (nit.next()) |entry| {
            const neighbor = entry.key_ptr.*;
            if (neighbor.f < best_f) {
                best_f = neighbor.f;
                best = neighbor;
            }
        }

        if (best) |b| return b.config;
        return null;
    }

    fn getEdgeCost(self: *const Solver, q_from: Config, q_to: Config) i32 {
        var cost: i32 = 0;
        for (0..self.num_agents) |i| {
            const idx: u32 = @intCast(i);
            const at_goal = self.goals.get(idx).eql(q_from.get(idx)) and
                self.goals.get(idx).eql(q_to.get(idx));
            if (!at_goal) cost += 1;
        }
        return cost;
    }

    fn shuffleCoords(self: *Solver, slice: []Coord) void {
        if (slice.len <= 1) return;
        var rng = self.prng.random();
        var i: usize = slice.len - 1;
        while (i > 0) : (i -= 1) {
            const j = rng.intRangeAtMost(usize, 0, i);
            const tmp = slice[i];
            slice[i] = slice[j];
            slice[j] = tmp;
        }
    }
};

fn getHValue(cfg: Config, dist_tables: []DistTable, n: u32) i32 {
    var cost: i32 = 0;
    for (0..n) |i| {
        const d = dist_tables[i].get(cfg.get(@intCast(i)));
        if (d == NIL_DIST) return NIL_DIST;
        cost += d;
    }
    return cost;
}

fn getOrder(allocator: Allocator, cfg: Config, dist_tables: []DistTable, n: u32, prng: *std.Random.DefaultPrng) ![]u32 {
    const order = try allocator.alloc(u32, n);
    for (0..n) |i| {
        order[i] = @intCast(i);
    }

    // Shuffle for tie-breaking
    var rng = prng.random();
    if (n > 1) {
        var i: usize = n - 1;
        while (i > 0) : (i -= 1) {
            const j = rng.intRangeAtMost(usize, 0, i);
            const tmp = order[i];
            order[i] = order[j];
            order[j] = tmp;
        }
    }

    // Sort by descending distance to goal
    const Context = struct {
        tables: []DistTable,
        config: Config,
    };
    const ctx = Context{ .tables = dist_tables, .config = cfg };
    std.mem.sort(u32, order, ctx, struct {
        fn lessThan(c: Context, a: u32, b: u32) bool {
            return c.tables[a].get(c.config.get(a)) > c.tables[b].get(c.config.get(b));
        }
    }.lessThan);

    return order;
}

// --- Tests ---

const testing = std.testing;

test "solver: single agent reaches goal" {
    const grid = try Grid.initAllPassable(testing.allocator, 5, 5);

    var starts_pos = [_]Coord{.{ .y = 0, .x = 0 }};
    var goals_pos = [_]Coord{.{ .y = 4, .x = 4 }};
    const starts = Config{ .positions = &starts_pos };
    const goals = Config{ .positions = &goals_pos };

    const solver = try Solver.initFromConfigs(
        testing.allocator,
        grid,
        starts,
        goals,
        false,
        42,
    );
    defer solver.deinit();

    // Run with generous deadline
    const result = solver.stepInternal(5000);
    try testing.expect(result != null);
}

test "solver: explored grows across step calls" {
    const grid = try Grid.initAllPassable(testing.allocator, 5, 5);

    var starts_pos = [_]Coord{.{ .y = 0, .x = 0 }};
    var goals_pos = [_]Coord{.{ .y = 4, .x = 4 }};
    const starts = Config{ .positions = &starts_pos };
    const goals = Config{ .positions = &goals_pos };

    const solver = try Solver.initFromConfigs(
        testing.allocator,
        grid,
        starts,
        goals,
        false,
        42,
    );
    defer solver.deinit();

    const size_before = solver.exploredSize();
    _ = solver.stepInternal(100);
    const size_after = solver.exploredSize();

    try testing.expect(size_after >= size_before);
}

test "solver: two agents on 5x5 grid" {
    const grid = try Grid.initAllPassable(testing.allocator, 5, 5);

    var starts_pos = [_]Coord{ .{ .y = 0, .x = 0 }, .{ .y = 4, .x = 4 } };
    var goals_pos = [_]Coord{ .{ .y = 4, .x = 4 }, .{ .y = 0, .x = 0 } };
    const starts = Config{ .positions = &starts_pos };
    const goals = Config{ .positions = &goals_pos };

    const solver = try Solver.initFromConfigs(
        testing.allocator,
        grid,
        starts,
        goals,
        false,
        42,
    );
    defer solver.deinit();

    const result = solver.stepInternal(5000);
    try testing.expect(result != null);
}

// --- Edge case tests ---

test "solver: agent already at goal returns null (no move needed)" {
    const grid = try Grid.initAllPassable(testing.allocator, 3, 3);

    var starts_pos = [_]Coord{.{ .y = 1, .x = 1 }};
    var goals_pos = [_]Coord{.{ .y = 1, .x = 1 }}; // same as start
    const starts = Config{ .positions = &starts_pos };
    const goals = Config{ .positions = &goals_pos };

    const solver = try Solver.initFromConfigs(
        testing.allocator,
        grid,
        starts,
        goals,
        false,
        42,
    );
    defer solver.deinit();

    const result = solver.stepInternal(5000);
    // Already at goal: extractNextConfig returns null (current_node IS goal_node)
    try testing.expect(result == null);
    // Verify goal_node was found
    try testing.expect(solver.goal_node != null);
}

test "solver: timeout zero does not crash" {
    const grid = try Grid.initAllPassable(testing.allocator, 10, 10);

    var starts_pos = [_]Coord{.{ .y = 0, .x = 0 }};
    var goals_pos = [_]Coord{.{ .y = 9, .x = 9 }};
    const starts = Config{ .positions = &starts_pos };
    const goals = Config{ .positions = &goals_pos };

    const solver = try Solver.initFromConfigs(
        testing.allocator,
        grid,
        starts,
        goals,
        false,
        42,
    );
    defer solver.deinit();

    // With 0ms deadline, should not crash
    _ = solver.stepInternal(0);
}

test "solver: narrow 1-wide corridor" {
    const grid = try Grid.initAllPassable(testing.allocator, 7, 1);

    var starts_pos = [_]Coord{.{ .y = 0, .x = 0 }};
    var goals_pos = [_]Coord{.{ .y = 0, .x = 6 }};
    const starts = Config{ .positions = &starts_pos };
    const goals = Config{ .positions = &goals_pos };

    const solver = try Solver.initFromConfigs(
        testing.allocator,
        grid,
        starts,
        goals,
        false,
        42,
    );
    defer solver.deinit();

    const result = solver.stepInternal(5000);
    try testing.expect(result != null);
}

test "solver: two agents 1-wide corridor cannot swap" {
    const grid = try Grid.initAllPassable(testing.allocator, 5, 1);

    var starts_pos = [_]Coord{ .{ .y = 0, .x = 0 }, .{ .y = 0, .x = 4 } };
    var goals_pos = [_]Coord{ .{ .y = 0, .x = 4 }, .{ .y = 0, .x = 0 } };
    const starts = Config{ .positions = &starts_pos };
    const goals = Config{ .positions = &goals_pos };

    const solver = try Solver.initFromConfigs(
        testing.allocator,
        grid,
        starts,
        goals,
        false,
        42,
    );
    defer solver.deinit();

    // Unsolvable in 1-wide corridor — should not crash
    var found = false;
    for (0..20) |_| {
        if (solver.stepInternal(100) != null) {
            found = true;
            break;
        }
    }
    // In 1-wide corridor, two agents cannot swap
    try testing.expect(!found);
}

test "solver: multiple steps accumulate explored state" {
    const grid = try Grid.initAllPassable(testing.allocator, 5, 5);

    var starts_pos = [_]Coord{.{ .y = 0, .x = 0 }};
    var goals_pos = [_]Coord{.{ .y = 4, .x = 4 }};
    const starts = Config{ .positions = &starts_pos };
    const goals = Config{ .positions = &goals_pos };

    const solver = try Solver.initFromConfigs(
        testing.allocator,
        grid,
        starts,
        goals,
        false,
        42,
    );
    defer solver.deinit();

    var sizes: [5]u32 = undefined;
    for (0..5) |i| {
        _ = solver.stepInternal(50);
        sizes[i] = solver.exploredSize();
    }

    // State should monotonically non-decrease
    for (1..5) |i| {
        try testing.expect(sizes[i] >= sizes[i - 1]);
    }
}

test "solver: lacam star mode" {
    const grid = try Grid.initAllPassable(testing.allocator, 5, 5);

    var starts_pos = [_]Coord{.{ .y = 0, .x = 0 }};
    var goals_pos = [_]Coord{.{ .y = 4, .x = 4 }};
    const starts = Config{ .positions = &starts_pos };
    const goals = Config{ .positions = &goals_pos };

    const solver = try Solver.initFromConfigs(
        testing.allocator,
        grid,
        starts,
        goals,
        true, // LaCAM*
        42,
    );
    defer solver.deinit();

    const result = solver.stepInternal(5000);
    try testing.expect(result != null);
}

test "solver: three agents on 5x5 grid" {
    const grid = try Grid.initAllPassable(testing.allocator, 5, 5);

    var starts_pos = [_]Coord{ .{ .y = 0, .x = 0 }, .{ .y = 0, .x = 4 }, .{ .y = 4, .x = 0 } };
    var goals_pos = [_]Coord{ .{ .y = 4, .x = 4 }, .{ .y = 4, .x = 0 }, .{ .y = 0, .x = 4 } };
    const starts = Config{ .positions = &starts_pos };
    const goals = Config{ .positions = &goals_pos };

    const solver = try Solver.initFromConfigs(
        testing.allocator,
        grid,
        starts,
        goals,
        false,
        42,
    );
    defer solver.deinit();

    var found = false;
    for (0..50) |_| {
        if (solver.stepInternal(200) != null) {
            found = true;
            break;
        }
    }
    try testing.expect(found);
}

// --- Phase 4: RT-LaCAM specific tests ---

test "rt-lacam: reroot updates current_node" {
    const grid = try Grid.initAllPassable(testing.allocator, 5, 5);

    var starts_pos = [_]Coord{.{ .y = 0, .x = 0 }};
    var goals_pos = [_]Coord{.{ .y = 4, .x = 4 }};
    const starts = Config{ .positions = &starts_pos };
    const goals = Config{ .positions = &goals_pos };

    const solver = try Solver.initFromConfigs(
        testing.allocator,
        grid,
        starts,
        goals,
        false,
        42,
    );
    defer solver.deinit();

    // Record initial current_node
    const initial_node = solver.current_node;
    try testing.expect(initial_node.config.get(0).eql(.{ .y = 0, .x = 0 }));

    // Reroot to a new position
    var new_pos = [_]Coord{.{ .y = 0, .x = 1 }};
    const new_cfg = Config{ .positions = &new_pos };
    solver.reroot(new_cfg);

    // current_node should have changed
    try testing.expect(solver.current_node != initial_node);
    try testing.expect(solver.current_node.config.get(0).eql(.{ .y = 0, .x = 1 }));
}

test "rt-lacam: reroot to explored config reuses node" {
    const grid = try Grid.initAllPassable(testing.allocator, 5, 5);

    var starts_pos = [_]Coord{.{ .y = 0, .x = 0 }};
    var goals_pos = [_]Coord{.{ .y = 4, .x = 4 }};
    const starts = Config{ .positions = &starts_pos };
    const goals = Config{ .positions = &goals_pos };

    const solver = try Solver.initFromConfigs(
        testing.allocator,
        grid,
        starts,
        goals,
        false,
        42,
    );
    defer solver.deinit();

    // Step to explore some nodes
    _ = solver.stepInternal(500);
    const explored_before = solver.exploredSize();

    // Reroot back to start (already in explored)
    var start_pos = [_]Coord{.{ .y = 0, .x = 0 }};
    solver.reroot(Config{ .positions = &start_pos });

    // Should not create new node — explored size unchanged
    try testing.expectEqual(explored_before, solver.exploredSize());
}

test "rt-lacam: reroot to new config adds to explored" {
    const grid = try Grid.initAllPassable(testing.allocator, 5, 5);

    var starts_pos = [_]Coord{.{ .y = 0, .x = 0 }};
    var goals_pos = [_]Coord{.{ .y = 4, .x = 4 }};
    const starts = Config{ .positions = &starts_pos };
    const goals = Config{ .positions = &goals_pos };

    const solver = try Solver.initFromConfigs(
        testing.allocator,
        grid,
        starts,
        goals,
        false,
        42,
    );
    defer solver.deinit();

    const explored_before = solver.exploredSize();

    // Reroot to a position not yet explored
    var new_pos = [_]Coord{.{ .y = 3, .x = 3 }};
    solver.reroot(Config{ .positions = &new_pos });

    // Should have added one new node
    try testing.expectEqual(explored_before + 1, solver.exploredSize());
}

test "rt-lacam: reroot establishes bidirectional neighbor link" {
    const grid = try Grid.initAllPassable(testing.allocator, 5, 5);

    var starts_pos = [_]Coord{.{ .y = 0, .x = 0 }};
    var goals_pos = [_]Coord{.{ .y = 4, .x = 4 }};
    const starts = Config{ .positions = &starts_pos };
    const goals = Config{ .positions = &goals_pos };

    const solver = try Solver.initFromConfigs(
        testing.allocator,
        grid,
        starts,
        goals,
        false,
        42,
    );
    defer solver.deinit();

    const old_node = solver.current_node;

    var new_pos = [_]Coord{.{ .y = 1, .x = 0 }};
    solver.reroot(Config{ .positions = &new_pos });

    const new_node = solver.current_node;

    // New node should be neighbor of old node
    try testing.expect(old_node.neighbors.get(new_node) != null);
    // Old node should be neighbor of new node
    try testing.expect(new_node.neighbors.get(old_node) != null);
}

test "rt-lacam: step then reroot then step continues search" {
    const grid = try Grid.initAllPassable(testing.allocator, 5, 5);

    var starts_pos = [_]Coord{.{ .y = 0, .x = 0 }};
    var goals_pos = [_]Coord{.{ .y = 4, .x = 4 }};
    const starts = Config{ .positions = &starts_pos };
    const goals = Config{ .positions = &goals_pos };

    const solver = try Solver.initFromConfigs(
        testing.allocator,
        grid,
        starts,
        goals,
        false,
        42,
    );
    defer solver.deinit();

    // Step 1: explore
    const result1 = solver.stepInternal(200);

    if (result1) |cfg| {
        // Simulate agent moving to the suggested position
        solver.reroot(cfg);
    }

    // Step 2: continue exploring from new root
    const explored_after_reroot = solver.exploredSize();
    _ = solver.stepInternal(200);

    // Search should continue (explored may grow)
    try testing.expect(solver.exploredSize() >= explored_after_reroot);
}

test "rt-lacam: incremental solve over many small steps" {
    // Simulate RT-LaCAM usage: many small budget steps
    const grid = try Grid.initAllPassable(testing.allocator, 5, 5);

    var starts_pos = [_]Coord{.{ .y = 0, .x = 0 }};
    var goals_pos = [_]Coord{.{ .y = 4, .x = 4 }};
    const starts = Config{ .positions = &starts_pos };
    const goals = Config{ .positions = &goals_pos };

    const solver = try Solver.initFromConfigs(
        testing.allocator,
        grid,
        starts,
        goals,
        false,
        42,
    );
    defer solver.deinit();

    // Many small steps should eventually find a solution
    var found = false;
    for (0..100) |_| {
        if (solver.stepInternal(10) != null) {
            found = true;
            break;
        }
    }
    try testing.expect(found);
}

test "rt-lacam: stepFlat writes correct flat output" {
    const grid = try Grid.initAllPassable(testing.allocator, 5, 5);

    var starts_pos = [_]Coord{.{ .y = 0, .x = 0 }};
    var goals_pos = [_]Coord{.{ .y = 4, .x = 4 }};
    const starts = Config{ .positions = &starts_pos };
    const goals = Config{ .positions = &goals_pos };

    const solver = try Solver.initFromConfigs(
        testing.allocator,
        grid,
        starts,
        goals,
        false,
        42,
    );
    defer solver.deinit();

    var out_buf: [2]i32 = undefined;
    const moved = solver.stepFlat(5000, &out_buf);

    if (moved > 0) {
        // Output should be valid grid coordinates
        try testing.expect(out_buf[0] >= 0 and out_buf[0] < 5); // y
        try testing.expect(out_buf[1] >= 0 and out_buf[1] < 5); // x
    }
}

test "rt-lacam: rerootFlat matches reroot behavior" {
    const grid = try Grid.initAllPassable(testing.allocator, 5, 5);

    var starts_pos = [_]Coord{.{ .y = 0, .x = 0 }};
    var goals_pos = [_]Coord{.{ .y = 4, .x = 4 }};
    const starts = Config{ .positions = &starts_pos };
    const goals = Config{ .positions = &goals_pos };

    const solver = try Solver.initFromConfigs(
        testing.allocator,
        grid,
        starts,
        goals,
        false,
        42,
    );
    defer solver.deinit();

    // Use flat API
    const flat_pos = [_]i32{ 1, 0 }; // y=1, x=0
    solver.rerootFlat(&flat_pos);

    try testing.expect(solver.current_node.config.get(0).eql(.{ .y = 1, .x = 0 }));
}

test "rt-lacam: repeated reroot does not leak or crash" {
    const grid = try Grid.initAllPassable(testing.allocator, 5, 5);

    var starts_pos = [_]Coord{.{ .y = 0, .x = 0 }};
    var goals_pos = [_]Coord{.{ .y = 4, .x = 4 }};
    const starts = Config{ .positions = &starts_pos };
    const goals = Config{ .positions = &goals_pos };

    const solver = try Solver.initFromConfigs(
        testing.allocator,
        grid,
        starts,
        goals,
        false,
        42,
    );
    defer solver.deinit();

    // Reroot many times
    for (0..4) |y| {
        for (0..4) |x| {
            var pos = [_]Coord{.{ .y = @intCast(y), .x = @intCast(x) }};
            solver.reroot(Config{ .positions = &pos });
        }
    }

    // Should still be functional
    try testing.expect(solver.exploredSize() > 0);
    _ = solver.stepInternal(100);
}
