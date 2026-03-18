//! rt-lacam: Real-Time LaCAM for Multi-Agent Path Finding.
//!
//! Zig-powered incremental MAPF solver with completeness guarantees.
//! Reference: "Real-Time LaCAM for Real-Time MAPF" (Liang et al., SOCS 2025)

// Import all modules to trigger comptime tests
pub const deque = @import("deque.zig");
pub const config = @import("config.zig");
pub const grid = @import("grid.zig");
pub const dist_table = @import("dist_table.zig");
pub const pibt = @import("pibt.zig");
pub const low_level_node = @import("low_level_node.zig");
pub const high_level_node = @import("high_level_node.zig");
pub const solver = @import("solver.zig");
pub const traffic_map = @import("traffic_map.zig");
pub const local_guidance = @import("local_guidance.zig");

// Pull in all tests from submodules
comptime {
    _ = deque;
    _ = config;
    _ = grid;
    _ = dist_table;
    _ = pibt;
    _ = low_level_node;
    _ = high_level_node;
    _ = solver;
    _ = traffic_map;
    _ = local_guidance;
}
