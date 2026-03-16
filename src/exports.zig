//! C ABI exports for Python FFI (cffi).
//!
//! All exported functions use C calling convention and flat
//! array representations for cross-language compatibility.
//!
//! Usage from Python (cffi):
//!   ffi.cdef("""
//!     uint32_t rtlacam_version(void);
//!     void* rtlacam_create(const uint8_t* grid, uint32_t w, uint32_t h,
//!                          const int32_t* starts, const int32_t* goals,
//!                          uint32_t n_agents, bool flg_star);
//!     void rtlacam_destroy(void* solver);
//!     int32_t rtlacam_step(void* solver, uint32_t deadline_ms, int32_t* out);
//!     void rtlacam_reroot(void* solver, const int32_t* current);
//!     uint32_t rtlacam_explored_size(void* solver);
//!   """)

const std = @import("std");
const Solver = @import("solver.zig").Solver;

/// Return library version as packed integer: 0xMMmmpp (major, minor, patch).
export fn rtlacam_version() callconv(.c) u32 {
    return 0x000100; // 0.1.0
}

/// Create a new RT-LaCAM solver instance.
///
/// Parameters:
///   grid_ptr:  flat row-major u8 array (1=passable, 0=obstacle), length = w * h
///   width, height: grid dimensions
///   starts_ptr: flat i32 array [y0,x0,y1,x1,...], length = n_agents * 2
///   goals_ptr:  flat i32 array [y0,x0,y1,x1,...], length = n_agents * 2
///   n_agents:   number of agents
///   flg_star:   true = LaCAM* (Dijkstra cost refinement), false = LaCAM
///
/// Returns: opaque solver handle, or null on invalid input / allocation failure.
export fn rtlacam_create(
    grid_ptr: [*]const u8,
    width: u32,
    height: u32,
    starts_ptr: [*]const i32,
    goals_ptr: [*]const i32,
    n_agents: u32,
    flg_star: bool,
) callconv(.c) ?*Solver {
    if (!validateInput(width, height, n_agents)) return null;
    return Solver.initFromFlat(
        std.heap.c_allocator,
        grid_ptr,
        width,
        height,
        starts_ptr,
        goals_ptr,
        n_agents,
        flg_star,
        0,
    ) catch null;
}

/// Create solver with explicit PRNG seed (for reproducible tests).
export fn rtlacam_create_seeded(
    grid_ptr: [*]const u8,
    width: u32,
    height: u32,
    starts_ptr: [*]const i32,
    goals_ptr: [*]const i32,
    n_agents: u32,
    flg_star: bool,
    seed: u64,
) callconv(.c) ?*Solver {
    if (!validateInput(width, height, n_agents)) return null;
    return Solver.initFromFlat(
        std.heap.c_allocator,
        grid_ptr,
        width,
        height,
        starts_ptr,
        goals_ptr,
        n_agents,
        flg_star,
        seed,
    ) catch null;
}

fn validateInput(width: u32, height: u32, n_agents: u32) bool {
    if (width == 0 or height == 0 or n_agents == 0) return false;
    // Prevent grid size overflow (max ~1M cells)
    if (@as(u64, width) * @as(u64, height) > 1_000_000) return false;
    // Prevent n_agents * 2 overflow in flat array indexing
    if (n_agents > std.math.maxInt(u32) / 2) return false;
    return true;
}

/// Destroy solver and free all memory.
export fn rtlacam_destroy(solver: ?*Solver) callconv(.c) void {
    if (solver) |s| {
        s.deinit();
    }
}

/// Run one incremental DFS step within deadline_ms milliseconds.
///
/// Writes next agent positions to out_ptr as flat [y0,x0,y1,x1,...].
/// Returns:
///   >0 = number of agents that moved
///    0 = no solution found yet (time budget exhausted)
///   -1 = error (null solver)
export fn rtlacam_step(
    solver: ?*Solver,
    deadline_ms: u32,
    out_ptr: [*]i32,
) callconv(.c) i32 {
    const s = solver orelse return -1;
    return s.stepFlat(deadline_ms, out_ptr);
}

/// Re-root the search tree after agents have moved.
///
/// Call this after agents execute one step to inform the solver
/// of the actual new positions (which may differ from planned).
///
/// current_ptr: flat i32 array [y0,x0,y1,x1,...] of actual positions.
export fn rtlacam_reroot(
    solver: ?*Solver,
    current_ptr: [*]const i32,
) callconv(.c) void {
    const s = solver orelse return;
    s.rerootFlat(current_ptr);
}

/// Return number of explored configurations in the search tree.
export fn rtlacam_explored_size(solver: ?*Solver) callconv(.c) u32 {
    const s = solver orelse return 0;
    return s.exploredSize();
}

/// Return number of agents.
export fn rtlacam_num_agents(solver: ?*Solver) callconv(.c) u32 {
    const s = solver orelse return 0;
    return s.num_agents;
}

/// Check if goal has been found in current search tree.
export fn rtlacam_has_goal(solver: ?*Solver) callconv(.c) bool {
    const s = solver orelse return false;
    return s.goal_node != null;
}
