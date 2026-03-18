# Changelog

## [0.2.0] - 2026-03-18

### Added

- **Lightweight Traffic Map** (`traffic_map=True`): Tracks directed edge usage
  during PIBT execution and normalizes congestion penalties to bias action
  selection away from frequently-traversed edges. Addresses oscillation in
  dense lifelong MAPF by penalizing back-and-forth movement patterns.
  Based on: Shen et al., "A Lightweight Traffic Map for Efficient Anytime
  LaCAM*" (arXiv:2603.07891, 2026).

- **Local Guidance** (`local_guidance=True`): Computes per-agent directional
  hints using sequential reservation-based conflict resolution. Each agent
  receives a preferred next-step that avoids directing multiple agents to the
  same cell. PIBT's candidate sorting prioritizes guidance-matching moves.
  Based on: Arita & Okumura, "Local Guidance for Configuration-Based
  Multi-Agent Pathfinding" (arXiv:2510.19072, 2025).

- New C ABI export `rtlacam_create_guided()` accepting `flg_traffic_map` and
  `flg_local_guidance` boolean flags alongside existing parameters.

- New Zig modules: `traffic_map.zig`, `local_guidance.zig` with unit tests.

- `SortCtx` and `guidedLessThan` in PIBT for traffic-map and guidance-aware
  candidate ordering. Falls back to original `compareDist` when both features
  are disabled.

### Changed

- PIBT struct now carries optional `traffic_map` and `guidance_hints` fields
  (default `null`, backward compatible).

- Solver records committed actions to the traffic map after each PIBT
  configuration generation.

- Solver computes local guidance hints at the start of each `stepInternal()`
  call from the current physical position.

### Backward Compatibility

- Existing `rtlacam_create` and `rtlacam_create_seeded` C ABI functions are
  unchanged. All existing tests pass without modification.

- Python: `RTLaCAM()` constructor accepts new keyword-only arguments
  `traffic_map=False` and `local_guidance=False`. Default behavior is
  identical to v0.1.2.

## [0.1.2] - 2025-12-01

- Initial release with RT-LaCAM incremental DFS solver.
- Python bindings via cffi.
- Persistent search state across `step()` calls with `reroot()`.
- LaCAM* mode (`flg_star=True`) with Dijkstra cost refinement.
