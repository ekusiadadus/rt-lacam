# rt-lacam

Real-Time LaCAM: an incremental Multi-Agent Path Finding (MAPF) solver with completeness guarantees.

High-performance Zig core with Python bindings via cffi.

## Overview

**rt-lacam** implements the Real-Time LaCAM algorithm, which enables incremental, anytime MAPF solving with persistent search state across planning steps. Unlike naive per-step replanning (which discards the search tree each iteration), RT-LaCAM preserves the explored configuration graph and re-roots it after agent execution, achieving theoretical completeness even under strict per-step time budgets.

### Key Features

- **Incremental DFS** with configurable per-step time budgets (milliseconds)
- **Persistent search state** across step() calls — the core innovation over naive replanning
- **Re-rooting** after agent execution to continue search from actual positions
- **LaCAM\* mode** with Dijkstra cost refinement for improved solution quality
- **PIBT** (Priority Inheritance with Backtracking) as the configuration generator
- **Lazy BFS distance tables** for efficient heuristic computation
- **Zero-copy C ABI** for integration with any language via FFI
- **~120KB** native library (ReleaseFast), no runtime dependencies

## Installation

### From PyPI

```bash
pip install rt-lacam
```

> **Note:** Pre-built wheels are currently available for macOS (Apple Silicon). Other platforms will build from source (requires Zig >= 0.15.0).

### From Source (requires [Zig >= 0.15.0](https://ziglang.org/download/))

```bash
git clone https://github.com/ekusiadadus/rt-lacam.git
cd rt-lacam
zig build                          # build native library
zig build test --summary all       # run 86 Zig tests
pip install -e ".[dev]"            # install Python package with dev deps
pytest -v                          # run 29 Python tests
```

### Build Wheel

```bash
zig build -Doptimize=ReleaseFast
cp zig-out/lib/librt_lacam.* python/rt_lacam/
pip install .
```

## Quick Start

```python
from rt_lacam import RTLaCAM

# 5x5 open grid, 1 agent
grid = [[1]*5 for _ in range(5)]

with RTLaCAM(grid, starts=[(0, 0)], goals=[(4, 4)]) as solver:
    pos = [(0, 0)]
    for step in range(100):
        # Incremental DFS with 50ms budget per step
        result = solver.step(deadline_ms=50)
        if result is not None:
            pos = result
            solver.reroot(pos)  # re-root search tree
        # Check if agents are physically at goals (not just path found)
        if solver.is_solved(pos):
            print(f"All agents reached goals at step {step}!")
            break
```

### Multi-Agent Example

```python
from rt_lacam import RTLaCAM

grid = [[1]*10 for _ in range(10)]  # 10x10 open grid

solver = RTLaCAM(
    grid,
    starts=[(0, 0), (0, 9), (9, 0)],
    goals=[(9, 9), (9, 0), (0, 9)],
    flg_star=True,  # LaCAM* for better solution quality
)

positions = [(0, 0), (0, 9), (9, 0)]
for _ in range(200):
    result = solver.step(deadline_ms=100)
    if result:
        positions = result
        solver.reroot(positions)
    if solver.is_solved(positions):
        break

solver.close()
```

## API Reference

### `RTLaCAM(grid, starts, goals, *, flg_star=False, seed=None)`

Create a solver instance.

- **grid**: 2D list of int (1=passable, 0=obstacle)
- **starts**: list of (y, x) tuples — initial agent positions
- **goals**: list of (y, x) tuples — target positions
- **flg_star**: enable LaCAM\* cost refinement (default: False)
- **seed**: PRNG seed for reproducibility (default: None)

### `solver.step(deadline_ms=100) -> list[(y,x)] | None`

Run incremental DFS for up to `deadline_ms` milliseconds. Returns next positions or None if no solution found yet.

### `solver.reroot(current_positions)`

Re-root the search tree after agents have moved. Call this with the actual new positions.

### `solver.is_solved(current_positions) -> bool`

Whether all agents are physically at their respective goal positions. Use this to determine when to stop the planning loop.

### `solver.has_goal -> bool`

Whether a goal configuration has been found in the search tree. **Note:** this does NOT mean agents have reached the goal — it means the solver has found a path. Use `is_solved()` to check physical arrival.

### `solver.explored_size -> int`

Number of explored configurations.

## Architecture

```
rt-lacam/
├── src/                     # Zig core (~3000 lines, 86 tests)
│   ├── solver.zig           # RT-LaCAM: incremental DFS + reroot
│   ├── pibt.zig             # PIBT configuration generator
│   ├── high_level_node.zig  # Configuration-space search node
│   ├── low_level_node.zig   # Constraint DFS node
│   ├── dist_table.zig       # Lazy BFS distance table
│   ├── grid.zig             # 2D obstacle grid
│   ├── config.zig           # Coord/Config types with hashing
│   ├── deque.zig            # Generic ring-buffer deque
│   └── exports.zig          # C ABI exports (9 functions)
├── python/rt_lacam/         # Python bindings via cffi
│   ├── __init__.py
│   └── _bindings.py         # RTLaCAM class
└── tests/                   # Python integration tests
```

## Algorithm

RT-LaCAM (Liang et al., 2025) extends LaCAM (Okumura, 2023) for real-time, lifelong MAPF:

1. **LaCAM** performs a two-level search in configuration space. The high-level DFS explores joint configurations; the low-level constraint DFS generates candidate next configurations via PIBT.

2. **RT-LaCAM** adds three key mechanisms:
   - **Persistent state**: The explored graph and open list survive across planning steps.
   - **Time-bounded iteration**: Each `step()` call runs DFS only within a millisecond budget, returning the best known next configuration.
   - **Re-rooting**: After agents execute one step, `reroot()` updates the search tree's root to the actual configuration, enabling continued exploration.

3. **Completeness**: Because the search tree grows monotonically across steps and no explored state is discarded, RT-LaCAM is provably complete — if a solution exists, it will eventually be found (given sufficient total computation).

## References

This implementation is based on the following papers. We are grateful to the authors for their foundational research in MAPF:

1. **Runzhe Liang, Rishi Veerapaneni, Daniel Harabor, Jiaoyang Li, Maxim Likhachev.** "Real-Time LaCAM for Real-Time MAPF." *International Symposium on Combinatorial Search (SoCS)*, 2025. [arXiv:2504.06091](https://arxiv.org/abs/2504.06091)
   — The primary reference for the real-time incremental DFS with re-rooting and completeness guarantees.

2. **Keisuke Okumura.** "LaCAM: Search-Based Algorithm for Quick Multi-Agent Pathfinding." *AAAI Conference on Artificial Intelligence*, 2023. [arXiv:2211.13432](https://arxiv.org/abs/2211.13432)
   — The original LaCAM algorithm: two-level lazy DFS in configuration space with PIBT.

3. **Keisuke Okumura.** "Improving LaCAM for Scalable Eventually Optimal Multi-Agent Pathfinding." *IJCAI*, 2023. [arXiv:2305.03632](https://arxiv.org/abs/2305.03632)
   — LaCAM\* extension with Dijkstra cost refinement for improved solution quality.

4. **Keisuke Okumura, Manao Machida, Xavier Défago, Yasumasa Tamura.** "Priority Inheritance with Backtracking for Iterative Multi-agent Path Finding." *Artificial Intelligence*, 2022. [arXiv:1901.11282](https://arxiv.org/abs/1901.11282)
   — PIBT: the single-step configuration generator used inside LaCAM.

5. **He Jiang, Yulun Zhang, Rishi Veerapaneni, Jiaoyang Li.** "Scaling Lifelong Multi-Agent Path Finding to More Realistic Settings: Research Challenges and Opportunities." *Symposium on Combinatorial Search (SoCS)*, 2024. [arXiv:2404.16162](https://arxiv.org/abs/2404.16162)
   — Analysis of windowed MAPF completeness and the need for persistent search state in lifelong settings.

6. **Bojie Shen, Yue Zhang, Zhe Chen, Daniel Harabor.** "A Lightweight Traffic Map for Efficient Anytime LaCAM*." 2026. [arXiv:2603.07891](https://arxiv.org/abs/2603.07891)
   — Dynamic congestion guidance for LaCAM\*, showing continued research interest in the LaCAM family.

## Acknowledgments

This project was inspired by the excellent work of:

- **Keisuke Okumura** ([@Kei18](https://github.com/Kei18)) for the original LaCAM and [py-lacam](https://github.com/Kei18/py-lacam) reference implementation.
- **Runzhe Liang, Rishi Veerapaneni, Daniel Harabor, Jiaoyang Li, and Maxim Likhachev** for the Real-Time LaCAM paper that made incremental lifelong MAPF practical.
- **Jiaoyang Li** and the MAPF research community for their continued work on scalable multi-agent coordination.

The Zig implementation was built from scratch following the algorithm descriptions in the above papers, with the [py-lacam](https://github.com/Kei18/py-lacam) Python implementation serving as a reference for correctness verification.

## License

MIT License. See [LICENSE](LICENSE) for details.
