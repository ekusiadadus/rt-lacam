"""Low-level cffi bindings to librt_lacam."""

from __future__ import annotations

import ctypes
import os
import platform
from pathlib import Path
from typing import Optional, Sequence

import cffi

_CDEF = """
    uint32_t rtlacam_version(void);

    void* rtlacam_create(
        const uint8_t* grid, uint32_t width, uint32_t height,
        const int32_t* starts, const int32_t* goals,
        uint32_t n_agents, bool flg_star
    );

    void* rtlacam_create_seeded(
        const uint8_t* grid, uint32_t width, uint32_t height,
        const int32_t* starts, const int32_t* goals,
        uint32_t n_agents, bool flg_star, uint64_t seed
    );

    void rtlacam_destroy(void* solver);

    int32_t rtlacam_step(
        void* solver, uint32_t deadline_ms, int32_t* out
    );

    void rtlacam_reroot(void* solver, const int32_t* current);

    uint32_t rtlacam_explored_size(void* solver);
    uint32_t rtlacam_num_agents(void* solver);
    bool rtlacam_has_goal(void* solver);
"""


def _find_lib() -> Path:
    """Locate librt_lacam shared library."""
    # Check environment override first
    env_path = os.environ.get("RTLACAM_LIB")
    if env_path:
        p = Path(env_path)
        if p.exists():
            return p

    # Platform-specific library name
    system = platform.system()
    if system == "Darwin":
        lib_name = "librt_lacam.dylib"
    elif system == "Linux":
        lib_name = "librt_lacam.so"
    elif system == "Windows":
        lib_name = "rt_lacam.dll"
    else:
        lib_name = "librt_lacam.so"

    # Search relative to this file (development layout)
    pkg_dir = Path(__file__).resolve().parent
    candidates = [
        pkg_dir / lib_name,  # installed alongside Python package
        pkg_dir.parent.parent / "zig-out" / "lib" / lib_name,  # dev build
    ]

    for candidate in candidates:
        if candidate.exists():
            return candidate

    raise FileNotFoundError(
        f"Cannot find {lib_name}. Build with `zig build` first or set "
        f"RTLACAM_LIB environment variable."
    )


def _load_lib():
    """Load the shared library via cffi."""
    ffi = cffi.FFI()
    ffi.cdef(_CDEF)
    lib_path = _find_lib()
    lib = ffi.dlopen(str(lib_path))
    return ffi, lib


_ffi, _lib = _load_lib()


Coord = tuple[int, int]  # (y, x)


class RTLaCAM:
    """Real-Time LaCAM solver.

    Maintains persistent search state across step() calls.
    Each step() runs incremental DFS within a time budget,
    and reroot() updates the search tree root after agent execution.

    Example::

        grid = [[1,1,1],[1,1,1],[1,1,1]]  # 3x3 open
        solver = RTLaCAM(grid, starts=[(0,0)], goals=[(2,2)])
        while not solver.has_goal:
            next_pos = solver.step(deadline_ms=50)
            if next_pos:
                # Execute movement
                solver.reroot(actual_positions)
    """

    def __init__(
        self,
        grid: Sequence[Sequence[int]],
        starts: Sequence[Coord],
        goals: Sequence[Coord],
        *,
        flg_star: bool = False,
        seed: Optional[int] = None,
    ) -> None:
        height = len(grid)
        width = len(grid[0]) if height > 0 else 0
        n_agents = len(starts)

        if len(goals) != n_agents:
            raise ValueError(
                f"starts ({n_agents}) and goals ({len(goals)}) length mismatch"
            )
        if height == 0 or width == 0:
            raise ValueError("Grid must have at least 1 row and 1 column")
        if n_agents == 0:
            raise ValueError("Must have at least 1 agent")

        # Validate coordinates are within grid bounds and on passable cells
        for i, (sy, sx) in enumerate(starts):
            if not (0 <= sy < height and 0 <= sx < width):
                raise ValueError(
                    f"Start position {i} ({sy}, {sx}) is outside "
                    f"grid bounds ({height}x{width})"
                )
            if not grid[sy][sx]:
                raise ValueError(
                    f"Start position {i} ({sy}, {sx}) is on an obstacle"
                )
        for i, (gy, gx) in enumerate(goals):
            if not (0 <= gy < height and 0 <= gx < width):
                raise ValueError(
                    f"Goal position {i} ({gy}, {gx}) is outside "
                    f"grid bounds ({height}x{width})"
                )
            if not grid[gy][gx]:
                raise ValueError(
                    f"Goal position {i} ({gy}, {gx}) is on an obstacle"
                )

        # Validate no duplicate starts (physical constraint)
        start_set = set(tuple(s) for s in starts)
        if len(start_set) != n_agents:
            raise ValueError("Duplicate start positions are not allowed")
        # Duplicate goals are allowed — idle agents may share goals
        # with active agents (stay-goal model for lifelong MAPF)

        # Flatten grid to uint8 array
        grid_buf = _ffi.new("uint8_t[]", width * height)
        for r in range(height):
            for c in range(width):
                grid_buf[r * width + c] = 1 if grid[r][c] else 0

        # Flatten starts/goals to int32 [y0,x0,y1,x1,...]
        starts_buf = _ffi.new("int32_t[]", n_agents * 2)
        goals_buf = _ffi.new("int32_t[]", n_agents * 2)
        for i, (sy, sx) in enumerate(starts):
            starts_buf[i * 2] = sy
            starts_buf[i * 2 + 1] = sx
        for i, (gy, gx) in enumerate(goals):
            goals_buf[i * 2] = gy
            goals_buf[i * 2 + 1] = gx

        if seed is not None:
            self._handle = _lib.rtlacam_create_seeded(
                grid_buf, width, height,
                starts_buf, goals_buf,
                n_agents, flg_star, seed,
            )
        else:
            self._handle = _lib.rtlacam_create(
                grid_buf, width, height,
                starts_buf, goals_buf,
                n_agents, flg_star,
            )

        if self._handle == _ffi.NULL:
            raise MemoryError("Failed to create RT-LaCAM solver")

        self._n_agents = n_agents
        self._goals = list(goals)
        self._out_buf = _ffi.new("int32_t[]", n_agents * 2)

    def __del__(self) -> None:
        self.close()

    def close(self) -> None:
        """Destroy solver and free native memory."""
        if hasattr(self, "_handle") and self._handle != _ffi.NULL:
            _lib.rtlacam_destroy(self._handle)
            self._handle = _ffi.NULL

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    def step(self, deadline_ms: int = 100) -> Optional[list[Coord]]:
        """Run incremental DFS for up to deadline_ms milliseconds.

        Returns list of (y, x) next positions, or None if no solution yet.
        """
        if self._handle == _ffi.NULL:
            raise RuntimeError("Solver has been destroyed")

        moved = _lib.rtlacam_step(self._handle, deadline_ms, self._out_buf)

        if moved <= 0:
            return None

        result = []
        for i in range(self._n_agents):
            y = self._out_buf[i * 2]
            x = self._out_buf[i * 2 + 1]
            result.append((y, x))
        return result

    def reroot(self, current: Sequence[Coord]) -> None:
        """Re-root search tree to actual agent positions.

        Call after agents have executed one step.
        """
        if self._handle == _ffi.NULL:
            raise RuntimeError("Solver has been destroyed")
        if len(current) != self._n_agents:
            raise ValueError(
                f"Expected {self._n_agents} positions, got {len(current)}"
            )

        buf = _ffi.new("int32_t[]", self._n_agents * 2)
        for i, (y, x) in enumerate(current):
            buf[i * 2] = y
            buf[i * 2 + 1] = x
        _lib.rtlacam_reroot(self._handle, buf)

    @property
    def explored_size(self) -> int:
        """Number of explored configurations."""
        if self._handle == _ffi.NULL:
            return 0
        return _lib.rtlacam_explored_size(self._handle)

    @property
    def num_agents(self) -> int:
        """Number of agents."""
        return self._n_agents

    @property
    def has_goal(self) -> bool:
        """Whether a goal configuration has been found in the search tree.

        NOTE: This does NOT mean agents have physically reached the goal.
        It means the solver has found a path to the goal configuration.
        Use is_solved(current_positions) to check if agents are at goals.
        """
        if self._handle == _ffi.NULL:
            return False
        return bool(_lib.rtlacam_has_goal(self._handle))

    def is_solved(self, current: Sequence[Coord]) -> bool:
        """Check if agents are physically at their goal positions.

        Args:
            current: actual agent positions as list of (y, x) tuples.

        Returns:
            True if all agents are at their respective goals.
        """
        if len(current) != self._n_agents:
            return False
        for (cy, cx), (gy, gx) in zip(current, self._goals):
            if cy != gy or cx != gx:
                return False
        return True

    @staticmethod
    def version() -> str:
        """Return library version string."""
        v = _lib.rtlacam_version()
        major = (v >> 16) & 0xFF
        minor = (v >> 8) & 0xFF
        patch = v & 0xFF
        return f"{major}.{minor}.{patch}"
