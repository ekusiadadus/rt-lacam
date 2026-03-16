"""Tests for rt-lacam Python bindings."""

import sys
from pathlib import Path

import pytest

# Add python/ to path for development
sys.path.insert(0, str(Path(__file__).parent.parent / "python"))

from rt_lacam import RTLaCAM


class TestVersion:
    def test_version_string(self):
        assert RTLaCAM.version() == "0.1.0"


class TestSingleAgent:
    def test_single_agent_reaches_goal(self):
        grid = [[1] * 5 for _ in range(5)]
        with RTLaCAM(grid, starts=[(0, 0)], goals=[(4, 4)]) as solver:
            result = solver.step(deadline_ms=5000)
            assert result is not None
            assert len(result) == 1
            y, x = result[0]
            assert 0 <= y < 5
            assert 0 <= x < 5

    def test_agent_already_at_goal(self):
        grid = [[1] * 3 for _ in range(3)]
        with RTLaCAM(grid, starts=[(1, 1)], goals=[(1, 1)]) as solver:
            result = solver.step(deadline_ms=5000)
            # Already at goal — returns None (no move needed)
            assert result is None
            assert solver.has_goal


class TestTwoAgents:
    def test_two_agents_swap_on_open_grid(self):
        grid = [[1] * 5 for _ in range(5)]
        with RTLaCAM(
            grid,
            starts=[(0, 0), (4, 4)],
            goals=[(4, 4), (0, 0)],
        ) as solver:
            result = solver.step(deadline_ms=5000)
            assert result is not None
            assert len(result) == 2
            # No vertex collision
            assert result[0] != result[1]

    def test_unsolvable_1wide_corridor(self):
        grid = [[1] * 5]  # 1x5 corridor
        with RTLaCAM(
            grid,
            starts=[(0, 0), (0, 4)],
            goals=[(0, 4), (0, 0)],
        ) as solver:
            found = False
            for _ in range(20):
                if solver.step(deadline_ms=100) is not None:
                    found = True
                    break
            assert not found


class TestReroot:
    def test_reroot_updates_state(self):
        grid = [[1] * 5 for _ in range(5)]
        with RTLaCAM(grid, starts=[(0, 0)], goals=[(4, 4)]) as solver:
            result = solver.step(deadline_ms=1000)
            if result:
                explored_before = solver.explored_size
                solver.reroot(result)
                # After reroot, exploration continues
                solver.step(deadline_ms=100)
                assert solver.explored_size >= explored_before

    def test_step_reroot_cycle(self):
        """Simulate the real RT-LaCAM usage loop."""
        grid = [[1] * 5 for _ in range(5)]
        with RTLaCAM(grid, starts=[(0, 0)], goals=[(4, 4)], seed=42) as solver:
            pos = [(0, 0)]
            for _ in range(20):
                result = solver.step(deadline_ms=200)
                if result is not None:
                    pos = result
                    solver.reroot(pos)
                if solver.has_goal:
                    break


class TestProperties:
    def test_explored_size_grows(self):
        grid = [[1] * 5 for _ in range(5)]
        with RTLaCAM(grid, starts=[(0, 0)], goals=[(4, 4)]) as solver:
            initial = solver.explored_size
            solver.step(deadline_ms=500)
            assert solver.explored_size >= initial

    def test_num_agents(self):
        grid = [[1] * 5 for _ in range(5)]
        with RTLaCAM(
            grid,
            starts=[(0, 0), (1, 1), (2, 2)],
            goals=[(4, 4), (3, 3), (0, 0)],
        ) as solver:
            assert solver.num_agents == 3


class TestContextManager:
    def test_double_close_safe(self):
        grid = [[1] * 3 for _ in range(3)]
        solver = RTLaCAM(grid, starts=[(0, 0)], goals=[(2, 2)])
        solver.close()
        solver.close()  # Should not crash

    def test_use_after_close_raises(self):
        grid = [[1] * 3 for _ in range(3)]
        solver = RTLaCAM(grid, starts=[(0, 0)], goals=[(2, 2)])
        solver.close()
        with pytest.raises(RuntimeError):
            solver.step()


class TestValidation:
    def test_empty_grid_raises(self):
        with pytest.raises(ValueError, match="at least 1"):
            RTLaCAM([], starts=[(0, 0)], goals=[(0, 0)])

    def test_start_out_of_bounds_raises(self):
        grid = [[1] * 3 for _ in range(3)]
        with pytest.raises(ValueError, match="outside grid bounds"):
            RTLaCAM(grid, starts=[(5, 0)], goals=[(0, 0)])

    def test_goal_out_of_bounds_raises(self):
        grid = [[1] * 3 for _ in range(3)]
        with pytest.raises(ValueError, match="outside grid bounds"):
            RTLaCAM(grid, starts=[(0, 0)], goals=[(0, 10)])

    def test_reroot_wrong_length_raises(self):
        grid = [[1] * 3 for _ in range(3)]
        with RTLaCAM(grid, starts=[(0, 0)], goals=[(2, 2)]) as solver:
            with pytest.raises(ValueError, match="Expected 1"):
                solver.reroot([(0, 0), (1, 1)])

    def test_mismatched_starts_goals_raises(self):
        grid = [[1] * 3 for _ in range(3)]
        with pytest.raises(ValueError, match="mismatch"):
            RTLaCAM(grid, starts=[(0, 0)], goals=[(1, 1), (2, 2)])

    def test_start_on_obstacle_raises(self):
        grid = [[1, 0], [1, 1]]
        with pytest.raises(ValueError, match="obstacle"):
            RTLaCAM(grid, starts=[(0, 1)], goals=[(1, 1)])

    def test_goal_on_obstacle_raises(self):
        grid = [[1, 0], [1, 1]]
        with pytest.raises(ValueError, match="obstacle"):
            RTLaCAM(grid, starts=[(0, 0)], goals=[(0, 1)])

    def test_duplicate_starts_raises(self):
        grid = [[1] * 3 for _ in range(3)]
        with pytest.raises(ValueError, match="Duplicate start"):
            RTLaCAM(grid, starts=[(0, 0), (0, 0)], goals=[(2, 2), (1, 1)])

    def test_duplicate_goals_raises(self):
        grid = [[1] * 3 for _ in range(3)]
        with pytest.raises(ValueError, match="Duplicate goal"):
            RTLaCAM(grid, starts=[(0, 0), (1, 1)], goals=[(2, 2), (2, 2)])


class TestIsSolved:
    def test_is_solved_when_at_goal(self):
        grid = [[1] * 3 for _ in range(3)]
        with RTLaCAM(grid, starts=[(0, 0)], goals=[(2, 2)]) as solver:
            assert not solver.is_solved([(0, 0)])
            assert solver.is_solved([(2, 2)])

    def test_is_solved_multi_agent(self):
        grid = [[1] * 5 for _ in range(5)]
        with RTLaCAM(
            grid,
            starts=[(0, 0), (4, 4)],
            goals=[(4, 4), (0, 0)],
        ) as solver:
            assert not solver.is_solved([(0, 0), (4, 4)])
            assert not solver.is_solved([(4, 4), (4, 4)])
            assert solver.is_solved([(4, 4), (0, 0)])

    def test_is_solved_wrong_length_returns_false(self):
        grid = [[1] * 3 for _ in range(3)]
        with RTLaCAM(grid, starts=[(0, 0)], goals=[(2, 2)]) as solver:
            assert not solver.is_solved([(0, 0), (1, 1)])

    def test_has_goal_vs_is_solved_semantics(self):
        """has_goal means path found in search tree, NOT physical arrival."""
        grid = [[1] * 5 for _ in range(5)]
        with RTLaCAM(grid, starts=[(0, 0)], goals=[(4, 4)], seed=42) as solver:
            result = solver.step(deadline_ms=5000)
            if solver.has_goal:
                # has_goal can be True even when agent hasn't moved to goal
                assert result is not None
                # Agent just took one step — unlikely to be at (4,4) already
                if result[0] != (4, 4):
                    assert not solver.is_solved(result)


class TestLaCAMStar:
    def test_lacam_star_finds_solution(self):
        grid = [[1] * 5 for _ in range(5)]
        with RTLaCAM(
            grid,
            starts=[(0, 0)],
            goals=[(4, 4)],
            flg_star=True,
        ) as solver:
            result = solver.step(deadline_ms=5000)
            assert result is not None
