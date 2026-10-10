"""
Extra tests I added on top of the provided ones (tests/ provided files are untouched).

Run:  python -m unittest tests.test_my_planner -v

Each test pins down a design decision the provided tests don't cover, so a
future change that breaks one of those decisions fails loudly.
"""
import math
import unittest

from grid_utils import Grid, inflate
from replan import next_waypoint_index, path_is_valid
from search import astar, heuristic
from sim.grid_values import FREE, OCCUPIED, UNKNOWN

SQ2 = math.sqrt(2.0)


def arena() -> Grid:
    """Same geometry as the simulator: 60 x 60 cells of 0.5 m, lower-left corner at (-15, -15)."""
    return Grid(60, 60, 0.5, -15.0, -15.0, [FREE] * 3600, [1.0] * 3600)


def cells_to_world(grid, cells):
    return [grid.cell_to_world(r, c) for r, c in cells]


class TestInflationMatchesScorer(unittest.TestCase):
    """
    scoring.py calls a path "not fine" if a waypoint is ON or directly BESIDE
    (up/down/left/right) an occupied cell. With robot_radius_m = 0.5 and 0.5 m
    cells, my inflation should mark exactly that cross shape, so path_is_valid
    on the inflated grid agrees with the scorer.
    """

    def test_radius_one_cell_marks_exactly_the_cross(self):
        g = arena()
        g.occupancy[g.index(30, 30)] = OCCUPIED
        out = inflate(g, 0.5)
        marked = {(r, c) for r in range(60) for c in range(60) if out.is_occupied(r, c)}
        self.assertEqual(marked, {(30, 30), (29, 30), (31, 30), (30, 29), (30, 31)})

    def test_diagonal_neighbour_is_left_free_and_that_is_safe(self):
        # Rover centre at the diagonal neighbour's centre is sqrt(2)*0.25 = 0.354 m
        # from the obstacle square's nearest corner. world.py's BODY_RADIUS_M is 0.3,
        # so this is collision-free even though inflation leaves it free.
        g = arena()
        g.occupancy[g.index(30, 30)] = OCCUPIED
        out = inflate(g, 0.5)
        self.assertFalse(out.is_occupied(31, 31))
        corner_dist = math.hypot(0.25, 0.25)
        self.assertGreater(corner_dist, 0.3)


class TestUnknownIsFreeOnTickZero(unittest.TestCase):
    """Why the node passes unknown_is_free=True: on tick 0 the goal is unknown."""

    def setUp(self):
        # Everything unknown except an 8 m circle around the start, like tick 0.
        g = arena()
        g.occupancy = [UNKNOWN] * 3600
        sx, sy = -13.2, -12.7
        for r in range(60):
            for c in range(60):
                x, y = g.cell_to_world(r, c)
                if math.hypot(x - sx, y - sy) <= 8.0:
                    g.occupancy[g.index(r, c)] = FREE
        self.g = g
        self.start = g.world_to_cell(sx, sy)
        self.goal = g.world_to_cell(12.8, 13.3)

    def test_unknown_blocked_means_no_route_at_all(self):
        self.assertIsNone(astar(self.g, self.start, self.goal, unknown_is_free=False))

    def test_unknown_free_gives_the_straight_diagonal(self):
        path = astar(self.g, self.start, self.goal, unknown_is_free=True)
        self.assertIsNotNone(path)
        self.assertAlmostEqual(len(path) - 1, 52)                    # 52 diagonal steps
        self.assertAlmostEqual(heuristic(self.start, self.goal), 52 * SQ2)


class TestRoverInsideInflatedZone(unittest.TestCase):
    """
    The case behind checking validity from idx + 1 in the node: the rover drives
    past a rock and ends up standing in an inflated cell.
    """

    def setUp(self):
        g = arena()
        g.occupancy[g.index(10, 11)] = OCCUPIED      # rock right next to the rover
        self.g = inflate(g, 0.5)
        self.start = (10, 10)                        # inflated: beside the rock
        self.goal = (10, 30)

    def test_start_cell_is_inflated(self):
        self.assertTrue(self.g.is_occupied(*self.start))

    def test_astar_can_still_leave(self):
        path = astar(self.g, self.start, self.goal, unknown_is_free=True)
        self.assertIsNotNone(path)
        self.assertEqual(path[0], self.start)

    def test_checking_from_rover_cell_would_replan_every_tick(self):
        path_xy = cells_to_world(self.g, astar(self.g, self.start, self.goal, unknown_is_free=True))
        idx = next_waypoint_index(path_xy, path_xy[0])
        self.assertEqual(idx, 0)
        # The brand-new path looks "invalid" if the rover's own cell is checked...
        self.assertFalse(path_is_valid(self.g, path_xy, idx))
        # ...and valid if checking starts at the next waypoint, which is what the node does.
        self.assertTrue(path_is_valid(self.g, path_xy, idx + 1))


class TestReplanTriggerIgnoresBehind(unittest.TestCase):
    """The popup scenario's 'rock behind': an obstacle on waypoints already passed."""

    def test_rock_on_driven_part_does_not_invalidate(self):
        g = arena()
        cells = [(5, c) for c in range(5, 40)]
        path_xy = cells_to_world(g, cells)
        g.occupancy[g.index(5, 10)] = OCCUPIED       # on waypoint 5
        g = inflate(g, 0.5)
        rover = path_xy[20]                          # rover has driven past it
        idx = next_waypoint_index(path_xy, rover)
        self.assertTrue(path_is_valid(g, path_xy, idx + 1))

    def test_rock_ahead_does_invalidate(self):
        g = arena()
        cells = [(5, c) for c in range(5, 40)]
        path_xy = cells_to_world(g, cells)
        g.occupancy[g.index(6, 30)] = OCCUPIED       # beside waypoint 25, ahead
        g = inflate(g, 0.5)
        idx = next_waypoint_index(path_xy, path_xy[20])
        self.assertFalse(path_is_valid(g, path_xy, idx + 1))


class TestHeuristicIsConsistent(unittest.TestCase):
    """
    A* only skips re-opening closed cells safely if h is CONSISTENT:
    h(a) <= step_cost(a, b) + h(b) for every neighbour b. Admissible alone is not enough.
    """

    def test_consistent_for_every_move(self):
        goal = (7, 3)
        moves = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                 (1, 1, SQ2), (1, -1, SQ2), (-1, 1, SQ2), (-1, -1, SQ2)]
        for r in range(-5, 15):
            for c in range(-5, 15):
                for dr, dc, step in moves:
                    self.assertLessEqual(heuristic((r, c), goal),
                                         step + heuristic((r + dr, c + dc), goal) + 1e-9)


if __name__ == "__main__":
    unittest.main()