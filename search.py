"""
A* search on a Grid — pure Python, NO ROS in this file.

    python -m unittest tests.test_search -v

Movement rules (fixed, so the tests can check them)
---------------------------------------------------
* 8-connected: a cell has up to 8 neighbours (up, down, left, right and the 4 diagonals).
* Cost of a move: 1.0 for up/down/left/right, sqrt(2) for a diagonal. (Cost is in
  cell-widths, not metres — multiply by grid.resolution if you need metres.)
* No corner cutting: a diagonal move is only allowed if BOTH of the two cells it
  squeezes between are passable. (A real rover cannot slip between two obstacles
  that touch at a corner.)
* Cells outside the grid don't exist.
"""

import heapq
import math
from typing import Dict, List, Optional
from grid_utils import Grid, Cell
from sim.grid_values import OCCUPIED, UNKNOWN

SQRT2 = math.sqrt(2.0)
NEIGHBOURS = [
    (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
    (1, 1, SQRT2), (1, -1, SQRT2), (-1, 1, SQRT2), (-1, -1, SQRT2),
]


def heuristic(a: Cell, b: Cell) -> float:
    """
    Estimate of the cost to travel from cell `a` to cell `b` on an EMPTY grid
    under the movement rules above.

    Choose and justify your choice in the README. The one hard requirement:
    it must NEVER OVERESTIMATE the true cost (that's what "admissible" means, and
    A* is only guaranteed to return the shortest path if it holds). The test
    checks this, and it is worth understanding why some very common heuristics
    fail it on an 8-connected grid.
    """
    dr = abs(a[0] - b[0])
    dc = abs(a[1] - b[1])
    return SQRT2 * min(dr, dc) + (max(dr, dc) - min(dr, dc))


def astar(grid: Grid, start: Cell, goal: Cell, unknown_is_free: bool = False) -> Optional[List[Cell]]:
    """
    Find the cheapest path from `start` to `goal`.

    Args:
        grid             the grid to search. OCCUPIED cells can't be entered.
        start, goal      (row, col) cells.
        unknown_is_free  how to treat UNKNOWN cells: True = passable, False = blocked.
                         Which is right for a rover that has only seen a small
                         circle of the world so far? Look at the very first tick
                         of a run before you decide.

    Returns:
        The path as a list of (row, col) cells, INCLUDING both start and goal, or
        None if there is no path. If start == goal, returns [start].

    Things to think about (the tests poke at each):
      * What if the goal is occupied?
      * The rover might already be standing on a cell that your obstacle
        inflation marked as blocked. Should search refuse to leave it?
      * Speed: a 100 x 100 grid must finish in well under a second. What data
        structure makes "give me the cheapest open node" fast?
    """
    def passable(r: int, c: int) -> bool:
        if not grid.in_bounds(r, c):
            return False
        v = grid.get(r, c)
        if v == OCCUPIED:
            return False
        if v == UNKNOWN:
            return unknown_is_free
        return True

    if not grid.in_bounds(*start) or not grid.in_bounds(*goal):
        return None
    if start == goal:
        return [start]
    if not passable(*goal):
        return None

    g_cost: Dict[Cell, float] = {start: 0.0}
    came_from: Dict[Cell, Cell] = {}
    closed = set()
    counter = 0                                  
    open_heap = [(heuristic(start, goal), counter, start)]

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current in closed:                     
            continue
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path
        closed.add(current)

        r, c = current
        for dr, dc, step in NEIGHBOURS:
            nr, nc = r + dr, c + dc
            if (nr, nc) in closed or not passable(nr, nc):
                continue
            if dr != 0 and dc != 0 and not (passable(r + dr, c) and passable(r, c + dc)):
                continue                        
            new_g = g_cost[current] + step
            if new_g < g_cost.get((nr, nc), math.inf):
                g_cost[(nr, nc)] = new_g
                came_from[(nr, nc)] = current
                counter += 1
                heapq.heappush(open_heap, (new_g + heuristic((nr, nc), goal), counter, (nr, nc)))

    return None
