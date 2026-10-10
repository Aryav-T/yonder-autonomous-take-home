# Methodology

How to run my planner, why it's built the way it is, and what it doesn't do yet.
The README's "Your write-up" section has the numbers from each run; this file is
the how-to and the reasoning.

## 1. How to run it

**Tested on:** macOS 26.5.1, Python 3.13.0 (in a virtual environment).
Run every command from the repo root.

```bash
# Get the code
git clone https://github.com/Aryav-T/yonder-autonomous-take-home.git
cd yonder-autonomous-take-home

# Set up (numpy/matplotlib are only needed for --visualize)
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt

# Unit tests: should end with OK
python -m unittest -v

# Every scenario plus random seeds 1-3, as one table
python sim/launch.py --all

# One scenario at a time
python sim/launch.py --scenario open
python sim/launch.py --scenario boulders
python sim/launch.py --scenario wall
python sim/launch.py --scenario popup
python sim/launch.py --scenario goal-blocked

# Watch a run live
python sim/launch.py --scenario boulders --visualize
python sim/launch.py --scenario wall --visualize
```

**What you should see:** every row of `--all` says `REACHED` with `blocked 0` and
`needless 0`, matching the table in the README's write-up.

## 2. Thought process

**Reading before writing.**
- I read `sim/messages.py` first. The grid is row-major, the origin is the
  **lower-left corner** of cell (0, 0), and row 0 is the **bottom** row. That's
  why `world_to_cell` uses `floor` (not `int`, which truncates −0.02 to 0 instead
  of −1) and `cell_to_world` adds half a cell to return the centre.
- I read `grid_feed_publisher.py` before writing the subscriber: the feed
  publishes **BEST_EFFORT, depth 1**, so my subscription uses the same.
  A RELIABLE subscriber would silently receive nothing.
- I read `rover.py` (an empty path means stop; the rover joins the path at its
  nearest point), `scoring.py` (what "needless" means), and `world.py` (the body
  radius is 0.3 m, the planning radius 0.5 m; obstacles are full squares).

**Search.**
- Octile heuristic: the exact cost on an empty 8-connected grid, so admissible
  and consistent. I confirmed it matters by swapping in Manhattan: 3 tests
  failed and A* returned paths 7% and 14% longer than optimal.
- `heapq` open set with lazy deletion and a closed set, so the 150×150 speed
  test runs well under the limit.
- No corner cutting; the start cell is never checked (the rover may already be
  inside an inflated zone); goal occupied or out of bounds returns `None` at once.

**Making the plan safe for a disc.**
- Inflate obstacles by the 0.5 m radius (circular). Mark the outer ring of cells
  as a wall, since their centres are only 0.25 m from the arena edge.
- I checked circular inflation against `world.py`'s collision rule: a diagonal
  neighbour is 0.354 m from the obstacle's corner, more than the 0.3 m body.
- Turning inflation off made every obstacle scenario stall with 150 blocked
  ticks, which confirmed it's essential (details in the README).

**The replan trigger.** This was the main design decision.
- I read `scoring.py` and found that its "not needless" rule (a waypoint on or
  beside an occupied cell) is exactly the cross shape a 1-cell inflation marks.
  So I check validity on the **same inflated grid I plan on**. Every replan my
  check triggers is one the scorer agrees was justified.
- I only check waypoints **ahead** of the rover, starting at `idx + 1`. Skipping
  everything behind handles `popup`'s rock behind. Skipping the rover's own cell
  avoids replanning every tick while the rover drives through an inflated zone.
- No route: publish an empty path (stop), retry every tick, and don't republish
  an unchanged empty path.
- I kept my counters to the scorer's definitions (a path after a stop is a
  recovery, not a replan), and checked they matched in every run.

**Unknown cells are free.** On tick 0 the goal is 36.8 m away and the sensor
sees 8 m, so the goal is unknown. Treating unknown as blocked means no route on
the first tick, and I confirmed it deadlocks: on `open` the rover never moves
and the run stalls at 30 s. Treating it as free means the plan is optimistic and
gets corrected by replanning as the map fills in.

**What the runs showed.**
- `boulders`: I predicted 4–6 replans and got 3. Boulders #1 and #2 behaved as
  predicted; #3 and #4 never triggered, because the detour around #2 left the
  path on an offset diagonal (y = x + 3) that passes beside them.
- `wall`: 13 replans in 4.2 s as the wall is revealed bit by bit. All justified
  (0 needless), but it's the clearest cost of being optimistic about unknown space.
- Pop-up rocks are answered on the same tick they appear; the rock behind is ignored.

**Process.** I built the pure files first, one commit per step, with the
provided tests as the check at each step. Then the node in two steps (plan once,
then replanning), the boulders prediction committed on its own before running
it, and the AI log updated as I went.

## 3. Known limitations

- **Gradual reveal causes bursts of replans.** `wall` replans 13 times in 4.2 s;
  random seed 42 replans 3 times in 0.4 s. None are needless by the scorer's
  rule, but each one re-runs A* and changes the path. **Next:** only treat a
  blocked waypoint as urgent if it's within some distance ahead (e.g. 3–4 m,
  about 2–3 s of driving at 1.5 m/s), and let farther blocks wait for more of
  the obstacle to be revealed. That trades safety margin for fewer replans, so it
  needs testing for blocked ticks across many seeds before I'd trust it.
- **Unknown space is treated as fully free.** Plans head confidently into
  unexplored areas, which is what causes those bursts. **Next:** give unknown
  cells a small extra cost (stretch goal B), so plans prefer seen-free ground
  when the detour is cheap. The heuristic would stay admissible, since extra cost
  only makes real paths more expensive.
- **No confidence handling.** I ignore `confidence`, which is correct for the
  clean sensor but would chase phantom obstacles under `--noise`. **Next
  (stretch 2A):** treat an occupied cell as blocking only if its confidence is
  above `confidence_threshold`, or if it was seen above the threshold before and
  has only decayed (a real rock doesn't disappear because I stopped looking);
  don't let a confidence change with no occupancy change trigger a replan; and
  during a dropout, keep driving the current path rather than replanning on a
  grid that holds no new information.
- **Grid-aligned paths.** A* paths only move in 8 directions, so they have kinks
  and many waypoints (53 for a straight diagonal). **Next:** Theta* (stretch C),
  checking each long segment against the grid along its whole length, since the
  rover drives straight lines between waypoints.
- **Replans from scratch.** Every replan runs full A* and re-inflates the whole
  grid. That's fast enough here (60×60), but on a bigger map an incremental
  planner like D* Lite would reuse the previous search.
- **Equal-cost paths are chosen arbitrarily.** Which of the many equal-cost
  paths A* returns depends on tie-breaking order. That decided whether boulders
  #3 and #4 mattered in `boulders`. A tie-break that prefers staying near the
  straight line would make paths more predictable.
- **`next_waypoint_index` is "nearest waypoint".** If a path ever doubled back
  close to itself, the nearest waypoint could be a later one, and the check
  would skip a stretch that's still ahead. A* paths here don't do that, but
  tracking progress along the path (never moving the index backward) would be
  more robust.
- **No-route retry runs every tick.** Cheap here because A* returns at once when
  the goal is occupied, but if the goal were unreachable for other reasons, each
  retry would search the whole grid. Retrying only when the grid has changed
  would fix that.