# AI Usage Log

I used **Claude** (Anthropic) throughout, as a tutor and pair programmer: it
explained concepts, verified and wrote some code. I ran
every test and scenario myself, read the provided `sim/` files it pointed me to,
and checked its claims against the outputs. Below is what I asked, what I kept,
what it got wrong, and how I checked.

Things it got wrong are collected in the summary at the end.

## 1. Step 1: Overview
**What I asked:** Give a full rundown of A*, how to implement, and how it works


## 2. Step 2: `grid_utils.py`

**What I asked:** Explain world ↔ cell conversion and inflation from the
`GridSnapshot` docstring. Then I asked it to verify my implementations of `world_to_cell`, `cell_to_world`, `inflate`.

**What the AI got wrong that I had to catch:** Nothing wrong in the code, but it
raised a concern it couldn't settle: circular inflation leaves diagonal neighbours
free, and a rover centre there is only 0.354 m from the obstacle's corner. I
resolved it later by reading `world.py`: the body radius that triggers an emergency
stop is 0.3 m, so 0.354 m is safe. It's now a test
(`test_diagonal_neighbour_is_left_free_and_that_is_safe`).

**How I verified it:** All 17 tests in `tests.test_grid_utils` passed.

## 3. Step 3: `search.py` (A* and heuristic)

**What I asked:** Explain A* step by step (including why the heuristic must not
overestimate), then ask to fix my version `heuristic` and `astar`.

**How I verified it:** 18/18 passed after the fix, including the 150×150 speed test.
Then I **broke the heuristic on purpose**, swapping in Manhattan (`dr + dc`):

```
FAIL: test_never_overestimates   h((0,0),(1,1)) = 2.000 overestimates the true cost 1.414
FAIL: test_tricky_grid_seed_199  9.414213562373096 != 8.2426
FAIL: test_tricky_grid_seed_223  8.82842712474619  != 8.2426
```

That confirmed the optimality tests really depend on the heuristic. I reverted to
octile and re-ran: 18/18.

## 4. Step 4: `replan.py` and reading `scoring.py`

**What I asked:** Implement `next_waypoint_index` and `path_is_valid`, and work out
what "needless replan" means.

**What I kept vs. rewrote, and why:** Kept both functions. The important decision
came from reading `scoring.py` together: its "not needless" rule (a waypoint on or
directly beside an occupied cell) is exactly what a 1-cell inflation marks. So I
decided to check validity on the **inflated** grid, not the raw one.

**What the AI got wrong that I had to catch:** Nothing wrong here. It flagged a
possible problem in advance (rover standing in an inflated cell → replan every tick),
which shaped Step 6.

**How I verified it:** 14/14 in `tests.test_replan`, and the full suite 49/49.

## 5. Step 5: `planner_node.py`, plan once

**What I asked:** Exlain what is needed to be done up: subscription, publisher, plan once, status output.

**What the AI got wrong that I had to catch:**
- It predicted the `open` path would be "a bit above 36.8 m". It came out
  **exactly 36.8 m**: start and goal are on an exact diagonal (52 diagonal steps),
  where octile equals straight-line distance. It corrected itself when I showed it
  the output.

**How I verified it:** `open` reached the goal: 1 plan, 0 replans, 0 blocked, and
`Your node reports replan_count=0 (the scoreboard counted 0)`. No QoS warning.

## 6. Steps 6–8: replanning, popup, goal-blocked, random seeds

**What I asked:** Explain how `needs_replan` work and what is needed to be done.

**What I kept vs. rewrote, and why:** Kept the three rules (initial plan; retry
every tick after no route; replan only if a waypoint **ahead** is blocked, checking
from `idx + 1`) and the "don't republish an unchanged empty path" rule. I had it
justify `idx + 1` with a concrete case before accepting it.

**What the AI got wrong that I had to catch:**
- `wall` produced **13 replans**, which looked like dithering. Claude's explanation
  (the wall revealed bit by bit, each plan going over the top of the part seen so
  far) was plausible, so I checked it: all 13 are between 6.8 s and 11.0 s, the path
  lengths creep 26.7 → 24.1 m as the crossing point moves up, and the scorer counts
  0 needless.

**How I verified it:** Every scenario, the `--all` table and 3 unseen seeds
(11, 42, 99): all reached the goal, 0 blocked, 0 needless, and my count matched the
scorer's every time. I checked the specific behaviours each scenario tests:
- `popup`: rock behind at tick 80 → no replan (count stays at 3 from 7.2 s to 19.0 s).
- `goal-blocked`: empty path at tick 102, recovery at tick 142 (the tick it cleared).
- Seeds 42 and 99: every pop-up rock answered on the same tick it appeared.

## 7. The `boulders` prediction

**What I asked:** Read world.py and explain where the boulders are

**What I kept vs. rewrote, and why:**  I checked the boulder positions and distances against `world.py` myself, asked if 5 works, and committed it alone as `prediction: boulders` before running the scenario.

**What the AI got wrong that I had to catch:** The prediction said 4–6; I got **3**.
One assumption was wrong: that each detour returns to the original diagonal and so
crosses the next boulder. In the `--visualize` picture, the path after the detour
around boulder #2 ran on the line y = x + 3, about 1.8 m off the route, and passed
0.35–0.7 m beside boulders #3 and #4 without touching them.

**How I verified it:** The picture, plus the timestamps: replans at 0.4 s (boulder
#1), 4.2 s and 4.4 s (boulder #2), and nothing after.

## 7. Asked claude to give more test cases



## Summary: what the AI got wrong

| What | How I caught it |
|---|---|
| Predicted `open` path "a bit above 36.8 m"; it was exactly 36.8 m | the `open` run output |
| Prediction assumed detours return to the original diagonal (predicted 4–6, got 3) | the `boulders --visualize` picture |
| Expected "some blocked ticks" without inflation; every obstacle scenario stalled at 150 and never recovered | the no-inflation `--all` table |
| Muddled explanation of why some tests passed during the placeholder error | ignored it; the traceback showed the real cause |
| Raised a corner-collision worry about circular inflation it couldn't settle | read `world.py`: body radius 0.3 m < 0.354 m; added a test |