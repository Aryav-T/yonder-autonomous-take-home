# Autonomous Take-Home: Path Planning Track

Welcome! This assessment is designed to measure how you think and problem-solve, not whether you already know ROS or robotics. Novices and experienced candidates are both expected to succeed here — we care much more about your process than your prior background.

**Time expectation:** roughly one day of focused work (~6–8 hours). Stretch goals are optional and unbounded beyond that.

**AI tools are allowed and expected.** See the AI Usage Log section below — how you use AI is part of what we're evaluating, not something to hide.

---

## Part 0: Base Knowledge Primer

### What is ROS 2?

ROS 2 is a framework where independent programs called **nodes** send messages to each other over named channels called **topics**. A **publisher** sends messages; a **subscriber** receives them. For this task, think of a topic as a live data stream you need to visualize, similar conceptually to a websocket feed. Instead of one giant program controlling everything, you have small focused programs — one for motor control, one for GPS, one for camera processing — that communicate over named channels.

### QoS (Quality of Service)

ROS 2 lets you configure delivery guarantees per topic (e.g., `RELIABLE` vs `BEST_EFFORT`).

1. `RELIABLE` guarantees that every message reaches its destination. The publisher keeps re-sending until the receiver acknowledges. If a packet gets lost, the sender resends it.
2. `BEST_EFFORT` sends data without checking it arrives. The publisher transmits once and moves on, spending no extra time or network resources tracking delivery.

A publisher and subscriber on the same topic must have *compatible* settings, or they silently won't connect. The rule: the subscriber may not ask for more than the publisher offers (a `BEST_EFFORT` publisher cannot serve a `RELIABLE` subscriber). Real ROS 2 prints a warning when this happens, and so does our simulator. Read your terminal output.

### Occupancy Grids

An occupancy grid represents the environment as a 2D grid of cells, where each cell is marked free, occupied, or unknown. Path planners search this grid to find a route from a start cell to a goal cell without crossing occupied cells. In the real world, a rover doesn't see the whole map at once — it builds up the grid incrementally as it moves and its sensors observe new areas. This task simulates that incremental view: the grid starts out almost entirely **unknown** and fills in as the rover drives.

A grid is just a flat list of numbers plus a few facts that say how to read it: how wide a cell is (`resolution`), and where in the world the grid sits (`origin`). Turning a world position into a cell, and a cell back into a world position, sounds trivial and is where a surprising number of bugs live. Read the `GridSnapshot` docstring in `sim/messages.py` before you write any code.

### Path Planning — A* (what you'll implement)

A* is a graph search algorithm that finds the shortest path between two points by exploring the most promising paths first, using a cost function that combines "distance traveled so far" with "estimated distance remaining" (a *heuristic*, an optimistic guess at the distance to the goal). Whether you get the *shortest* path depends entirely on that heuristic being optimistic. That is worth understanding, not just copying.

**Theta\*** (a stretch goal, and what we actually use on the rover) is a variant of A* that allows "any-angle" paths instead of only grid-aligned moves, producing smoother, more realistic paths — but it's meaningfully harder to implement, hence stretch-only.

### The rover has a size

A path planner treats the rover as a dimensionless point moving through cell centres. The real rover is a disc. A path that skims the edge of a boulder is fine for a point and impossible for a disc. The standard fix is to *inflate* every obstacle by the rover's radius so that a point moving through the inflated map is safe for the real disc. The outer edge of the arena is a wall in the same sense: the rover's body cannot hang over it or scrape along it, so hugging the outermost row of cells is unsafe. Nothing marks it in the grid for you.

### Replanning

Because the rover's view of the world changes as it moves (new obstacles appear, unknown ground turns out to have boulders on it), a real planner can't just plan once at the start — it has to detect when its current path is no longer valid and replan. This is the core of what makes this task different from a textbook static-grid A* exercise, and it's the core of what makes our real autonomy stack hard. There is a second half to it that is just as important: **not** replanning when the path is still fine. Planning costs compute, and a path that changes every tick makes the rover dither.

### How Yonder actually uses this

Our rover builds a rolling 30m×30m occupancy grid from stereo camera depth data, where each cell has a confidence value that degrades over time/distance from the camera's current view. We run Theta\* with a pure pursuit controller to plan smooth paths around obstacles while continuously remapping as the rover moves and its view updates. This task is a simplified version of that real pipeline.

**Resources:**
- [ROS 2 Publisher/Subscriber tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [A* Pathfinding for Beginners (Red Blob Games)](https://www.redblobgames.com/pathfinding/a-star/introduction.html)
- [ROS 2 QoS docs](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html)

---

## Starter Repo

### What's in the repo

```
planner_node.py        YOURS — the ROS node: receives grids, decides when to
                       replan, publishes paths, prints monitoring output.

grid_utils.py          YOURS — world <-> cell conversion, obstacle inflation.
search.py              YOURS — the heuristic and A*.
replan.py              YOURS — "where am I on my path?" and "is my path still valid?"
                       These three files are plain Python with no ROS in them,
                       on purpose: they are what the tests exercise.

tests/                 Unit tests for the three files above. Run them as you go.
                       Add your own.

sim/                   PROVIDED — please do not modify anything in here.
  launch.py            Entry point. Run this to start everything.
  simulation.py        The tick loop that ties everything together.
  world.py             The ground truth (which obstacles really exist) and the scenarios.
                       Your planner never sees this.
  grid_feed_publisher.py  The rover's "mapping stack": sensor model -> GridSnapshot.
  rover.py             A minimal rover that drives whatever path you last published.
  scoring.py           Judges your run from the outside. Read it to know how you're scored.
  visualizer.py        Live matplotlib picture of the run.
  messages.py          GridSnapshot and helpers for the nav_msgs/Path you publish.
  rclpy_lite/          A lightweight simulator shim with the same API as real rclpy.
                       Lets you write ROS-style code without installing ROS.
  nav_msgs/  geometry_msgs/  std_msgs/    Standard ROS message types as dataclasses.

requirements.txt       pip dependencies (numpy, matplotlib — only for the visualizer).
AI_LOG.md              Template for your AI usage log (Part 3).
METHODOLOGY.md         Template for how to run your work and your thought process (Part 4).
```

### Familiarisation — read these before you start

**`sim/messages.py`** — the most important file to read first. The `GridSnapshot` docstring defines exactly how the grid is laid out (row-major? which corner is `origin`? which way is `row 0`?) and what every field means. Every bug you can have in the coordinate maths is answered in that docstring.

**`sim/grid_feed_publisher.py`** — how the grid is built each tick, and the QoS it publishes with (this is the thing most likely to silently break your node).

**`sim/rover.py`** — what the rover does with the path you publish. Note what an *empty* path means.

**`sim/scoring.py`** — exactly how your run is judged: what counts as a plan, a replan, and a *needless* replan.

**`sim/rclpy_lite/`** — you don't need to understand this in depth. It's a lightweight shim with the same API as real rclpy (Node, Publisher, Subscriber, QoS). We name it `rclpy_lite` rather than `rclpy` to avoid ambiguity if you have ROS installed. To port your code to a real ROS 2 system, swap `rclpy_lite` → `rclpy` in your imports.

### Running it

You need Python 3.10 or newer (check with `python3 --version`, or `python --version` on Windows) and `git`. The simulator and the tests use only the standard library; numpy and matplotlib are needed only to draw pictures.

1. **Choose how you'll run Python, and install dependencies.** Pick **one** of the two options. Both work; the rest of the steps are the same.

   **Option A: directly in your terminal** (no virtual environment). The simplest route. Type `python3` wherever this README says `python` on macOS and Linux. **On Windows the command is usually `python` (or `py`); if typing `python` opens the Microsoft Store, use `py`.**

   Part 1 needs **nothing installed**. Only the pictures (`--visualize`, `--save-png`) need numpy and matplotlib, so install them if you want pictures:

   - **Ubuntu / WSL:** install the system packages. (A plain `pip install` is refused on recent Ubuntu with `error: externally-managed-environment`.)

     ```bash
     sudo apt install python3-numpy python3-matplotlib python3-tk
     ```

   - **macOS / Windows / other Linux:**

     ```bash
     pip install -r requirements.txt
     ```

   If `pip` says `externally-managed-environment` or you'd rather not touch your system Python, use Option B.

   **Option B: a Python virtual environment.** Keeps everything inside the project and works the same on every OS. On Ubuntu / WSL you may first need `sudo apt install python3-pip python3-venv`.

   ```bash
   python3 -m venv .venv
   source .venv/bin/activate          # Windows PowerShell:  .venv\Scripts\Activate.ps1   (Windows cmd.exe: .venv\Scripts\activate.bat)
   pip install -r requirements.txt
   ```

   On Windows, PowerShell may refuse the activate script with "running scripts is disabled on this system". Either use `cmd.exe` and the `activate.bat` line, or run `Set-ExecutionPolicy -Scope Process Bypass` first (it only affects that one window).

   Your prompt now starts with `(.venv)`. Run the `source` line again in every new terminal, and type `deactivate` to leave. Inside the environment the command is called `python`, which is what the rest of this README uses.

2. **Run the tests** (from the repo root). They all fail at first. Your job is to turn them green:

   ```bash
   python -m unittest -v
   ```

   You should see `FAILED (errors=48)` to begin with. That is correct: 48 tests are waiting for you, and 1 already passes.

3. **Start the simulator** (from the repo root):

   ```bash
   python sim/launch.py --scenario open
   ```

   This loads your `planner_node.py` and runs the whole scenario in a fraction of a second. Until you implement the node, it prints a summary saying what is missing (at first: no subscription to `/grid_feed`). That is expected.

4. **Enable the live visualizer** once your node publishes a path:

   ```bash
   python sim/launch.py --scenario boulders --visualize
   ```

   Opens a matplotlib window showing the rover's map (dark gray is unknown, white is free, black is occupied), your planned path in blue, the trail the rover has driven, and the goal. Replans are marked on the trail. Faint red squares are real obstacles the rover has not seen yet.

   > **No display?** `--visualize` needs one (Windows 11 has one built into WSL through WSLg, and you may also need `sudo apt install python3-tk`). Use `--save-png out.png` instead to write a picture of the final state of a run.

| Command | What it does |
| --- | --- |
| `python sim/launch.py` | Default scenario (`boulders`), headless, prints a summary |
| `python sim/launch.py --scenario NAME` | Pick a world: `open`, `boulders`, `wall`, `popup`, `goal-blocked`, `random` |
| `python sim/launch.py --scenario random --seed 7` | A generated world; every seed is a different one |
| `python sim/launch.py --all` | Every scenario plus random seeds 1–3, as one table |
| `python sim/launch.py --visualize` | Live matplotlib window (`--speed 3` for 3× faster) |
| `python sim/launch.py --save-png out.png` | Run, then save a picture of the final state |
| `python sim/launch.py --noise` | Part 2: noisy sensor (see Part 2A) |
| `python sim/launch.py --no-node` | Simulator only, no planner |

### The feeds

| Topic | Rate | Message | QoS |
| --- | --- | --- | --- |
| `/grid_feed` (you subscribe) | 5 Hz of *simulated* time | `GridSnapshot` (grid + rover position + goal) | check the source |
| `/planned_path` (you publish) | when your plan changes | `nav_msgs/Path` (waypoints in world metres) | your choice |

Some numbers worth knowing (all also in the messages): the arena is 30 m × 30 m, cells are 0.5 m, the rover sees everything within 8 m of itself, drives at 1.5 m/s, and has a planning radius of 0.5 m. The start and goal are the same in every scenario.

**Simulated time.** One tick is 0.2 s of *simulated* time. The simulator does not wait for the wall clock, so a whole run takes a fraction of a second (`--visualize` slows it to real time so you can watch). Any "time" in your monitoring output must be simulated time (`msg.header.stamp`), not `time.time()`.

### What the grid tells you

Each `GridSnapshot` is everything the rover knows at that moment:

- **`occupancy`**: per cell, `-1` unknown, `0` free, `100` occupied.
- **`confidence`**: per cell, 0.0–1.0, how much to trust that occupancy value.

> ### Part 1 uses a perfect sensor. You do not need `confidence` for Part 1.
>
> With the default (clean) feed, every cell inside the rover's sensor range is reported **exactly** as it is in the world. An obstacle is always `occupied`, empty ground is always `free`, and confidence is simply `1.0` for anything the rover has seen and `0.0` for anything it hasn't. Nothing is ever wrong, and nothing decays. **Ignore the `confidence` array entirely in Part 1.**
>
> Confidence-based planning is **Part 2**, which turns on a noisy sensor (`--noise`) where confidence starts to mean something.

What is *not* perfect in Part 1 is the rover's **knowledge**: it only sees within 8 m, so most of the map starts out unknown, and things can change after it last looked.

---

## Part 1: Core Task (required)

Complete `planner_node.py` (and the three pure-Python files it uses) so that it:

1. **Subscribes to `/grid_feed`**, which delivers a new occupancy grid snapshot and the rover's current position at each tick.
2. **Implements A\*** to plan a path from the rover's current position to the goal, respecting occupied cells and the rover's size.
3. **Detects when the current path is no longer valid** — because a cell along the part of the path still ahead of the rover became occupied — and **replans** from the rover's current position when this happens. Don't replan when the path is still fine.
4. **Publishes the current planned path** on `/planned_path` each time it changes.
5. **Provides basic monitoring output** (terminal is fine): current path length, number of replans triggered so far, and time since last replan.

### Suggested order of work

Each step has a check that tells you it's right before you move on, and ends with a **commit** (see "Commit as you go" below).

| Step | What to do | How you know it's done | Commit message |
| --- | --- | --- | --- |
| 1 | Read `sim/messages.py`. Run `python sim/launch.py --scenario open --no-node` and watch the rover sit still. | The `!!` line at the bottom of the summary says why. | *(nothing to commit yet)* |
| 2 | `grid_utils.py`: `world_to_cell`, `cell_to_world`, `inflate`. | `python -m unittest tests.test_grid_utils` | `step 2: grid_utils` |
| 3 | `search.py`: `heuristic`, `astar`. | `python -m unittest tests.test_search` | `step 3: search` |
| 4 | `replan.py`: `next_waypoint_index`, `path_is_valid`. | `python -m unittest tests.test_replan` | `step 4: replan helpers` |
| 5 | `planner_node.py`: subscribe, publish, plan once. Get the rover across the `open` scenario. | `--scenario open` reaches the goal, 1 plan, 0 replans. | `step 5: plan once` |
| 6 | Add the replan logic. Run `boulders`, `wall`. | Reaches the goal, 0 blocked ticks. | `step 6: replanning` |
| 7 | `popup` and `goal-blocked`. | Reaches the goal, 0 blocked ticks, 0 needless replans. | `step 7: popup and goal-blocked` |
| 8 | `--all`, then some `--scenario random --seed N` you haven't tried. | Every row reaches the goal. | `step 8: random seeds` |
| 9 | Write your write-up (in this README), finish `METHODOLOGY.md` and `AI_LOG.md`. | Every question in the write-up has an answer with numbers from your own runs. | `step 9: write-up, methodology and AI log` |

Use `--visualize` (or `--save-png`) constantly. You will learn more from watching one replan happen than from re-reading your code.

**Predict, then run.** Before you first run `boulders`, write down (in your README write-up) how many replans you expect and why, and **commit that prediction on its own** (`prediction: boulders`) before you run it. After the run, compare. If the number surprises you, that surprise is the interesting part.

### Commit as you go

We read your commit history as well as your code. It shows how you worked, and it is the honest record behind your write-up. Commit at the end of each step in the table above, using the message shown.

One-time setup (git refuses to commit until it knows who you are). Use your own name and email:

```bash
git config --global user.name "Your Name"
git config --global user.email "you@example.com"
```

At the end of each step:

```bash
git status                  # what changed? nothing surprising?
git add -A
git commit -m "step 2: grid_utils"
```

Rules of the road:

- **Small and honest beats tidy.** A history with `step 6: replanning` followed by `fix: replan fired on the rock behind us` is exactly what we like to see. Fixing your own bug in a later commit is normal.
- **Do not squash, amend or force-push** to make the history look cleaner. Do not commit everything in one go at the end.
- **Commit your AI log as you go too.** When an AI tool gets something wrong, write the `AI_LOG.md` entry in the same commit as the fix.
- Do not edit `sim/` or the provided tests to make things pass. Put your own tests in new files under `tests/`. We grade against our own copy of both.

### What "done" looks like

The summary at the end of a run is your scoreboard:

```
 Result             REACHED THE GOAL in 130 ticks (26.0 s simulated)
 Blocked ticks      0
 Paths published    4   (4 plans, 3 of them replans)
 Needless replans   0
```

- **Result** — did the rover get there.
- **Blocked ticks** — the number of ticks the rover had to emergency-stop because your path led into a real obstacle. This should be **0** everywhere.
- **Replans / needless replans** — a replan is *needless* if the path it replaced was still fine according to the grid you were given (`sim/scoring.py` defines exactly what that means). This should be **0** everywhere. Note that more replans is not the same as needless: a world full of boulders legitimately needs many.

We will also run your planner on worlds you haven't seen (`--scenario random` with other seeds), so don't tune it to the visible scenarios.

### What we're looking for

- Does the planner produce a valid path at each tick, and correctly replan when the grid changes underneath it?
- Does it avoid replanning needlessly on every single tick — only when the current path is actually invalidated?
- Correct pub/sub setup against the provided scaffold.
- Reasonable code structure — is the grid/search logic separated from the ROS plumbing?
- A short README write-up (see "Your write-up") explaining your heuristic choice, your replan-trigger logic, and any design decisions or assumptions.

---

## Part 2: Stretch Goals (optional)

Pick any/all. Partial, well-reasoned attempts are valued over none.

### A. Noisy sensor: confidence-based planning

Run `python sim/launch.py --noise`. Same worlds, same rover, but the sensor is now imperfect, and this is where the `confidence` array starts to matter. Read the `NOISE` table in `sim/grid_feed_publisher.py`; every number is explained. In short:

- Confidence falls with distance: a cell right next to the rover is ≈ 1.0, one at the edge of sensor range ≈ 0.25.
- **Low confidence means the reading is more likely to be wrong**: phantom obstacles that aren't there, and real ones that are missed. A wrong reading usually lasts a single tick.
- A cell that isn't currently being observed loses confidence over time. There are also short sensor **dropouts** where nothing is observed at all.
- `confidence_threshold` (0.6) is the value below which we call a reading *unreliable*.

**Step 1 — observe.** Run your Part 1 planner, unmodified, with `--noise`. What happens to the replan count? Where do the replans come from? (The summary now has a **Phantom replans** line: replans triggered by something in your grid that wasn't really there.) Look at `--visualize`: low-confidence cells fade toward gray.

**Step 2 — fix it.** Use confidence so that you stop chasing phantoms without becoming blind to real obstacles. There is no single right answer; questions worth answering in your write-up:

- Which cells should count as blocking: everything marked occupied? Only cells above the threshold? What about an obstacle you saw clearly a moment ago whose confidence has since decayed. Has it gone away?
- Should a change in confidence, with no change in occupancy, ever trigger a replan? What could a replan achieve if *every* cell has lost confidence (a dropout)?
- Unknown cells also have confidence 0. Are they the same thing as low-confidence cells?

We grade the outcome (reached the goal, 0 blocked ticks, few phantom and needless replans), not one specific technique.

### B. Cost-aware planning

Instead of treating cells as purely free/occupied, use the confidence value as a soft cost (e.g., low-confidence cells are traversable but penalised), so the planner prefers well-observed routes rather than only reacting after a replan is forced. This changes `astar` (cells no longer all cost the same) and, with it, what a good heuristic is. Explain how.

### C. Theta\*

Implement Theta\* instead of (or in addition to) A\* for smoother, any-angle paths. This is a meaningfully harder algorithm — attempt it only if the core task felt comfortable. Note that the rover drives in straight lines between the waypoints you publish, and the simulator checks that against the real world, so a long segment has to be checked for obstacles along its whole length, not just at its ends.

### D. Live Webots simulation

We can provide access to a lightweight version of our Webots simulation environment (via Docker) publishing live occupancy grids instead of this simulator. Flag interest in your README and we'll follow up with setup instructions. This one has real infra dependencies, so we're treating it as fully optional, and there is nothing to do for it in this repo.

---

## Part 3: AI Usage Log (required)

Submit a short `AI_LOG.md` with your code (there is a template in the repo root). For each significant use of AI tools, note:

- What you asked
- What you kept vs. rewrote, and why
- Anything the AI got wrong that you had to catch. A very common one is generating a planner that assumes a static map with no replanning logic at all unless explicitly prompted, since that's the default assumption in most training data and tutorials. There are plenty of others; the scoreboard and the tests are there to help you find them.
- How you verified it actually worked (not just that it ran). Which test, scenario or picture showed you the problem?

This is not graded on whether you used AI — it is graded on whether you can tell us what it got wrong and why you fixed it.

---

## Part 4: METHODOLOGY.md (required)

Edit the `METHODOLOGY.md` in the repo root (there is a template) so it covers:

- **How to run your code.** The exact steps for a reviewer to install everything and run your work from a fresh clone and see it working. Say which Python version and OS you tested on.
- **Your thought process, in bullet points.** Why you made the choices you did, and what your own runs showed.
- **Known limitations.** What doesn't work, and what you'd do next. Being upfront counts in your favor.

The "Your write-up" section of this README answers specific questions with numbers from your runs; `METHODOLOGY.md` is the how-to-run and the reasoning, in your own words. Write it for a teammate who has never seen your code.

## Rubric

| Criterion | What we're scoring |
| --- | --- |
| **Correctness** | Planner finds valid paths, reaches the goal in every scenario (including unseen seeds) with 0 blocked ticks; pub/sub actually connects |
| **Replanning behavior** | Correctly detects invalidated paths and replans; doesn't replan needlessly every tick |
| **Design judgment** | Evidence of intentional choices beyond the minimum ask (heuristic choice, replan-trigger design, code structure) |
| **Handling ambiguity** | How did they resolve underspecified parts of the task? Did they make a reasonable call and explain it? |
| **Understanding, not just output** | Can they explain their heuristic, their replan logic, their tradeoffs? Does the write-up show real comprehension, backed by numbers from their own runs? |
| **AI verification** | Evidence they tested/verified AI-assisted code rather than taking it on faith (from log + code quality + tests they added + commit history) |
| **Stretch engagement** (bonus) | Attempted or completed any stretch goal — even partial attempts count positively |

We don't expect a perfect implementation. Those who show genuine effort and learning are the ones who will have a leg up!

---

## Your write-up

All numbers below come from my own runs on the clean sensor (Part 1).

### Prediction

*Written and committed (`prediction: boulders`) before I first ran `boulders`.*

**Expected: 5.**

From `sim/world.py`: `boulders` has 10 boulders. 4 sit directly on the
start→goal line (at 22%, 42%, 62%, 83% of the route, 1.5–2.0 m wide). The
other 6 are 3.9–12.4 m off it, far outside the scorer's 1 m (2-cell) threat
zone, so I expected them to trigger nothing.

Reasoning:
- My first plan treats unknown cells as free, so it runs along the diagonal
  through all 4 on-route boulders. Each one invalidates the path when the
  rover sees it (8 m sensor range), so at least ~1 replan per on-route boulder.
- Boulder #1's nearest corner is ~7 m from the start, so part of it is
  visible on tick 0 and the initial plan already dodges that corner. The
  unseen rest of it is still "free", so I still expected a replan for it.
- The sensor edge sweeps across each boulder over several ticks (the rover
  moves 0.3 m per tick). A detour planned around the first visible cells
  can be clipped by the next ones, so some boulders may cost 2 replans.
- After each detour the new path heads back toward the goal roughly
  parallel to the diagonal, so it still crosses the next on-route boulder.
- I expected 0 needless replans: I only replan when a waypoint still ahead is
  on or beside an obstacle in the inflated grid, which matches the
  scorer's own definition of a path that is no longer fine.

**Result: 3 replans** 0 needless, 0 blocked, goal reached in 25.2 s.

| t (s) | Event | Matches prediction? |
|---|---|---|
| 0.0 | initial plan is **37.6 m**, not open's 36.8 m: it already dodges the visible corner of boulder #1 | yes, partial visibility |
| 0.4 | replan: more of boulder #1 comes into view and clips the detour | yes |
| 4.2, 4.4 | two replans on consecutive ticks for boulder #2 | yes, gradual reveal costs 2 |
| after 4.4 | **boulders #3 and #4 never trigger a replan** | **no** |

The wrong assumption was the last bullet: that each detour returns to the
original diagonal. In the `--visualize` picture, the path after t = 4.4 s goes
briefly north, then runs one long diagonal on the line **y = x + 3**, then east
to the goal. The original route is y = x + 0.5, so the path shifted about
1.8 m sideways. There are many equal-cost paths on an 8-connected grid, and A*
happened to put all the straight steps at the start and end, which made the
long diagonal an offset parallel line. Checking the nearest corners against
that line: boulder #3 is 0.7 m below it, boulder #4 is 0.35 m below it, and the
off-route boulder at (3, 9) is 0.7 m above it. The path threads between them
and never touches #3 or #4, so neither invalidates it.

### Heuristic

**Octile distance**: `h = √2·min(dr, dc) + |dr − dc|` (in `search.py`).

This is the exact cost of the cheapest path on an *empty* 8-connected grid:
use `min(dr, dc)` diagonal steps (√2 each) to close the shorter gap, then
straight steps (1 each) for what's left. Obstacles can only make the real path
longer, never shorter, so octile **never overestimates**, which is what A*
needs to guarantee the shortest path. It is also *consistent*
(`h(a) ≤ step(a, b) + h(b)` for every neighbour), which is why A* can safely
finalise a cell the first time it pops it. My test
`test_consistent_for_every_move` checks this for every move from a 20×20 block
of cells.

**What goes wrong with an overestimate (I tried it).** I temporarily replaced
it with Manhattan (`dr + dc`), which charges 2 for a diagonal step that really
costs √2. Three provided tests failed:

| Test | Octile | Manhattan |
|---|---|---|
| `test_never_overestimates` (0,0)→(1,1) | h = 1.414 | h = **2.000** (true cost 1.414) |
| `test_tricky_grid_seed_199` | 8.243 (optimal) | **9.414**, 14% longer |
| `test_tricky_grid_seed_223` | 8.243 (optimal) | **8.828**, 7% longer |

Why: A* stops when it pops the goal with cost C, trusting that every cell
still waiting has `f ≥ C` and therefore can't lead to anything cheaper. An
overestimating `h` inflates `f` on diagonal-heavy routes, so A* never explores
them and finishes on a straighter, longer route. Manhattan's paths used fewer
diagonals (1 and 2) than the optimal one (3). The speed test still passed,
because a greedier heuristic explores fewer cells. That is the trade weighted
A* makes on purpose, but it gives up the shortest-path guarantee. I reverted
to octile.

### Replan trigger

Every tick, `needs_replan` (in `planner_node.py`) decides, on the **inflated**
grid with the border marked as a wall:

**Replan when:**
1. **Never planned** → initial plan.
2. **The last attempt found no route** (an empty path) → retry every tick,
   because the world can change (the goal in `goal-blocked` clears after 8 s).
   While still stuck, I don't republish the empty path, since nothing changed.
3. **A waypoint ahead of the rover is blocked.** `next_waypoint_index` finds
   the waypoint nearest the rover, then `path_is_valid` checks every waypoint
   from **`idx + 1`** to the goal. Any one that is occupied or out of bounds
   invalidates the path.

**Deliberately do NOT replan when:**
- The map changed somewhere that doesn't touch the remaining path. In `wall`,
  the bottom of the wall comes into view around t ≈ 4.4 s but isn't on the
  path, and the first replan is at 6.8 s, when the part of the wall the path
  crosses comes into range.
- An obstacle appears **behind** the rover. In `popup`, the "rock behind"
  drops onto the rover's trail at tick 80, and my replan count stays at 3 from
  7.2 s until 19.0 s.
- The rover's **own** cell is inside an inflated zone. That's why checking
  starts at `idx + 1`, not `idx`. If the rover drives past a newly seen rock,
  its own cell gets inflated. Checking it would invalidate the path; the new
  path would start from that same cell (A* is allowed to leave a blocked start),
  so the next tick would invalidate it again, every tick until the rover drives
  out. Replanning can't move the rover out of the cell it is already in, so I
  skip it. My test `test_checking_from_rover_cell_would_replan_every_tick`
  shows the brand-new path is "invalid" from `idx` and valid from `idx + 1`.

**Why checking the inflated grid is the right test.** `sim/scoring.py` calls a
replaced path "not fine" if a remaining waypoint is on or directly beside
(up/down/left/right) an occupied cell. With a 0.5 m radius and 0.5 m cells, my
circular inflation marks exactly that cross shape (test
`test_radius_one_cell_marks_exactly_the_cross`). So whenever my check says a
waypoint is blocked, the scorer agrees the old path was not fine, and none of my
replans can be needless.

```
scenario      result           ticks  blocked  plans  replans  needless
-----------------------------------------------------------------------
open          REACHED            121        0      1        0         0
boulders      REACHED            126        0      4        3         0
wall          REACHED            134        0     14       13         0
popup         REACHED            129        0      5        4         0
goal-blocked  REACHED            164        0      4        2         0
random 1      REACHED            137        0     10        9         0
random 2      REACHED            131        0     10        9         0
random 3      REACHED            138        0      6        5         0
```

Unseen seeds I also ran: **11 → 4 replans, 42 → 9, 99 → 5**. All reached the
goal, 0 blocked, 0 needless. In every run, my node's `replan_count` matched the
scoreboard's count.

**Why `wall` needs 13 replans, and why they're not needless.** All 13 happen
between 6.8 s and 11.0 s. The wall (x ∈ [−0.5, 0.5], y from −15 up to 8) is
revealed bit by bit as the rover drives up beside it. Since unknown is treated
as free, each new plan goes over the top of the *part of the wall seen so far*.
The rover moves 0.3 m per tick, a bit more wall appears, the planned crossing is
now blocked, and it replans again. The path lengths show the crossing point
creeping upward: 26.7 → 27.3 → 26.5 → … → 24.1 m. Once the real top and the gap
are visible, there are no more replans. Each one was triggered by a new obstacle
cell on the path ahead, which is why the scorer counts 0 needless. The same
clustering on consecutive ticks shows up in random seed 42 (12.2, 12.4, 12.6 s).

**Pop-ups get a same-tick reaction.** Seed 42: `rock ahead #1` appears at tick
55 and I replan at tick 55; `#2` at tick 72, replan at tick 72. Seed 99: ticks 26
and 86, both same tick. `popup`: tick 17 and tick 95, both same tick.

### Unknown cells

My planner treats **unknown cells as free** (`astar(..., unknown_is_free=True)`),
and `path_is_valid` treats them as fine.

The rover only sees an 8 m circle, but the goal is 36.8 m away. On the very first
tick, the goal cell and almost the whole map are unknown. Under the other choice
(unknown = blocked), the goal itself counts as blocked, so `astar` returns `None`
immediately and the planner publishes an empty path: the rover never moves.

I checked this on `open` by temporarily switching the flag to `False`:

```
[planner t=   0.0s] NO ROUTE (initial plan) -> published empty path (stop)
...
 Result             STALLED: rover made no progress for 30 s (gave up at t=30.0 s)
 Distance driven    0.0 m   (straight line start to goal is 36.8 m)
```

No route on tick 0, and none for the next 30 s either, even though my planner
retries every tick. It's a deadlock, not a slow start: the rover never moves, so
it never observes new ground, so the goal stays unknown (= blocked) forever. With
the flag on, the same scenario reaches the goal with 1 plan.

My test `test_unknown_blocked_means_no_route_at_all` reproduces it offline with
a tick-0-style grid (an 8 m known circle around the start, everything else
unknown): `None` with the flag off, the 52-step diagonal with it on.

The optimism has a cost, and replanning is what pays it: the plan is only as
good as what's been seen, so it gets corrected as the map fills in (e.g. the 13
replans in `wall`).

### Rover size

Two things in `safe_grid`, which builds the grid both planning and validity
checks use:

1. **Inflate every obstacle by `msg.robot_radius_m` (0.5 m = 1 cell)**, a
   circular neighbourhood around each occupied cell. A* then plans for a point,
   and any route the point can take keeps the rover's centre at least one cell
   from every obstacle.
2. **Mark the arena border as a wall.** Nothing in the grid marks it. The
   outermost ring of cell centres is 0.25 m from the edge, less than the 0.5 m
   radius, so I mark that ring occupied. The next ring is 0.75 m from the edge,
   which is safe. The ring count is computed from the radius, not hard-coded.

Plus one rule in A*: **no corner cutting**. A diagonal move needs both side
cells passable, so the rover can't slip between two obstacles that touch at a
corner.

**Is circular inflation enough?** It leaves an obstacle's diagonal neighbours
free. A rover centre at a diagonal neighbour is √2·0.25 = 0.354 m from the
obstacle square's nearest corner. `world.py` emergency-stops only when the body
(`BODY_RADIUS_M = 0.3`) gets closer than 0.3 m to a cell's square, so 0.354 m is
safe. The 0.5 m planning radius already includes a 0.2 m margin. This is also
covered by a test.

**Without it:** I temporarily set the inflation radius to 0 (keeping the border
wall) and ran `--all`:

```
scenario      result           ticks  blocked  plans  replans  needless
-----------------------------------------------------------------------
open          REACHED            121        0      1        0         0
boulders      STALLED            173      150      3        2         0
wall          STALLED            208      150     14       13         0
popup         STALLED            182      150      2        1         0
goal-blocked  STALLED            187      150      3        2         0
random 1      STALLED            182      150      8        7         0
random 2      STALLED            198      150     10        9         0
random 3      STALLED            180      150      2        1         0
```

Every scenario with an obstacle stalls with exactly **150 blocked ticks**, which
is 30 s at 0.2 s per tick: the simulator's give-up timer. Without inflation, A*
plans through cells directly beside a boulder. A rover centre there is 0.25 m
from the boulder's square, inside its 0.3 m body, so the sim emergency-stops it.
The rover never recovers because my validity check uses the same un-inflated
grid: the waypoint isn't *on* an occupied cell, so the path still looks valid and
no replan is triggered. Inflation is what makes "valid" in my planner mean the
same thing as "collision-free" in the simulator.

### No route

When A* finds no path, I **publish an empty path**, which `rover.py` treats as
"stop". I then retry every tick, without republishing the empty path while
there's still no route.

In `goal-blocked`, a box lands on the goal at tick 102. My path becomes invalid
that same tick, A* finds the goal occupied and returns `None`, and I publish an
empty path (the scorer records a `stop`). I retry each tick until tick 142, the
tick the box clears, and publish a 6.9 m path (scorer: `recovery`). The rover
waited about 8 s and reached the goal with **0 blocked ticks**.

If I published nothing, the rover would keep following its last path, which
leads straight into the box over the goal. Every tick of that would be a
blocked tick, because the rover only stops for an empty path or a collision.

### Time

The monitoring output uses **simulated time, `msg.header.stamp`**. Status
lines print about once per simulated second, plus a line on every plan, and
"time since last replan" is `stamp − last_replan_stamp`.

The simulator doesn't wait for the wall clock: a whole 25 s run finishes in a
fraction of a second. `time.time()` would report near-zero gaps between replans
that are seconds apart in the rover's world, and would give different numbers
with and without `--visualize`, which slows the sim to real time.

### Ambiguity

Calls I had to make where the task left room:

- **Inflate before checking validity, or check the raw grid?** The docstring
  leaves it open. I check the **inflated** grid, because that matches the
  scorer's own "on or beside an obstacle" rule exactly (see Replan trigger),
  and a raw-grid check would keep a path "valid" while it skims a rock.
- **Square or circular inflation?** Either passes the tests. I chose
  **circular**: it's less conservative (it doesn't block diagonal neighbours),
  and I checked against `world.py` that it's still collision-free (0.354 m >
  0.3 m body).
- **How wide is the border wall?** Computed from the radius (cells whose centre
  is closer than `robot_radius_m` to the edge), which is 1 ring here.
- **Which waypoint does "ahead" start at?** `idx + 1`, the waypoint after the
  one nearest the rover, for the reason in Replan trigger.
- **When to retry after no route?** Every tick: A* fails instantly when the goal
  is occupied, so it's cheap, and the goal can clear at any time. I don't
  republish an unchanged empty path.
- **What counts as a "replan" in my counter?** I followed `scoring.py`: only a
  non-empty path replacing a non-empty one. A path after a stop is a recovery,
  not a replan. That's why my count matches the scoreboard in every run.
- **What does the path start and end with?** The rover's cell centre and the
  goal's cell centre, as the `plan` docstring asks. The rover joins the path at
  its nearest point (`rover.py`, within 0.75 m), and the rover's cell centre is
  always within 0.36 m.

### Stretch goals

I didn't attempt the stretch goals; I focused on making Part 1 correct and
explaining it. My planner ignores `confidence` entirely, which is correct for
the clean sensor but would chase every phantom obstacle under `--noise`. What
I'd try first for 2A is in `METHODOLOGY.md` under Known limitations.

---

## Submitting

1. **Create a public repository on your own GitHub account.**
2. **Point your clone at it.** Your clone's `origin` is our repo, which you can't push to:

   ```bash
   git remote set-url origin https://github.com/<your-username>/<your-repo>.git
   git push -u origin main
   ```

3. **Check that it's public.** Open your repo's link in a private/incognito browser window. If you can see the code without logging in, so can we.
4. **Send us the link** in the Google Form you'll be asked to fill out.

Your repo should include your code, your write-up (the section above, in this README), your `METHODOLOGY.md`, your `AI_LOG.md`, and your **full commit history** (push all of it; do not squash).
