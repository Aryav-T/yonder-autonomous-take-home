"""
Planner node — your implementation goes here.

Run via:
    python sim/launch.py                       # default scenario
    python sim/launch.py --scenario open       # easiest world — start here
    python sim/launch.py --visualize           # with a live plot
    python sim/launch.py --all                 # every scenario, one summary table

This file uses rclpy_lite, a lightweight simulator shim that mirrors the real
rclpy (ROS 2 Python) API. To run on a real ROS 2 system, swap the imports back
to plain rclpy.

Division of labour
------------------
    grid_utils.py   grid maths (world <-> cell, inflating obstacles)     no ROS
    search.py       A*                                                    no ROS
    replan.py       "is my plan still valid? where am I on it?"           no ROS
    planner_node.py THIS FILE: receives grids, decides when to replan,
                    publishes paths, prints monitoring output              ROS glue
"""

import rclpy_lite as rclpy
from rclpy_lite.node import Node
from rclpy_lite.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sim.messages import GridSnapshot, make_path_msg
from sim.grid_values import OCCUPIED
from nav_msgs.msg import Path

from grid_utils import Grid, inflate, path_length_m
from search import astar
from replan import next_waypoint_index, path_is_valid

# Must be compatible with the feed: grid_feed_publisher.py publishes BEST_EFFORT,
# depth 1. A RELIABLE subscriber would ask for more than is offered -> no connection.
GRID_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=1,
)

STATUS_PERIOD_S = 1.0   # print a status line about once per simulated second


class PlannerNode(Node):
    """
    Plans a path from the rover to the goal and keeps it up to date as the map changes.

    Subscribes:
        /grid_feed     (sim.messages.GridSnapshot)   5 Hz, BEST_EFFORT
    Publishes:
        /planned_path  (nav_msgs/Path)               only when the plan changes
                       waypoints in WORLD coordinates; an EMPTY path means "stop"
    """

    def __init__(self):
        super().__init__("planner_node")

        self.grid_sub = self.create_subscription(
            GridSnapshot, "/grid_feed", self.on_grid, GRID_QOS
        )
        # Default (RELIABLE) is fine: the rover subscribes RELIABLE.
        self.path_pub = self.create_publisher(Path, "/planned_path", 10)

        # State
        self.path_xy = None            # current plan (world points). None = never planned, [] = no route
        self.plan_count = 0            # non-empty plans published (initial included)
        self.replan_count = 0          # plans that replaced a non-empty plan
        self.last_replan_stamp = None  # sim time of most recent replan
        self.last_print_stamp = None   # sim time of last status line

    # -----------------------------------------------------------------------
    # The callback: runs once per tick
    # -----------------------------------------------------------------------

    def on_grid(self, msg: GridSnapshot) -> None:
        stamp = msg.header.stamp
        grid = self.safe_grid(msg)
        rover_xy = (msg.rover_x, msg.rover_y)
        goal_xy = (msg.goal_x, msg.goal_y)

        replan, reason = self.needs_replan(grid, rover_xy)
        if replan:
            new_path = self.plan(grid, rover_xy, goal_xy)
            self.publish_path(new_path, stamp)
            self.record_plan(new_path, stamp, reason)

        self.print_status(stamp)

    def safe_grid(self, msg: GridSnapshot) -> Grid:
        """
        The grid the planner treats as truth: obstacles inflated by the rover's
        radius, plus the arena border marked as a wall.
        """
        grid = inflate(Grid.from_msg(msg), msg.robot_radius_m)

        # Border: a cell is unsafe if its centre is closer than the radius to the
        # arena edge. Ring k (0 = outermost) has its centre (k + 0.5) * res from
        # the edge. With 0.5 m cells and 0.5 m radius that is ring 0 only.
        rings = 0
        while (rings + 0.5) * grid.resolution < msg.robot_radius_m:
            rings += 1
        W, H = grid.width, grid.height
        for k in range(rings):
            for c in range(W):
                grid.occupancy[k * W + c] = OCCUPIED
                grid.occupancy[(H - 1 - k) * W + c] = OCCUPIED
            for r in range(H):
                grid.occupancy[r * W + k] = OCCUPIED
                grid.occupancy[r * W + (W - 1 - k)] = OCCUPIED
        return grid

    def needs_replan(self, grid: Grid, rover_xy) -> tuple:
        """
        Step 5: plan once. Only plan if we never have.
        (Validity checking / replanning is added in step 6.)
        """
        if self.path_xy is None:
            return True, "initial plan"
        return False, ""

    def plan(self, grid: Grid, rover_xy, goal_xy):
        """
        A* from the rover's cell to the goal's cell on the safe grid.
        Returns a list of (x, y) world points (cell centres), or [] if no route.
        Unknown cells are treated as free: the rover has only seen an 8 m circle,
        so treating unknown as blocked would make the goal unreachable on tick 1.
        """
        start = grid.world_to_cell(*rover_xy)
        goal = grid.world_to_cell(*goal_xy)
        cells = astar(grid, start, goal, unknown_is_free=True)
        if cells is None:
            return []
        return [grid.cell_to_world(r, c) for r, c in cells]

    def publish_path(self, points_xy, stamp: float) -> None:
        """Publish a list of (x, y) world points as nav_msgs/Path. (provided)"""
        self.path_pub.publish(make_path_msg(points_xy, stamp))

    def record_plan(self, new_path, stamp: float, reason: str) -> None:
        """Update counters to match the scoreboard's definitions, and log the event."""
        had_route = bool(self.path_xy)          # previous plan existed and was non-empty
        if new_path:
            self.plan_count += 1
            if had_route:
                self.replan_count += 1
                self.last_replan_stamp = stamp
            kind = "REPLAN" if had_route else "PLAN"
            print(f"[planner t={stamp:6.1f}s] {kind}: {reason} -> "
                  f"{len(new_path)} waypoints, {path_length_m(new_path):.1f} m")
        else:
            print(f"[planner t={stamp:6.1f}s] NO ROUTE ({reason}) -> published empty path (stop)")
        self.path_xy = new_path
        self.last_print_stamp = None            # force a status line this tick

    # -----------------------------------------------------------------------
    # Monitoring
    # -----------------------------------------------------------------------

    def print_status(self, stamp: float) -> None:
        """About once per simulated second (msg.header.stamp, never time.time())."""
        if self.last_print_stamp is not None and stamp - self.last_print_stamp < STATUS_PERIOD_S:
            return
        self.last_print_stamp = stamp

        length = path_length_m(self.path_xy) if self.path_xy else 0.0
        since = ("never" if self.last_replan_stamp is None
                 else f"{stamp - self.last_replan_stamp:.1f}s ago")
        print(f"[planner t={stamp:6.1f}s] path {length:5.1f} m | "
              f"replans {self.replan_count} | last replan {since}")