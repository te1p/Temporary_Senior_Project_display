"""Navigate with the team's D* Lite planner (dstar_lite_planner.py at the repo root).

Every frame:
  1. Obstacles = every other robot and puck the camera sees, as circles of points.
  2. The planner works on a grid over the field (--dstar-cell-mm), with each obstacle
     grown by our robot's radius, so a path of cell centers is a path the robot's
     center can follow without touching anything.
  3. The robot steers at a "carrot": the first path point at least --lookahead-mm
     away. Near the end of the path it steers at the goal itself.

The same planner object is reused while the goal stays in the same cell, so D* Lite
only repairs the part of its search that changed (robot moved, obstacle moved).
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from dstar_lite_planner import DStarPlanner  # noqa: E402


def circle_points(center, radius, spacing):
    """Points filling a circle, no further apart than spacing, so the planner sees a solid disc."""
    cx, cy = center
    pts = [(cx, cy)]
    r = spacing
    while r <= radius + 1e-6:
        n = max(6, math.ceil(2 * math.pi * r / spacing))
        pts += [(cx + r * math.cos(2 * math.pi * k / n), cy + r * math.sin(2 * math.pi * k / n)) for k in range(n)]
        r += spacing
    return pts


class DStarNav:
    def __init__(self, field_w, field_h, cell_mm, robot_radius_mm, lookahead_mm):
        self.field_w, self.field_h = field_w, field_h
        self.cell = cell_mm
        self.robot_radius = robot_radius_mm
        self.lookahead = lookahead_mm
        self.planner = None
        self.path = []        # last path, field mm, start to goal
        self.obstacles = []   # last obstacles: (center, radius) in mm

    def _new_planner(self):
        # The grid must be a whole number of cells wide and tall. If it isn't,
        # DStarPlanner lets paths use one more row and column than its obstacle map
        # covers, so a path can run along that edge straight through an obstacle.
        # So the grid stops at the last whole cell (at most one cell short of the field).
        self.grid_w = math.floor(self.field_w / self.cell) * self.cell
        self.grid_h = math.floor(self.field_h / self.cell) * self.cell
        # Grow obstacles by half a cell more than the robot's radius: the planner only checks
        # cell centers, and the robot drives between them, so this keeps real paths clear.
        return DStarPlanner(0.0, 0.0, float(self.grid_w), float(self.grid_h), float(self.cell),
                            float(self.robot_radius + self.cell / 2))

    def _inside(self, p):
        """Clamp to a point whose grid cell the planner accepts."""
        top_x = self.grid_w - self.cell * 0.51
        top_y = self.grid_h - self.cell * 0.51
        return float(np.clip(p[0], 0, top_x)), float(np.clip(p[1], 0, top_y))

    def step(self, robot_xy, goal_xy, obstacles):
        """obstacles: list of ((x, y), radius_mm).
        Returns (target_xy, is_final, note, remaining_mm): where to steer now, whether that is
        the goal itself, a note when something is wrong, and the path length still to go.
        target_xy is None when there is no way to the goal (note says why)."""
        self.obstacles = obstacles
        robot, goal = np.array(robot_xy, float), np.array(goal_xy, float)

        # Touching something already (inside its grown radius): back straight away first.
        for center, radius in obstacles:
            away = robot - np.array(center)
            reach = radius + self.robot_radius
            if np.hypot(*away) < reach:
                away = away / np.hypot(*away) if away.any() else np.array([0.0, -1.0])
                self.path = []
                return self._inside(np.array(center) + (reach + 40) * away), False, \
                    "backing away from an obstacle", None
        for center, radius in obstacles:
            if np.hypot(*(goal - np.array(center))) < radius + self.robot_radius:
                self.path = []
                return None, False, "target is blocked: too close to a robot or puck", None

        points = [p for center, radius in obstacles for p in circle_points(center, radius, self.cell / 2)]
        if self.planner is None:
            self.planner = self._new_planner()
        path = self.planner.plan(self._inside(robot), self._inside(goal), points)
        if not path:
            # Its search can get stuck after big changes; one fresh try before giving up.
            self.planner = self._new_planner()
            path = self.planner.plan(self._inside(robot), self._inside(goal), points)
        if not path:
            self.path = []
            return None, False, "no path found", None

        path = [np.array(p, float) for p in path]
        path[-1] = goal  # end exactly on the goal, not its cell center
        path[0] = robot
        self.path = path
        remaining = float(sum(np.hypot(*(b - a)) for a, b in zip(path, path[1:])))
        for p in path[1:]:
            if np.hypot(*(p - robot)) >= self.lookahead:
                return (float(p[0]), float(p[1])), p is path[-1], "", remaining
        return (float(goal[0]), float(goal[1])), True, "", remaining
