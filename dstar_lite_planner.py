"""
dstar_lite_planner.py

The D* Lite search structure and update logic are retained from the original
DStarPlanner class. LIMO-specific navigation, sensing, communication,
visualization, and motion-control code are intentionally excluded.

Coordinates:
    World coordinates are (x, y).
    Grid nodes are integer (ix, iy) indices.
    Obstacles are supplied as parallel x/y coordinate collections, matching
    the original planner's process(..., ox, oy) interface.

Typical use:
    planner = DStarPlanner(
        min_x=0.0, min_y=0.0,
        max_x=8.0, max_y=6.0,
        resolution=0.2,
        rr=0.15,
    )

    path_x, path_y = planner.process(
        sx=1.0, sy=1.0,
        gx=7.0, gy=5.0,
        ox=[3.0, 3.0, 3.0],
        oy=[2.0, 2.2, 2.4],
    )

    waypoints = list(zip(path_x, path_y))

Call process again with the robot's updated start position and the latest
obstacle coordinates to update the path. Reuse the same planner instance
while the goal remains the same to retain D* Lite's incremental search state.
"""

import heapq
import math


class DStarPlanner:
    """Incremental D* Lite planner operating on a two-dimensional grid.

    Args:
        min_x, min_y: Lower world-coordinate bounds of the grid.
        max_x, max_y: Upper world-coordinate bounds of the grid.
        resolution: World distance represented by one grid cell.
        rr: Obstacle inflation radius. Obstacle points within this radius of
            a grid-cell center mark that cell as occupied.
    """

    def __init__(self, min_x, min_y, max_x, max_y, resolution, rr):
        self.min_x = min_x
        self.min_y = min_y
        self.max_x = max_x
        self.max_y = max_y
        self.resolution = resolution
        self.rr = rr

        if resolution <= 0:
            raise ValueError("resolution must be positive")
        if max_x <= min_x or max_y <= min_y:
            raise ValueError("grid bounds must have positive width and height")
        if rr < 0:
            raise ValueError("obstacle inflation radius cannot be negative")

        self.x_width = round((max_x - min_x) / resolution)
        self.y_width = round((max_y - min_y) / resolution)

        self.map = [
            [False for _ in range(self.y_width)]
            for _ in range(self.x_width)
        ]

        self.g = {}
        self.rhs = {}
        self.U = []
        self.km = 0.0
        self.s_start = None
        self.s_goal = None

        # Eight-connected grid: four cardinal and four diagonal movements.
        self.motion = [
            [1, 0, 1],
            [0, 1, 1],
            [-1, 0, 1],
            [0, -1, 1],
            [-1, -1, 1.414],
            [-1, 1, 1.414],
            [1, -1, 1.414],
            [1, 1, 1.414],
        ]

    # Convert continuous coordinates to a grid index.
    def calc_grid_index(self, x, y):
        return (
            (y - self.min_y) * self.x_width
            + (x - self.min_x)
        )

    def calc_xy_index(self, pos, min_pos):
        return round((pos - min_pos) / self.resolution)

    def calc_grid_pos(self, index, min_pos):
        return index * self.resolution + min_pos

    def heuristic(self, s1, s2):
        return math.hypot(s1[0] - s2[0], s1[1] - s2[1])

    # Calculate priority keys for the heap.
    def calculate_key(self, s):
        k1 = (
            min(
                self.g.get(s, float("inf")),
                self.rhs.get(s, float("inf")),
            )
            + self.heuristic(self.s_start, s)
            + self.km
        )
        k2 = min(
            self.g.get(s, float("inf")),
            self.rhs.get(s, float("inf")),
        )
        return (k1, k2)

    # Update the consistency of a vertex.
    def update_vertex(self, u):
        if u != self.s_goal:
            self.rhs[u] = float("inf")

            for dx, dy, cost in self.motion:
                s_prime = (u[0] + dx, u[1] + dy)
                if not self.is_valid(s_prime):
                    continue

                if self.is_obstacle(s_prime):
                    edge_cost = float("inf")
                else:
                    edge_cost = cost

                self.rhs[u] = min(
                    self.rhs[u],
                    self.g.get(s_prime, float("inf")) + edge_cost,
                )

        self.U = [item for item in self.U if item[1] != u]
        heapq.heapify(self.U)

        if self.g.get(u, float("inf")) != self.rhs.get(u, float("inf")):
            heapq.heappush(self.U, (self.calculate_key(u), u))

    # Check whether a grid node lies inside the configured world bounds.
    def is_valid(self, s):
        px = self.calc_grid_pos(s[0], self.min_x)
        py = self.calc_grid_pos(s[1], self.min_y)

        return not (
            px < self.min_x
            or py < self.min_y
            or px >= self.max_x
            or py >= self.max_y
        )

    # Check whether a grid cell is blocked.
    def is_obstacle(self, s):
        if 0 <= s[0] < self.x_width and 0 <= s[1] < self.y_width:
            return self.map[s[0]][s[1]]
        return False

    # Compute the shortest path by processing inconsistent vertices.
    def compute_shortest_path(self):
        while (
            self.U
            and (
                self.U[0][0] < self.calculate_key(self.s_start)
                or self.rhs.get(self.s_start, float("inf"))
                != self.g.get(self.s_start, float("inf"))
            )
        ):
            k_old, u = heapq.heappop(self.U)
            k_new = self.calculate_key(u)

            if k_old < k_new:
                heapq.heappush(self.U, (k_new, u))

            elif self.g.get(u, float("inf")) > self.rhs.get(u, float("inf")):
                self.g[u] = self.rhs[u]

                for dx, dy, cost in self.motion:
                    s_prime = (u[0] + dx, u[1] + dy)
                    if self.is_valid(s_prime):
                        self.update_vertex(s_prime)

            else:
                self.g[u] = float("inf")
                self.update_vertex(u)

                for dx, dy, cost in self.motion:
                    s_prime = (u[0] + dx, u[1] + dy)
                    if self.is_valid(s_prime):
                        self.update_vertex(s_prime)

    # Main planner interface: update the obstacle map and find a path.
    def process(self, sx, sy, gx, gy, ox, oy):
        """Calculate a path from (sx, sy) to (gx, gy).

        Args:
            sx, sy: Start position in world coordinates.
            gx, gy: Goal position in world coordinates.
            ox, oy: Parallel iterables of obstacle x and y coordinates.

        Returns:
            (rx, ry): Parallel lists of world-coordinate path waypoints.
            Both lists are empty if no path can be extracted.
        """
        ox = list(ox)
        oy = list(oy)
        if len(ox) != len(oy):
            raise ValueError("ox and oy must contain the same number of points")

        start_node = (
            self.calc_xy_index(sx, self.min_x),
            self.calc_xy_index(sy, self.min_y),
        )
        goal_node = (
            self.calc_xy_index(gx, self.min_x),
            self.calc_xy_index(gy, self.min_y),
        )

        if not self.is_valid(start_node) or not self.is_valid(goal_node):
            raise ValueError("start and goal must lie inside the planner bounds")

        # Initialize on the first call or when the goal changes.
        if self.s_goal is None or self.s_goal != goal_node:
            self.s_start = start_node
            self.s_goal = goal_node
            self.U = []
            self.km = 0.0
            self.rhs = {}
            self.g = {}

            self.rhs[self.s_goal] = 0
            heapq.heappush(
                self.U,
                (self.calculate_key(self.s_goal), self.s_goal),
            )

        # Update the start position as the robot moves.
        if self.s_start != start_node:
            self.km += self.heuristic(self.s_start, start_node)
            self.s_start = start_node

        # Rebuild occupancy grid from the latest obstacle coordinates.
        new_map = [
            [False for _ in range(self.y_width)]
            for _ in range(self.x_width)
        ]

        for x, y in zip(ox, oy):
            min_ix = self.calc_xy_index(x - self.rr, self.min_x)
            max_ix = self.calc_xy_index(x + self.rr, self.min_x)
            min_iy = self.calc_xy_index(y - self.rr, self.min_y)
            max_iy = self.calc_xy_index(y + self.rr, self.min_y)

            for i in range(max(0, min_ix), min(self.x_width, max_ix + 1)):
                for j in range(max(0, min_iy), min(self.y_width, max_iy + 1)):
                    cx = self.calc_grid_pos(i, self.min_x)
                    cy = self.calc_grid_pos(j, self.min_y)

                    if math.hypot(cx - x, cy - y) <= self.rr:
                        new_map[i][j] = True

        # Find changed cells and update those cells and their neighbors.
        changed_nodes = []
        for x in range(self.x_width):
            for y in range(self.y_width):
                if self.map[x][y] != new_map[x][y]:
                    changed_nodes.append((x, y))

        self.map = new_map

        if changed_nodes:
            for u in changed_nodes:
                self.update_vertex(u)

                for dx, dy, _ in self.motion:
                    v = (u[0] + dx, u[1] + dy)
                    if self.is_valid(v):
                        self.update_vertex(v)

        self.compute_shortest_path()
        return self.extract_path()

    # Reconstruct a path from start to goal by following the cost gradient.
    def extract_path(self):
        if self.g.get(self.s_start, float("inf")) == float("inf"):
            return [], []

        rx = []
        ry = []
        curr = self.s_start

        rx.append(self.calc_grid_pos(curr[0], self.min_x))
        ry.append(self.calc_grid_pos(curr[1], self.min_y))

        visited = {curr}

        while curr != self.s_goal:
            min_cost = float("inf")
            next_node = None

            for dx, dy, cost in self.motion:
                s_prime = (curr[0] + dx, curr[1] + dy)
                if not self.is_valid(s_prime):
                    continue

                if self.is_obstacle(s_prime):
                    edge_cost = float("inf")
                else:
                    edge_cost = cost

                candidate_cost = (
                    self.g.get(s_prime, float("inf")) + edge_cost
                )

                if candidate_cost < min_cost:
                    min_cost = candidate_cost
                    next_node = s_prime

            if (
                next_node is None
                or next_node == curr
                or next_node in visited
            ):
                return [], []

            curr = next_node
            visited.add(curr)

            rx.append(self.calc_grid_pos(curr[0], self.min_x))
            ry.append(self.calc_grid_pos(curr[1], self.min_y))

            if len(rx) > self.x_width * self.y_width:
                return [], []

        return rx, ry

    def plan(self, start, goal, obstacles=()):
        """Convenience interface.

        Args:
            start: (x, y).
            goal: (x, y).
            obstacles: Iterable of (x, y) obstacle points.

        Returns:
            List of (x, y) waypoints. Empty list means no path.
        """
        obstacles = list(obstacles)
        ox = [point[0] for point in obstacles]
        oy = [point[1] for point in obstacles]
        rx, ry = self.process(start[0], start[1], goal[0], goal[1], ox, oy)
        return list(zip(rx, ry))
