"""
astar.py

A* search on the 3D voxel grid. Uses Manhattan distance as heuristic.
Elevation changes cost a bit more (1.5 vs 1.0 for flat movement).
"""

import heapq
from environment import Environment


def heuristic(a: tuple, b: tuple) -> float:
    """3D Manhattan distance heuristic."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1]) + abs(a[2] - b[2])


def astar(env, start, goal):
    """
    A* on the 3D grid. Returns list of (x,y,z) from start to goal,
    or None if there's no path.
    """
    if not env.is_walkable(*start) or not env.is_walkable(*goal):
        return None

    open_set = []
    heapq.heappush(open_set, (0, start))

    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    closed_set = set()

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            return _reconstruct_path(came_from, current)

        if current in closed_set:
            continue
        closed_set.add(current)

        for neighbor in env.get_walkable_neighbors(*current):
            if neighbor in closed_set:
                continue

            # movement cost: 1 for horizontal, 1.5 for elevation change
            dz = abs(neighbor[2] - current[2])
            move_cost = 1.5 if dz > 0 else 1.0

            tentative_g = g_score[current] + move_cost

            if tentative_g < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None  # no path found


def _reconstruct_path(came_from: dict, current: tuple) -> list:
    """Trace back from goal to start."""
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path
