"""
hybrid.py - Adaptive Hybrid Pathfinding Helpers

Contains the terrain analysis logic (distance transforms) used to determine
when the HybridGroup should switch between formation and flow field modes.
"""

from environment import Environment
from collections import deque


def compute_walkable_width(env: Environment) -> dict:
    """
    Compute the "clearance" (distance to nearest obstacle) for every
    walkable cell using a BFS-based distance transform.
    """
    width_map = {}
    queue = deque()

    walkable = set()
    for x in range(env.width):
        for y in range(env.depth):
            for z in range(env.height):
                if env.is_walkable(x, y, z):
                    walkable.add((x, y, z))

    for cell in walkable:
        x, y, z = cell
        is_border = False
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = x + dx, y + dy
            if not env.is_walkable(nx, ny, z):
                is_border = True
                break
        if is_border:
            width_map[cell] = 1
            queue.append(cell)

    while queue:
        cell = queue.popleft()
        current_width = width_map[cell]
        x, y, z = cell
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nb = (x + dx, y + dy, z)
            if nb in walkable and nb not in width_map:
                width_map[nb] = current_width + 1
                queue.append(nb)

    for cell in walkable:
        if cell not in width_map:
            width_map[cell] = max(width_map.values()) if width_map else 1

    return width_map


def classify_terrain(width_map: dict, threshold: int = 3) -> dict:
    """
    Classify each cell as 'open' or 'constrained' based on clearance.
    """
    return {
        cell: ("constrained" if w <= threshold else "open")
        for cell, w in width_map.items()
    }


def analyze_path_terrain(path: list, terrain_classes: dict) -> dict:
    """
    Analyze what fraction of a path passes through open vs constrained terrain.
    """
    if not path:
        return {"open_pct": 0.0, "constrained_pct": 0.0, "total_cells": 0}

    open_count = sum(
        1 for cell in path
        if terrain_classes.get(cell, "open") == "open"
    )
    total = len(path)

    return {
        "open_pct": round(100 * open_count / total, 1),
        "constrained_pct": round(100 * (total - open_count) / total, 1),
        "total_cells": total,
    }