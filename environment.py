"""
environment.py

3D voxel grid for the pathfinding project. 

The grid is a numpy array where each cell has a terrain type. 

Agents require something solid beneath them to stand on, no floating through the air.

Also includes preset environment-generators for testing.
"""

import numpy as np
from enum import IntEnum


class Terrain(IntEnum):
    """Terrain types for voxel cells."""
    EMPTY = 0       # walkable open space
    BLOCKED = 1     # wall / obstacle
    RAMP = 2        # connects elevation levels
    FLOOR = 3       # solid ground (agents stand on top)


class Environment:
    """
    3D voxel grid environment.
    
    The grid is indexed as (x, y, z) where:
        x, y = horizontal plane
        z = elevation

    A cell is walkable if...
        1) it is EMPTY
        2) has FLOOR or BLOCKED beneath it
    (the agent needs something to stand on).
    """

    def __init__(self, width: int, depth: int, height: int):
        self.width = width
        self.depth = depth
        self.height = height
        self.grid = np.full((width, depth, height), Terrain.EMPTY, dtype=np.int8)
        
        # Ground floor
        # Everything at z=0 is solid
        self.grid[:, :, 0] = Terrain.FLOOR

    def in_bounds(self, x: int, y: int, z: int) -> bool:
        return 0 <= x < self.width and 0 <= y < self.depth and 0 <= z < self.height

    def is_walkable(self, x: int, y: int, z: int) -> bool:
        """Check if an agent can stand at (x, y, z)."""
        if not self.in_bounds(x, y, z):
            return False
        if self.grid[x, y, z] != Terrain.EMPTY:
            return False
        # need solid ground or ramp below
        if z == 0:
            return False  # z=0 is the floor layer itself
        below = self.grid[x, y, z - 1]
        return below in (Terrain.FLOOR, Terrain.BLOCKED, Terrain.RAMP)

    def get_walkable_neighbors(self, x: int, y: int, z: int):
        """
        Return walkable neighbors for pathfinding.
        
        Supports 4-directional movement on the same level,
        plus vertical transitions via ramps.
        """
        neighbors = []
        # cardinal directions on same elevation
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = x + dx, y + dy
            if self.is_walkable(nx, ny, z):
                neighbors.append((nx, ny, z))
            # check ramp: can we go up or down one level?
            if self.in_bounds(nx, ny, z) and self.grid[nx, ny, z] == Terrain.RAMP:
                # going up
                if self.is_walkable(nx, ny, z + 1):
                    neighbors.append((nx, ny, z + 1))
            if z > 1 and self.in_bounds(nx, ny, z - 1) and self.grid[nx, ny, z - 1] == Terrain.RAMP:
                # going down
                if self.is_walkable(nx, ny, z - 1):
                    neighbors.append((nx, ny, z - 1))

        return neighbors

    def set_block(self, x: int, y: int, z: int):
        """Place a solid block (wall/obstacle)."""
        if self.in_bounds(x, y, z):
            self.grid[x, y, z] = Terrain.BLOCKED

    def set_ramp(self, x: int, y: int, z: int):
        """Place a ramp for vertical transitions."""
        if self.in_bounds(x, y, z):
            self.grid[x, y, z] = Terrain.RAMP
            # make sure the cell above the ramp is walkable (clear it)
            if self.in_bounds(x, y, z + 1):
                self.grid[x, y, z + 1] = Terrain.EMPTY
                # and there's floor on top of the ramp destination level
                # (the ramp itself acts as floor for z+1)

    def build_wall(self, x: int, y: int, z_base: int, wall_height: int):
        """Build a vertical wall starting at z_base."""
        for dz in range(wall_height):
            self.set_block(x, y, z_base + dz)

    def __repr__(self):
        return f"Environment({self.width}x{self.depth}x{self.height})"


# preset environment generators for testing below

def create_open_terrain(width=30, depth=30, height=5) -> Environment:
    """Wide open field — baseline test for formation movement."""
    env = Environment(width, depth, height)
    # just a flat ground - already set up by __init__
    return env


def create_chokepoint(width=30, depth=30, height=5, gap_width=3) -> Environment:
    """
    Open field with a wall across the middle, with a narrow gap.
    
    This is the core test case — formation-based pathfinding should
    struggle here while flow field should handle it better.
    """
    env = Environment(width, depth, height)
    mid_y = depth // 2
    gap_start = (width - gap_width) // 2

    for x in range(width):
        if x < gap_start or x >= gap_start + gap_width:
            for z in range(1, height):
                env.build_wall(x, mid_y, 1, height - 1)
    return env


def create_urban(width=30, depth=30, height=5) -> Environment:
    """
    Grid of buildings with streets/alleys between them.
    Buildings are 4x4 blocks with 2-wide streets.
    """
    env = Environment(width, depth, height)
    building_size = 4
    street_width = 2
    cell_size = building_size + street_width

    for bx in range(0, width, cell_size):
        for by in range(0, depth, cell_size):
            for x in range(bx, min(bx + building_size, width)):
                for y in range(by, min(by + building_size, depth)):
                    for z in range(1, height):
                        env.set_block(x, y, z)
    return env


def create_multilevel(width=20, depth=20, height=8) -> Environment:
    """
    Two-level environment connected by ramps.
    Lower level at z=1, upper platform at z=3, ramp connecting them.
    """
    env = Environment(width, depth, height)

    # upper platform: raised floor on the right half
    platform_start_x = width // 2
    for x in range(platform_start_x, width):
        for y in range(depth):
            env.grid[x, y, 2] = Terrain.FLOOR  # platform floor at z=2
            # walkable space is at z=3

    # ramp connecting z=1 (ground) to z=3 (on top of platform)
    ramp_y = depth // 2
    env.set_ramp(platform_start_x - 1, ramp_y, 1)  # ramp at ground level
    env.grid[platform_start_x - 1, ramp_y, 2] = Terrain.RAMP  # ramp continues
    # clear space above ramp
    env.grid[platform_start_x, ramp_y, 3] = Terrain.EMPTY

    return env
