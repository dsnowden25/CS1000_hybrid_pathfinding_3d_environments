"""
environment_complex.py

Advanced environments for multi-group and complex MAPF testing.
"""

from environment import Environment, Terrain

def create_crossroads(width=40, depth=40, height=5, corridor_width=6):
    """
    A four-way intersection.
    Creates a cross-shaped walkable area surrounded by walls.
    Perfect for testing two groups crossing paths perpendicularly.
    """
    env = Environment(width, depth, height)
    
    # First, fill the entire environment with solid blocks above the floor
    for x in range(width):
        for y in range(depth):
            for z in range(1, height):
                env.set_block(x, y, z)
                
    # Carve out the East-West corridor
    start_y = (depth - corridor_width) // 2
    end_y = start_y + corridor_width
    for x in range(width):
        for y in range(start_y, end_y):
            for z in range(1, height):
                env.grid[x, y, z] = Terrain.EMPTY
                
    # Carve out the North-South corridor
    start_x = (width - corridor_width) // 2
    end_x = start_x + corridor_width
    for x in range(start_x, end_x):
        for y in range(depth):
            for z in range(1, height):
                env.grid[x, y, z] = Terrain.EMPTY
                
    return env


def create_serpentine(width=40, depth=40, height=5):
    """
    Creates an S-shaped winding canyon.
    Formations will struggle with the 180-degree turns and coherence loss,
    while flow fields should smoothly hug the inside curves.
    """
    env = Environment(width, depth, height)
    
    # Wall 1: Blocks the left side, forces path to the right
    for x in range(0, width - 10):
        for y in range(10, 14):
            for z in range(1, height):
                env.set_block(x, y, z)
                
    # Wall 2: Blocks the right side, forces path back to the left
    for x in range(10, width):
        for y in range(24, 28):
            for z in range(1, height):
                env.set_block(x, y, z)
                
    return env


def create_pillar_field(width=40, depth=40, height=5):
    """
    A field filled with evenly spaced pillars.
    Highly porous but constantly forces agents to adjust, 
    shattering rigid formations and testing local collision avoidance.
    """
    env = Environment(width, depth, height)
    
    # Leave a safe spawn zone at the bottom (y < 8) 
    # and safe goal zone at the top (y > 32)
    for x in range(2, width - 2, 5):
        for y in range(8, depth - 8, 5):
            # Create a 2x2 pillar
            for px in range(2):
                for py in range(2):
                    for z in range(1, height):
                        if env.in_bounds(x + px, y + py, z):
                            env.set_block(x + px, y + py, z)
                            
    return env