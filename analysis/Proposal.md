# Pathfinding for Multiple Agents in 3D Environments

**Daymian Snowden**
**CS 5100 - Foundations of AI**
**02/04/2026**

## Project Overview

So, I play a lot of video games. One of my favorite video games is Mount and Blade: Bannerlord. It's a silly game where you start as basically nothing and eventually can command armies in great battles. Now, my issue is that the pathfinding of NPCs in chokepoints...well, it's bad. Granted, there are a lot of things going on - 100s of 3D NPCs can be actively fighting for their lives. Projectiles are going everywhere, there are people on horses, etc. I understand now there are practical limits to what computers can do. But I'm also learning that most problems are just a matter of time. They might not have had time to figure out a better solution, but I might!

For my project, I want to look at the different pathfinding strategies available, specifically concerning groups of agents in varied 3D-environments. There are three approaches I would consider:
1) formation-based pathfinding (used in Mount and Blade: Bannerlord)
2) flow field pathfinding (used in games like Supreme Commander 2)
3) an adaptive hybrid approach (switches between formation-based and flow field pathfinding depending on the terrain)

## The Problem

As someone who used to be in the Army, I can tell you that moving large groups of people can be a pain. There are two primary interests, and they stand at odds with each other. Do we maintain organized formations because they look nice and can be tactically advantageous, or do we give up coherence in favor of speed and efficiency in constrained spaces? Formation-based methods maintain organization, but they inevitably bottleneck in tight spaces. Flow field methods handle constrained and crowded environments better, but the result looks messy.

## Proposed Solution

I will implement and compare three approaches in a variety of environments.

1) **Formation-Based (Leader-Follower):** There is a single A* pathfinding call for the formation leader. Meanwhile, the other agents (followers) maintain their relative positions within a formation template.

2) **Flow Field:** We generate a direction field toward the goal, allowing all agents in the group to follow the flow independently with local collision avoidance.

3) **Adaptive Hybrid:** Dynamically switch between formation-based and flow field approaches based on terrain analysis. We can use available path widths and transition between maintaining formation structure in open terrain and prioritizing throughput in constrained spaces.

## Implementation

**The Environment:** A 3D voxel grid (numpy array) will represent walkable/blocked spaces, with multiple elevation levels.

**Agents:** Units with their position, unit type (melee/ranged), and collision radius. Units will be assigned to a formation. There will be at least two formation types, column and line.

**Potential Test Scenarios include:**
- Open terrain navigation
- Narrow pass/chokepoint traversal  
- Multi-level pathfinding (stairs/ramps/hills)
- Urban navigation (buildings/alleys/streets)
- Multiple formations with similar/different goals

**Potential Evaluation Metrics include:**
- Computation time
- Path length
- Time to goal
- Formation coherence (the deviation from ideal positions)
- Unit type positioning preservation (melee/ranged order)
- Collision frequency
- Scalability (# of agents, # of formations)

**Goal:**
Determine if there is a good middle ground for pathfinding that more games could use, especially those with groups of agents navigating complex, realistic environments.