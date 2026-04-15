# Pathfinding for Multiple Agents in 3D Environments
# Planning

Daymian Snowden — CS 5100, Foundations of AI

## Overview

We are comparing three pathfinding strategies for groups of agents in 3D environments:

1. **Formation-based (leader-follower)** — one A* call for the leader, followers maintain offsets - Mount & Blade: Bannerlord.
2. **Flow field** — shared direction field toward the goal, agents follow independently with local collision avoidance.
    * *To-Do*
3. **Adaptive hybrid** — switches between formation and flow field based on terrain width.
    * *To-Do*

Formation pathfinding breaks down in chokepoints — agents pile up instead of adapting.
Can we fix that?

## Structure

```
pathfinding_project/
├── environment.py          # 3D voxel grid and terrain generators
├── agents.py               # Agent & Formation classes
├── pathfinding/
│   ├── __init__.py
│   ├── astar.py            # A* on 3D grid
│   ├── flow_field.py       # (stub)
│   └── hybrid.py           # (stub)
├── simulation.py           # scenario runner and metrics
└── README.md
```

## Required tools

Python 3.10+, NumPy

```bash
pip install numpy
```

## Main Program

```bash
python simulation.py
```

Runs the formations across test environments and reports evaluation metrics.

## Current status

- [x] 3D voxel environment with terrain types
- [x] Preset environments (open, chokepoint, urban, multi-level)
- [x] A* pathfinding with elevation support
- [x] Leader-follower formations (column and line)
- [x] Metrics: completion, travel distance, coherence, unit positioning, collisions, scalability
- [ ] Flow field pathfinding
- [ ] Local collision avoidance
- [ ] Adaptive hybrid
- [ ] Visualization
- [ ] Anything else?

## Current References

Sigurdson, D., Bulitko, V., Yeoh, W., Hernandez, C., & Koenig, S. (2018). Multi-agent pathfinding with real-time heuristic search. *IEEE CIG*, pp. 1–8.
