# Pathfinding for Multiple Agents in 3D Environments
# Planning

Daymian Snowden — CS 5100, Foundations of AI

## Overview

Formation pathfinding breaks down in chokepoints — agents pile up instead of adapting.

Our size-aware hybrid solution fixes this by scaling switching thresholds to unit size and enforcing global clearance before reforming.

We are comparing three pathfinding strategies for groups of agents in 3D environments:

1. **Formation-based (leader-follower)** — one A* call for the leader, followers maintain offsets. This centralized logic maintains tactical cohesion (templates: Column and Line) but suffers from "Unit Entanglement" in high-congestion chokepoints.
   
2. **Flow field** — shared direction field toward the goal using a Dijkstra-based cost-integration field. Agents follow the local gradient independently. This decentralized approach maximizes throughput and eliminates collisions by treating units as a fluid.
   
3. **Adaptive hybrid** — switches between formation and flow field based on terrain width and unit physical footprint. It uses proactive path analysis and distance transforms to detect bottlenecks before entry, "liquefying" the unit for traversal and "crystallizing" back into formation in open ground.

## Structure
pathfinding_project/
├── environment.py           # 3D voxel grid logic, stability checks, and basic terrain
├── environment_complex.py   # Advanced stress-test scenario generators (Canyon, Crossroads)
├── agents.py                # Core Agent classes and Size-Aware HybridGroup controller
├── pathfinding/
│   ├── init.py
│   ├── astar.py             # 3D A* implementation with elevation weighting
│   ├── flow_field.py        # Dijkstra cost-field generation and gradient descent
│   └── hybrid.py            # Distance transforms and obstacle-clearance analysis
├── simulation_large.py      # Main stress-test driver (7 scenarios) and metrics collection
├── overhead_exploration.py  # Performance profiler comparing O(N) vs O(1) logic
├── generate_plots.py        # Matplotlib script for generating technical visualizations
└── README.md                # Consolidated technical documentation

## Required tools

Python 3.10+, NumPy, Matplotlib

```bash
pip install numpy matplotlib
```

## Main Program
python simulation_large.py

## Technical Performance & Metrics

Collision Mitigation

The hybrid model successfully mitigates the formation paradox. In head-on canyon clashes, formations reached 340+ collisions while the Hybrid system maintained zero collisions.

State Distribution

The system proactively identifies path constraints, remaining in Flow Field mode for 100% of chokepoint navigation and Urban traversal while reserving Formation mode for open terrain.

Logic Scalability Profiling
To ensure the system handles large-scale battles, the switching logic was profiled for scalability. Transitioning from $O(N)$ clearance checks to $O(1)$ Representative Sampling (Head & Tail check) reduced CPU overhead by 99.4% for 500-agent groups.

## Current status
[x] 3D voxel environment with terrain types

[x] Preset environments (open, chokepoint, urban, multi-level)

[x] A* pathfinding with elevation support

[x] Leader-follower formations (column and line)

[x] Metrics: completion, travel distance, coherence, unit positioning, collisions, scalability

[x] Flow field pathfinding (Dijkstra integration field)

[x] Local collision avoidance (Priority-based cell booking)

[x] Adaptive hybrid (Size-aware state-switching)

[x] Visualization suite (Matplotlib data generation)

[x] Scalability Optimization (Representative Sampling Roadmap)

## References

- Sigurdson, D., Bulitko, V., Yeoh, W., Hernandez, C., & Koenig, S. (2018). Multi-agent pathfinding with real-time heuristic search. IEEE CIG, pp. 1–8.
- Treuille, A., Cooper, S., & Popović, Z. (2006). Continuum Crowds. ACM Transactions on Graphics.