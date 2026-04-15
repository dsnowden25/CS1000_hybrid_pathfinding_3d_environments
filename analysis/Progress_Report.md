# Progress Report 1: Pathfinding for Multiple Agents in 3D Environments

**Daymian Snowden**
**CS 5100 – Foundations of AI**
**March 14, 2026**

## What I Have Already Achieved

I completed the project proposal - to compare formation-based, flow field, and adaptive hybrid pathfinding for groups of agents in 3D environments.
We also did a literature review of Sigurdson et al.'s (2018) work on Bounded Multi-Agent A* (BMAA*), which confirmed the tradeoffs between central and decentralized pathfinding.
As predicted, this was particularly noticeable around chokepoint congestion.

On the implementation side, I have the foundational framework built in Python. 
There is a 3D voxel environment that uses a NumPy array. 
Each cell has a terrain type (empty, blocked, ramp, floor) - agents need solid ground beneath them to stand on. 
So far, I built four preset test environments from my proposal: open terrain, a chokepoint with a narrow gap in a wall, an urban grid with buildings and streets, and a multi-level environment with ramps.
I also made the first pathfinding approach — a formation-based leader-follower A* algorithm, like what Mount & Blade: Bannerlord uses.
As far as I can tell, it is working. 
The leader runs A* on the 3D grid, and followers try to maintain their positions in either column or line formation. 
This is performed by a simulation program (driver), which also collects the following evaluation metrics: completion rate, steps to goal, computation time, leader and follower travel distance, formation coherence, unit type positioning preservation, and collision frequency.
The preliminary results already suggest that line formations degrade in constrained spaces with more agents, which is a core project problem.

The codebase is structured with the pathfinding approaches in their own subpackage, where each strategy is its own separate module:

```
pathfinding_project/
├── environment.py          # 3D voxel grid and terrain generators
├── agents.py               # Agent and Formation classes
├── pathfinding/
│   ├── astar.py            # A* (implemented)
│   ├── flow_field.py       # flow field (next)
│   └── hybrid.py           # adaptive switcher (last)
├── simulation.py           # scenario runner and metrics
└── README.md
```

## Next Steps

My immediate priorities are implementing the flow field pathfinding algorithm.
I also need to add local collision avoidance between agents. 
Once both formation-based and flow field are working, I will start on the adaptive hybrid system that switches between them based on terrain analysis.

## Challenges/Adjustments

One challenge will be designing a terrain analysis for the hybrid approach — specifically, how do we classify terrain as "open" versus "constrained" in real time.
I am considering a precomputation step using distance transforms on the voxel grid and annotates path widths. 
Otherwise, there haven't been any major adjustments to the original proposal.

## Completion Plan

Over the next month, I need to: 
1. Implement flow field pathfinding
2. Implement hybrid switcher
3. Build any remaining test scenarios
4. May need to add/remove/adjust test scenarios and/or metrics
5. agents.py only configured for formation-based, will need to make more general or refactor
6. Compare results between the three approaches
7. Write the final report with analysis.
