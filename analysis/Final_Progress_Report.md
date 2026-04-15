# Progress Report 2: Pathfinding for Multiple Agents in 3D Environments

**Daymian Snowden**
**CS 5100 – Foundations of AI**
**March 30, 2026**

## Completion Plan (From last report)
 
1. Implement flow field pathfinding
   1. Done - notes below
2. Implement hybrid switcher
   1. Completed - note below
3. Build any remaining test scenarios
   1. May add one or two more
4. May need to add/remove/adjust test scenarios and/or metrics
   1. Adjusted
5. agents.py only configured for formation-based, will need to make more general or refactor
   1. Refactored
6. Compare results between the three approaches
   1. Preliminary results - notes below
7. Write the final report with analysis.
   1. Pending

## Literature Update - Continuum Crowds

**Authors:** Adrien Treuille, Seth Cooper, and Zoran Popović

**Peer-Reviewed:** ACM SIGGRAPH 2006, published in ACM Transactions on Graphics, Vol. 25, No. 3, pp. 1160–1168.

**DOI:** 10.1145/1141911.1142008

**Summary:** They present a real-time crowd simulation model that replaces traditional agent-based dynamics with dynamic potential fields. Rather than computing individual paths for each agent, the system generates a global potential field derived from the eikonal equation. The system simultaneously encodes both the optimal direction toward the goal and the presence of moving obstacles (including other people). 

Each agent simply follows the gradient of this potential field, scaled by a density-dependent speed function. One important piece is the velocity-dependent speed term; an agent's maximum speed is influenced by the flow of nearby agents, so that movement against crowd flow is impeded and movement with crowd flow is not impeded. This mechanism naturally produces lane formation when groups walk in opposite directions (which actually happens in real crowds). They also used "predictive discomfort," projecting a small discomfort region ahead of each agent to improve behavior during perpendicular crossings.

The system was tested on scenarios ranging from hallway crossings to a 10,000-person city environment, running at acceptable frame rates. The algorithm's computational cost scales with the number of grid cells rather than the number of agents, making it efficient for very large groups sharing a common goal.

**Relevance:** This paper provided the theoretical foundation for the flow field pathfinding approach - it is one of the oldest and most cited papers on flow field pathfinding specifically in digital spaces. The key advantage they were able to prove tracks with my experiences; field-based navigation scales well for groups because the field is computed once and shared by all agents. The result is that flow field pathfinding should outperform formation-based A* in constrained environments with many agents. They confirm that their approach using flow field agents navigates chokepoints more smoothly than formation-based agents.

My flow field implementation uses a version of the same core idea: however, I have to account for the space being 3D rather than 2D. For this, we use Dijkstra instead of the eikonal equation. Specifically, we use Dijkstra to get the cost-to-goal (integration field) for every walkable cell, and then we can compute a numerical gradient over that cost field to get smooth flow directions. The gradient points "uphill" (away from goal), so we negate it and normalize to get a flow direction. Also, I am using discrete agent groups with unit types (melee/ranged), which the continuum model does not address specifically.

## Literature Update - When to Switch: Planning and Learning for Partially Observable Multi-Agent Pathfinding

**Authors:** Alexey Skrynnik, Anton Andreychuk, Konstantin Yakovlev, and Aleksandr I. Panov

**Peer-Reviewed:** IEEE Transactions on Neural Networks and Learning Systems, Vol. 35, No. 12, December 2024, pp. 17411–17424.

**DOI:** 10.1109/TNNLS.2023.3303502

**Summary:** The authors address multi-agent pathfinding under partial observability (PO-MAPF), where each agent can only observe a limited area around itself and cannot communicate with other agents. They propose two distinct policies. The first is RePlan, a search-based policy that re-runs A* at every timestep using memorized observations. The second is EPOM, a reinforcement learning policy trained with PPO and augmented with a grid memory module. 

However, we are most interested in their hybrid "switching" framework, which runs both policies in parallel and selects which action to execute at each timestep. Three switching strategies are tested: a heuristic switcher that uses EPOM when the local agent density exceeds a threshold, an assistant switcher (ASwitcher) that defaults to RePlan but falls back to EPOM when A* fails to find a path or detects a loop, and a learnable switcher that trains value estimators for each policy and greedily picks the higher-valued one.

The authors evaluated 239 maps of varying topology (game maps, mazes, city streets, random grids) with up to 500 agents. Their key finding is that ASwitcher consistently had the best performance because it leverages RePlan's efficiency in open areas while relying on EPOM's learned collision-resolution ability in congested areas.

**Relevance:** This paper directly validates the core hypothesis of my adaptive hybrid approach: that no single pathfinding strategy dominates across all environment conditions, and dynamically switching between strategies based on local conditions can outperform either strategy alone. My hybrid system switches between formation-based A* (analogous to their RePlan) and flow field pathfinding (analogous to their EPOM) based on terrain width analysis.

While they use agent density and path-planning failure as their switching triggers, I use precomputed distance transforms to classify terrain as "open" versus "constrained". It is a simpler, more interpretable mechanism that does not require reinforcement learning and takes up less resources. Their findings that the assistant switcher works best by defaulting to the search-based policy and only switching when it struggles is particularly relevant. This suggests my hybrid approach should similarly default to formation-based movement and only switch to flow field when certain conditions are met.

The paper also reinforces the importance of evaluation metrics like completion rate and scalability across varying agent counts, which align closely with my own evaluation framework.

### Flow Field Update

The flow field pathfinding approach is fully implemented! It generates a direction field toward the goal using Dijkstra's algorithm to build a cost-to-goal field (integration field), then computes approximate gradients to produce smooth flow directions.

The approach was inspired by Treuille et al. (2006), who use the eikonal equation to produce a continuous potential field. However, the eikonal equation operates on 2D continuous domains, and adapting it to a 3D voxel grid with discrete elevation changes and ramp transitions is nontrivial. There were options, and I may still explore them, but I wanted to make sure I had a working project first. 

Instead, we run Dijkstra over the walkable graph produced by the environment's `get_walkable_neighbors()` method, which naturally handles ramps and elevation. This gives a cost-to-goal value at every reachable cell. Then, for each cell, we approximate the gradient by computing a weighted average of direction vectors pointing toward each walkable neighbor, weighted by how much cheaper that neighbor is. The resulting flow direction is smoother than a naive "pick the cheapest neighbor" approach because it blends influence from multiple directions, making it less discrete and more of a gradient. It is not as exact as the eikonal equation, but the differences are rather marginal.

Agents follow the flow field independently with priority-based local collision avoidance. Agents closer to the goal move first and claim cells; if the preferred cell is occupied, the agent tries alternative neighbors that still decrease cost, or waits. Agents that have reached the goal are removed from the occupied set so they do not block others.

## Hybrid Approach Update
 
The adaptive hybrid approach is implemented as the `HybridGroup` class. It uses both formation-based movement and flow field movement, switching between them based on local terrain clearance. The terrain clearance is precomputed using a BFS-based distance transform: each walkable cell is assigned a value representing its Manhattan distance to the nearest obstacle. Cells with clearance below a (configurable) determined threshold are classified as "constrained," and the rest as "open."
 
The group defaults to formation mode, where the leader follows an A* path and followers maintain their formation offsets. At each step, the system checks the terrain clearance at the leader's current position and several cells ahead on the leader's path. If the clearance drops below the threshold, the group switches to flow field mode, where all agents (including the leader) follow the precomputed flow field independently. When the terrain opens back up and the followers have regrouped close to the leader, the system switches back to formation mode and recomputes the leader's A* path from its current position.
 
This design was directly inspired by Skrynnik et al. (2024), whose ASwitcher defaults to a planning-based policy and falls back to a learned policy when planning struggles. My approach uses terrain width as the switching trigger rather than planning failure, which is a simpler mechanism, although maybe there are a better triggers to use.
 
## Preliminary Results
 
All three approaches were tested across four environments (open terrain, chokepoint, urban grid, multi-level) with varying agent counts (6 and 15 agents). All 20 scenario/strategy combinations completed successfully with no errors. 

Some key findings from the *chokepoint scenario with 15 agents* include:
 
- **Formation (column):** 25 steps to complete, 15 collisions. 
  - Fast because agents ride the leader's path, but followers pile up at the narrow gap.
- **Formation (line):** 25 steps, 43 collisions. 
  - Line formation breaks down badly in the chokepoint because the formation is wider than the gap.
- **Flow field:** 39 steps, 0 collisions. 
  - Slower because there is no leader blazing a trail, but agents navigate the chokepoint independently without any collisions.
- **Hybrid (column):** 41 steps, 13 collisions, 13 mode switches, 37% formation / 63% flow field. 
  - Reduced collisions compared to pure formation by switching to flow field in the constrained area.
- **Hybrid (line):** 39 steps, 7 collisions, 5 mode switches, 5% formation / 95% flow field.       
  - Correctly detected the chokepoint and spent most of the time in flow field mode, reducing collisions from 43 (pure line) to 7.
 
In the urban environment, the hybrid approach identified that the entire grid of narrow streets was constrained and spent 100% of its time in flow field mode, achieving 0 collisions. This is good!
 
## Potential Tuning
 
In open terrain, the hybrid approach is switching modes more than expected (7 switches, 57% flow field time), which adds overhead without benefit. The clearance threshold (currently set to 3 for column formations) is too sensitive for wide-open environments. 

Possible improvements include raising the threshold, adding hysteresis (requiring clearance to drop further before switching and rise higher before switching back), or using the average clearance over a window of cells rather than a single-point check.
 
## Remaining Work
 
1. Tune the hybrid switching threshold
2. Add additional test scenarios and larger agent counts
3. Write the final report with full analysis
4. Update the README.md
