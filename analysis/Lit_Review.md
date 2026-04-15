# Literature Review

**Daymian Snowden**
**CS 5100 – Foundations of AI**
**02/27/2026**

## Paper Details

**Paper Reviewed:** Multi-Agent Pathfinding with Real-Time Heuristic Search

**Authors:** Devon Sigurdson, Vadim Bulitko, William Yeoh, Carlos Hernandez, and Sven Koenig

**Peer-Reviewed**: 2018 IEEE Conference on Computational Intelligence and Games (CIG), Maastricht, Netherlands, 2018, pp. 1-8, 

**DOI:** 10.1109/CIG.2018.8490436.

## Summary

Sigurdson et al. (2018) offer Bounded Multi-Agent A* (BMAA*) as a real-time heuristic search algorithm that is able to find collision-free paths for a large numbers of video game agents in various environments. The paper addresses a core challenge found in existing multi-agent pathfinding (MAPF) techniques. Both Windowed Hierarchical Cooperative A* (WHCA*) and Flow Annotated Replanning (FAR) require complete paths before the agents start moving - as the number of agents grows, this calculation becomes increasingly expensive.

Their BMAA* approach addresses this challenge. Each agent independently runs its own bounded-depth A* search, where other agents are treated as moving obstacles and movement starts after finding a partial path (rather than a complete path). The algorithm is parameterized by a search depth (lookahead), a vision radius for detecting nearby agents, and a replanning interval. Flow annotations impose directional movement constraints, similar to one-way streets, and reduce the number of head-on collisions. BMAA* also has a push mechanic, which allows agents to temporarily displace other agents blocking goal locations.

BMAA* was tested on ten maps from three popular commercial video games: Dragon Age: Origins, Warcraft III, and Baldur's Gate II. The agent counts ranged from 25 to 2,000. Their key finding is that BMAA* achieves substantially higher completion rates than FAR when more than 200 agents are present. However, the tradeoff is that BMAA* agents travel longer distances. Sometimes their partial searches lead to suboptimal routes.

The authors argue this is an acceptable tradeoff. Players are more likely to notice agents failing to reach their destinations compared to agents taking slightly longer paths. There was a particularly noteworthy result concerning a chokepoint behaviors: FAR agents all find globally optimal paths that converge in the same narrow corridors, creating severe congestion. Meanwhile, BMAA* agents' local, real-time searches route them around areas that appear blocked by other agents.

## Relevance to My Project

This paper directly informs my project, which compares three pathfinding strategies for groups of agents in 3D environments: formation-based, flow field, and adaptive hybrid . My project is motivated by the chokepoint congestion problem, where hundreds of NPCs bottleneck themselves in narrow passages. 

The BMAA* paper provides experimental evidence that I'm not the only one who cares about chokepoint congestion. In fact, it is a central performance bottleneck in MAPF. Their comparison of real-time versus complete-path approaches is similar to my comparison of flow field pathfinding versus formation-based pathfinding. In flow field pathfinding, agents follow a shared direction field independently, similar to BMAA*'s decentralized philosophy. In formation-based pathfinding, a leader computes a single path and followers maintain positions, which is similar to FAR's centralized planning. 

Furthermore, the paper's finding that no single algorithm dominates across all scenarios supports my hypothesis that an adaptive hybrid approach can outperform any individual method.

## Answered Questions

This paper helped clarify several questions relevant to my project.

First, it confirmed that centralized path-planning methods tend to create congestion at chokepoints because many agents converge on the same optimal routes. This validates my concern that formation-based pathfinding alone will likely struggle in constrained spaces. 

Second, the paper demonstrated that decentralized, real-time approaches scale better with larger numbers of agents. This is an important consideration, as my project targets scenarios with 50–200+ agents across multiple formations.

Third, the evaluation metrics used by Sigurdson et al.—completion rate, completion time, and travel distance—overlap are extremely similar to my originally proposed metrics (computation time, time to goal, path length, and scalability). It is good to know I am on the right track!

## Remaining Questions

Despite its relevance, the BMAA* paper leaves me with several lingering questions.

The paper treats all agents as independent individuals with no group structure—there is no concept of formations, unit types, or coordinated group movement. My project specifically investigates how formations (columns, lines) behave as cohesive units, which introduces challenges around formation coherence that BMAA* does not consider.

Additionally, the paper operates primarily on 2D grid maps - technically, Dragon Age: Origins uses 3D environments and NPCs, but the camera was designed with isometric combat in mind (little to no element of verticality). My project applies MAPF to 3D voxel environments with multiple elevation levels, stairs, and ramps. It remains unclear how well BMAA*'s findings about chokepoint behavior could generalize to three-dimensional terrain and vertical movement. 

Finally, the paper does not explore the idea of dynamically switching between algorithms based on terrain analysis, which is the core of my adaptive hybrid approach. Determining when and how to trigger transitions between formation-based and flow field requires further investigation.

## Reference

Sigurdson, D., Bulitko, V., Yeoh, W., Hernandez, C., & Koenig, S. (2018). Multi-agent pathfinding with real-time heuristic search. In *Proceedings of the IEEE Conference on Computational Intelligence and Games (CIG)* (pp. 1–8). IEEE. https://doi.org/10.1109/CIG.2018.8490436