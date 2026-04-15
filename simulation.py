"""
simulation.py

Runs pathfinding strategies through test environments and compares metrics.

Strategy-agnostic: works with any AgentGroup subclass (FormationGroup,
FlowFieldGroup, HybridGroup) through the shared interface.

Metrics collected:
    Shared (all strategies):
        - success: did all agents reach the goal?
        - steps: number of simulation steps
        - num_agents: group size
        - computation_time_ms: time for precomputation (A*, flow field, etc.)
        - avg_travel_dist: average distance traveled per agent
        - total_collisions: number of cell-sharing events
    Formation-specific:
        - leader_path_length: length of leader's A* path
        - avg_formation_coherence: avg deviation from ideal positions
        - unit_positioning_preserved: % of time melee is closer than ranged
"""

import time
from environment import (
    Environment,
    create_open_terrain,
    create_chokepoint,
    create_urban,
    create_multilevel,
)
from agents import (
    Agent, AgentGroup, FormationGroup, FlowFieldGroup, HybridGroup,
    FormationType, UnitType,
)


# ---------- group creation helpers ----------

def create_agents(start_x: int, start_y: int, z: int,
                  num_agents: int = 6,
                  env: Environment = None) -> list[Agent]:
    """
    Create a list of agents at staggered positions near (start_x, start_y, z).
    Alternates melee/ranged. Agent 0 is always melee (potential leader).

    If env is provided, verifies each position is walkable and searches
    nearby if not. Agents are placed in a column behind the start point,
    wrapping to adjacent columns if they run out of room.
    """
    agents = []
    placed = set()

    # generate candidate positions: column behind start, then adjacent columns
    def candidate_positions():
        for col_offset in range(num_agents):
            for row in range(num_agents):
                cx = start_x + col_offset
                cy = start_y - row
                yield (cx, cy, z)
                if col_offset > 0:
                    yield (start_x - col_offset, cy, z)

    positions = []
    for pos in candidate_positions():
        if len(positions) >= num_agents:
            break
        if pos in placed:
            continue
        if env is not None and not env.is_walkable(*pos):
            continue
        positions.append(pos)
        placed.add(pos)

    # fallback: if we couldn't find enough walkable spots, just stack
    while len(positions) < num_agents:
        positions.append((start_x, start_y, z))

    for i in range(num_agents):
        if i == 0:
            unit = UnitType.MELEE
        else:
            unit = UnitType.RANGED if i % 2 == 0 else UnitType.MELEE

        agent = Agent(
            agent_id=i,
            position=positions[i],
            unit_type=unit,
        )
        agents.append(agent)

    return agents


def create_formation_group(agents: list[Agent],
                           formation_type: FormationType) -> FormationGroup:
    """Wrap a list of agents into a FormationGroup (first agent = leader)."""
    group = FormationGroup(formation_type)
    for i, agent in enumerate(agents):
        group.add_agent(agent, as_leader=(i == 0))
    return group


def create_flow_field_group(agents: list[Agent]) -> FlowFieldGroup:
    """Wrap a list of agents into a FlowFieldGroup."""
    group = FlowFieldGroup()
    for agent in agents:
        group.add_agent(agent)
    return group


def create_hybrid_group(agents: list[Agent],
                        formation_type: FormationType) -> HybridGroup:
    """Wrap a list of agents into a HybridGroup (first agent = leader)."""
    group = HybridGroup(formation_type)
    for i, agent in enumerate(agents):
        group.add_agent(agent, as_leader=(i == 0))
    return group


# ---------- scenario runner ----------

def run_scenario(env: Environment, group: AgentGroup, goal: tuple,
                 max_steps: int = 500) -> dict:
    """
    Run one scenario and collect metrics.

    Works with any AgentGroup subclass through the shared interface:
        group.prepare(env, goal)  -> precomputation
        group.step_all(env)       -> one simulation tick
        group.all_agents          -> list of agents
        group.is_complete         -> done check

    Returns a dict of metrics, or None if preparation failed.
    """
    # --- precomputation (timed) ---
    t0 = time.perf_counter()
    ready = group.prepare(env, goal)
    compute_time = time.perf_counter() - t0

    if not ready:
        print("  Preparation failed (no path / unreachable goal).")
        return None

    # --- tracking variables ---
    prev_positions = {a.agent_id: a.position for a in group.all_agents}
    travel_distances = {a.agent_id: 0.0 for a in group.all_agents}
    collision_count = 0

    # formation-specific tracking
    coherence_samples = []
    positioning_correct = 0
    positioning_total = 0
    is_formation = isinstance(group, FormationGroup)

    # --- simulation loop ---
    steps_taken = 0
    for step in range(max_steps):
        if group.is_complete:
            break

        group.step_all(env)
        steps_taken = step + 1

        # --- shared metrics ---

        # travel distance
        for agent in group.all_agents:
            prev = prev_positions[agent.agent_id]
            if agent.position != prev:
                dx = agent.position[0] - prev[0]
                dy = agent.position[1] - prev[1]
                dz = agent.position[2] - prev[2]
                travel_distances[agent.agent_id] += (
                    (dx**2 + dy**2 + dz**2) ** 0.5
                )
            prev_positions[agent.agent_id] = agent.position

        # collisions (agents sharing a cell, excluding arrived agents)
        active_positions = [
            a.position for a in group.all_agents
            if a.position != goal
        ]
        unique = set(active_positions)
        collision_count += len(active_positions) - len(unique)

        # --- formation-specific metrics ---
        if is_formation and group.leader is not None:
            # formation coherence
            targets = group.compute_follower_targets(env)
            deviations = []
            for follower in group.followers:
                ideal = targets.get(follower.agent_id)
                if ideal:
                    dx = follower.position[0] - ideal[0]
                    dy = follower.position[1] - ideal[1]
                    dz = follower.position[2] - ideal[2]
                    deviations.append((dx**2 + dy**2 + dz**2) ** 0.5)
            if deviations:
                coherence_samples.append(
                    sum(deviations) / len(deviations)
                )

            # unit type positioning (melee closer to leader than ranged)
            leader_pos = group.leader.position
            for i, a in enumerate(group.followers):
                for b in group.followers[i + 1:]:
                    if a.unit_type == b.unit_type:
                        continue
                    positioning_total += 1
                    melee = a if a.unit_type == UnitType.MELEE else b
                    ranged = b if a.unit_type == UnitType.MELEE else a
                    dist_m = (abs(melee.position[0] - leader_pos[0])
                              + abs(melee.position[1] - leader_pos[1]))
                    dist_r = (abs(ranged.position[0] - leader_pos[0])
                              + abs(ranged.position[1] - leader_pos[1]))
                    if dist_m <= dist_r:
                        positioning_correct += 1

    # --- compute final metrics ---
    all_dists = list(travel_distances.values())
    avg_travel = 0.0
    if all_dists:
        avg_travel = sum(all_dists) / len(all_dists)

    results = {
        "strategy": group.strategy_name,
        "success": group.is_complete,
        "steps": steps_taken,
        "num_agents": group.size,
        "computation_time_ms": round(compute_time * 1000, 3),
        "avg_travel_dist": round(avg_travel, 2),
        "total_collisions": collision_count,
    }

    # formation-specific results
    if is_formation:
        leader_path_len = 0
        if group.leader and group.leader.path:
            leader_path_len = len(group.leader.path)

        avg_coherence = 0.0
        if coherence_samples:
            avg_coherence = sum(coherence_samples) / len(coherence_samples)

        pos_pct = 100.0
        if positioning_total > 0:
            pos_pct = round(
                100 * positioning_correct / positioning_total, 1
            )

        results["leader_path_length"] = leader_path_len
        results["avg_formation_coherence"] = round(avg_coherence, 3)
        results["unit_positioning_preserved"] = f"{pos_pct}%"

    # hybrid-specific results
    if isinstance(group, HybridGroup):
        results["mode_switches"] = group.switch_count
        if group.mode_history:
            formation_pct = round(
                100 * group.mode_history.count("formation")
                / len(group.mode_history), 1
            )
            results["formation_mode_pct"] = f"{formation_pct}%"
            results["flow_field_mode_pct"] = f"{round(100 - formation_pct, 1)}%"

    return results


# ---------- output ----------

def print_results(name: str, results: dict):
    """Pretty-print a scenario's results."""
    print(f"\n--- {name} ---")
    if results is None:
        print("  Failed.\n")
        return
    for key, val in results.items():
        label = key.replace("_", " ")
        print(f"  {label}: {val}")
    print()


# ---------- main ----------

def main():
    """
    Run all scenarios across all strategies and compare.

    Each scenario is tested with:
        - Formation (column)
        - Formation (line)   [where applicable]
        - Flow Field
    """

    # shared scenario definitions
    scenarios = [
        {
            "name": "Open terrain",
            "env_fn": lambda: create_open_terrain(30, 30, 5),
            "start": (5, 2, 1),
            "goal": (25, 27, 1),
            "num_agents": 6,
        },
        {
            "name": "Chokepoint (6 agents)",
            "env_fn": lambda: create_chokepoint(30, 30, 5, gap_width=3),
            "start": (15, 2, 1),
            "goal": (15, 27, 1),
            "num_agents": 6,
        },
        {
            "name": "Chokepoint (15 agents)",
            "env_fn": lambda: create_chokepoint(30, 30, 5, gap_width=3),
            "start": (15, 2, 1),
            "goal": (15, 27, 1),
            "num_agents": 15,
        },
        {
            "name": "Urban",
            "env_fn": lambda: create_urban(30, 30, 5),
            "start": (4, 5, 1),
            "goal": (22, 23, 1),
            "num_agents": 6,
        },
    ]

    # strategies to test for each scenario
    strategy_configs = [
        {"label": "column",       "make_group": lambda agents: create_formation_group(agents, FormationType.COLUMN)},
        {"label": "line",         "make_group": lambda agents: create_formation_group(agents, FormationType.LINE)},
        {"label": "flow",         "make_group": lambda agents: create_flow_field_group(agents)},
        {"label": "hybrid_col",   "make_group": lambda agents: create_hybrid_group(agents, FormationType.COLUMN)},
        {"label": "hybrid_line",  "make_group": lambda agents: create_hybrid_group(agents, FormationType.LINE)},
    ]

    print("=" * 60)
    print("Multi-Agent Pathfinding Comparison")
    print("CS 5100 - Foundations of AI")
    print("=" * 60)

    for scenario in scenarios:
        print(f"\n{'=' * 60}")
        print(f"SCENARIO: {scenario['name']}")
        print(f"  Agents: {scenario['num_agents']}, "
              f"Goal: {scenario['goal']}")
        print("=" * 60)

        for strat in strategy_configs:
            # fresh environment and agents for each strategy
            env = scenario["env_fn"]()
            agents = create_agents(
                *scenario["start"],
                num_agents=scenario["num_agents"],
                env=env,
            )
            group = strat["make_group"](agents)

            display_name = f"{scenario['name']} [{group.strategy_name}]"
            results = run_scenario(env, group, scenario["goal"])
            print_results(display_name, results)


if __name__ == "__main__":
    main()
