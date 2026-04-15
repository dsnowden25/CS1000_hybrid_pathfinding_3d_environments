"""
sim_large.py (Master Simulation Suite)

Runs the complete suite of tests for the CS 5100 Final Project:
Part 1: Single-Group Baseline Tests (Open Terrain, Chokepoints, Urban)
Part 2: Multi-Group Extreme Stress Tests (Crossroads, Serpentine, Pillar Field)
"""

import time
from environment import create_open_terrain, create_chokepoint, create_urban
from environment_complex import create_crossroads, create_serpentine, create_pillar_field
from simulation import (
    create_agents, create_formation_group, create_flow_field_group, 
    create_hybrid_group, run_scenario, print_results
)
from agents import FormationType

def get_groups_for_strategy(strat_type, agents_1, agents_2=None):
    """Helper to instantiate the right group types."""
    if strat_type == "formation_col":
        g1 = create_formation_group(agents_1, FormationType.COLUMN)
        g2 = create_formation_group(agents_2, FormationType.COLUMN) if agents_2 else None
    elif strat_type == "formation_line":
        g1 = create_formation_group(agents_1, FormationType.LINE)
        g2 = create_formation_group(agents_2, FormationType.LINE) if agents_2 else None
    elif strat_type == "flow":
        g1 = create_flow_field_group(agents_1)
        g2 = create_flow_field_group(agents_2) if agents_2 else None
    elif strat_type == "hybrid_col":
        g1 = create_hybrid_group(agents_1, FormationType.COLUMN)
        g2 = create_hybrid_group(agents_2, FormationType.COLUMN) if agents_2 else None
    elif strat_type == "hybrid_line":
        g1 = create_hybrid_group(agents_1, FormationType.LINE)
        g2 = create_hybrid_group(agents_2, FormationType.LINE) if agents_2 else None
    
    return (g1, g2) if agents_2 else g1

def run_multi_scenario(env, group_goal_pairs, max_steps=800) -> dict:
    """Runs a scenario with multiple distinct groups, tracking global collisions."""
    t0 = time.perf_counter()
    
    for group, goal in group_goal_pairs:
        if not group.prepare(env, goal):
            return None
            
    compute_time = time.perf_counter() - t0

    all_agents = []
    for group, _ in group_goal_pairs:
        all_agents.extend(group.all_agents)
        
    prev_positions = {a.agent_id: a.position for a in all_agents}
    travel_distances = {a.agent_id: 0.0 for a in all_agents}
    global_collision_count = 0
    
    steps_taken = 0
    for step in range(max_steps):
        if all(g.is_complete for g, _ in group_goal_pairs):
            break

        for group, _ in group_goal_pairs:
            if not group.is_complete:
                group.step_all(env)
                
        steps_taken = step + 1

        active_positions = [
            a.position for a, (g, goal) in zip(all_agents, [(g, goal) for g, goal in group_goal_pairs for _ in g.all_agents])
            if a.position != goal
        ]
        unique = set(active_positions)
        global_collision_count += len(active_positions) - len(unique)

        for agent in all_agents:
            prev = prev_positions[agent.agent_id]
            if agent.position != prev:
                dx, dy, dz = agent.position[0] - prev[0], agent.position[1] - prev[1], agent.position[2] - prev[2]
                travel_distances[agent.agent_id] += ((dx**2 + dy**2 + dz**2) ** 0.5)
            prev_positions[agent.agent_id] = agent.position

    all_dists = list(travel_distances.values())
    avg_travel = sum(all_dists) / len(all_dists) if all_dists else 0.0

    return {
        "strategy": group_goal_pairs[0][0].strategy_name + " (Multi-Group)",
        "success": all(g.is_complete for g, _ in group_goal_pairs),
        "steps": steps_taken,
        "total_agents": len(all_agents),
        "computation_time_ms": round(compute_time * 1000, 3),
        "avg_travel_dist": round(avg_travel, 2),
        "global_collisions": global_collision_count,
        "combined_mode_switches": sum(g.switch_count for g, _ in group_goal_pairs if hasattr(g, 'switch_count'))
    }

def main():
    strategies = [
        {"label": "Formation (Column)", "type": "formation_col"},
        {"label": "Formation (Line)", "type": "formation_line"},
        {"label": "Flow Field", "type": "flow"},
        {"label": "Hybrid (Column)", "type": "hybrid_col"},
        {"label": "Hybrid (Line)", "type": "hybrid_line"}
    ]

    print("============================================================")
    print("PART 1: SINGLE-GROUP BASELINE TESTS")
    print("============================================================")

    single_scenarios = [
        {"name": "Open Terrain", "env_fn": lambda: create_open_terrain(30, 30, 5), "start": (2, 2, 1), "goal": (25, 27, 1), "num_agents": 6},
        {"name": "Chokepoint (6 agents)", "env_fn": lambda: create_chokepoint(30, 30, 5, gap_width=3), "start": (15, 2, 1), "goal": (15, 27, 1), "num_agents": 6},
        {"name": "Chokepoint (15 agents)", "env_fn": lambda: create_chokepoint(30, 30, 5, gap_width=3), "start": (15, 2, 1), "goal": (15, 27, 1), "num_agents": 15},
        {"name": "Urban", "env_fn": lambda: create_urban(30, 30, 5), "start": (4, 5, 1), "goal": (22, 23, 1), "num_agents": 6}
    ]

    for scenario in single_scenarios:
        print(f"\n\n{'=' * 60}\nSCENARIO: {scenario['name']}\n{'=' * 60}")
        for strat in strategies:
            env = scenario["env_fn"]()
            agents = create_agents(*scenario["start"], num_agents=scenario["num_agents"], env=env)
            group = get_groups_for_strategy(strat["type"], agents)
            
            results = run_scenario(env, group, scenario["goal"])
            print_results(f"{scenario['name']} [{strat['label']}]", results)


    print("\n\n============================================================")
    print("PART 2: MULTI-GROUP EXTREME STRESS TESTS")
    print("============================================================")

    # 1. Crossroads
    print(f"\n\n{'=' * 60}\nSCENARIO: Crossroads (30 Agents, Perpendicular)\n{'=' * 60}")
    for strat in strategies:
        env = create_crossroads(width=40, depth=40, height=5, corridor_width=6)
        agents_1 = create_agents(start_x=2, start_y=20, z=1, num_agents=15, env=env)
        agents_2 = create_agents(start_x=20, start_y=38, z=1, num_agents=15, env=env)
        for a in agents_2: a.agent_id += 100 
        
        g1, g2 = get_groups_for_strategy(strat["type"], agents_1, agents_2)
        results = run_multi_scenario(env, [(g1, (38, 20, 1)), (g2, (20, 2, 1))])
        print_results(f"Crossroads - {strat['label']}", results)

    # 2. Serpentine
    print(f"\n\n{'=' * 60}\nSCENARIO: Serpentine Canyon (30 Agents, Head-On)\n{'=' * 60}")
    for strat in strategies:
        env = create_serpentine(width=40, depth=40, height=5)
        agents_1 = create_agents(start_x=35, start_y=5, z=1, num_agents=15, env=env)
        agents_2 = create_agents(start_x=5, start_y=35, z=1, num_agents=15, env=env)
        for a in agents_2: a.agent_id += 100 

        g1, g2 = get_groups_for_strategy(strat["type"], agents_1, agents_2)
        results = run_multi_scenario(env, [(g1, (5, 35, 1)), (g2, (35, 5, 1))])
        print_results(f"Serpentine - {strat['label']}", results)

    # 3. Pillar Field
    print(f"\n\n{'=' * 60}\nSCENARIO: Pillar Field (30 Agents, Diagonal X)\n{'=' * 60}")
    for strat in strategies:
        env = create_pillar_field(width=40, depth=40, height=5)
        agents_1 = create_agents(start_x=5, start_y=5, z=1, num_agents=15, env=env)
        agents_2 = create_agents(start_x=35, start_y=5, z=1, num_agents=15, env=env)
        for a in agents_2: a.agent_id += 100 

        g1, g2 = get_groups_for_strategy(strat["type"], agents_1, agents_2)
        results = run_multi_scenario(env, [(g1, (35, 35, 1)), (g2, (5, 35, 1))])
        print_results(f"Pillar Field - {strat['label']}", results)

if __name__ == "__main__":
    main()