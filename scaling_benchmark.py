"""
scaling_benchmark.py

A proper scaling analysis for the multi-agent pathfinding project.

The original overhead_exploration.py benchmarked Python list iteration —
not pathfinding. This script benchmarks the actual system:

    1. Flow field precomputation time vs. environment size
       (Dijkstra over the walkable graph — the real bottleneck)

    2. A* follower cost vs. agent count
       (FormationGroup runs one A* per follower per step)

    3. Steps to completion vs. agent count — across three environments
       (Chokepoint, Urban, Serpentine — does throughput degrade at scale?)

    4. Collision count vs. agent count
       (Does the hybrid stay safe as groups get bigger?)

    5. prepare() precomputation time vs. agent count, all strategies
       (How much does startup cost grow with group size?)

    6. prepare() precomputation time vs. number of simultaneous groups
       (Does spinning up more groups at once compound the cost?)

    7. Formation coherence vs. environment type
       (Avg deviation from ideal positions — how well does formation hold?)

    8. Unit positioning preserved vs. environment type
       (% of time melee is closer to leader than ranged)

Each test uses real environment and agent objects from the project.
Run from the project root:

    python scaling_benchmark.py
"""

import time
import sys
import os
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np

# Make sure the project root is on the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from environment import Environment, create_open_terrain, create_chokepoint, create_urban
from environment_complex import create_serpentine, create_pillar_field, create_crossroads
from agents import (
    Agent, FormationGroup, FlowFieldGroup, HybridGroup,
    FormationType, UnitType,
)
from pathfinding.astar import astar
from pathfinding.flow_field import FlowField
from pathfinding.hybrid import compute_walkable_width


# ── Helpers ───────────────────────────────────────────────────────────────────

def make_agents(env, start_x, start_y, z, n):
    """Place n agents near (start_x, start_y, z), skipping unwalkable cells."""
    agents, placed = [], set()
    candidates = [
        (start_x + col_off + (1 if col_off > 0 else 0) * sign, start_y - row, z)
        for col_off in range(n)
        for sign in [1, -1]
        for row in range(n)
    ]
    # simpler: just a grid search
    candidates = []
    for col_off in range(n + 5):
        for row in range(n + 5):
            candidates.append((start_x + col_off, start_y - row, z))
            if col_off > 0:
                candidates.append((start_x - col_off, start_y - row, z))

    seen = set()
    for pos in candidates:
        if len(agents) >= n:
            break
        if pos in seen:
            continue
        seen.add(pos)
        if env.is_walkable(*pos):
            unit = UnitType.MELEE if len(agents) % 2 == 0 else UnitType.RANGED
            agents.append(Agent(agent_id=len(agents), position=pos, unit_type=unit))

    while len(agents) < n:
        agents.append(Agent(agent_id=len(agents), position=(start_x, start_y, z)))

    return agents


def run_sim(group, env, goal, max_steps=600):
    """Run until complete or timeout. Returns (steps, collisions, success)."""
    if not group.prepare(env, goal):
        return None, None, False

    collision_count = 0
    for step in range(max_steps):
        if group.is_complete:
            return step, collision_count, True
        group.step_all(env)
        active = [a.position for a in group.all_agents if a.position != goal]
        collision_count += len(active) - len(set(active))

    return max_steps, collision_count, False


def make_groups(agents, ftype=FormationType.COLUMN):
    """Return one of each strategy type sharing the same agent list (copied)."""
    import copy

    def fresh(agent_list):
        return [copy.deepcopy(a) for a in agent_list]

    fg = FormationGroup(ftype)
    for i, a in enumerate(fresh(agents)):
        fg.add_agent(a, as_leader=(i == 0))

    ff = FlowFieldGroup()
    for a in fresh(agents):
        ff.add_agent(a)

    hg = HybridGroup(ftype)
    for i, a in enumerate(fresh(agents)):
        hg.add_agent(a, as_leader=(i == 0))

    return fg, ff, hg


# ── Test 1: Flow Field Precomputation vs. Environment Size ────────────────────

def test_flow_field_scaling():
    """
    How does Dijkstra-based flow field build time scale with the number of
    walkable cells? Tests square open environments from 10x10 to 60x60.
    """
    print("\n[1/6] Flow field precomputation vs. environment size...")
    # Chosen so walkable cell count roughly doubles each step
    sizes = [50, 70, 100, 140, 200, 280, 350, 430, 500]
    times_ms = []
    cell_counts = []
    GOAL_OFFSET = 4

    for s in sizes:
        env = create_open_terrain(s, s, 3)  # height=3 keeps cell count cleaner
        goal = (s - GOAL_OFFSET, s - GOAL_OFFSET, 1)

        runs = 3 if s <= 200 else 1
        timings = []
        for _ in range(runs):
            t0 = time.perf_counter()
            ff = FlowField(env, goal)
            timings.append(time.perf_counter() - t0)

        avg_ms = (sum(timings) / len(timings)) * 1000
        walkable = sum(
            1 for x in range(s) for y in range(s) for z in range(3)
            if env.is_walkable(x, y, z)
        )
        times_ms.append(avg_ms)
        cell_counts.append(walkable)
        print(f"  {s:>4}x{s:<4} grid | {walkable:>7} walkable cells | {avg_ms:>9.1f} ms")

    return sizes, cell_counts, times_ms


# ── Test 2: A* Follower Cost vs. Agent Count ──────────────────────────────────

def test_astar_follower_scaling():
    """
    FormationGroup runs one A* call per follower per step.
    This measures the total per-step A* cost as agent count grows.
    Uses a medium environment so paths are non-trivial.
    """
    print("\n[2/6] A* follower cost vs. agent count...")
    # Double each time: 5, 10, 20, 40, 80, 160, 320
    agent_counts = [5, 10, 20, 40, 80, 160, 320]
    costs_ms = []

    # Large env so A* paths are long and non-trivial even at high agent counts
    ENV_SIZE = 150
    env = create_open_terrain(ENV_SIZE, ENV_SIZE, 5)
    goal = (ENV_SIZE - 5, ENV_SIZE - 5, 1)
    SAMPLES = 8

    for n in agent_counts:
        agents = make_agents(env, 5, 5, 1, n)
        group = FormationGroup(FormationType.COLUMN)
        for i, a in enumerate(agents):
            group.add_agent(a, as_leader=(i == 0))

        if not group.prepare(env, goal):
            costs_ms.append(0)
            continue

        step_times = []
        for _ in range(SAMPLES):
            if group.is_complete:
                break
            t0 = time.perf_counter()
            group.step_all(env)
            step_times.append((time.perf_counter() - t0) * 1000)

        avg = sum(step_times) / len(step_times) if step_times else 0
        costs_ms.append(avg)
        print(f"  {n:>3} agents | avg step cost: {avg:>7.3f} ms")

    return agent_counts, costs_ms


# ── Test 3: Steps to Completion vs. Agent Count ───────────────────────────────

def test_throughput_scaling():
    """
    Steps to completion vs. agent count, across three environment types.

    Chokepoint (gap=6): bottleneck forces serialized passage — all strategies
    slow down as more agents must queue through the gap.

    Urban (street grid): many narrow corridors, formation collapses early,
    hybrid switches permanently to flow field.

    Serpentine (winding canyon): most hostile to formation — 180-degree turns
    shatter coherence; flow field and hybrid should scale much better.
    """
    print("\n[3/8] Steps to completion vs. agent count (three environments)...")
    agent_counts = [5, 10, 20, 40, 80]
    MAX_STEPS = 2000
    strategies = ["Formation (Col)", "Flow Field", "Hybrid (Col)"]

    env_configs = {
        "Chokepoint": {
            "fn":    lambda: create_chokepoint(120, 120, 5, gap_width=6),
            "start": (60, 5, 1),
            "goal":  (60, 115, 1),
        },
        "Urban": {
            "fn":    lambda: create_urban(60, 60, 5),
            "start": (5, 5, 1),
            "goal":  (53, 53, 1),
        },
        "Serpentine": {
            "fn":    lambda: create_serpentine(120, 120, 5),
            "start": (110, 8, 1),
            "goal":  (8, 110, 1),
        },
    }

    # results[env_name][strategy] = [steps per agent count]
    results = {env: {s: [] for s in strategies} for env in env_configs}

    for env_name, cfg in env_configs.items():
        print(f"\n  Environment: {env_name}")
        for n in agent_counts:
            for label in strategies:
                env = cfg["fn"]()
                agents = make_agents(env, *cfg["start"], n)

                if label == "Formation (Col)":
                    g = FormationGroup(FormationType.COLUMN)
                    for i, a in enumerate(agents): g.add_agent(a, as_leader=(i == 0))
                elif label == "Flow Field":
                    g = FlowFieldGroup()
                    for a in agents: g.add_agent(a)
                else:
                    g = HybridGroup(FormationType.COLUMN)
                    for i, a in enumerate(agents): g.add_agent(a, as_leader=(i == 0))

                steps, _, ok = run_sim(g, env, cfg["goal"], max_steps=MAX_STEPS)
                result = steps if ok else MAX_STEPS
                results[env_name][label].append(result)
                print(f"    {n:>4} agents | {label:<20} | steps: {result:>5} | {'OK' if ok else 'DNF'}")

    return agent_counts, results


# ── Test 4: Collision Count vs. Agent Count ───────────────────────────────────

def test_collision_scaling():
    """
    Does the hybrid keep collisions low as group size increases?
    Uses the serpentine (two groups head-on) — worst case for formations.
    """
    print("\n[4/6] Collision count vs. agent count (serpentine, two groups)...")
    # Double each time: 5, 10, 20, 40, 80 per group (10–160 total)
    per_group_counts = [5, 10, 20, 40, 80]
    results = {
        "Formation (Col)": [],
        "Hybrid (Col)":    [],
        "Flow Field":      [],
    }

    # Large serpentine so there's room for big groups to spawn
    ENV_SIZE = 120

    for n in per_group_counts:
        for label in results:
            env = create_serpentine(ENV_SIZE, ENV_SIZE, 5)

            agents1 = make_agents(env, ENV_SIZE - 10, 8, 1, n)
            agents2 = make_agents(env, 8, ENV_SIZE - 10, 1, n)
            for a in agents2:
                a.agent_id += 500

            def make_group(agent_list, lbl):
                if lbl == "Formation (Col)":
                    g = FormationGroup(FormationType.COLUMN)
                    for i, a in enumerate(agent_list): g.add_agent(a, as_leader=(i == 0))
                elif lbl == "Flow Field":
                    g = FlowFieldGroup()
                    for a in agent_list: g.add_agent(a)
                else:
                    g = HybridGroup(FormationType.COLUMN)
                    for i, a in enumerate(agent_list): g.add_agent(a, as_leader=(i == 0))
                return g

            g1 = make_group(agents1, label)
            g2 = make_group(agents2, label)

            goal1 = (8, ENV_SIZE - 10, 1)
            goal2 = (ENV_SIZE - 10, 8, 1)

            ok1 = g1.prepare(env, goal1)
            ok2 = g2.prepare(env, goal2)
            if not ok1 or not ok2:
                results[label].append(None)
                print(f"  {n*2:>4} agents | {label:<20} | PREP FAILED")
                continue

            all_agents_list = g1.all_agents + g2.all_agents
            total_collisions = 0

            for step in range(2000):
                done1 = g1.is_complete
                done2 = g2.is_complete
                if done1 and done2:
                    break
                if not done1: g1.step_all(env)
                if not done2: g2.step_all(env)

                active = [
                    a.position for a in all_agents_list
                    if a.position not in (goal1, goal2)
                ]
                total_collisions += len(active) - len(set(active))

            results[label].append(total_collisions)
            print(f"  {n*2:>4} total agents | {label:<20} | collisions: {total_collisions}")

    group_totals = [n * 2 for n in per_group_counts]
    return group_totals, results


# ── Test 5: prepare() Time vs. Agent Count, All Strategies ───────────────────

def test_prepare_vs_agents():
    """
    How long does prepare() take for each strategy as group size grows,
    across three environment types?

    Open terrain: baseline — no obstacles, distance transform is trivial.
    Urban: dense obstacles mean the distance transform has many border cells,
           making compute_walkable_width() more expensive.
    Pillar field: highly porous obstacles, many constrained cells — worst case
                  for the hybrid's width map computation.

    Formation should stay flat across all (just one A* for the leader).
    Flow field and hybrid should also stay flat but at a higher base cost
    that varies by environment complexity, not agent count.
    """
    print("\n[5/6] prepare() time vs. agent count (three environments)...")
    agent_counts = [5, 10, 20, 40, 80, 160, 320]
    RUNS = 3
    ENV_SIZE = 60  # consistent size across all three so results are comparable

    env_configs = {
        "Open Terrain": lambda: create_open_terrain(ENV_SIZE, ENV_SIZE, 5),
        "Urban":        lambda: create_urban(ENV_SIZE, ENV_SIZE, 5),
        "Pillar Field": lambda: create_pillar_field(ENV_SIZE, ENV_SIZE, 5),
    }

    # start/goal that works in all three environments
    START = (5, 5, 1)
    GOAL  = (ENV_SIZE - 7, ENV_SIZE - 7, 1)

    strategies = ["Formation (Col)", "Flow Field", "Hybrid (Col)"]

    # results[env_name][strategy] = [time per agent count]
    results = {env: {s: [] for s in strategies} for env in env_configs}

    for env_name, env_fn in env_configs.items():
        print(f"\n  Environment: {env_name}")
        env = env_fn()

        for n in agent_counts:
            for label in strategies:
                times = []
                for _ in range(RUNS):
                    agents = make_agents(env, *START, n=n)
                    if label == "Formation (Col)":
                        g = FormationGroup(FormationType.COLUMN)
                        for i, a in enumerate(agents):
                            g.add_agent(a, as_leader=(i == 0))
                    elif label == "Flow Field":
                        g = FlowFieldGroup()
                        for a in agents:
                            g.add_agent(a)
                    else:
                        g = HybridGroup(FormationType.COLUMN)
                        for i, a in enumerate(agents):
                            g.add_agent(a, as_leader=(i == 0))

                    t0 = time.perf_counter()
                    g.prepare(env, GOAL)
                    times.append((time.perf_counter() - t0) * 1000)

                avg = sum(times) / len(times)
                results[env_name][label].append(avg)

            print(f"    {n:>4} agents | "
                  f"Formation: {results[env_name]['Formation (Col)'][-1]:>7.2f} ms | "
                  f"FlowField: {results[env_name]['Flow Field'][-1]:>7.2f} ms | "
                  f"Hybrid: {results[env_name]['Hybrid (Col)'][-1]:>7.2f} ms")

    return agent_counts, results


# ── Test 6: prepare() Time vs. Number of Simultaneous Groups ─────────────────

def test_prepare_vs_groups():
    """
    How does total precomputation cost scale when spinning up multiple groups,
    across three environment types?

    Formation scales cheap and linearly in all cases — one A* per group.
    Flow field and hybrid each build a full environment-wide field per group,
    so cost compounds linearly but at a much steeper rate.

    The environment type changes the base cost (urban and pillar field have
    more obstacle geometry to process), which shifts where hybrid becomes
    prohibitive relative to formation.
    """
    print("\n[6/6] prepare() time vs. number of simultaneous groups (10 agents each)...")
    group_counts = [1, 2, 4, 8, 16, 32]
    AGENTS_PER_GROUP = 10
    RUNS = 3
    ENV_SIZE = 60  # consistent across all three

    env_configs = {
        "Open Terrain": lambda: create_open_terrain(ENV_SIZE, ENV_SIZE, 5),
        "Urban":        lambda: create_urban(ENV_SIZE, ENV_SIZE, 5),
        "Pillar Field": lambda: create_pillar_field(ENV_SIZE, ENV_SIZE, 5),
    }

    strategies = ["Formation (Col)", "Flow Field", "Hybrid (Col)"]

    # Well-separated start/goal pairs, valid in all three environments
    # (all near edges, away from buildings/pillars)
    # Coordinates chosen to fall on street intersections in urban grid
    # (building_size=4, street_width=2 → streets at 4,5,10,11,16,17,...)
    spawn_configs = [
        {"start": (5,  5,  1), "goal": (53, 53, 1)},
        {"start": (53, 5,  1), "goal": (5,  53, 1)},
        {"start": (5,  53, 1), "goal": (53, 5,  1)},
        {"start": (53, 53, 1), "goal": (5,  5,  1)},
        {"start": (29, 5,  1), "goal": (29, 53, 1)},
        {"start": (5,  29, 1), "goal": (53, 29, 1)},
        {"start": (53, 29, 1), "goal": (5,  29, 1)},
        {"start": (29, 53, 1), "goal": (29, 5,  1)},
        {"start": (17, 5,  1), "goal": (41, 53, 1)},
        {"start": (41, 5,  1), "goal": (17, 53, 1)},
        {"start": (5,  17, 1), "goal": (53, 41, 1)},
        {"start": (5,  41, 1), "goal": (53, 17, 1)},
        {"start": (17, 17, 1), "goal": (41, 41, 1)},
        {"start": (41, 17, 1), "goal": (17, 41, 1)},
        {"start": (17, 41, 1), "goal": (41, 17, 1)},
        {"start": (41, 41, 1), "goal": (17, 17, 1)},
        {"start": (11, 5,  1), "goal": (47, 53, 1)},
        {"start": (47, 5,  1), "goal": (11, 53, 1)},
        {"start": (5,  11, 1), "goal": (53, 47, 1)},
        {"start": (5,  47, 1), "goal": (53, 11, 1)},
        {"start": (23, 5,  1), "goal": (35, 53, 1)},
        {"start": (35, 5,  1), "goal": (23, 53, 1)},
        {"start": (5,  23, 1), "goal": (53, 35, 1)},
        {"start": (5,  35, 1), "goal": (53, 23, 1)},
        {"start": (11, 11, 1), "goal": (47, 47, 1)},
        {"start": (47, 11, 1), "goal": (11, 47, 1)},
        {"start": (11, 47, 1), "goal": (47, 11, 1)},
        {"start": (47, 47, 1), "goal": (11, 11, 1)},
        {"start": (23, 11, 1), "goal": (35, 47, 1)},
        {"start": (35, 11, 1), "goal": (23, 47, 1)},
        {"start": (23, 23, 1), "goal": (35, 35, 1)},
        {"start": (35, 23, 1), "goal": (23, 35, 1)},
    ]

    # results[env_name][strategy] = [time per group count]
    results = {env: {s: [] for s in strategies} for env in env_configs}

    for env_name, env_fn in env_configs.items():
        print(f"\n  Environment: {env_name}")
        env = env_fn()

        for num_groups in group_counts:
            configs = spawn_configs[:num_groups]

            for label in strategies:
                times = []
                for _ in range(RUNS):
                    groups = []
                    for idx, cfg in enumerate(configs):
                        # find a walkable start — urban/pillar may block edge cells
                        sx, sy, sz = cfg["start"]
                        if not env.is_walkable(sx, sy, sz):
                            found = None
                            for r in range(1, 6):
                                for ddx in range(-r, r+1):
                                    for ddy in range(-r, r+1):
                                        if env.is_walkable(sx+ddx, sy+ddy, sz):
                                            found = (sx+ddx, sy+ddy, sz)
                                            break
                                    if found: break
                                if found: break
                            if found:
                                sx, sy, sz = found

                        agents = make_agents(env, sx, sy, sz, AGENTS_PER_GROUP)
                        for a in agents:
                            a.agent_id += idx * 200

                        if label == "Formation (Col)":
                            g = FormationGroup(FormationType.COLUMN)
                            for i, a in enumerate(agents):
                                g.add_agent(a, as_leader=(i == 0))
                        elif label == "Flow Field":
                            g = FlowFieldGroup()
                            for a in agents:
                                g.add_agent(a)
                        else:
                            g = HybridGroup(FormationType.COLUMN)
                            for i, a in enumerate(agents):
                                g.add_agent(a, as_leader=(i == 0))

                        groups.append((g, cfg["goal"]))

                    t0 = time.perf_counter()
                    for g, goal in groups:
                        g.prepare(env, goal)
                    times.append((time.perf_counter() - t0) * 1000)

                avg = sum(times) / len(times)
                results[env_name][label].append(avg)

            print(f"    {num_groups:>3} groups | "
                  f"Formation: {results[env_name]['Formation (Col)'][-1]:>7.2f} ms | "
                  f"FlowField: {results[env_name]['Flow Field'][-1]:>7.2f} ms | "
                  f"Hybrid: {results[env_name]['Hybrid (Col)'][-1]:>7.2f} ms")

    return group_counts, results


# ── Test 7: Formation Coherence vs. Environment Type ─────────────────────────

def test_formation_coherence():
    """
    Average deviation of followers from their ideal formation positions,
    compared across Open Terrain, Chokepoint, Urban, and Serpentine.

    Tests both FormationGroup and HybridGroup — the hybrid should show
    degraded coherence relative to formation in open terrain (time spent
    in flow field mode breaks positional offsets) but comparable or better
    coherence in constrained environments where pure formation breaks down.

    Only meaningful for formation-based movement; flow field has no
    concept of formation positions.
    """
    print("\n[7/8] Formation coherence vs. environment type...")

    N_AGENTS = 10
    MAX_STEPS = 300

    env_configs = {
        "Open Terrain": {
            "fn":    lambda: create_open_terrain(30, 30, 5),
            "start": (3, 3, 1),
            "goal":  (27, 27, 1),
        },
        "Chokepoint": {
            "fn":    lambda: create_chokepoint(30, 30, 5, gap_width=3),
            "start": (15, 2, 1),
            "goal":  (15, 27, 1),
        },
        "Urban": {
            "fn":    lambda: create_urban(30, 30, 5),
            "start": (1, 1, 1),
            "goal":  (27, 27, 1),
        },
        "Serpentine": {
            "fn":    lambda: create_serpentine(40, 40, 5),
            "start": (35, 5, 1),
            "goal":  (5, 35, 1),
        },
    }

    strategies = ["Formation (Col)", "Hybrid (Col)"]
    # results[strategy][env_name] = avg coherence
    results = {s: {} for s in strategies}

    for env_name, cfg in env_configs.items():
        print(f"\n  Environment: {env_name}")
        for label in strategies:
            env = cfg["fn"]()
            agents = make_agents(env, *cfg["start"], N_AGENTS)

            if label == "Formation (Col)":
                g = FormationGroup(FormationType.COLUMN)
                for i, a in enumerate(agents): g.add_agent(a, as_leader=(i == 0))
            else:
                g = HybridGroup(FormationType.COLUMN)
                for i, a in enumerate(agents): g.add_agent(a, as_leader=(i == 0))

            if not g.prepare(env, cfg["goal"]):
                results[label][env_name] = None
                print(f"    {label:<20} | PREP FAILED")
                continue

            coherence_samples = []
            for _ in range(MAX_STEPS):
                if g.is_complete:
                    break
                g.step_all(env)

                # Compute coherence: avg follower deviation from ideal position
                if isinstance(g, FormationGroup):
                    targets = g.compute_follower_targets(env)
                    followers = g.followers
                else:
                    targets = g._compute_follower_targets(env)
                    followers = g.followers

                deviations = []
                for f in followers:
                    ideal = targets.get(f.agent_id)
                    if ideal:
                        dx = f.position[0] - ideal[0]
                        dy = f.position[1] - ideal[1]
                        dz = f.position[2] - ideal[2]
                        deviations.append((dx**2 + dy**2 + dz**2) ** 0.5)
                if deviations:
                    coherence_samples.append(sum(deviations) / len(deviations))

            avg = sum(coherence_samples) / len(coherence_samples) if coherence_samples else 0
            results[label][env_name] = round(avg, 3)
            print(f"    {label:<20} | avg coherence deviation: {avg:.3f} cells")

    return list(env_configs.keys()), results


# ── Test 8: Unit Positioning Preserved vs. Environment Type ──────────────────

def test_unit_positioning():
    """
    Percentage of simulation steps where melee units are closer to the
    leader than ranged units — the tactical ordering constraint.

    This is only meaningful for FormationGroup and HybridGroup. In open
    terrain, formation should preserve ordering nearly perfectly. In
    constrained environments, the ordering degrades as followers scramble
    for walkable positions.

    Hybrid should preserve ordering better than pure formation in constrained
    environments, because it dissolves into flow field mode rather than
    forcing followers into impossible offset positions.
    """
    print("\n[8/8] Unit positioning preserved vs. environment type...")

    N_AGENTS = 10
    MAX_STEPS = 300

    env_configs = {
        "Open Terrain": {
            "fn":    lambda: create_open_terrain(30, 30, 5),
            "start": (3, 3, 1),
            "goal":  (27, 27, 1),
        },
        "Chokepoint": {
            "fn":    lambda: create_chokepoint(30, 30, 5, gap_width=3),
            "start": (15, 2, 1),
            "goal":  (15, 27, 1),
        },
        "Urban": {
            "fn":    lambda: create_urban(30, 30, 5),
            "start": (1, 1, 1),
            "goal":  (27, 27, 1),
        },
        "Serpentine": {
            "fn":    lambda: create_serpentine(40, 40, 5),
            "start": (35, 5, 1),
            "goal":  (5, 35, 1),
        },
    }

    strategies = ["Formation (Col)", "Hybrid (Col)"]
    results = {s: {} for s in strategies}

    for env_name, cfg in env_configs.items():
        print(f"\n  Environment: {env_name}")
        for label in strategies:
            env = cfg["fn"]()
            agents = make_agents(env, *cfg["start"], N_AGENTS)

            if label == "Formation (Col)":
                g = FormationGroup(FormationType.COLUMN)
                for i, a in enumerate(agents): g.add_agent(a, as_leader=(i == 0))
            else:
                g = HybridGroup(FormationType.COLUMN)
                for i, a in enumerate(agents): g.add_agent(a, as_leader=(i == 0))

            if not g.prepare(env, cfg["goal"]):
                results[label][env_name] = None
                print(f"    {label:<20} | PREP FAILED")
                continue

            correct = 0
            total = 0

            for _ in range(MAX_STEPS):
                if g.is_complete:
                    break
                g.step_all(env)

                leader_pos = g.leader.position
                followers = g.followers
                for i, a in enumerate(followers):
                    for b in followers[i + 1:]:
                        if a.unit_type == b.unit_type:
                            continue
                        total += 1
                        melee  = a if a.unit_type == UnitType.MELEE else b
                        ranged = b if a.unit_type == UnitType.MELEE else a
                        dist_m = abs(melee.position[0]  - leader_pos[0]) + abs(melee.position[1]  - leader_pos[1])
                        dist_r = abs(ranged.position[0] - leader_pos[0]) + abs(ranged.position[1] - leader_pos[1])
                        if dist_m <= dist_r:
                            correct += 1

            pct = round(100 * correct / total, 1) if total > 0 else 0.0
            results[label][env_name] = pct
            print(f"    {label:<20} | unit positioning preserved: {pct:.1f}%")

    return list(env_configs.keys()), results


# ── Plotting ──────────────────────────────────────────────────────────────────

def plot_all(t1, t2, t3, t4, t5, t6, t7, t8):
    sizes, cell_counts, ff_times = t1
    astar_counts, astar_costs = t2
    throughput_counts, throughput_results = t3
    collision_counts, collision_results = t4
    prepare_agent_counts, prepare_agent_results = t5
    prepare_group_counts, prepare_group_results = t6
    coherence_envs, coherence_results = t7
    positioning_envs, positioning_results = t8

    plt.style.use('ggplot')
    colors = {
        "Formation (Col)": "#3498db",
        "Flow Field":       "#e74c3c",
        "Hybrid (Col)":     "#9b59b6",
        "flow_field_build": "#2ecc71",
        "astar_follower":   "#f39c12",
    }
    env_colors = {
        "Chokepoint":   "#e74c3c",
        "Urban":        "#9b59b6",
        "Serpentine":   "#2ecc71",
    }
    env_styles = {
        "Open Terrain": "-",
        "Urban":        "--",
        "Pillar Field": ":",
    }

    fig = plt.figure(figsize=(18, 22))
    fig.suptitle("Multi-Agent Pathfinding: Scaling Analysis", fontsize=15, fontweight='bold', y=0.995)
    gs = gridspec.GridSpec(4, 2, figure=fig, hspace=0.45, wspace=0.32)

    # ── Plot 1: Flow field build time vs. cell count ──────────────────────────
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(cell_counts, ff_times, 'o-', color=colors["flow_field_build"],
             linewidth=2, markersize=6, label="Flow field build")
    coeffs = np.polyfit(cell_counts, ff_times, 1)
    trend = np.poly1d(coeffs)
    xs = np.linspace(min(cell_counts), max(cell_counts), 100)
    ax1.plot(xs, trend(xs), '--', color='gray', alpha=0.6, label="Linear fit")
    ax1.set_xlabel("Walkable cells in environment")
    ax1.set_ylabel("Build time (ms)")
    ax1.set_title("Flow Field: Build Time vs. Environment Size")
    ax1.legend(fontsize=8)

    # ── Plot 2: A* per-step cost vs. agent count ──────────────────────────────
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(astar_counts, astar_costs, 's-', color=colors["astar_follower"],
             linewidth=2, markersize=6)
    ax2.set_xlabel("Number of agents in formation")
    ax2.set_ylabel("Avg step cost (ms)")
    ax2.set_title("Formation: Per-Step A* Cost vs. Agent Count")
    ax2.annotate("One A* call\nper follower\nper step",
                 xy=(astar_counts[-1], astar_costs[-1]),
                 xytext=(astar_counts[-3], astar_costs[-1] * 0.7),
                 fontsize=8, color='gray',
                 arrowprops=dict(arrowstyle='->', color='gray'))

    # ── Plot 3: Steps to completion vs. agent count, per environment ──────────
    ax3 = fig.add_subplot(gs[1, 0])
    strat_lines = {
        "Formation (Col)": "-",
        "Flow Field":       "--",
        "Hybrid (Col)":     ":",
    }
    for env_name, strat_data in throughput_results.items():
        for label, vals in strat_data.items():
            clean_x = [throughput_counts[i] for i, v in enumerate(vals) if v is not None]
            clean_y = [v for v in vals if v is not None]
            ax3.plot(clean_x, clean_y,
                     linestyle=strat_lines[label],
                     color=env_colors.get(env_name, "#333333"),
                     linewidth=2, markersize=5, marker='o',
                     label=f"{label} / {env_name}")
    ax3.axhline(2000, color='black', linestyle=':', alpha=0.4, linewidth=1)
    ax3.text(throughput_counts[0], 1900, "timeout (DNF)", fontsize=7, color='gray')
    ax3.set_xlabel("Number of agents")
    ax3.set_ylabel("Steps to completion")
    ax3.set_title("Throughput: Steps to Complete vs. Agent Count\n(Three environments)")
    ax3.legend(fontsize=6, ncol=2)

    # ── Plot 4: Collisions vs. agent count (serpentine, two groups) ──────────
    ax4 = fig.add_subplot(gs[1, 1])
    for label, vals in collision_results.items():
        clean_x = [collision_counts[i] for i, v in enumerate(vals) if v is not None]
        clean_y = [v for v in vals if v is not None]
        ax4.plot(clean_x, clean_y, 'o-', color=colors[label],
                 linewidth=2, markersize=6, label=label)
    ax4.set_xlabel("Total agents (two groups)")
    ax4.set_ylabel("Total collisions")
    ax4.set_title("Safety at Scale: Collisions vs. Agent Count\n(Serpentine, head-on)")
    ax4.legend(fontsize=8)

    # ── Plot 5: prepare() time vs. agent count, per environment ──────────────
    ax5 = fig.add_subplot(gs[2, 0])
    for env_name, strat_results in prepare_agent_results.items():
        for label, vals in strat_results.items():
            ax5.plot(prepare_agent_counts, vals,
                     linestyle=env_styles[env_name],
                     color=colors[label],
                     linewidth=2, markersize=5, marker='o',
                     label=f"{label} / {env_name}")
    ax5.set_xlabel("Number of agents in group")
    ax5.set_ylabel("prepare() time (ms)")
    ax5.set_title("Overhead: prepare() Cost vs. Agent Count\n(Three environments)")
    ax5.legend(fontsize=6, ncol=2)

    # ── Plot 6: prepare() time vs. group count, per environment ──────────────
    ax6 = fig.add_subplot(gs[2, 1])
    for env_name, strat_results in prepare_group_results.items():
        for label, vals in strat_results.items():
            ax6.plot(prepare_group_counts, vals,
                     linestyle=env_styles[env_name],
                     color=colors[label],
                     linewidth=2, markersize=5, marker='o',
                     label=f"{label} / {env_name}")
    ax6.set_xlabel("Number of simultaneous groups (10 agents each)")
    ax6.set_ylabel("Total prepare() time (ms)")
    ax6.set_title("Overhead: prepare() Cost vs. Group Count\n(Three environments)")
    ax6.legend(fontsize=6, ncol=2)

    # ── Plot 7: Formation coherence vs. environment ───────────────────────────
    ax7 = fig.add_subplot(gs[3, 0])
    x = np.arange(len(coherence_envs))
    width = 0.35
    for i, (label, env_data) in enumerate(coherence_results.items()):
        vals = [env_data.get(e, 0) or 0 for e in coherence_envs]
        offset = (i - 0.5) * width
        bars = ax7.bar(x + offset, vals, width, label=label,
                       color=colors[label], alpha=0.85)
    ax7.set_xticks(x)
    ax7.set_xticklabels(coherence_envs, fontsize=8)
    ax7.set_ylabel("Avg deviation from ideal position (cells)")
    ax7.set_title("Formation Coherence by Environment\n(Lower is better — 10 agents)")
    ax7.legend(fontsize=8)

    # ── Plot 8: Unit positioning preserved vs. environment ────────────────────
    ax8 = fig.add_subplot(gs[3, 1])
    for i, (label, env_data) in enumerate(positioning_results.items()):
        vals = [env_data.get(e, 0) or 0 for e in positioning_envs]
        offset = (i - 0.5) * width
        ax8.bar(x + offset, vals, width, label=label,
                color=colors[label], alpha=0.85)
    ax8.set_xticks(x)
    ax8.set_xticklabels(positioning_envs, fontsize=8)
    ax8.set_ylim(0, 105)
    ax8.axhline(100, color='gray', linestyle='--', alpha=0.4, linewidth=1)
    ax8.set_ylabel("Steps with melee closer than ranged (%)")
    ax8.set_title("Unit Positioning Preserved by Environment\n(Higher is better — 10 agents)")
    ax8.legend(fontsize=8)

    os.makedirs("plots", exist_ok=True)
    out = "plots/scaling_benchmark.png"
    plt.savefig(out, dpi=150, bbox_inches='tight')
    print(f"\nSaved: {out}")
    plt.show()


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("=" * 60)
    print("Multi-Agent Pathfinding — Scaling Benchmark")
    print("=" * 60)

    t1 = test_flow_field_scaling()
    t2 = test_astar_follower_scaling()
    t3 = test_throughput_scaling()
    t4 = test_collision_scaling()
    t5 = test_prepare_vs_agents()
    t6 = test_prepare_vs_groups()
    t7 = test_formation_coherence()
    t8 = test_unit_positioning()

    plot_all(t1, t2, t3, t4, t5, t6, t7, t8)

    print("\nDone.")