"""
generate_plots.py

Generates publication-quality plots from the simulation results.
Data sourced from the final simulation_large.py run.
"""

import matplotlib.pyplot as plt
import numpy as np
import os

if not os.path.exists('plots'):
    os.makedirs('plots')

plt.style.use('ggplot')
colors = ['#3498db', '#2ecc71', '#e74c3c', '#9b59b6', '#f1c40f']

# ---------------------------------------------------------
# DATA (from final simulation_large.py run)
# ---------------------------------------------------------
scenarios = ["Crossroads", "Serpentine", "Pillar Field"]
strategies = ["Formation (Col)", "Formation (Line)", "Flow Field", "Hybrid (Col)", "Hybrid (Line)"]

# Global Collisions
collision_data = {
    "Crossroads":   [260, 177,  13, 206, 104],
    "Serpentine":    [231, 343,   0,  45,  98],
    "Pillar Field":  [105,  49,  23,  38,  22],
}

# Computation Time (ms)
compute_data = {
    "Crossroads":   [  1.5,   1.8,  48.3,  89.1,  77.2],
    "Serpentine":    [ 16.7,  23.0, 110.7, 217.1, 223.1],
    "Pillar Field":  [ 17.4,  21.6, 120.7, 240.2, 225.7],
}

# ---------------------------------------------------------
# PLOT 1: Global Collisions
# ---------------------------------------------------------
def plot_collisions():
    x = np.arange(len(scenarios))
    width = 0.15
    fig, ax = plt.subplots(figsize=(10, 6))

    for i, strat in enumerate(strategies):
        counts = [collision_data[s][i] for s in scenarios]
        ax.bar(x + i * width, counts, width, label=strat, color=colors[i])

    ax.set_ylabel('Total Global Collisions')
    ax.set_title('Safety Comparison: Collisions by Strategy')
    ax.set_xticks(x + width * 2)
    ax.set_xticklabels(scenarios)
    ax.legend()

    plt.tight_layout()
    plt.savefig('plots/collision_comparison.png', dpi=300)
    print("Saved: plots/collision_comparison.png")


# ---------------------------------------------------------
# PLOT 2: Computation Time
# ---------------------------------------------------------
def plot_compute():
    x = np.arange(len(scenarios))
    width = 0.15
    fig, ax = plt.subplots(figsize=(10, 6))

    for i, strat in enumerate(strategies):
        times = [compute_data[s][i] for s in scenarios]
        ax.bar(x + i * width, times, width, label=strat, color=colors[i])

    ax.set_ylabel('Precomputation Time (ms)')
    ax.set_title('Performance Overhead: A* vs. Flow Fields')
    ax.set_xticks(x + width * 2)
    ax.set_xticklabels(scenarios)
    ax.set_yscale('log')
    ax.legend()

    plt.tight_layout()
    plt.savefig('plots/compute_overhead.png', dpi=300)
    print("Saved: plots/compute_overhead.png")


# ---------------------------------------------------------
# PLOT 3: Hybrid Behavior (Mode Distribution)
# ---------------------------------------------------------
def plot_hybrid_mode():
    labels = ['Open Terrain', 'Chokepoint (15)', 'Urban', 'Serpentine']
    formation_pct = [100.0, 72.0, 50.0, 0.0]
    flow_pct      = [  0.0, 28.0, 50.0, 100.0]

    fig, ax = plt.subplots(figsize=(8, 6))
    ax.bar(labels, formation_pct, label='Formation Mode', color='#3498db')
    ax.bar(labels, flow_pct, bottom=formation_pct, label='Flow Field Mode', color='#e74c3c')

    ax.set_ylabel('Percentage of Simulation Steps')
    ax.set_title('Adaptive Hybrid Decision Making')
    ax.legend(loc='lower right')

    plt.tight_layout()
    plt.savefig('plots/hybrid_behavior.png', dpi=300)
    print("Saved: plots/hybrid_behavior.png")


# ---------------------------------------------------------
# PLOT 4: Single-Group Baseline Summary
# ---------------------------------------------------------
def plot_single_group_summary():
    """Bar chart of collisions across single-group scenarios."""
    single_scenarios = ['Open\n(6 agents)', 'Chokepoint\n(6 agents)',
                        'Chokepoint\n(15 agents)', 'Urban\n(6 agents)']
    strats = ['Formation (Col)', 'Formation (Line)', 'Flow Field',
              'Hybrid (Col)', 'Hybrid (Line)']

    collision_vals = {
        'Formation (Col)':  [2,  2, 15,  0],
        'Formation (Line)': [2,  2, 43, 17],
        'Flow Field':       [0,  0,  0,  0],
        'Hybrid (Col)':     [2,  2, 15,  0],
        'Hybrid (Line)':    [2,  2, 22, 14],
    }

    x = np.arange(len(single_scenarios))
    width = 0.15
    fig, ax = plt.subplots(figsize=(10, 6))

    for i, s in enumerate(strats):
        ax.bar(x + i * width, collision_vals[s], width, label=s, color=colors[i])

    ax.set_ylabel('Total Collisions')
    ax.set_title('Single-Group Baseline: Collisions by Scenario')
    ax.set_xticks(x + width * 2)
    ax.set_xticklabels(single_scenarios)
    ax.legend(fontsize=8)

    plt.tight_layout()
    plt.savefig('plots/single_group_collisions.png', dpi=300)
    print("Saved: plots/single_group_collisions.png")


# ---------------------------------------------------------
# RUN ALL
# ---------------------------------------------------------
if __name__ == "__main__":
    plot_collisions()
    plot_compute()
    plot_hybrid_mode()
    plot_single_group_summary()
    print("\nAll visuals generated in the /plots folder!")
