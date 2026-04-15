"""
agents.py

Agent classes and group strategies for multi-agent pathfinding.

Defines a base AgentGroup class with a common interface, and three
strategy-specific subclasses:
    - FormationGroup: Leader-follower with A* (like Mount & Blade: Bannerlord)
    - FlowFieldGroup: Shared direction field, agents follow independently
    - HybridGroup: Switches between formation and flow field based on
                   collision detection and terrain clearance

Each group implements prepare() to do any precomputation and step_all()
to advance all agents by one simulation tick.
"""
import math
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

from environment import Environment
from pathfinding.astar import astar
from pathfinding.flow_field import FlowField
from pathfinding.hybrid import compute_walkable_width, classify_terrain


# ── Enums ─────────────────────────────────────────────────────────────────────

class UnitType(Enum):
    MELEE = "melee"
    RANGED = "ranged"


class FormationType(Enum):
    LINE = "line"       # side by side, perpendicular to direction of travel
    COLUMN = "column"   # single file, along direction of travel


# ── Agent ─────────────────────────────────────────────────────────────────────

@dataclass
class Agent:
    """A single unit in the simulation."""
    agent_id: int
    position: tuple
    unit_type: UnitType = UnitType.MELEE
    collision_radius: float = 0.4
    path: list = field(default_factory=list)
    path_index: int = 0

    @property
    def has_path(self) -> bool:
        return len(self.path) > 0 and self.path_index < len(self.path)

    @property
    def at_destination(self) -> bool:
        if not self.path:
            return True
        return self.path_index >= len(self.path) - 1

    def step(self):
        """Advance one step along the precomputed path."""
        if self.has_path and not self.at_destination:
            self.path_index += 1
            self.position = self.path[self.path_index]

    def set_path(self, path: list):
        self.path = path
        self.path_index = 0
        if path:
            self.position = path[0]


# ── Base Group ────────────────────────────────────────────────────────────────

class AgentGroup(ABC):
    """
    Base class for a group of agents sharing a movement strategy.

    Subclasses implement:
        prepare(env, goal) — precomputation (A* for leader, flow field, etc.)
        step_all(env)      — advance every agent by one simulation step
    """

    def __init__(self):
        self._agents: list[Agent] = []
        self.goal: Optional[tuple] = None

    @property
    def all_agents(self) -> list[Agent]:
        return list(self._agents)

    @property
    def size(self) -> int:
        return len(self._agents)

    @property
    def is_complete(self) -> bool:
        """True when all agents have reached the goal (or close enough)."""
        return all(a.at_destination for a in self._agents)

    @property
    @abstractmethod
    def strategy_name(self) -> str:
        """Human-readable name for this strategy."""
        ...

    def add_agent(self, agent: Agent):
        """Add an agent to the group."""
        self._agents.append(agent)

    @abstractmethod
    def prepare(self, env: Environment, goal: tuple) -> bool:
        """
        Do any precomputation needed before the simulation loop.
        Returns True if the group is ready to move, False if setup failed.
        """
        ...

    @abstractmethod
    def step_all(self, env: Environment):
        """Advance all agents by one simulation step."""

# ── Formation Group ───────────────────────────────────────────────────────────

class FormationGroup(AgentGroup):
    """
    Leader-follower formation pathfinding.

    One agent (the leader) runs A* to get a full path to the goal.
    All other agents (followers) maintain their relative positions
    within a formation template (column or line), recomputing
    short-range A* paths toward their ideal positions each step.

    Completion: leader reaches the goal. Followers are not required
    to arrive — the leader reaching the goal represents mission success.
    """

    def __init__(self, formation_type: FormationType = FormationType.COLUMN):
        super().__init__()
        self.formation_type = formation_type
        self.leader: Optional[Agent] = None
        self.followers: list[Agent] = []

    @property
    def strategy_name(self) -> str:
        return f"Formation ({self.formation_type.value})"

    @property
    def all_agents(self) -> list[Agent]:
        if self.leader is None:
            return list(self.followers)
        return [self.leader] + self.followers

    @property
    def size(self) -> int:
        return len(self.all_agents)

    @property
    def is_complete(self) -> bool:
        """Done when the leader reaches the goal."""
        if self.leader is None:
            return True
        return self.leader.at_destination

    def add_agent(self, agent: Agent, as_leader: bool = False):
        if as_leader:
            if self.leader is not None:
                self.followers.insert(0, self.leader)
            self.leader = agent
        else:
            self.followers.append(agent)
        self._agents = self.all_agents

    def prepare(self, env: Environment, goal: tuple) -> bool:
        self.goal = goal
        if self.leader is None:
            return False
        path = astar(env, self.leader.position, goal)
        if path is None:
            return False
        self.leader.set_path(path)
        return True

    def step_all(self, env: Environment):
        if self.leader is None:
            return
        self.leader.step()
        targets = self.compute_follower_targets(env)
        for follower in self.followers:
            target = targets.get(follower.agent_id)
            if target is None or follower.position == target:
                continue
            path = astar(env, follower.position, target)
            if path and len(path) > 1:
                follower.set_path(path)
                follower.step()

    def get_formation_offsets(self) -> list[tuple]:
        offsets = []
        n_followers = len(self.followers)
        if self.formation_type == FormationType.COLUMN:
            for i in range(n_followers):
                offsets.append((0, -(i + 1)))
        elif self.formation_type == FormationType.LINE:
            for i in range(n_followers):
                side = -1 if i % 2 == 0 else 1
                dist = (i // 2) + 1
                offsets.append((side * dist, 0))
        return offsets

    def get_facing_direction(self) -> tuple:
        if self.leader is None or not self.leader.has_path:
            return (0, 1)
        idx = self.leader.path_index
        path = self.leader.path
        if idx + 1 < len(path):
            curr, nxt = path[idx], path[idx + 1]
        elif idx > 0:
            curr, nxt = path[idx - 1], path[idx]
        else:
            return (0, 1)
        dx = nxt[0] - curr[0]
        dy = nxt[1] - curr[1]
        if abs(dx) >= abs(dy):
            return (1 if dx > 0 else -1, 0)
        else:
            return (0, 1 if dy > 0 else -1)

    def _rotate_offset(self, offset: tuple, facing: tuple) -> tuple:
        ox, oy = offset
        fx, fy = facing
        if fy == 1:   return (ox, oy)
        if fy == -1:  return (-ox, -oy)
        if fx == 1:   return (-oy, ox)
        if fx == -1:  return (oy, -ox)
        return (ox, oy)

    def compute_follower_targets(self, env: Environment) -> dict:
        if self.leader is None:
            return {}
        facing = self.get_facing_direction()
        offsets = self.get_formation_offsets()
        leader_pos = self.leader.position
        targets = {}
        for i, follower in enumerate(self.followers):
            if i >= len(offsets):
                break
            rotated = self._rotate_offset(offsets[i], facing)
            tx = leader_pos[0] + rotated[0]
            ty = leader_pos[1] + rotated[1]
            tz = leader_pos[2]
            if not env.is_walkable(tx, ty, tz):
                found = self._find_nearest_walkable(env, tx, ty, tz)
                tx, ty, tz = found if found else leader_pos
            targets[follower.agent_id] = (tx, ty, tz)
        return targets

    def _find_nearest_walkable(self, env, x, y, z, max_radius=3):
        for r in range(1, max_radius + 1):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    if abs(dx) == r or abs(dy) == r:
                        if env.is_walkable(x + dx, y + dy, z):
                            return (x + dx, y + dy, z)
        return None


# ── Flow Field Group ──────────────────────────────────────────────────────────

class FlowFieldGroup(AgentGroup):
    """
    Flow field pathfinding for a group of agents.

    A single flow field is precomputed from the goal using Dijkstra
    with approximate gradients. All agents independently follow the
    field with priority-based local collision avoidance (agents closer
    to the goal move first and claim cells).

    Completion: all agents reach the goal cell. Arrived agents vacate
    the occupied set each step, so subsequent agents can move in.
    """

    def __init__(self):
        super().__init__()
        self.flow_field: Optional[FlowField] = None

    @property
    def strategy_name(self) -> str:
        return "Flow Field"

    def prepare(self, env: Environment, goal: tuple) -> bool:
        self.goal = goal
        if not env.is_walkable(*goal):
            return False
        self.flow_field = FlowField(env, goal)
        for agent in self._agents:
            agent.path = [agent.position, goal]
            agent.path_index = 0
        for agent in self._agents:
            if self.flow_field.is_reachable(agent.position):
                return True
        return False

    def step_all(self, env: Environment):
        if self.flow_field is None:
            return

        arrived = set()
        active = []
        for agent in self._agents:
            if agent.position == self.goal:
                agent.path = [self.goal]
                agent.path_index = 0
                arrived.add(agent.agent_id)
            else:
                active.append(agent)

        # Agents closest to goal move first — prevents priority inversion
        active.sort(key=lambda a: self.flow_field.get_cost(a.position))
        occupied = {a.position for a in self._agents if a.agent_id not in arrived}

        for agent in active:
            pos = agent.position
            target = self.flow_field.get_next_cell(pos)
            if target is not None and target not in occupied:
                occupied.discard(pos)
                occupied.add(target)
                agent.position = target
            else:
                my_cost = self.flow_field.get_cost(pos)
                alternatives = env.get_walkable_neighbors(*pos)
                alternatives.sort(key=lambda n: self.flow_field.get_cost(n))
                for alt in alternatives:
                    if alt not in occupied and self.flow_field.get_cost(alt) < my_cost:
                        occupied.discard(pos)
                        occupied.add(alt)
                        agent.position = alt
                        break

    @property
    def is_complete(self) -> bool:
        if self.goal is None:
            return True
        return all(a.position == self.goal for a in self._agents)


# ── Hybrid Group ──────────────────────────────────────────────────────────────

class HybridGroup(AgentGroup):
    """
    Adaptive hybrid pathfinding using collision-based mode switching.

    Defaults to formation mode (A* leader + follower offsets). Switches
    to flow field mode when a collision is detected OR the leader's
    immediate terrain clearance drops below a threshold — whichever
    comes first. Switches back to formation only after a sustained
    collision-free streak AND the leader has enough open space ahead.

    This means:
        - In open terrain: formation mode, tactical cohesion preserved
        - Approaching a bottleneck: proactive switch on clearance drop
        - Inside a bottleneck: flow field, agents navigate independently
        - Transition back: a few collision steps are possible and expected
          while agents regroup after exiting the constraint

    Completion: leader reaches the goal (same as FormationGroup).
    Followers are not required to arrive simultaneously.
    """
    GOAL_SWITCH_DISTANCE = 4      # Manhattan distance in grid cells


    # Fixed cooldown between mode switches regardless of group size.
    # Prevents rapid oscillation without scaling into a deadlock.
    MODE_COOLDOWN = 15

    # How many consecutive collision-free steps are required before
    # switching back from flow field to formation.
    COLLISION_FREE_REQUIRED = 10

    def __init__(self, formation_type: FormationType = FormationType.COLUMN,
                 flow_threshold: int = None, form_threshold: int = None):
        super().__init__()
        self.formation_type = formation_type
        self.leader: Optional[Agent] = None
        self.followers: list[Agent] = []

        self.flow_threshold = flow_threshold  # clearance below which we switch to flow
        self.form_threshold = form_threshold  # clearance above which we can switch back

        self.flow_field: Optional[FlowField] = None
        self.width_map: dict = {}

        self.mode: str = "formation"
        self.mode_history: list[str] = []
        self.switch_count: int = 0

        self._mode_steps: int = 0           # steps since last switch
        self._collision_free_steps: int = 0  # consecutive steps with 0 collisions
        self._last_step_collisions: int = 0  # collisions detected on the last step
           # sustained collision tracking
        self._collision_steps = 0

        # size-scaled thresholds (initialized in prepare)
        self.collision_to_flow_steps = 0
        self.collision_free_required = 0
        self.flow_threshold = 0
        self.form_threshold = 0

    @property
    def strategy_name(self) -> str:
        return f"Hybrid ({self.formation_type.value})"

    @property
    def all_agents(self) -> list[Agent]:
        if self.leader is None:
            return list(self.followers)
        return [self.leader] + self.followers

    @property
    def size(self) -> int:
        return len(self.all_agents)

    @property
    def is_complete(self) -> bool:
        """Done when the leader reaches the goal, matching FormationGroup."""
        if self.leader is None:
            return True
        return self.leader.at_destination

    def add_agent(self, agent: Agent, as_leader: bool = False):
        if as_leader:
            if self.leader is not None:
                self.followers.insert(0, self.leader)
            self.leader = agent
        else:
            self.followers.append(agent)
        self._agents = self.all_agents

    def prepare(self, env: Environment, goal: tuple) -> bool:
        self.goal = goal
        if self.leader is None:
            return False

        # A* for leader path
        path = astar(env, self.leader.position, goal)
        if path is None:
            return False
        self.leader.set_path(path)

        # Flow field covers the whole environment — one Dijkstra pass
        self.flow_field = FlowField(env, goal)

        # Distance transform for clearance values
        self.width_map = compute_walkable_width(env)

        # Reset all tracking state
        self.mode_history = []
        self.switch_count = 0
        self._mode_steps = 0

        N = len(self.all_agents)

        # sustained congestion required before breaking formation
        self.collision_to_flow_steps = max(2, int(math.sqrt(N)))

        # stable time required before reforming (scales with group size)
        self.collision_free_required = max(3, int(2 * math.sqrt(N)))

        # spatial requirements scale with group size
        self.flow_threshold = max(2, int(math.sqrt(N)))
        self.form_threshold = self.flow_threshold + 2

        # formation-first behavior
        self.mode = "formation"

        for agent in self.followers:
            agent.path = [agent.position, goal]
            agent.path_index = 0

        return True

    # ── Clearance helpers ──────────────────────────────────────────────────────

    def _get_clearance(self, position: tuple) -> int:
        return self.width_map.get(position, 0)

    # ── Switching conditions ───────────────────────────────────────────────────

    def _should_switch_to_flow(self) -> bool:
        """
        Switch to flow field if:
          - A collision was detected on the last step, OR
          - The leader's current cell is already in constrained terrain.

        No lookahead window — keep it simple and reactive.
        """
        # require sustained congestion, not single bumps
        if self._collision_steps >= self.collision_to_flow_steps:
            return True

        if self.leader is not None:
            return self._get_clearance(self.leader.position) <= self.flow_threshold

        return False

    def _should_switch_to_formation(self) -> bool:
        """
        Switch back to formation if:
          - No collisions for COLLISION_FREE_REQUIRED consecutive steps, AND
          - The leader currently has enough open space to reform.

        The collision-free streak is the primary gate. If agents are still
        bumping into each other after the bottleneck, it's too early to reform.
        """
        if self._collision_free_steps < self.collision_free_required:
            return False

        if self.leader is None:
            return False

        return self._get_clearance(self.leader.position) >= self.form_threshold

    def _switch_to_formation(self, env: Environment):
        """
        Immediately switch to formation mode and replan the leader's path.
        Used for goal convergence and post-bottleneck regrouping.
        """
        if self.leader is None or self.goal is None:
            return
        new_path = astar(env, self.leader.position, self.goal)
        if new_path is None:
            return

        self.leader.set_path(new_path)
        self.mode = "formation"
        self.switch_count += 1
        self._mode_steps = 0

    # ── Step ──────────────────────────────────────────────────────────────────
    def step_all(self, env: Environment):
        self._mode_steps += 1

        # --- Goal proximity switching ---
        if self.leader and self.goal:
            lx, ly, lz = self.leader.position
            gx, gy, gz = self.goal

            dist_to_goal = abs(lx - gx) + abs(ly - gy) + abs(lz - gz)

            if dist_to_goal <= self.GOAL_SWITCH_DISTANCE:
                if self.mode != "formation":
                    self._switch_to_formation(env)

        # Evaluate switching only after cooldown
        if self._mode_steps >= self.MODE_COOLDOWN:
            if self.mode == "formation" and self._should_switch_to_flow():
                self.mode = "flow_field"
                self.switch_count += 1
                self._mode_steps = 0

            elif self.mode == "flow_field" and self._should_switch_to_formation():
                new_path = astar(env, self.leader.position, self.goal)
                if new_path:
                    self.leader.set_path(new_path)
                    self.mode = "formation"
                    self.switch_count += 1
                    self._mode_steps = 0

        self.mode_history.append(self.mode)

        # Execute mode
        if self.mode == "formation":
            self._step_formation(env)
        else:
            self._step_flow_field(env)

        # Collision accounting
        active_positions = [
            a.position for a in self.all_agents
            if a.position != self.goal
        ]
        unique_positions = set(active_positions)
        self._last_step_collisions = len(active_positions) - len(unique_positions)

        if self._last_step_collisions > 0:
            self._collision_steps += 1
            self._collision_free_steps = 0
        else:
            self._collision_steps = 0
            self._collision_free_steps += 1
    # ── Formation step ────────────────────────────────────────────────────────

    def _step_formation(self, env: Environment):
        self.leader.step()
        targets = self._compute_follower_targets(env)
        for follower in self.followers:
            target = targets.get(follower.agent_id)
            if target and follower.position != target:
                path = astar(env, follower.position, target)
                if path and len(path) > 1:
                    follower.set_path(path)
                    follower.step()

    # ── Flow field step ───────────────────────────────────────────────────────

    def _step_flow_field(self, env: Environment):
        if self.flow_field is None:
            return

        all_units = self.all_agents
        arrived = {a.agent_id for a in all_units if a.position == self.goal}
        active = [a for a in all_units if a.agent_id not in arrived]

        # Agents closest to goal get priority — prevents deadlock near goal
        active.sort(key=lambda a: self.flow_field.get_cost(a.position))
        occupied = {a.position for a in all_units if a.agent_id not in arrived}

        for agent in active:
            pos = agent.position
            target = self.flow_field.get_next_cell(pos)
            if target and target not in occupied:
                occupied.discard(pos)
                occupied.add(target)
                agent.position = target
            else:
                my_cost = self.flow_field.get_cost(pos)
                alternatives = sorted(
                    env.get_walkable_neighbors(*pos),
                    key=lambda n: self.flow_field.get_cost(n)
                )
                for alt in alternatives:
                    if alt not in occupied and self.flow_field.get_cost(alt) < my_cost:
                        occupied.discard(pos)
                        occupied.add(alt)
                        agent.position = alt
                        break

    # ── Formation geometry ────────────────────────────────────────────────────

    def _get_facing_direction(self) -> tuple:
        if self.leader is None or not self.leader.has_path:
            return (0, 1)
        idx, path = self.leader.path_index, self.leader.path
        if idx + 1 < len(path):
            curr, nxt = path[idx], path[idx + 1]
        elif idx > 0:
            curr, nxt = path[idx - 1], path[idx]
        else:
            return (0, 1)
        dx, dy = nxt[0] - curr[0], nxt[1] - curr[1]
        if abs(dx) >= abs(dy):
            return (1 if dx > 0 else -1, 0)
        else:
            return (0, 1 if dy > 0 else -1)

    def _rotate_offset(self, offset, facing):
        ox, oy = offset
        fx, fy = facing
        if fy == 1:   return (ox, oy)
        if fy == -1:  return (-ox, -oy)
        if fx == 1:   return (-oy, ox)
        if fx == -1:  return (oy, -ox)
        return (ox, oy)

    def _compute_follower_targets(self, env: Environment) -> dict:
        if self.leader is None:
            return {}
        facing = self._get_facing_direction()
        leader_pos = self.leader.position
        n = len(self.followers)
        offsets = []
        if self.formation_type == FormationType.COLUMN:
            for i in range(n):
                offsets.append((0, -(i + 1)))
        else:
            for i in range(n):
                offsets.append(((-1 if i % 2 == 0 else 1) * ((i // 2) + 1), 0))

        targets = {}
        for i, follower in enumerate(self.followers):
            rotated = self._rotate_offset(offsets[i], facing)
            tx = leader_pos[0] + rotated[0]
            ty = leader_pos[1] + rotated[1]
            tz = leader_pos[2]
            if not env.is_walkable(tx, ty, tz):
                found = self._find_nearest_walkable(env, tx, ty, tz)
                tx, ty, tz = found if found else leader_pos
            targets[follower.agent_id] = (tx, ty, tz)
        return targets

    def _find_nearest_walkable(self, env, x, y, z, max_radius=3):
        for r in range(1, max_radius + 1):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    if (abs(dx) == r or abs(dy) == r) and env.in_bounds(x+dx, y+dy, z) and env.is_walkable(x+dx, y+dy, z):
                        return (x+dx, y+dy, z)
        return None