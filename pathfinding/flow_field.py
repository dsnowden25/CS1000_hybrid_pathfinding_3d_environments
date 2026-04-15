"""
flow_field.py - Flow Field Pathfinding

Generates a direction field toward the goal using Dijkstra's algorithm
to build a cost-to-goal (integration) field, then computes approximate
gradients to produce smooth flow directions. Agents independently follow
the field with local collision avoidance.
"""

import heapq
from environment import Environment


class FlowField:
    """
    A precomputed direction field that guides agents toward a goal.

    The field is built in two stages:
        1. Dijkstra from the goal computes cost-to-goal for every
           reachable walkable cell (the "integration field").
        2. For each cell, the gradient of the cost field is approximated
           by looking at walkable neighbors and computing a weighted
           direction vector pointing toward decreasing cost. This produces
           smoother paths than simply picking the single cheapest neighbor.

    Agents query the field at their position to get a next-step cell.
    """

    def __init__(self, env: Environment, goal: tuple):
        self.env = env
        self.goal = goal
        # cost_to_goal: dict mapping (x, y, z) -> float
        self.cost_to_goal = {}
        # flow_dir: dict mapping (x, y, z) -> (nx, ny, nz) best next cell
        self.flow_dir = {}

        self._build(env, goal)

    def _build(self, env: Environment, goal: tuple):
        """
        Build the flow field using Dijkstra + gradient approximation.

        Stage 1 (Integration field):
            Run Dijkstra backwards from the goal to compute the minimum
            cost-to-goal for every reachable cell. Uses the same movement
            costs as A* (1.0 for flat, 1.5 for elevation change).

        Stage 2 (Flow directions via approximate gradient):
            For each cell, examine all walkable neighbors. Compute a
            direction vector as the weighted sum of unit vectors pointing
            toward each neighbor, weighted by how much cheaper that
            neighbor is (cost_current - cost_neighbor). Neighbors with
            higher cost than the current cell contribute nothing (clamped
            to zero). The best next cell is the walkable neighbor that
            lies closest to this gradient direction.

            This produces smoother flow than naive "pick cheapest neighbor"
            because diagonal and multi-neighbor influences are blended -
            similar to how the eikonal equation's gradient integrates
            information from multiple directions.
        """
        if not env.is_walkable(*goal):
            return

        # --- Stage 1: Dijkstra from goal ---
        cost = {goal: 0.0}
        heap = [(0.0, goal)]

        while heap:
            current_cost, current = heapq.heappop(heap)

            if current_cost > cost.get(current, float('inf')):
                continue

            for neighbor in env.get_walkable_neighbors(*current):
                dz = abs(neighbor[2] - current[2])
                move_cost = 1.5 if dz > 0 else 1.0
                new_cost = current_cost + move_cost

                if new_cost < cost.get(neighbor, float('inf')):
                    cost[neighbor] = new_cost
                    heapq.heappush(heap, (new_cost, neighbor))

        self.cost_to_goal = cost

        # --- Stage 2: Approximate gradient -> flow directions ---
        for cell in cost:
            if cell == goal:
                # at the goal, no direction needed
                self.flow_dir[cell] = None
                continue

            neighbors = env.get_walkable_neighbors(*cell)
            if not neighbors:
                self.flow_dir[cell] = None
                continue

            my_cost = cost[cell]

            # compute weighted gradient direction
            # each neighbor contributes a vector pointing from cell -> neighbor,
            # weighted by how much cheaper that neighbor is
            grad_x = 0.0
            grad_y = 0.0
            grad_z = 0.0
            total_weight = 0.0

            for nb in neighbors:
                nb_cost = cost.get(nb, float('inf'))
                # weight = how much cost decreases by going to this neighbor
                # clamp to zero: neighbors that are more expensive don't pull
                weight = max(0.0, my_cost - nb_cost)
                if weight <= 0:
                    continue

                # unit vector from cell toward neighbor
                dx = nb[0] - cell[0]
                dy = nb[1] - cell[1]
                dz = nb[2] - cell[2]
                dist = (dx * dx + dy * dy + dz * dz) ** 0.5
                if dist == 0:
                    continue

                grad_x += weight * (dx / dist)
                grad_y += weight * (dy / dist)
                grad_z += weight * (dz / dist)
                total_weight += weight

            if total_weight == 0:
                # no neighbor is cheaper (local minimum / unreachable)
                # fall back to cheapest neighbor directly
                best = min(neighbors, key=lambda n: cost.get(n, float('inf')))
                if cost.get(best, float('inf')) < my_cost:
                    self.flow_dir[cell] = best
                else:
                    self.flow_dir[cell] = None
                continue

            # normalize the gradient
            grad_x /= total_weight
            grad_y /= total_weight
            grad_z /= total_weight

            # pick the walkable neighbor closest to the gradient direction
            best_nb = None
            best_dot = -float('inf')

            for nb in neighbors:
                nb_cost = cost.get(nb, float('inf'))
                if nb_cost >= my_cost:
                    # don't go uphill
                    continue

                dx = nb[0] - cell[0]
                dy = nb[1] - cell[1]
                dz = nb[2] - cell[2]
                dist = (dx * dx + dy * dy + dz * dz) ** 0.5
                if dist == 0:
                    continue

                # dot product with gradient = alignment
                dot = (dx / dist) * grad_x + (dy / dist) * grad_y + (dz / dist) * grad_z

                if dot > best_dot:
                    best_dot = dot
                    best_nb = nb

            self.flow_dir[cell] = best_nb

    def get_next_cell(self, position: tuple):
        """
        Get the next cell an agent at `position` should move to.
        Returns (x, y, z) or None if at goal / unreachable.
        """
        return self.flow_dir.get(position, None)

    def is_reachable(self, position: tuple) -> bool:
        """Check if a position was reached by the Dijkstra expansion."""
        return position in self.cost_to_goal

    def get_cost(self, position: tuple) -> float:
        """Get the cost-to-goal from a position. Inf if unreachable."""
        return self.cost_to_goal.get(position, float('inf'))


def move_agents_flow_field(agents: list, env: Environment,
                           flow_field: FlowField):
    """
    Move a list of agents one step along the flow field with
    local collision avoidance.

    Collision avoidance strategy:
        - Build a set of currently occupied cells.
        - For each agent (processed in order of distance to goal,
          closest first), look up the flow direction.
        - If the target cell is unoccupied, move there.
        - If occupied, try alternate neighbors that still decrease
          cost-to-goal.
        - If no move is possible, stay put.

    This is a simple priority-based avoidance scheme. Agents closer
    to the goal get priority, which helps prevent the "freezing"
    problem where agents block each other indefinitely.

    Args:
        agents: list of Agent objects (must have .position, .agent_id,
                .at_destination attributes)
        env: the Environment
        flow_field: precomputed FlowField for the shared goal

    Returns:
        dict mapping agent_id -> new position
    """
    # sort agents by cost-to-goal (closest first = highest priority)
    sorted_agents = sorted(
        agents,
        key=lambda a: flow_field.get_cost(a.position)
    )

    occupied = set(a.position for a in agents)
    moves = {}

    for agent in sorted_agents:
        if agent.at_destination:
            moves[agent.agent_id] = agent.position
            continue

        pos = agent.position
        target = flow_field.get_next_cell(pos)

        if target is not None and target not in occupied:
            # preferred direction is free
            occupied.discard(pos)
            occupied.add(target)
            moves[agent.agent_id] = target
        else:
            # try alternate neighbors that decrease cost
            my_cost = flow_field.get_cost(pos)
            alternatives = env.get_walkable_neighbors(*pos)
            # sort by cost (prefer cheaper = closer to goal)
            alternatives.sort(key=lambda n: flow_field.get_cost(n))

            moved = False
            for alt in alternatives:
                if alt not in occupied and flow_field.get_cost(alt) < my_cost:
                    occupied.discard(pos)
                    occupied.add(alt)
                    moves[agent.agent_id] = alt
                    moved = True
                    break

            if not moved:
                # stay put
                moves[agent.agent_id] = pos

    return moves
