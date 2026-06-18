from utils.path_utils import get_neighbors, get_energy_cost
from utils.probability_map import get_probability_map

def csp_search(maze, start, goal, prob_map=None, max_danger=0.5, step_budget=150, energy_budget=100):
    """
    Constraint Satisfaction Problem (CSP) Solver for Maze Pathfinding.
    
    Variables:
      - X_d: The coordinate (r, c) of the path at step 'd'.
      
    Domains:
      - The set of all open (non-wall) coordinates in the maze.
      
    Constraints:
      - Adjacency: X_d must be orthogonally adjacent to X_{d-1}.
      - No-Loop: X_d must be unique (no repeated cells in the path).
      - Safety (Real-Life): The danger probability of any cell in the path must be <= max_danger.
      - Step Budget (Real-Life): The total path steps must not exceed step_budget.
      - Energy Budget (Real-Life): The total path energy must not exceed energy_budget.
      
    Heuristics:
      - Value Ordering: Neighbors are sorted by their Manhattan distance to the goal
        (guiding the search towards the destination efficiently, similar to LCV/MRV).
    """
    if prob_map is None:
        prob_map = get_probability_map(maze)

    def backtrack(path, current_energy):
        current = path[-1]
        
        # Goal Constraint check
        if current == goal:
            return path
            
        # Step Budget Constraint check (steps = len(path) - 1)
        if len(path) - 1 >= step_budget:
            return None

        # Determine domain (valid neighbors)
        neighbors = []
        for n in get_neighbors(current, maze):
            # No-Loop Constraint check
            if n not in path:
                # Safety Constraint check (S, G, C have danger 0.0, always allowed if not wall)
                p_danger = prob_map.get(n, 0.05)
                if maze[n[0]][n[1]] in ('S', 'G', 'C') or p_danger <= max_danger:
                    e_cost = get_energy_cost(n, prob_map)
                    if current_energy + e_cost <= energy_budget:
                        neighbors.append((n, e_cost))

        # Apply Value Ordering heuristic (Manhattan distance to goal)
        neighbors.sort(key=lambda item: abs(item[0][0] - goal[0]) + abs(item[0][1] - goal[1]))

        # Try assigning next step
        for next_cell, e_cost in neighbors:
            path.append(next_cell)
            result = backtrack(path, current_energy + e_cost)
            if result:
                return result
            # Backtrack
            path.pop()

        return None

    initial_path = [start]
    # Verify start itself doesn't violate safety constraints
    start_danger = prob_map.get(start, 0.0)
    if maze[start[0]][start[1]] not in ('S', 'G', 'C') and start_danger > max_danger:
        return None

    solution = backtrack(initial_path, 0)
    return solution

