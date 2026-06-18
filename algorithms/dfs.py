from utils.path_utils import get_neighbors, reconstruct_path, get_energy_cost


def dfs(maze, start, goal, prob_map=None, max_danger=1.0, step_budget=150, energy_budget=100):

    stack = [start]

    visited = {start}

    parent = {start: None}

    # Track steps and energy for path pruning
    steps = {start: 0}
    energy = {start: 0}

    # Verify start itself doesn't violate safety constraints
    if prob_map and maze[start[0]][start[1]] not in ('S', 'G', 'C'):
        if prob_map.get(start, 0.0) > max_danger:
            return None

    while stack:

        current = stack.pop()

        if current == goal:
            return reconstruct_path(parent, goal)

        # If we already reached or exceeded budget limits, do not expand neighbors
        if steps[current] >= step_budget or energy[current] >= energy_budget:
            continue

        for neighbor in get_neighbors(current, maze):
            # Check danger constraint
            p_danger = prob_map.get(neighbor, 0.05) if prob_map else 0.05
            if maze[neighbor[0]][neighbor[1]] not in ('S', 'G', 'C') and p_danger > max_danger:
                continue

            next_steps = steps[current] + 1
            next_energy = energy[current] + get_energy_cost(neighbor, prob_map)

            # Only visit if it stays within budgets
            if next_steps <= step_budget and next_energy <= energy_budget:
                if neighbor not in visited:
                    visited.add(neighbor)
                    parent[neighbor] = current
                    steps[neighbor] = next_steps
                    energy[neighbor] = next_energy
                    stack.append(neighbor)

    return None