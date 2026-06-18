import heapq

from utils.path_utils import (
    get_neighbors,
    reconstruct_path,
    get_energy_cost
)


def heuristic(a, b):

    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def astar(maze, start, goal, prob_map=None, max_danger=1.0, step_budget=150, energy_budget=100):

    open_set = []

    heapq.heappush(
        open_set,
        (0, start)
    )

    parent = {start: None}

    g_score = {start: 0}
    energy_score = {start: 0}

    # Verify start itself doesn't violate safety constraints
    if prob_map and maze[start[0]][start[1]] not in ('S', 'G', 'C'):
        if prob_map.get(start, 0.0) > max_danger:
            return None

    while open_set:

        _, current = heapq.heappop(open_set)

        if current == goal:
            return reconstruct_path(parent, goal)

        for neighbor in get_neighbors(current, maze):
            # Check danger constraint
            p_danger = prob_map.get(neighbor, 0.05) if prob_map else 0.05
            if maze[neighbor[0]][neighbor[1]] not in ('S', 'G', 'C') and p_danger > max_danger:
                continue

            tentative_g = g_score[current] + 1
            tentative_energy = energy_score[current] + get_energy_cost(neighbor, prob_map)

            # Only visit if it stays within budgets
            if tentative_g <= step_budget and tentative_energy <= energy_budget:
                if (
                    neighbor not in g_score
                    or
                    tentative_g < g_score[neighbor]
                ):

                    parent[neighbor] = current
                    g_score[neighbor] = tentative_g
                    energy_score[neighbor] = tentative_energy

                    f_score = (
                        tentative_g
                        +
                        heuristic(neighbor, goal)
                    )

                    heapq.heappush(
                        open_set,
                        (f_score, neighbor)
                    )

    return None