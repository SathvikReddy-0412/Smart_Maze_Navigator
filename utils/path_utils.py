def get_neighbors(node, maze):

    rows = len(maze)
    cols = len(maze[0])

    row, col = node

    directions = [
        (-1, 0),
        (1, 0),
        (0, -1),
        (0, 1)
    ]

    neighbors = []

    for dr, dc in directions:

        nr = row + dr
        nc = col + dc

        if (
            0 <= nr < rows and
            0 <= nc < cols and
            maze[nr][nc] != 1
        ):
            neighbors.append((nr, nc))

    return neighbors


def reconstruct_path(parent, goal):

    path = []

    current = goal

    while current is not None:

        path.append(current)
        current = parent[current]

    path.reverse()

    return path


def get_energy_cost(cell, prob_map):
    """
    Computes energy consumption for a given cell based on its danger probability.
    - Danger <= 0.05 (baseline or start/goal/checkpoint): 1 energy unit.
    - Danger > 0.05: 1 + int(p_danger * 10) energy units.
    """
    if not prob_map:
        return 1
    p_danger = prob_map.get(cell, 0.05)
    if p_danger <= 0.05:
        return 1
    return 1 + int(p_danger * 10)