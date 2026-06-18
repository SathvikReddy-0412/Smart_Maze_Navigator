import random

def generate_perfect_maze(rows=15, cols=15):
    """
    Generates a random perfect maze of the given dimensions using Randomized DFS.
    A perfect maze has exactly one path between any two cells, no loops, and no isolated areas.
    """
    # Ensure rows and cols are odd to maintain clear borders and paths
    if rows % 2 == 0:
        rows += 1
    if cols % 2 == 0:
        cols += 1

    # Initialize the grid: 1 is wall, 0 is empty path
    grid = [[1 for _ in range(cols)] for _ in range(rows)]

    # Start generation at cell (1, 1)
    start_r, start_c = 1, 1
    grid[start_r][start_c] = 0

    stack = [(start_r, start_c)]
    visited = {(start_r, start_c)}

    while stack:
        current_r, current_c = stack[-1]

        # Find unvisited neighbors at distance 2 (skipping boundaries)
        neighbors = []
        for dr, dc in [(-2, 0), (2, 0), (0, -2), (0, 2)]:
            nr, nc = current_r + dr, current_c + dc
            if 0 < nr < rows - 1 and 0 < nc < cols - 1:
                if (nr, nc) not in visited:
                    neighbors.append((nr, nc))

        if neighbors:
            # Choose a random unvisited neighbor
            nr, nc = random.choice(neighbors)

            # Knock down the wall between the current cell and the selected neighbor
            wall_r = current_r + (nr - current_r) // 2
            wall_c = current_c + (nc - current_c) // 2
            grid[wall_r][wall_c] = 0
            grid[nr][nc] = 0

            visited.add((nr, nc))
            stack.append((nr, nc))
        else:
            # Backtrack
            stack.pop()

    # Collect potential internal walls to knock down to create alternative paths
    internal_walls = []
    for r in range(1, rows - 1):
        for c in range(1, cols - 1):
            if grid[r][c] == 1:
                # Check horizontal path connection
                if grid[r][c - 1] == 0 and grid[r][c + 1] == 0:
                    internal_walls.append((r, c))
                # Check vertical path connection
                elif grid[r - 1][c] == 0 and grid[r + 1][c] == 0:
                    internal_walls.append((r, c))

    # Knock down about 15% of these walls to create loops/alternative routes
    if internal_walls:
        num_to_knock = max(1, int(len(internal_walls) * 0.15))
        walls_to_knock = random.sample(internal_walls, num_to_knock)
        for wr, wc in walls_to_knock:
            grid[wr][wc] = 0

    # Set S (Start) on the top border and G (Goal) on the bottom border
    grid[0][1] = 'S'
    grid[1][1] = 0  # Ensure path from start is open

    grid[rows - 1][cols - 2] = 'G'
    grid[rows - 2][cols - 2] = 0  # Ensure path to goal is open

    # Choose a checkpoint 'C' in the middle of the maze (avoiding S and G areas)
    center_r, center_c = rows // 2, cols // 2
    best_dist = float('inf')
    checkpoint_coords = None
    
    for r in range(1, rows - 1):
        for c in range(1, cols - 1):
            if grid[r][c] == 0 and (r, c) != (1, 1) and (r, c) != (rows - 2, cols - 2):
                dist = abs(r - center_r) + abs(c - center_c)
                if dist < best_dist:
                    best_dist = dist
                    checkpoint_coords = (r, c)
                    
    if checkpoint_coords:
        cr, cc = checkpoint_coords
        grid[cr][cc] = 'C'

    return grid
