def get_probability_map(maze):
    """
    Returns a probability map representing the likelihood of danger or
    obstacles in each cell of the maze.
    - S and G have 0.0 probability of danger.
    - Walls (1) have 1.0 probability.
    - Certain cells are designated as danger zones with high probability (e.g., 0.8).
    - Normal empty paths (0) have a low baseline danger probability (e.g., 0.05).
    """
    rows = len(maze)
    cols = len(maze[0])
    prob_map = {}
    
    for r in range(rows):
        for c in range(cols):
            val = maze[r][c]
            if val == 1:
                prob_map[(r, c)] = 1.0
            elif val in ('S', 'G', 'C'):
                prob_map[(r, c)] = 0.0
            elif val == 0 and (r * 7 + c * 13) % 11 == 0:
                # Deterministic dynamic danger zones: 40% or 80% risk
                prob_map[(r, c)] = 0.8 if (r + c) % 2 == 0 else 0.4
            else:
                prob_map[(r, c)] = 0.05  # baseline low danger
                
    return prob_map
