from utils.probability_map import get_probability_map
from utils.path_utils import get_energy_cost

def mdp_search(maze, start, goal, prob_map=None, max_danger=1.0, step_budget=150, energy_budget=100):
    """
    Markov Decision Process (MDP) solver using Value Iteration.
    
    States:
      - Coordinate tuples (r, c) representing traversable positions in the maze.
      
    Actions:
      - Up, Down, Left, Right.
      
    Transition Model:
      - 80% chance of successfully moving in the intended direction.
      - 10% chance each of sliding perpendicular to the left or right of the action.
      - Attempting to slide into a wall keeps the agent in its current state.
      
    Rewards:
      - Reaching Goal (G): +500.0
      - Entering a Danger Zone (0 < p_danger < 1.0): -15.0 * p_danger
      - Step Cost: -0.1 (encourages finding the shortest path)
    """
    if prob_map is None:
        prob_map = get_probability_map(maze)
        
    rows = len(maze)
    cols = len(maze[0])
    
    # States are all empty cells (non-walls) that do not exceed max_danger
    states = []
    for r in range(rows):
        for c in range(cols):
            if maze[r][c] != 1:
                p_danger = prob_map.get((r, c), 0.05)
                if maze[r][c] in ('S', 'G', 'C') or p_danger <= max_danger:
                    states.append((r, c))
                
    actions = [(-1, 0), (1, 0), (0, -1), (0, 1)] # Up, Down, Left, Right
    
    # Perpendicular moves for each action
    perp_actions = {
        (-1, 0): [(0, -1), (0, 1)],  # Up -> Left, Right
        (1, 0): [(0, -1), (0, 1)],   # Down -> Left, Right
        (0, -1): [(-1, 0), (1, 0)],  # Left -> Up, Down
        (0, 1): [(-1, 0), (1, 0)],   # Right -> Up, Down
    }
    
    # Tuned MDP Parameters
    GOAL_REWARD = 500.0
    STEP_COST = -0.1
    DANGER_FACTOR = 15.0
    GAMMA = 0.98
    
    # Initialize values
    V = {s: 0.0 for s in states}
    epsilon = 0.001
    max_iter = 1000
    
    # Check if start and goal are valid states
    if start not in V or goal not in V:
        return None
        
    # Value Iteration Loop
    for _ in range(max_iter):
        delta = 0
        V_new = V.copy()
        for s in states:
            if s == goal:
                V_new[s] = 0.0
                continue
                
            max_u = float('-inf')
            for a in actions:
                expected_utility = 0.0
                
                # 1. Intended target (80% prob)
                target = (s[0] + a[0], s[1] + a[1])
                if target not in V:
                    target = s  # bounce off wall
                
                r_target = GOAL_REWARD if target == goal else (STEP_COST - DANGER_FACTOR * prob_map.get(target, 0.05))
                expected_utility += 0.8 * (r_target + GAMMA * V[target])
                
                # 2. Perpendicular side-slips (10% prob each)
                for pa in perp_actions[a]:
                    ptarget = (s[0] + pa[0], s[1] + pa[1])
                    if ptarget not in V:
                        ptarget = s
                    r_ptarget = GOAL_REWARD if ptarget == goal else (STEP_COST - DANGER_FACTOR * prob_map.get(ptarget, 0.05))
                    expected_utility += 0.1 * (r_ptarget + GAMMA * V[ptarget])
                    
                if expected_utility > max_u:
                    max_u = expected_utility
                    
            V_new[s] = max_u
            delta = max(delta, abs(V_new[s] - V[s]))
            
        V = V_new
        if delta < epsilon:
            break
            
    # Derive optimal policy path
    path = [start]
    current = start
    visited = {start}
    accumulated_energy = 0
    
    while current != goal and (len(path) - 1) < step_budget and accumulated_energy < energy_budget:
        best_action = None
        max_u = float('-inf')
        
        for a in actions:
            expected_utility = 0.0
            
            # Intended action path evaluation
            target = (current[0] + a[0], current[1] + a[1])
            if target not in V:
                target = current
            r_target = GOAL_REWARD if target == goal else (STEP_COST - DANGER_FACTOR * prob_map.get(target, 0.05))
            expected_utility += 0.8 * (r_target + GAMMA * V[target])
            
            # Perpendicular side-slips evaluation
            for pa in perp_actions[a]:
                ptarget = (current[0] + pa[0], current[1] + pa[1])
                if ptarget not in V:
                    ptarget = current
                r_ptarget = GOAL_REWARD if ptarget == goal else (STEP_COST - DANGER_FACTOR * prob_map.get(ptarget, 0.05))
                expected_utility += 0.1 * (r_ptarget + GAMMA * V[ptarget])
                
            if expected_utility > max_u:
                max_u = expected_utility
                best_action = a
                
        if best_action is None:
            break
            
        next_cell = (current[0] + best_action[0], current[1] + best_action[1])
        if next_cell not in V or next_cell in visited:
            break
            
        # Enforce energy budget on rollout
        energy_cost = get_energy_cost(next_cell, prob_map)
        if accumulated_energy + energy_cost > energy_budget:
            break
            
        path.append(next_cell)
        visited.add(next_cell)
        accumulated_energy += energy_cost
        current = next_cell
        
    if not path or path[-1] != goal:
        return None
    return path



