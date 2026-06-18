import heapq
from utils.path_utils import get_neighbors, reconstruct_path, get_energy_cost
from utils.probability_map import get_probability_map

def utility_search(maze, start, goal, prob_map=None, max_danger=1.0, step_budget=150, energy_budget=100):
    """
    Utility Search algorithm (Decision-Making under Uncertainty).
    Calculates a path that minimizes total cost, where transition cost
    is computed as: cost = 1.0 + W * probability_of_danger.
    Balances path length and path safety.
    """
    if prob_map is None:
        prob_map = get_probability_map(maze)
        
    W = 10.0  # Weight given to danger penalty
    
    # Priority queue stores (cost, current_node)
    pq = []
    heapq.heappush(pq, (0.0, start))
    
    parent = {start: None}
    g_score = {start: 0.0}
    
    # Track steps and energy for path pruning
    steps = {start: 0}
    energy = {start: 0}

    # Verify start itself doesn't violate safety constraints
    if maze[start[0]][start[1]] not in ('S', 'G', 'C'):
        if prob_map.get(start, 0.0) > max_danger:
            return None
            
    while pq:
        cost, current = heapq.heappop(pq)
        
        if current == goal:
            return reconstruct_path(parent, goal)
            
        if cost > g_score.get(current, float('inf')):
            continue
            
        for neighbor in get_neighbors(current, maze):
            # Check danger constraint
            p_danger = prob_map.get(neighbor, 0.05)
            if maze[neighbor[0]][neighbor[1]] not in ('S', 'G', 'C') and p_danger > max_danger:
                continue
                
            # Step cost is path cost (1) + danger penalty (W * p_danger)
            step_cost = 1.0 + W * p_danger
            tentative_g = g_score[current] + step_cost
            
            next_steps = steps[current] + 1
            next_energy = energy[current] + get_energy_cost(neighbor, prob_map)
            
            # Only visit if it stays within budgets
            if next_steps <= step_budget and next_energy <= energy_budget:
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    parent[neighbor] = current
                    g_score[neighbor] = tentative_g
                    steps[neighbor] = next_steps
                    energy[neighbor] = next_energy
                    heapq.heappush(pq, (tentative_g, neighbor))
                
    return None

