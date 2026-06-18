from flask import Flask, render_template, request, redirect
import time
import webbrowser
from threading import Timer
import os

from utils.maze_loader import load_maze, find_start_goal
from utils.probability_map import get_probability_map

from algorithms.bfs import bfs
from algorithms.dfs import dfs
from algorithms.astar import astar
from algorithms.utility_search import utility_search
from algorithms.csp import csp_search
from algorithms.mdp import mdp_search

app = Flask(__name__)

def open_browser():
    webbrowser.open_new("http://127.0.0.1:5000/")

# Initialize maze on startup if needed
if not os.path.exists("mazes/maze1.txt") or len(load_maze("mazes/maze1.txt")) < 15:
    from utils.maze_generator import generate_perfect_maze
    maze = generate_perfect_maze(15, 15)
    os.makedirs("mazes", exist_ok=True)
    with open("mazes/maze1.txt", "w") as f:
        for row in maze:
            f.write(" ".join(map(str, row)) + "\n")

@app.route("/")
def home():
    maze = load_maze("mazes/maze1.txt")
    prob_map = get_probability_map(maze)
    
    return render_template(
        "index.html",
        maze=maze,
        prob_map=prob_map,
        path=[],
        path_safety=None,
        algorithm=None,
        execution_time=None,
        total_cost=None,
        use_checkpoint=False,
        max_danger=1.0,
        step_budget=150,
        energy_budget=100,
        path_energy=None,
        failure_reason=None
    )


@app.route("/change_maze")
def change_maze():
    from utils.maze_generator import generate_perfect_maze
    maze = generate_perfect_maze(15, 15)
    
    os.makedirs("mazes", exist_ok=True)
    with open("mazes/maze1.txt", "w") as f:
        for row in maze:
            f.write(" ".join(map(str, row)) + "\n")
            
    return redirect("/")


@app.route("/solve", methods=["POST"])
def solve():
    algorithm = request.form.get("algorithm")
    maze = load_maze("mazes/maze1.txt")
    start, goal = find_start_goal(maze)
    prob_map = get_probability_map(maze)

    use_checkpoint = request.form.get("use_checkpoint") == "true"

    # Extract constraints
    try:
        val = float(request.form.get("max_danger", 1.0))
        max_danger = val / 100.0 if val > 1.0 else val
    except (ValueError, TypeError):
        max_danger = 1.0
        
    try:
        step_budget = int(request.form.get("step_budget", 150))
    except (ValueError, TypeError):
        step_budget = 150
        
    try:
        energy_budget = int(request.form.get("energy_budget", 100))
    except (ValueError, TypeError):
        energy_budget = 100

    from utils.path_utils import get_energy_cost

    def calculate_path_energy(p):
        if not p:
            return 0
        energy = 0
        for cell in p[1:]:
            energy += get_energy_cost(cell, prob_map)
        return energy

    def run_solver(alg, start_node, goal_node, m_danger, s_budget, e_budget):
        if alg == "bfs":
            return bfs(maze, start_node, goal_node, prob_map, m_danger, s_budget, e_budget)
        elif alg == "dfs":
            return dfs(maze, start_node, goal_node, prob_map, m_danger, s_budget, e_budget)
        elif alg == "astar":
            return astar(maze, start_node, goal_node, prob_map, m_danger, s_budget, e_budget)
        elif alg == "utility":
            return utility_search(maze, start_node, goal_node, prob_map, m_danger, s_budget, e_budget)
        elif alg == "csp":
            return csp_search(maze, start_node, goal_node, prob_map, m_danger, s_budget, e_budget)
        elif alg == "mdp":
            return mdp_search(maze, start_node, goal_node, prob_map, m_danger, s_budget, e_budget)
        return None

    start_time = time.time()
    path = []

    if use_checkpoint:
        from utils.maze_loader import find_checkpoint
        checkpoint = find_checkpoint(maze)
        if checkpoint:
            # First leg S -> C
            path1 = run_solver(algorithm, start, checkpoint, max_danger, step_budget, energy_budget)
            if path1:
                steps_used = len(path1) - 1
                energy_used = calculate_path_energy(path1)
                
                # Second leg C -> G with remaining budgets
                path2 = run_solver(
                    algorithm, 
                    checkpoint, 
                    goal, 
                    max_danger, 
                    step_budget - steps_used, 
                    energy_budget - energy_used
                )
                if path2:
                    path = path1 + path2[1:]
                else:
                    path = None
            else:
                path = None
        else:
            path = run_solver(algorithm, start, goal, max_danger, step_budget, energy_budget)
    else:
        path = run_solver(algorithm, start, goal, max_danger, step_budget, energy_budget)

    end_time = time.time()
    execution_time = round(end_time - start_time, 6)

    total_cost = 0
    path_energy = None
    failure_reason = None

    if path:
        total_cost = len(path) - 1
        path_energy = calculate_path_energy(path)
    else:
        # Run diagnostic check without step/energy budget constraints to identify the cause of failure
        if use_checkpoint:
            from utils.maze_loader import find_checkpoint
            checkpoint = find_checkpoint(maze)
            if checkpoint:
                path1_unlim = run_solver(algorithm, start, checkpoint, max_danger, 9999, 9999)
                if path1_unlim:
                    path2_unlim = run_solver(algorithm, checkpoint, goal, max_danger, 9999, 9999)
                    if path2_unlim:
                        path_unlim = path1_unlim + path2_unlim[1:]
                    else:
                        path_unlim = None
                else:
                    path_unlim = None
            else:
                path_unlim = run_solver(algorithm, start, goal, max_danger, 9999, 9999)
        else:
            path_unlim = run_solver(algorithm, start, goal, max_danger, 9999, 9999)

        if not path_unlim:
            failure_reason = f"No path exists under Safety Constraint (Max Cell Danger <= {int(max_danger * 100)}%). Start or goal might be isolated."
        else:
            req_steps = len(path_unlim) - 1
            req_energy = calculate_path_energy(path_unlim)
            reasons = []
            if req_steps > step_budget:
                reasons.append(f"required {req_steps} steps vs budget of {step_budget}")
            if req_energy > energy_budget:
                reasons.append(f"required {req_energy} energy vs budget of {energy_budget}")
            failure_reason = "Constraint Violation: " + " and ".join(reasons) + "."

    # Calculate path safety percentage (product of cell survival probabilities)
    path_safety = None
    if path:
        survival_prob = 1.0
        for cell in path:
            r, c = cell
            survival_prob *= (1.0 - prob_map.get((r, c), 0.05))
        path_safety = round(survival_prob * 100, 1)

    return render_template(
        "index.html",
        maze=maze,
        prob_map=prob_map,
        path=path if path is not None else [],
        path_safety=path_safety,
        algorithm=algorithm.upper() if algorithm else None,
        execution_time=execution_time,
        total_cost=total_cost,
        use_checkpoint=use_checkpoint,
        max_danger=max_danger,
        step_budget=step_budget,
        energy_budget=energy_budget,
        path_energy=path_energy,
        failure_reason=failure_reason
    )



if __name__ == "__main__":
    # Auto-open browser exactly 1 second after starting the server
    # Werkzeug run-main check prevents double launches during debug reloads
    if not os.environ.get("WERKZEUG_RUN_MAIN"):
        Timer(1.0, open_browser).start()
        print("\n* Starting server and redirecting to browser at http://127.0.0.1:5000/ ...\n")

    app.run(
        debug=True,
        port=5000
    )