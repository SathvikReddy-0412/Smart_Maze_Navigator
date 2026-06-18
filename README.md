# Smart Maze Navigator 🌐
### AI Decision Making & Route Optimization under Uncertainty and Constraints

Welcome to **Smart Maze Navigator**, an interactive web-based path planning application designed to explore decision-making under uncertainty, safety constraints, and resource budgets. Built for first-year Project-Based Learning (Semester 3), this project implements and compares classical and probabilistic AI search algorithms on grid-based environments.

---

## 🚀 Key Features

* **Vibrant Modern UI**: Features a dark-themed glassmorphism interface with HSL tailored color schemes, hover states, linear gradient highlights, and micro-animations for grid layout rendering.
* **Six Navigation Algorithms**: Compare classical graph searches with utility-based, constraint-satisfaction, and Markovian decision models:
  * **BFS** (Breadth-First Search)
  * **DFS** (Depth-First Search)
  * **A\* Search** (Informed Heuristic)
  * **Utility Search** (Decision Theory balancing path length and risk)
  * **CSP** (Constraint Satisfaction Problem Search)
  * **MDP** (Markov Decision Process Value Iteration under transition uncertainty)
* **Resource and Safety Budgets**:
  * **Max Cell Danger Limit (%)**: Prevent paths from traversing through risky zones.
  * **Step Budget (Max Steps)**: Constrains path length.
  * **Energy Budget (Units)**: Incorporates danger-dependent energy consumption.
* **Checkpoint Stopover (C)**: Support for multi-leg routing (Start → Checkpoint → Goal).
* **Fault Diagnostics**: In case a route cannot be found, the app executes a diagnostics algorithm to explain whether it failed due to safety isolation or because of step/energy budget violations.
* **Perfect Maze Generator**: Generate randomized perfect grid mazes on the fly.

---

## 📂 Project Architecture

```plaintext
Smart_Maze_Navigator/
│
├── algorithms/                 # Core Pathfinding Algorithms
│   ├── astar.py                # A* informed search using Manhattan heuristic
│   ├── bfs.py                  # Uniform BFS search
│   ├── csp.py                  # Constraint Satisfaction search enforcing local/global bounds
│   ├── dfs.py                  # Deep backtracking search
│   ├── mdp.py                  # MDP Value Iteration solver under slip probability
│   └── utility_search.py       # Cost vs Risk decision search
│
├── static/
│   └── style.css               # Styling rules for UI components and interactive elements
│
├── templates/
│   └── index.html              # Frontend page template
│
├── utils/                      # Helper modules
│   ├── maze_generator.py       # Randomized perfect maze generation algorithms
│   ├── maze_loader.py          # Parsers for file-based mazes (handling Start, Goal, Checkpoints)
│   ├── path_utils.py           # Coordinate neighbors, paths, and energy computations
│   └── probability_map.py      # Danger/probability density mapping
│
├── mazes/                      # Storage folder for current maze config
│   └── maze1.txt               # Currently loaded maze grid file
│
├── app.py                      # Main Flask application and server entrypoint
├── requirements.txt            # Package dependencies
└── README.md                   # Project documentation
```

---

## 🛠️ Installation & Setup

### Prerequisites
* Python 3.8 or above installed on your system.

### Steps
1. **Clone the Repository**:
   ```bash
   git clone https://github.com/SathvikReddy-0412/Smart_Maze_Navigator.git
   cd Smart_Maze_Navigator
   ```

2. **Set up Virtual Environment** *(Optional but recommended)*:
   ```bash
   python -m venv .venv
   # On Windows
   .venv\Scripts\activate
   # On macOS/Linux
   source .venv/bin/activate
   ```

3. **Install Dependencies**:
   ```bash
   pip install flask
   ```

4. **Run the Application**:
   ```bash
   python app.py
   ```

5. **Interact**: 
   The application will automatically start the server and open your default web browser to `http://127.0.0.1:5000/`.

---

## 🔬 Algorithm Comparison & Real-World Analogs

| Algorithm | Traversal Strategy | Constraints Handled | Real-World Application |
| :--- | :--- | :--- | :--- |
| **BFS** | Explores frontier uniformly in layers. | None. Guarantees fewest steps. | **Emergency Evacuation**: Fast escape routing when corridors are equally safe. |
| **DFS** | Backtracking exploration down deep paths. | None. Does not guarantee optimal path. | **Cave / Pipe Mapping**: Autonomous robot surveying linear subterranean networks. |
| **A\*** | Guided heuristic search ($f(n) = g(n) + h(n)$). | Cell Danger, Step/Energy Budgets. | **GPS Navigation**: Used in Google Maps and video games for obstacle avoidance. |
| **Utility** | Evaluates tradeoffs: $U = \text{Steps} + W \times \text{Danger}$. | Cell Danger, Step/Energy Budgets. | **Hazmat Transport**: Routing dangerous cargo away from urban zones at a slight cost to distance. |
| **CSP** | Treats constraints as hard rules. | Strict limits. Fails early if bounds are breached. | **Drone Flight Planning**: Restricting path planning to stay within wind and battery limits. |
| **MDP** | Models stochastic movement (e.g., 80% success, 10% slide). | Safety and Energy rollout bounds. | **Mars Rover / Icy Roads**: Navigating slippery terrains where actions can have uncertain outcomes. |

---

## ⚙️ How It Works (The Technical Math)

### 1. Probability Map
The system assigns a risk parameter ($P_{\text{danger}}$) to each cell:
* **Walls**: $P_{\text{danger}} = 1.0$ (impassable)
* **Start, Goal, and Checkpoint**: $P_{\text{danger}} = 0.0$
* **Danger Zones (orange outline)**: $P_{\text{danger}} = 0.8$ or $0.4$ (deterministic dynamic assignment)
* **Standard Paths**: $P_{\text{danger}} = 0.05$

### 2. Path Safety Evaluation
Overall path safety is computed as the joint product of cell survival rates:
$$\text{Path Safety (\%)} = \left( \prod_{c \in \text{Path}} (1.0 - P_{\text{danger}}(c)) \right) \times 100$$

### 3. Dynamic Energy Consumption
The energy consumption matches real-world friction:
$$\text{Energy Cost}(c) = \begin{cases} 
1 & \text{if } P_{\text{danger}}(c) \le 0.05 \\
1 + \lfloor P_{\text{danger}}(c) \times 10 \rfloor & \text{if } P_{\text{danger}}(c) > 0.05 
\end{cases}$$

### 4. Non-Deterministic Transition (MDP)
For the MDP, when the agent commands action $A$ (e.g., Up), the transition probabilities are:
* **Intended movement ($A$)**: $80\%$
* **Slip perpendicular left**: $10\%$
* **Slip perpendicular right**: $10\%$
If the slip goes into a wall, the agent stays in its current cell.

---

## 🤝 Contributing
Feel free to fork this project, submit issues, or issue pull requests.

## 📝 License
This project is licensed under the MIT License.
