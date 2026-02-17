# AI Pathfinding Visualizer

A Python-based application that visualizes various graph search algorithms on an interactive grid.

## Features
- **Visualizations for 6 Search Algorithms**:
  - Breadth-First Search (BFS)
  - Depth-First Search (DFS)
  - Uniform-Cost Search (UCS)
  - Depth-Limited Search (DLS)
  - Iterative Deepening DFS (IDDFS)
  - Bi-directional Search
- **Interactive Grid**: Randomly generated obstacles, start, and goal positions.
- **Real-time Feedback**: Watch the algorithms explore the grid step-by-step.

## Installation

1.  **Clone the repository** (if applicable) or download the source code.
2.  **Install dependencies**:
    ```bash
    pip install -r requirements.txt
    ```
    *(Note: This project requires `matplotlib` and `numpy`)*

## Usage

Run the main script to launch the graphical interface:

```bash
python main.py
```

### Controls
- Use the buttons on the right side of the window to select an algorithm to run.
- **Reset Grid**: Generates a new random map with new start/goal positions.
- **Exit**: Closes the application.

## Algorithms Implemented

1.  **BFS (Breadth-First Search)**: Explores all neighbor nodes at the present depth prior to moving on to the nodes at the next depth level.
2.  **DFS (Depth-First Search)**: Explores as far as possible along each branch before backtracking.
3.  **UCS (Uniform-Cost Search)**: Expands the node with the lowest path cost (dijkstra-like logic).
4.  **DLS (Depth-Limited Search)**: DFS with a predetermined limit on depth to prevent infinite loops.
5.  **IDDFS (Iterative Deepening DFS)**:  Repeatedly applies DLS with increasing depth limits.
6.  **Bi-directional Search**: Runs two simultaneous searches: one forward from the initial state and one backward from the goal.
