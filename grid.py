import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.widgets import Button, TextBox
import numpy as np
import random
from typing import Tuple, List, Set, Optional

class NavigationGrid:
    """Manages the environment and visual representation of the search space with GUI controls."""
    
    CELL_OPEN = 0
    CELL_BLOCK = 1
    CELL_BEGIN = 2
    CELL_GOAL = 3

    def __init__(self, height: int, width: int, block_rate: float = 0.2):
        self.rows = height
        self.cols = width
        self.matrix = np.zeros((height, width), dtype=int)
        self.start_pos, self.goal_pos = None, None

        self.front_nodes = set()
        self.history = set()
        self.active_node = None
        self.trajectory = []
        self.display_fig, self.display_ax = None, None
        
        # GUI elements
        self.buttons = []
        self.button_axes = []
        self.text_box = None
        self.text_ax = None

        self.initialize_map(block_rate)

    def initialize_map(self, density: float):
        """Generates the grid with random obstacles, start, and target."""
        self.matrix.fill(self.CELL_OPEN)
        for r in range(self.rows):
            for c in range(self.cols):
                if random.random() < density:
                    self.matrix[r][c] = self.CELL_BLOCK

        available_slots = [(r, c) for r in range(self.rows) for c in range(self.cols) if self.matrix[r][c] == self.CELL_OPEN]
        
        if len(available_slots) < 2:
            self.matrix.fill(self.CELL_OPEN)
            available_slots = [(r, c) for r in range(self.rows) for c in range(self.cols)]

        self.start_pos = random.choice(available_slots)
        available_slots.remove(self.start_pos)
        self.goal_pos = random.choice(available_slots)

        self.matrix[self.start_pos] = self.CELL_BEGIN
        self.matrix[self.goal_pos] = self.CELL_GOAL

    def is_traversable(self, pos: Tuple[int, int]) -> bool:
        """Checks if a position is within bounds and not a wall."""
        r, c = pos
        return 0 <= r < self.rows and 0 <= c < self.cols and self.matrix[r][c] != self.CELL_BLOCK

    def clear_render_state(self):
        """Resets the data used for visualization."""
        self.front_nodes.clear()
        self.history.clear()
        self.active_node = None
        self.trajectory = []

    def setup_gui_controls(self, search_engine):
        """Creates interactive buttons on the side of the visualization."""
        if self.display_fig is None:
            self.render_frame("Initializing Interface...")

        # Add buttons in a column on the right
        btn_labels = [
            "BFS", "DFS", "UCS", 
            "DLS", "IDDFS", "Bidirectional", 
            "Reset Grid", "Exit"
        ]
        
        # Functions to bind
        def run_bfs(event): search_engine.run_bfs()
        def run_dfs(event): search_engine.run_dfs()
        def run_ucs(event): search_engine.run_ucs()
        def run_dls(event): search_engine.run_dls() # Use default/configured depth
        def run_iddfs(event): search_engine.run_iddfs() # Use default/configured depth
        def run_bi(event): search_engine.run_bi_search()
        def reset_map(event):
            self.initialize_map(0.2)
            self.clear_render_state()
            self.render_frame("Environment Reset")
        def close_app(event): plt.close(self.display_fig)

        callbacks = [run_bfs, run_dfs, run_ucs, run_dls, run_iddfs, run_bi, reset_map, close_app]
        
        # Clear existing buttons
        for ax in self.button_axes:
            ax.remove()
        self.button_axes = []
        self.buttons = []
        if self.text_ax:
            self.text_ax.remove()
            self.text_ax = None

        # Position buttons
        x_pos = 0.82
        y_start = 0.85
        y_step = 0.08
        btn_width = 0.15
        btn_height = 0.05

        # Add Depth Limit Text Box above buttons
        self.text_ax = plt.axes([x_pos, 0.92, btn_width, 0.04])
        self.text_box = TextBox(self.text_ax, 'Depth: ', initial=str(search_engine.depth_limit))
        
        def submit_depth(text):
            try:
                val = int(text)
                search_engine.depth_limit = val
                print(f"Depth limit updated to {val}")
            except ValueError:
                print("Invalid depth!")
                self.text_box.set_val(str(search_engine.depth_limit))
        
        self.text_box.on_submit(submit_depth)

        for i, (label, callback) in enumerate(zip(btn_labels, callbacks)):
            ax_btn = plt.axes([x_pos, y_start - (i * y_step), btn_width, btn_height])
            btn = Button(ax_btn, label, color='#EEEEEE', hovercolor='#DDCCAA')
            btn.on_clicked(callback)
            self.button_axes.append(ax_btn)
            self.buttons.append(btn)

    def render_frame(self, header: str = "Search Visualization", delay: float = 0.05):
        """Updates the visual output of the pathfinding progress."""
        if self.display_fig is None or not plt.fignum_exists(self.display_fig.number):
            self.display_fig, self.display_ax = plt.subplots(figsize=(14, 9))
            plt.subplots_adjust(right=0.8, left=0.05)
        else:
            self.display_ax.clear()

        # Define color palette
        theme = np.ones((self.rows, self.cols, 3)) 

        for r in range(self.rows):
            for c in range(self.cols):
                point = (r, c)
                if self.matrix[r][c] == self.CELL_BLOCK:
                    theme[r, c] = [0.1, 0.1, 0.1]
                elif point == self.start_pos:
                    theme[r, c] = [0.2, 0.8, 0.2]
                elif point == self.goal_pos:
                    theme[r, c] = [0.8, 0.2, 0.2]
                elif point in self.trajectory:
                    theme[r, c] = [1.0, 0.9, 0.0]
                elif point == self.active_node:
                    theme[r, c] = [1.0, 0.5, 0.0]
                elif point in self.front_nodes:
                    theme[r, c] = [0.5, 0.8, 0.9]
                elif point in self.history:
                    theme[r, c] = [0.8, 0.8, 0.8]

        self.display_ax.imshow(theme, interpolation='nearest')
        self.display_ax.set_title(header, fontsize=16, fontweight='bold')
        self.display_ax.set_xticks(np.arange(-0.5, self.cols, 1), minor=True)
        self.display_ax.set_yticks(np.arange(-0.5, self.rows, 1), minor=True)
        self.display_ax.grid(which='minor', color='#CCCCCC', linestyle='-', linewidth=0.5)
        self.display_ax.tick_params(which='both', size=0, labelsize=0)

        if self.start_pos: 
            self.display_ax.text(self.start_pos[1], self.start_pos[0], 'START', ha='center', va='center', color='black', fontsize=8, fontweight='bold')
        if self.goal_pos: 
            self.display_ax.text(self.goal_pos[1], self.goal_pos[0], 'END', ha='center', va='center', color='black', fontsize=8, fontweight='bold')

        if self.trajectory:
            path_pts = np.array(self.trajectory)
            self.display_ax.plot(path_pts[:, 1], path_pts[:, 0], color='#FFD700', linewidth=4, alpha=0.7)

        visual_keys = [
            mpatches.Patch(color='#33CC33', label='Origin'),
            mpatches.Patch(color='#CC3333', label='Destination'),
            mpatches.Patch(color='#1A1A1A', label='Obstacle'),
            mpatches.Patch(color='#80CCE6', label='Queue/Frontier'),
            mpatches.Patch(color='#CCCCCC', label='Visited'),
            mpatches.Patch(color='#FF8000', label='Current Inspect'),
            mpatches.Patch(color='#FFD700', label='Resulting Path')
        ]
        self.display_ax.legend(handles=visual_keys, loc='upper left', bbox_to_anchor=(1.02, 0.2))

        plt.draw()
        if delay > 0:
            plt.pause(delay)
# grid initialized

# Grid rendering optimized

# UI colors updated

# Grid rendering optimized
