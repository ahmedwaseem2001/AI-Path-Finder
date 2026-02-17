import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from grid import NavigationGrid
from algorithms import SearchTechniques
import sys

def launch_graphic_interface():
    """Starts the pathfinding application with a graphical menu."""
    print("\n" + "="*40)
    print(" AI PATHFINDER: GRAPHICAL INTERFACE")
    print("="*40)

    # User input for grid configuration
    try:
        r_input = input("Enter Grid Rows (default 15): ")
        height = int(r_input) if r_input.strip() else 15
        
        c_input = input("Enter Grid Cols (default 15): ")
        width = int(c_input) if c_input.strip() else 15
        
        o_input = input("Enter Obstacle Ratio (0.0-1.0, default 0.2): ")
        obstacle_ratio = float(o_input) if o_input.strip() else 0.2
        
        d_input = input("Enter Default Depth Limit (default 15): ")
        depth_limit = int(d_input) if d_input.strip() else 15
    except ValueError:
        print("Invalid input! Using defaults: 15x15, 0.2 ratio, depth 15.")
        height, width, obstacle_ratio, depth_limit = 15, 15, 0.2, 15

    print(f"Initializing {height}x{width} grid...")
    print("Opening visualization window...")

    exec_speed = 0.05

    # Initialize components
    env_map = NavigationGrid(height, width, obstacle_ratio)
    engine = SearchTechniques(env_map, exec_speed)
    
    # Store the depth limit in the engine so it can be used/updated by GUI
    engine.depth_limit = depth_limit

    # Setup the graphical buttons and initial view
    env_map.render_frame("Pathfinding Menu")
    env_map.setup_gui_controls(engine)

    # Display status info in terminal
    print("Ready. Use the buttons in the window to select algorithms.")
    print("Close the window to exit.")

    # Enter Matplotlib event loop
    plt.show()

if __name__ == "__main__":
    try:
        launch_graphic_interface()
    except KeyboardInterrupt:
        print("\nApplication closed by user.")
        sys.exit(0)
# main loop ready

# Main loop initialized

# Ready for release

# UI ready

# Release 1.0
