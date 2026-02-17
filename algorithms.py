from collections import deque
from typing import Tuple, List, Set, Optional, Dict
from utils import SearchNode, PathfindingHeap, fetch_adjacent_nodes, compute_step_weight, trace_back_path
from grid import NavigationGrid
import time

class SearchTechniques:
    """Implementations of various graph search algorithms for navigation."""

    def __init__(self, nav_grid: NavigationGrid, wait_time: float = 0.05):
        self.nav_map = nav_grid
        self.latency = wait_time
        self.current_algo_tag = ""
        self.depth_limit = 15

    def _refresh_view(self, extra_info=""):
        """Triggers a grid visualization update."""
        if self.latency > 0:
            self.nav_map.render_frame(
                header=f"{self.current_algo_tag}{extra_info}",
                delay=self.latency
            )

    def run_bfs(self) -> Optional[List[Tuple[int, int]]]:
        """Executes Breadth-First Search."""
        self.current_algo_tag = "BFS Algorithm"
        self.nav_map.clear_render_state()

        origin, goal = self.nav_map.start_pos, self.nav_map.goal_pos
        work_queue = deque([SearchNode(origin)])
        seen_coords = {origin}
        path_tracker = {origin: None}
        self.nav_map.front_nodes.add(origin)

        while work_queue:
            node_to_check = work_queue.popleft()
            pos = node_to_check.coord

            self.nav_map.active_node = pos
            self.nav_map.history.add(pos)
            if pos in self.nav_map.front_nodes:
                self.nav_map.front_nodes.remove(pos)

            self._refresh_view()

            if pos == goal:
                final_route = trace_back_path(path_tracker, goal, origin)
                self.nav_map.trajectory = final_route
                self.nav_map.active_node = None
                self._refresh_view(" - Target Acquired!")
                return final_route

            for next_step in fetch_adjacent_nodes(pos, self.nav_map.rows, self.nav_map.cols):
                if next_step not in seen_coords and self.nav_map.is_traversable(next_step):
                    seen_coords.add(next_step)
                    path_tracker[next_step] = pos
                    work_queue.append(SearchNode(next_step, node_to_check))
                    self.nav_map.front_nodes.add(next_step)
        return None

    def run_dfs(self) -> Optional[List[Tuple[int, int]]]:
        """Executes Depth-First Search."""
        self.current_algo_tag = "DFS Algorithm"
        self.nav_map.clear_render_state()

        origin, goal = self.nav_map.start_pos, self.nav_map.goal_pos
        work_stack = [SearchNode(origin)]
        visited_registry = set()
        path_tracker = {origin: None}
        self.nav_map.front_nodes.add(origin)

        while work_stack:
            node_to_check = work_stack.pop()
            pos = node_to_check.coord

            if pos in visited_registry:
                continue
            visited_registry.add(pos)

            self.nav_map.active_node = pos
            self.nav_map.history.add(pos)
            if pos in self.nav_map.front_nodes:
                self.nav_map.front_nodes.remove(pos)

            self._refresh_view()

            if pos == goal:
                final_route = trace_back_path(path_tracker, goal, origin)
                self.nav_map.trajectory = final_route
                self.nav_map.active_node = None
                self._refresh_view(" - Goal Reached!")
                return final_route

            neighbors = fetch_adjacent_nodes(pos, self.nav_map.rows, self.nav_map.cols)
            for adj in reversed(neighbors):
                if adj not in visited_registry and self.nav_map.is_traversable(adj):
                    path_tracker[adj] = pos
                    work_stack.append(SearchNode(adj, node_to_check))
                    self.nav_map.front_nodes.add(adj)
        return None

    def run_ucs(self) -> Optional[List[Tuple[int, int]]]:
        """Executes Uniform-Cost Search."""
        self.current_algo_tag = "UCS Algorithm"
        self.nav_map.clear_render_state()

        origin, goal = self.nav_map.start_pos, self.nav_map.goal_pos
        priority_heap = PathfindingHeap()
        priority_heap.push(SearchNode(origin, weight=0), 0)

        permanent_set = set()
        path_tracker = {origin: None}
        accumulated_costs = {origin: 0}
        self.nav_map.front_nodes.add(origin)

        while not priority_heap.is_empty():
            node = priority_heap.pop()
            pos = node.coord

            if pos in permanent_set:
                continue
            permanent_set.add(pos)

            self.nav_map.active_node = pos
            self.nav_map.history.add(pos)
            if pos in self.nav_map.front_nodes:
                self.nav_map.front_nodes.remove(pos)

            self._refresh_view()

            if pos == goal:
                final_route = trace_back_path(path_tracker, goal, origin)
                self.nav_map.trajectory = final_route
                self.nav_map.active_node = None
                self._refresh_view(f" - Optimized Path Found! Cost: {accumulated_costs[goal]:.2f}")
                return final_route

            for neighbor in fetch_adjacent_nodes(pos, self.nav_map.rows, self.nav_map.cols):
                if self.nav_map.is_traversable(neighbor):
                    step_cost = compute_step_weight(pos, neighbor)
                    newly_computed_cost = accumulated_costs[pos] + step_cost
                    if neighbor not in accumulated_costs or newly_computed_cost < accumulated_costs[neighbor]:
                        accumulated_costs[neighbor] = newly_computed_cost
                        path_tracker[neighbor] = pos
                        priority_heap.push(SearchNode(neighbor, node, newly_computed_cost), newly_computed_cost)
                        self.nav_map.front_nodes.add(neighbor)
        return None

    def run_dls(self, depth_cap: int = None) -> Optional[List[Tuple[int, int]]]:
        """Executes Depth-Limited Search."""
        limit = depth_cap if depth_cap is not None else self.depth_limit
        self.current_algo_tag = f"DLS (Max Depth: {limit})"
        self.nav_map.clear_render_state()
        return self._perform_recursive_dls(self.nav_map.start_pos, limit, {self.nav_map.start_pos: None}, set())

    def _perform_recursive_dls(self, current, remaining_depth, links, explored) -> Optional[List[Tuple[int, int]]]:
        """Recursive helper for depth-limited search."""
        self.nav_map.active_node = current
        self.nav_map.history.add(current)
        if current in self.nav_map.front_nodes:
            self.nav_map.front_nodes.remove(current)
        self._refresh_view()

        if current == self.nav_map.goal_pos:
            route = trace_back_path(links, self.nav_map.goal_pos, self.nav_map.start_pos)
            self.nav_map.trajectory = route
            self.nav_map.active_node = None
            self._refresh_view(" - Path Found!")
            return route

        if remaining_depth <= 0:
            return None

        explored.add(current)
        adjacent = fetch_adjacent_nodes(current, self.nav_map.rows, self.nav_map.cols)
        
        # Identify frontier nodes for visualization
        for loc in adjacent:
            if loc not in explored and self.nav_map.is_traversable(loc):
                self.nav_map.front_nodes.add(loc)

        for loc in adjacent:
            if loc not in explored and self.nav_map.is_traversable(loc):
                links[loc] = current
                outcome = self._perform_recursive_dls(loc, remaining_depth - 1, links, explored)
                if outcome:
                    return outcome
        explored.remove(current)
        return None

    def run_iddfs(self, upper_limit: int = None) -> Optional[List[Tuple[int, int]]]:
        """Executes Iterative Deepening Depth-First Search."""
        # Use provided limit, or default to configured depth (strict user control)
        limit = upper_limit if upper_limit is not None else self.depth_limit
        self.current_algo_tag = f"IDDFS (Max Depth: {limit})"
        for current_max in range(limit + 1):
            self.nav_map.clear_render_state()
            found_path = self._perform_recursive_dls(self.nav_map.start_pos, current_max, {self.nav_map.start_pos: None}, set())
            if found_path:
                return found_path
        return None

    def run_bi_search(self) -> Optional[List[Tuple[int, int]]]:
        """Executes Bidirectional Breadth-First Search."""
        self.current_algo_tag = "Bidirectional Pathmaking"
        self.nav_map.clear_render_state()

        start, finish = self.nav_map.start_pos, self.nav_map.goal_pos
        fwd_work, bwd_work = deque([start]), deque([finish])
        fwd_map, bwd_map = {start: None}, {finish: None}
        self.nav_map.front_nodes.add(start)
        self.nav_map.front_nodes.add(finish)

        while fwd_work and bwd_work:
            # Step forward
            node_f = fwd_work.popleft()
            self.nav_map.active_node = node_f
            self.nav_map.history.add(node_f)
            self._refresh_view(" (Exploring Forward)")
            if node_f in bwd_map:
                return self._bridge_bi_directional_paths(fwd_map, bwd_map, node_f)
            
            for adj in fetch_adjacent_nodes(node_f, self.nav_map.rows, self.nav_map.cols):
                if adj not in fwd_map and self.nav_map.is_traversable(adj):
                    fwd_map[adj] = node_f
                    fwd_work.append(adj)
                    self.nav_map.front_nodes.add(adj)

            # Step backward
            node_b = bwd_work.popleft()
            self.nav_map.active_node = node_b
            self.nav_map.history.add(node_b)
            self._refresh_view(" (Exploring Backward)")
            if node_b in fwd_map:
                return self._bridge_bi_directional_paths(fwd_map, bwd_map, node_b)
            
            for adj in fetch_adjacent_nodes(node_b, self.nav_map.rows, self.nav_map.cols):
                if adj not in bwd_map and self.nav_map.is_traversable(adj):
                    bwd_map[adj] = node_b
                    bwd_work.append(adj)
                    self.nav_map.front_nodes.add(adj)
        return None

    def _bridge_bi_directional_paths(self, f_links, b_links, junction):
        """Combines paths from start and goal when they meet."""
        f_part = []
        scanner = junction
        while scanner is not None:
            f_part.append(scanner)
            scanner = f_links[scanner]
        f_part.reverse()

        b_part = []
        scanner = b_links[junction]
        while scanner is not None:
            b_part.append(scanner)
            scanner = b_links[scanner]

        complete_route = f_part + b_part
        self.nav_map.trajectory = complete_route
        self.nav_map.active_node = None
        self._refresh_view(" - Bi-directional Match Found!")
        return complete_route
# algorithms loaded

# Basic search strategies loaded

# Algorithms optimized

# BFS loaded

# DFS loaded

# UCS loaded

# Bi-directional loaded
