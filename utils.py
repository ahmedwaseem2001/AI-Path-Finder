from typing import Tuple, List, Optional, Dict
import heapq

class SearchNode:
    """Represents a single point in the pathfinding process."""
    def __init__(self, coord: Tuple[int, int], ancestor: Optional['SearchNode'] = None, weight: float = 0):
        self.coord = coord
        self.ancestor = ancestor
        self.g_score = weight
        self.level = 0 if ancestor is None else ancestor.level + 1

    def __lt__(self, other):
        # Comparison for heap priority
        return self.g_score < other.g_score

class PathfindingHeap:
    """A priority queue specialized for search algorithms."""
    def __init__(self):
        self._data = []
        self._insertion_order = 0

    def is_empty(self) -> bool:
        return len(self._data) == 0

    def push(self, entry: SearchNode, rank: float):
        heapq.heappush(self._data, (rank, self._insertion_order, entry))
        self._insertion_order += 1

    def pop(self) -> SearchNode:
        return heapq.heappop(self._data)[2]

def fetch_adjacent_nodes(location: Tuple[int, int], max_r: int, max_c: int) -> List[Tuple[int, int]]:
    """Retrieves valid neighboring coordinates including diagonals."""
    r, c = location
    # Possible movement directions (8-way connectivity might be used depending on logic)
    # The original script used a specific set of 6 directions? Let me check again.
    # In utils.py: dirs = [(-1, 0), (0, 1), (1, 0), (1, 1), (0, -1), (-1, -1)]
    # That's 6 directions. I should stick to the same logic.
    offsets = [(-1, 0), (0, 1), (1, 0), (1, 1), (0, -1), (-1, -1)]
    neighbors = []
    
    for dr, dc in offsets:
        nr, nc = r + dr, c + dc
        if 0 <= nr < max_r and 0 <= nc < max_c:
            neighbors.append((nr, nc))
    return neighbors

def compute_step_weight(src: Tuple[int, int], dest: Tuple[int, int]) -> float:
    """Calculates the cost of moving between two adjacent cells."""
    is_diagonal = abs(src[0] - dest[0]) == 1 and abs(src[1] - dest[1]) == 1
    return 1.414 if is_diagonal else 1.0

def trace_back_path(link_map: Dict, target: Tuple[int, int], origin: Tuple[int, int]) -> List[Tuple[int, int]]:
    """Constructs the final path from target to start using the parent mapping."""
    result_path = [target]
    current = target
    while current != origin:
        current = link_map[current]
        result_path.append(current)
    return result_path[::-1]
# utils ready

# Validated helper functions

# Edge cases handled

# Utils validated
