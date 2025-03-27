"""

This is an implementation of the A* algorithm in a static environment.

"""
import heapq

class Node:
    def __init__(self, position, parent=None, start_cost=0, heuristic_cost=0):
        self.position = position  # (x, y) grid coordinates
        self.parent = parent  # Parent node to reconstruct path
        self.start_cost = start_cost  # Cost from start to this node
        self.heuristic_cost = heuristic_cost  # Estimated cost from this node to end
        self.total_cost = start_cost + heuristic_cost  # Total cost

    # For comparison of two nodes
    def __lt__(self, other):
        return self.total_cost < other.total_cost
    
# Predict the distance between two points
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star(grid, start, end):
    open_list = []  # Priority queue for open nodes
    closed_set = set()  # Set of visited nodes

    # Create a start node
    start_node = Node(start, None, 0, heuristic(start, end))
    heapq.heappush(open_list, start_node)  # Add it to the heap

    # We will loop until all possible nodes have been visited
    while open_list:
        current_node = heapq.heappop(open_list)  # Get node with lowest total cost

        # If our end node has been reached
        if current_node.position == end:
            path = []  # To reconstruct the path

            # Reconstruct path using parent nodes
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent

            return path[::-1]  # Return path reversed
        
        closed_set.add(current_node.position)  # Add visited node to closed set

        # For all possible directions
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            # Find position of neighbor
            neighbor_position = (current_node.position[0] + dx, current_node.position[1] + dy)

            # Check if the neighbor is within bounds and visitable (==0)
            if (0 <= neighbor_position[0] < len(grid) and 0 <= neighbor_position[1] < len(grid[0]) and
                grid[neighbor_position[0]][neighbor_position[1]] == 0 and
                neighbor_position not in closed_set):

                start_cost = current_node.start_cost + 1  # Increment start cost
                heuristic_cost = heuristic(neighbor_position, end)  # Calculate heuristic cost
                # Create neighbor node
                neighbor_node = Node(neighbor_position, current_node, start_cost, heuristic_cost)

                # If the node is already in open list with lower cost
                if any(n.position == neighbor_node.position and n.total_cost <= neighbor_node.total_cost for n in open_list):
                    continue  # ... we skip it

                heapq.heappush(open_list, neighbor_node)  # Add neighbor to heap

    return None  # Return None if no path found


if __name__ == "__main__":
    # We use a matrix with visitable points 0's and obstacles 1's.
    grid = [
        [0, 1, 0, 0, 0],
        [0, 1, 0, 1, 0],
        [0, 0, 0, 1, 0],
        [1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0]
    ]

    start = (0, 0)  # Start point
    end = (4, 4)  # Desired end point

    path = a_star(grid, start, end)  # Run A* algorithm

    print("Path:", path)  # Print returned path
