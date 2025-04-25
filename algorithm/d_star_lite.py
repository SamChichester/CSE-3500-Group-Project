import heapq
from collections import defaultdict

class DynamicAStar:
    def __init__(self, grid, start, goal):
        self.grid = grid
        self.rows, self.cols = len(grid), len(grid[0])
        self.start = start
        self.goal = goal
        #g is cost from goal to given node
        self.g = defaultdict(lambda: float('inf'))
        #rhs one step lookahead value initialized with inf for all nodes except the goal
        self.rhs = defaultdict(lambda: float('inf'))
        self.g[goal] = float('inf')
        self.rhs[goal] = 0

        self.open = []
        self.closed = set()
        heapq.heappush(self.open, (self._key(goal), goal))
        self.compute_path()
    #Our heuristic we chose manhatten for basic left right up down calculations
    def _heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])  # Manhattan distance
    #This function returns if a coordinate is within bounds this is a function used in other functions to check.
    def _in_bounds(self, x, y):
        return 0 <= x < self.rows and 0 <= y < self.cols
    #This function checks the neighbors of the given coordinate 
    #This uses in_bounds to see if they are in bounds and also if they are the valid type != #
    def _neighbors(self, x, y):
        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
            nx, ny = x + dx, y + dy
            if self._in_bounds(nx, ny) and self.grid[nx][ny] != '#':
                yield (nx, ny)
    #This function checks between the rhs and g values and determines cost
    #If it then returns a tuple of this cost with our heuristic and the cost, used for sorting the open list
    def _key(self, pos):
        cost = min(self.g[pos], self.rhs[pos])
        return (cost + self._heuristic(pos, self.start), cost)
    #This is a function to determine the cost of moving from one coordinate to the other
    #This is very dynamic and allows for any cost value of each grid cell
    def _cost(self, a, b):
        x, y = b
        cell = self.grid[x][y]
        if cell == '#':  # Obstacle
            return float('inf')  # Can't traverse
        if isinstance(cell, (int, float)):  
            return cell
        else:
            return 1

    def _update(self, s):
        #This is used to prevent duplicate additions to the open list so we remove any existing entry of s
        self.open = [(k, v) for k, v in self.open if v != s]
        heapq.heapify(self.open)
        #We update the rhs based on the cost and our current g value for this cell
        #We check each neighbor for a more preferred path  
        if s != self.goal:
            options = [self._cost(s, n) + self.g[n] for n in self._neighbors(*s)]
            self.rhs[s] = min(options) if options else float('inf')
        #If g and rhs are different we put s back into the open list with an updated priority using key
        if self.g[s] != self.rhs[s]:
            heapq.heappush(self.open, (self._key(s), s))
    #This function will ensure that g[start] == rhs[start] meaning that we have an ideal path for the enviroment
    def compute_path(self):
        self.closed.clear()
        #This loop condition runs until the top of the open list has a lower priority key than the starts key
        #Or while the start node is still inconsistent in rhs and g values
        while self.open and (
            self.open[0][0] < self._key(self.start) or
            self.rhs[self.start] != self.g[self.start]
        ):
            #Take the position value not the priority of the cell with the best priority and add it to the visited list
            _, u = heapq.heappop(self.open)
            self.closed.add(u)
            #If our g value is overestimating we update with our rhs
            if self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                #We update neighbors with this new given information
                for n in self._neighbors(*u):
                    self._update(n)
            #If the cell was underconsistent we reset the g of this cell to infinity and recalculate rhs
            else:
                self.g[u] = float('inf')
                self._update(u)
                #We then update the neighbors of u with this information
                for n in self._neighbors(*u):
                    self._update(n)
    #This is where we add obstacles or change the value of a cell and recompute
    def update_grid(self, pos, value):
        x, y = pos
        #We change grid value
        self.grid[x][y] = value
        #Recompute affected nodes including the cell that changed
        for n in list(self._neighbors(x, y)) + [pos]:
            self._update(n)
        #Now we recompute path with our update instead of completely recalculating
        self.compute_path()
    #This function creates our path output
    def get_path(self):
        path = []
        curr = self.start
        #If our g[] at the start is infinite it means we have no path
        if self.g[curr] == float('inf'):
            return [] 
        #While we are not at goal go forward
        while curr != self.goal:
            #We append the current value
            path.append(curr)
            neighbors = list(self._neighbors(*curr))
            #If we have no neighbors (and we arent at goal) we reached a deadend and have no path 
            if not neighbors:
                return []
            #Now we update our current to the minimum of our neighbors given their g values
            curr = min(neighbors, key=lambda n: self.g[n])
        #We append the goal
        path.append(self.goal)
        return path

#Utility to find start and goal
def locate(grid):
    start = goal = None
    #Looks through the graph to set the goal coordinates, easier to set up the grid in this way
    for i, row in enumerate(grid):
        for j, val in enumerate(row):
            if val == 'S':
                start = (i, j)
            elif val == 'G':
                goal = (i, j)
    return start, goal

# Test Cases
if __name__ == "__main__":
    grid = [
        ['S', 1, 1, '#'],
        [1, '#', 1,  1],
        [1,  1,  1, 'G']
    ]

    start, goal = locate(grid)
    planner = DynamicAStar(grid, start, goal)
    print("Initial path:")
    print(planner.get_path())

    print("\nUpdating: changing cost at (2, 0) from 1 to 10 Total cost in this path: 15")
    planner.update_grid((2, 0), 10)
    print(planner.get_path())

    print("\nUpdating: changing cost at (0, 2) from 1 to 20 Total cost in this path: 25")
    planner.update_grid((0, 2), 20)
    print(planner.get_path())

    print("\nUpdating: adding wall at (1, 0)")
    planner.update_grid((1, 0), '#')
    print(planner.get_path())

    print("\nUpdating: adding wall at (0, 1)")
    planner.update_grid((0, 1), '#')
    print(planner.get_path())
