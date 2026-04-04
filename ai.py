from __future__ import print_function
from heapq import *  # Hint: Use heappop and heappush

ACTIONS = [(0, 1), (1, 0), (0, -1), (-1, 0)]


class AI:
    def __init__(self, grid, type):
        self.grid = grid
        self.set_type(type)
        self.set_search()

    def set_type(self, type):
        self.final_cost = 0
        self.type = type

    def set_search(self):
        self.final_cost = 0
        self.grid.reset()
        self.finished = False
        self.failed = False
        self.previous = {}

        # Initialization of algorithms goes here
        if self.type == "dfs":
            self.frontier = [self.grid.start]
            self.explored = set()
        elif self.type == "bfs":
            self.frontier = [self.grid.start]
            self.explored = set()
        elif self.type == "ucs":
            self.frontier = []
            heappush(self.frontier, (0, self.grid.start))
            self.explored = set()
            self.costs = {self.grid.start: 0}
        elif self.type == "astar":
            self.frontier = []
            h_cost = abs(self.grid.start[0] - self.grid.goal[0]) + abs(self.grid.start[1] - self.grid.goal[1])
            heappush(self.frontier, (h_cost, 0, self.grid.start))
            self.explored = set()
            self.costs = {self.grid.start: 0}

    def get_result(self):
        total_cost = 0
        current = self.grid.goal
        while not current == self.grid.start:
            if self.type == "bfs":
                total_cost += 1
            else:
                total_cost += self.grid.nodes[current].cost()
            current = self.previous[current]
            self.grid.nodes[current].color_in_path = (
                True  # This turns the color of the node to red
            )
        total_cost += self.grid.nodes[current].cost()
        self.final_cost = total_cost

    def make_step(self):
        if self.type == "dfs":
            self.dfs_step()
        elif self.type == "bfs":
            self.bfs_step()
        elif self.type == "ucs":
            self.ucs_step()
        elif self.type == "astar":
            self.astar_step()

    # TODO: Buggy DFS, fix it first
    def dfs_step(self):
        if not self.frontier:
            self.failed = True
            self.finished = True
            print("no path")
            return
            
        current = self.frontier.pop()
        
        if current in self.explored:
            return
            
        self.explored.add(current)

        # Finishes search if we've found the goal.
        if current == self.grid.goal:
            self.finished = True
            return

        children = [(current[0] + a[0], current[1] + a[1]) for a in ACTIONS]
        self.grid.nodes[current].color_checked = True
        self.grid.nodes[current].color_frontier = False

        for n in children:
            if n[0] in range(self.grid.row_range) and n[1] in range(self.grid.col_range):
                if not self.grid.nodes[n].puddle:
                    if n not in self.explored:
                        self.previous[n] = current
                        self.frontier.append(n)
                        self.grid.nodes[n].color_frontier = True

    # TODO: Implement BFS here (Don't forget to implement initialization in set_search function)
    def bfs_step(self):
        if not self.frontier:
            self.failed = True
            self.finished = True
            print("no path")
            return
        
        current = self.frontier.pop(0)
        self.explored.add(current)

        if current == self.grid.goal:
            self.finished = True
            return

        children = [(current[0] + a[0], current[1] + a[1]) for a in ACTIONS]
        self.grid.nodes[current].color_checked = True
        self.grid.nodes[current].color_frontier = False

        for n in children:
            if n[0] in range(self.grid.row_range) and n[1] in range(
                self.grid.col_range
            ):
                if not self.grid.nodes[n].puddle:
                    if n not in self.explored and n not in self.frontier:
                        self.previous[n] = current
                        self.frontier.append(n)
                        self.grid.nodes[n].color_frontier = True

    # TODO: Implement UCS here (Don't forget to implement initialization in set_search function)
    # Hint: You can use heappop and heappush from the heapq library (imported for you above)
    def ucs_step(self):
        if not self.frontier:
            self.failed = True
            self.finished = True
            print("no path")
            return
        
        current_cost, current = heappop(self.frontier)
        
        if current in self.explored:
            return
            
        self.explored.add(current)

        if current == self.grid.goal:
            self.finished = True
            return

        children = [(current[0] + a[0], current[1] + a[1]) for a in ACTIONS]
        self.grid.nodes[current].color_checked = True
        self.grid.nodes[current].color_frontier = False

        for n in children:
            if n[0] in range(self.grid.row_range) and n[1] in range(self.grid.col_range):
                if not self.grid.nodes[n].puddle:
                    new_cost = current_cost + self.grid.nodes[n].cost()
                    
                    if n not in self.explored and (n not in self.costs or new_cost < self.costs[n]):
                        self.costs[n] = new_cost
                        self.previous[n] = current
                        heappush(self.frontier, (new_cost, n))
                        self.grid.nodes[n].color_frontier = True

    # TODO: Implement Astar here (Don't forget to implement initialization in set_search function)
    # Hint: You can use heappop and heappush from the heapq library (imported for you above)
    def astar_step(self):
        if not self.frontier:
            self.failed = True
            self.finished = True
            print("no path")
            return
        
        current_f, current_cost, current = heappop(self.frontier)
        
        if current in self.explored:
            return
            
        self.explored.add(current)

        if current == self.grid.goal:
            self.finished = True
            return

        children = [(current[0] + a[0], current[1] + a[1]) for a in ACTIONS]
        self.grid.nodes[current].color_checked = True
        self.grid.nodes[current].color_frontier = False

        for n in children:
            if n[0] in range(self.grid.row_range) and n[1] in range(self.grid.col_range):
                if not self.grid.nodes[n].puddle:
                    new_cost = current_cost + self.grid.nodes[n].cost()
                    
                    if n not in self.explored and (n not in self.costs or new_cost < self.costs[n]):
                        self.costs[n] = new_cost
                        self.previous[n] = current
                        
                        h_cost = abs(n[0] - self.grid.goal[0]) + abs(n[1] - self.grid.goal[1])
                        f_cost = new_cost + h_cost
                        
                        heappush(self.frontier, (f_cost, new_cost, n))
                        self.grid.nodes[n].color_frontier = True
