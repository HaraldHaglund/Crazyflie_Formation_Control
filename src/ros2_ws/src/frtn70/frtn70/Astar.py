import heapq  # Importing the heapq module for priority queue implementation
import math   # Importing the math module for square root calculation
import numpy as np

class Node:
    def __init__(self, state, parent=None, action=None, cost=0, heuristic=0):
        self.state = state  # Current state (3D coordinates)
        self.parent = parent  # Parent node
        self.action = action  # Action taken from parent to reach current state
        self.cost = cost  # Cost to reach current state
        self.heuristic = heuristic  # Heuristic value

    def total_cost(self):
        return self.cost + self.heuristic  # Calculate total cost of the node
    
class Astar:
    def astar(self, initial_state, goal_state, obstacles):
        open_list = []  # Priority queue for open nodes
        closed_set = set()  # Set of visited nodes

        start_node = Node(initial_state)  # Initialize start node
        start_node.heuristic = self.manhattan_distance(initial_state, goal_state)  # Calculate heuristic for start node
        heapq.heappush(open_list, (start_node.total_cost(), id(start_node), start_node))  # Push start node to open list

        while open_list:
            _, _, current_node = heapq.heappop(open_list)  # Pop node with lowest total cost from open list
            if current_node.state == goal_state:
                path = []  # Reconstruct and store the path
                while current_node:
                    path.append(current_node.state)
                    current_node = current_node.parent
                return path[::-1]  # Reverse the path from start to goal

            closed_set.add(current_node.state)  # Add current node to closed set

            for action, successor_state, step_cost in self.successors(current_node.state, obstacles):
                if successor_state in closed_set:
                    continue  # Skip if successor state is already visited

                new_cost = current_node.cost + step_cost  # Calculate new cost to reach successor node
                new_node = Node(successor_state, current_node, action, new_cost)  # Create new node for successor state
                new_node.heuristic = self.manhattan_distance(successor_state, goal_state)  # Calculate heuristic for new node

                heapq.heappush(open_list, (new_node.total_cost(), id(new_node), new_node))  # Push new node to open list
        return None  # No path found

    def manhattan_distance(self, state, goal_state):
        #return np.sum(np.abs(np.array(state)) - np.abs(np.array(goal_state)))
        return abs(state[0] - goal_state[0]) + abs(state[1] - goal_state[1]) + abs(state[2] - goal_state[2])  # Manhattan distance heuristic

    def successors(self, state, obstacles):
        x, y, z = state  # Extract coordinates from the state
        moves = [(-1, 0, 0), (1, 0, 0), (0, -1, 0), (0, 1, 0), (0, 0, -1), (0, 0, 1)]  # Possible moves (6 directions)
        for dx, dy, dz in moves:
            new_x, new_y, new_z = x + dx, y + dy, z + dz  # Calculate new coordinates
            if (new_x, new_y, new_z) not in obstacles:  # Check if the new position is not an obstacle
                yield f"Move ({dx}, {dy}, {dz})", (new_x, new_y, new_z), math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)  # Yield valid move with step cost
