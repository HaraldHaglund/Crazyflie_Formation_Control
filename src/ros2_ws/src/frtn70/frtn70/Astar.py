import heapq
import math

import numpy as np
#import cProfile

class Node:
    def __init__(self, state, parent=None, action=None, g=0, h=0):
        self.state = state # Current position 
        self.parent = parent # Parent node
        self.action = action # Move made from parent->this node (look inside "moves")
        self.g = g # Cost
        self.h = h # Heuristic

    def f(self): # Total cost
        return self.g + self.h 

class Astar:
    def astar(self, initial_state, goal_state, obstacles: set):
        print('Initial state is ', initial_state)
        print('Goal state is ', goal_state)
        print("OBSTACLE SIZE", len(obstacles))
        # OPEN //the set of nodes to be evaluated
        open_list = []  # containing f, id, node
        heapq.heapify(open_list)  # Convert list to heap
        # CLOSED //the set of nodes already evaluated
        closed_set = set()
        open_set = set()

        # add the start node to OPEN
        start_node = Node(initial_state)
        start_node.h = self.heuristic(initial_state, goal_state)
        heapq.heappush(open_list, (start_node.f(), id(start_node), start_node))
        open_set.add(start_node.state)
        # loop
        while open_set:
            # current = node in OPEN with the lowest f_cost
            _, _, current_node = heapq.heappop(open_list)          # remove current from OPEN
            open_set.remove(current_node.state)
            # add current to CLOSED
            closed_set.add(current_node.state)

            # if current is the target node //path has been found
            if current_node.state == goal_state:
                # return
                path = []
                while current_node:
                    path.append(current_node.state)
                    current_node = current_node.parent
                return path[::-1] #reverse the list

            # foreach neighbour of the current node. where neighbours contains [action, state, g]
            i = 0
            #print('CURRENT NODE: ' , current_node.state)
            for neighbour_node in self.neighbours(current_node.state, obstacles):
                i +=1
                #print('Neighbour node number ', i, ' state: ', neighbour_node.state)
                #print('Neighbour node number ', i , 'in closed set? ', neighbour_node.state in closed_set)
                if (neighbour_node.state in closed_set) or (neighbour_node.state in obstacles):
                    # skip to the next neighbour
                    continue
                
                new_g = current_node.g + self.heuristic(current_node.state,neighbour_node.state) 

                # Check if the neighbor is not in the closed set or if the new path to the neighbor is shorter
                if new_g < neighbour_node.g or neighbour_node.state not in open_set:
                    neighbour_node.g = new_g 
                    neighbour_node.h = self.heuristic(neighbour_node.state, goal_state)
                    neighbour_node.parent = current_node
                    # If the neighbor is not in the open set
                    if neighbour_node.state not in open_set:
                        # Add the neighbor to the open list
                        heapq.heappush(open_list, (neighbour_node.f(), id(neighbour_node), neighbour_node))
                        open_set.add(neighbour_node.state)
        # No path found
        return None

    def heuristic(self, state, goal_state, weight=1): # 0 -> euclidian, 1 -> manhattan
        if weight:
            #Manhattan
            return abs(state[0] - goal_state[0]) + abs(state[1] - goal_state[1]) + abs(state[2] - goal_state[2])
        else:
        #euclidean_dist = math.sqrt((state[0] - goal_state[0])**2 + (state[1] - goal_state[1])**2 + (state[2] - goal_state[2])**2)
            return np.linalg.norm(np.array(state) - np.array(goal_state))

    #Retrives the neighbours of a current state
    def neighbours(self, state, obstacles):
        x, y, z = state
        neighbors = []  # Initialize an empty list to store the generated neighbors

        # Define possible changes in coordinates
        moves = [(-1, 0, 0), (1, 0, 0), (0, -1, 0), (0, 1, 0), (0, 0, -1), (0, 0, 1)]
                #(-1, -1, 0), (-1, 1, 0), (1, -1, 0), (1, 1, 0), (-1, 0, -1), (-1, 0, 1),
                #(1, 0, -1), (1, 0, 1), (0, -1, -1), (0, -1, 1), (0, 1, -1), (0, 1, 1)]

        for dx, dy, dz in moves:
            new_x, new_y, new_z = x + dx, y + dy, z + dz  # Calculate the new coordinates
            new_state = (new_x, new_y, new_z)

            # Check if the new state is valid (not obstructed)
            if new_state not in obstacles:
                action_description = f"Move ({dx}, {dy}, {dz})"  # Create the action description string
                g = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)  # Calculate the step cost
                neighbors.append(Node(new_state, action=action_description, g=g))
        return neighbors


#def profile_function():
#    astar_solver = Astar()
#    #initial_state = (16, 83, 140)
#    initial_state =  (0, 0, 0)
#    #goal_state = (-133, -66, 40)
#    goal_state = (149, 149, 149)
#    obstacles = {}  # Example obstacle positions
#    path = astar_solver.astar(initial_state, goal_state, obstacles)
#    print("Path:", path)

#Profile the function call
#cProfile.run('profile_function()')

