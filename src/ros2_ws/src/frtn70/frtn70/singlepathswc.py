# Needed to use ros and create nodes
from copy import deepcopy
import random
import rclpy
import rclpy.duration
from rclpy.node import Node

# Fullstate control
from crazyflie_interfaces.msg import FullState
from crazyflie_interfaces.srv import Takeoff
import rowan

# For initial delay in startup
import time

# Needed for calculations
import numpy as np
import math

# Import our Crazyflie
from Crazyflie import Crazyflie

# Import our graphics handler
from GraphicsHandler import GraphicsHandler
from Astar import Astar

from std_msgs.msg import Float64MultiArray

class Operation:
    def __init__(self, name, goal=[0.0, 0.0, 0.0], force=[0.0, 0.0, 0.0], type="Goal", countTo=0):
        self.name = name
        self.force = force
        self.goal = np.array(goal)
        self.inProgress = False
        self.completed = False
        self.waiting = False
        # For waiting until time has passed
        self.counter = 0
        self.countTo = countTo
        self.type = type


class Obstacle:
    def __init__(self, id, size=[0.5, 0.5, 0.5], location=[0.0, 0.0, 0.0]):
        self.id = id
        self.size = size
        self.location = location


# Contains code for controlling the group as a whole, and specific manuvers
# Is a node
class Controller(Node):
    def __init__(self):
        # Run constructor
        super().__init__('Swarm_controller')

        # Try to perform operations at 10 Hz
        self.operation_interval = 0.1

        # How far away from the goal we can be to be considered "there"
        self.goal_tolerance = 0.1

        # Initial goal
        self.goal = [0.0, 0.0, 0.0]

        # The height at which to consider us landed
        self.landing_height = 0.05

        # How far apart are the drones allowed to be
        self.safety_distance = 0.1

        # How big our area is
        self.bounding_box_size = [4.0/2, 3.0/2, 2.0]

        # How close to the edge of the area can the drone be
        self.edge_distance = 0.2

        # Tuning parameters for force model
        self.wcoh = 1.5#0.5
        self.walign = 0.2#0.02
        self.wsep = 0.1#0.05
        self.wgoal = 1.0
        self.boidDistance = 3.8 #How far apart the drones can be to be affected by boidForces
        self.maxForce = 0.1 #* operation_interval # How fast do we allow the drones to move per cycle?

        # What we should call to get the drones to takeoff
        self.takeoffHeight = 1.0
        self.takeoffRequest = Takeoff.Request()
        self.takeoffRequest.duration = rclpy.duration.Duration(seconds=4).to_msg()
        self.takeoffRequest.height = self.takeoffHeight

        # Get drone names, need to wait for them to show up        
        self._drones = set()
        while not len(self._drones):
            # Very convoluted, but essentialy makes a list of all topics
            # that contain 'cf' (as in crazyflie, specified in crazyflies.yaml)
            # and extracts the drone name from the topic
            # where the topic is '/cfx/xxx', and takes the unique drone names from that list
            time.sleep(1.0)
            self._drones = set([e[0].split('/')[1] 
                for e in self.get_topic_names_and_types() 
                if e[0].startswith('/cf')])
        
        self._crazyflies = {}

        print("CREATING LISTENERS")
        self.createDrones()
        self.graphics = GraphicsHandler(self)
        self.graphics.createMarkerPublishers()
        self.graphics.displayBoundingBox()

        self.obstacles = [
            Obstacle(0, size=[0.05, 0.05, 0.05], location=[2.0, 1.0, 1.5]),
            Obstacle(1, size=[0.05, 0.05, 0.05], location=[-1.5, -1.0, 1.0]),
        ]

        self.graphics.displayObstacles()
        
        self.simpleOpList = [
            Operation("Takeoff",        type="Takeoff", force=[0.0, 0.0, 1.0]),
            Operation("Wait 10 s",       type="Delay",   countTo=10.0/self.operation_interval),
            Operation("Land",           type="Land"),
            ]
        

        self.circle_op_start = [
            Operation("Takeoff",        type="Takeoff", force=[0.0, 0.0, 1.0]),
            Operation("Wait 1 s",       type="Delay",   countTo=1.0/self.operation_interval),            
            ]

        self.operations = self.circle_op_start
        self.operationsGenerated = False

        # We have now created our operation - so call the method for rendering them (pathplanner should do this after every updated path)
        #self.graphics.displayWaypoints()
        self.current_op_index = 0

        # Variables used to decrease computation time
        self.distances = None
        self.positions = None
        self.avgPos = None

        # Call performOperations function every operation_interval seconds
        self.operation_timer = self.create_timer(self.operation_interval, self.performPathOp)

        # Print debug info every 0.5 seconds, 2Hz
        self.debug_print_timer = self.create_timer(0.5, self.debugPrint)

        # Draw avgPoint marker every 0.05 seconds, 20Hz
        self.marker_timer = self.create_timer(0.05, self.graphics.displayAvgPoint)

        # Run safety checks at 20 Hz (as often as we get new measurements)
        self.safety_timer = self.create_timer(0.05, self.checkSafety)

        # Publisher for distance data for tuning boid forces
        self.dist_publisher = self.create_publisher(Float64MultiArray, "/distances", 10)


    def debugPrint(self):
        bf = self.boidforce()
        for cf in self._crazyflies.values():
            print("Crazyflie: " + cf._drone,
                "X: ", "{:.8f}".format(cf.position[0])[0:6],
                "Y: ", "{:.8f}".format(cf.position[1])[0:6],
                "Z: ", "{:.8f}".format(cf.position[2])[0:6],
                "X-rot: ", "{:.4f}".format(cf.rotation[0])[0:4],
                "Y-rot: ", "{:.4f}".format(cf.rotation[1])[0:4],
                "Z-rot: ", "{:.4f}".format(cf.rotation[2])[0:4],
                "W-rot: ", "{:.4f}".format(cf.rotation[3])[0:min(4, len(str(cf.rotation[3])))],
                "Velocity: ", cf.velocity)
            #print("Boid: ", bf[cf._drone])
        print("Average position: ", self.getAvgPosition(self.positions))
        


    def createDrones(self):
        for cf_name in self._drones:
            print("Created drone " + cf_name)
            # Create crazyflie node, with a goal tolerance of goal_tolerance
            self._crazyflies[cf_name] = Crazyflie(cf_name, self.goal_tolerance, self.takeoffHeight)

    
    def shutdown(self):
        for cf in self._crazyflies.values():
            cf.shutdown()
        self.destroy_node()
        exit()


    def getPositions(self):
        # Filter outliars
        valid_positions = {}
        for (cf_name, cf) in self._crazyflies.items():
            pos = cf.position
            # Filter?
            valid_positions[cf_name] = np.array(pos)

        return valid_positions
    

    def getDistances(self):
        pos = self.positions
        distances = {}
        for n in self._crazyflies.keys():
            for n2 in self._crazyflies.keys():
                if n == n2 or (n, n2) in distances.keys() or (n2, n) in distances.keys():
                    continue
                distances[(n, n2)] = np.linalg.norm(pos[n2] - pos[n])
                distances[(n2, n)] = np.linalg.norm(pos[n] - pos[n2])
        return distances


    def getAvgPosition(self, positions):
        # Create an average of all the points in the list positions, or send origo
        if len(positions) > 0:
            return np.array(list(positions.values())).mean(axis=0)
        else:
            return [0, 0, 0]


    def allReady(self):
        return all([cf.ready for cf in self._crazyflies.values()])
    

    def generateOperations(self):
        #goal = [-1.5, 1.0, 1.0]
        goal = self.avgPos + np.array([0.5, 0.5, 0.5])
        goal2 = goal + np.array([-1.5, -1.5, -1.0])
        for i in range(0,3):
            while goal[i] > (self.bounding_box_size[i] - 0.25):
                goal[i] -= 0.1
            while goal[i] < (-self.bounding_box_size[i] + 0.25):
                goal[i] += 0.1
            
        for i in range(0,3):
            while goal2[i] > (self.bounding_box_size[i] - 0.25):
                goal2[i] -= 0.1
            while goal2[i] < (-self.bounding_box_size[i] + 0.25):
                goal2[i] += 0.1
        astar = Astar()
        startPos = self.getAvgPosition(self.positions)
        path = astar.astar(tuple([int(c * 100) for c in startPos]), tuple([int(c * 100) for c in goal]), [])
        #Debug
        if path:
            print("Path found:")
            for step, position in enumerate(path):
                print(f"Step {step + 1}: Move to {position}")
        else:
            print("No path found.")

        for move in path:
            self.operations.append(Operation("Move", type="Goal", goal=[float(c / 100) for c in move]))
        print('length of operations is: ', len(self.operations))
        path = astar.astar(tuple([int(c * 100) for c in goal]), tuple([int(c * 100) for c in goal2]), [])
        for move in path:
            self.operations.append(Operation("Move", type="Goal", goal=[float(c / 100) for c in move]))
        self.operations.append(Operation("Land",           type="Land"))

        min_lenght = len(self.operations) / 2
        while len(self.operations) > min_lenght:
            toRemove = random.sample(self.operations[3:-3], 1)[0]
            self.operations.remove(toRemove)
        
        self.graphics.displayWaypoints()

    
    def publishDistances(self):
        message = Float64MultiArray()
        visited = []
        for c1 in self._crazyflies:
            for c2 in self._crazyflies:
                if c1 == c2 or (c1, c2) in visited:
                    continue
                visited.append((c1, c2))
                visited.append((c2, c1))
                message.data.append(self.distances[(c1, c2)])
        self.dist_publisher.publish(message)


    def performPathOp(self):
        self.positions = self.getPositions()
        self.distances = self.getDistances()
        self.avgPos = self.getAvgPosition(self.positions)
        self.publishDistances()
        if not self.allReady():
            return
        
        boidForces = self.boidforce()
        
        # If any drone has not yet taken off, do so
        op = self.operations[self.current_op_index]
        if op.type == "Takeoff":
            if not op.inProgress:
                print("Executing op " + op.name)
                #self.moveAll(op.force)
                #for cf in self._crazyflies.values():
                #    cf._goal = list(np.array(cf.position) + np.array([0.0, 0.0, 1.0]))
                c = self.create_client(Takeoff, "/all/takeoff")
                
                c.call_async(self.takeoffRequest)
                op.inProgress = True


            elif all([cf.taken_off for cf in self._crazyflies.values()]):
                print("All have taken off")
                self.current_op_index += 1
                op.completed = True
                if not self.operationsGenerated:
                    self.generateOperations()
                    self.graphics.displayWaypoints()
                    self.operationsGenerated = True
        
        elif op.type == "Goal":
            if not op.inProgress:
                print("Executing op " + op.name)
                op.inProgress = True
                self.graphics.displayOp(op)
            else:
                
                bfs = {}
                for (name, cf) in self._crazyflies.items():
                    bfs[name] = np.array(boidForces[name])

                gf = np.array(self.getForceAvgToGoal(op.goal, bfs))
                for (name, cf) in self._crazyflies.items():
                    tf = gf + bfs[name]
                    #print("Drone " + name + " has tf " + str(tf))
                    self.applyForce(cf, self.limitForce(tf))
                    # Check if we are at our goal, whilst considering boidforces as well
                    #print("Drone " + name + " has pos " + str(self.positions[name]) + " and goal " + str(op.goal) + " dist is " + str(np.linalg.norm((op.goal + bf) - self.positions[name])))
                if self.avgAtGoal(op.goal):
                    op.completed = True
                    self.current_op_index += 1
        
        elif op.type == "Delay":
            if not op.inProgress:
                print("Executing op " + op.name)
                op.inProgress = True
            else:
                op.counter += 1
                # Check if we are done counting
                if op.counter < op.countTo:
                    return
                op.completed = True
                self.current_op_index += 1

        elif op.type == "Land" and not op.inProgress:
            print("Executing op " + op.name)
            self.wgoal = 1000.0
            for cf in self._crazyflies.values():
                origoForce = self.getForceToGoal(cf, [0.0, 0.0, self.landing_height])
                self.applyForce(cf, [0.0, 0.0, origoForce[2]])
            op.inProgress = True
                    
    
    def getForceToGoal(self, cf, goal):
        return (goal - self.positions[cf._drone]) * self.wgoal
    

    def getForceAvgToGoal(self, goal, boidforces):
        newPos = {}
        for name in self._crazyflies.keys():
            newPos[name] = self.positions[name] + boidforces[name]
        newAvg = self.getAvgPosition(newPos)
        return (goal - newAvg) * self.wgoal

    
    def droneAtGoal(self, cf, goal):
        return np.linalg.norm(goal - self.positions[cf._drone]) < self.goal_tolerance
    

    def avgAtGoal(self, goal):
        return np.linalg.norm(goal - self.avgPos) < self.goal_tolerance
    
    
    def limitForce(self, force):
        norm = np.linalg.norm(force)
        if not norm:
            return [0.0, 0.0, 0.0]
        normForce = force/norm
        return min(norm, self.maxForce) * normForce
            
    
    #TODO Generate paths dynamically
    #TODO Extend safety system with drone distance checks - if too close, apply fsep on all drones and land
    #TODO Add obstacleForce = sum(1/(pos - obstaclePos)) for O, O=Obstacles within distance - maybe not. Pathplanning should handle.

    def boidforce(self):
        bforce = {}

        neighbours = {}
        for n in self._crazyflies.keys():
            neighbours[n] = []
        
        for n in self._crazyflies.keys():
            for n2 in self._crazyflies.keys():
                if n == n2:
                    continue
                #if self.distances[(n, n2)] < self.boidDistance:
                #    neighbours[n].append(n2)
                neighbours[n].append(n2)

        # Loop through all crazyflies and calculate the boidforces for them
        for (name, cf) in self._crazyflies.items():
            if len(neighbours[name]) == 0:
                bforce[name] = 0.0
                continue
           # 1/|N|
            nweight = 1/(len(neighbours[name]))
           # Fsep
            fsep = -sum([(np.array(cf2.position) - np.array(cf.position)) /
                        pow(np.linalg.norm(
                            np.array(cf2.position) - np.array(cf.position)), 2) 
                            for cf2 in self._crazyflies.values() if cf2 != cf])
            #fsep = -sum([self.positions[n2] - self.positions[name]
            #            / pow(np.linalg.norm(
            #                self.positions[n2] - self.positions[name]), 2) 
            #                for n2 in neighbours[name]])
            #print([np.sqrt(np.dot(self.positions[n2] - self.positions[name], self.positions[n2] - self.positions[name])) for n2 in neighbours[name]])
            #                for n2 in neighbours[name]])
            valign = nweight * sum([np.array(self._crazyflies[n2].velocity) - np.array(cf.velocity) 
                        for n2 in neighbours[name]])
    
            pcoh = nweight * sum([self.positions[n2] - self.positions[name]
                        for n2 in neighbours[name]])
            
            #for d in [self.distances[(name, n2)] for n2 in neighbours[name]]:
            #    print("Drone " + name + " has distance " + str(d) )
            #print("Drone " + name + " has pcoh " + str(pcoh*self.wcoh) + " valign " + str(valign*self.walign) + " fsep " + str(fsep*self.wsep))
    
            bforce[name] = self.wsep * fsep + self.walign * valign + self.wcoh * pcoh
            #print("Drone " + name + " has boidforce " + str(bforce[name]))
        return bforce
    

    def applyForce(self, cf, force, rotation=0):
        startPoint = cf.position

        # Do not move too close to the edge
        for i in range(0,3):
            if startPoint[i] > (self.bounding_box_size[i] - self.edge_distance) and force[i] > 0:
                print("Not allowing drone " + cf._drone + " to move closer to the edge, as it is closer than our margins")
                force[i] = 0
            elif startPoint[i] < (-self.bounding_box_size[i] + self.edge_distance) and force[i] < 0 and i != 2:
                force[i] = 0
                print("Not allowing drone " + cf._drone + " to move closer to the edge, as it is closer than our margins")
                
                
        goal = np.array(startPoint) + force
        self.graphics.displayForce(int(cf._drone[-1]), startPoint, goal)
        #print("Drone " + cf._drone + " is getting force " + str(force))

        cf.setGoal(goal)
        msg = cf.getNewStateMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = goal[0] #startPoint[0] + force[0]
        msg.pose.position.y = goal[1] #startPoint[1] + force[1]
        msg.pose.position.z = goal[2] #startPoint[2] + force[2]
        q = rowan.from_euler(0, 0, rotation)
        msg.pose.orientation.w = q[0]
        msg.pose.orientation.x = q[1]
        msg.pose.orientation.y = q[2]
        msg.pose.orientation.z = q[3]
            
        # Send message
        cf.stateMsg = msg


    def moveAll(self, force, rotation=0):
        for (name, cf) in self._crazyflies.items():
            startPoint = self.getPositions()[name]
            goal = np.array(startPoint) + np.array(force)

            cf.setGoal(goal)
            msg = cf.getNewStateMsg()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x = goal[0] #startPoint[0] + force[0]
            msg.pose.position.y = goal[1] #startPoint[1] + force[1]
            msg.pose.position.z = goal[2] #startPoint[2] + force[2]
            q = rowan.from_euler(0, 0, rotation)
            msg.pose.orientation.w = q[0]
            msg.pose.orientation.x = q[1]
            msg.pose.orientation.y = q[2]
            msg.pose.orientation.z = q[3]
            
            # Send message
            cf.stateMsg = msg


    # Runs faster than everything else, and shuts down if something is not within the bounding box
    def checkSafety(self):
        positions = self.positions
        dist = self.distances
        if not positions or not dist:
            return
        for (name, pos) in positions.items():
            if self._crazyflies[name].battery_warn:
                self.emergencyLand(name, "HAS LOW BATTERY")
            
            for (n2, p2) in positions.items():
                if p2[2] == -1000 or pos[2] == -1000:
                    break
                if (name, n2) in dist.keys() and dist[(name, n2)] < self.safety_distance:
                    self.emergencyLand(name, "IS TOO CLOSE TO DRONE " + n2 + str(p2) + str(pos))

            for i in range(3):
                if abs(pos[i]) > self.bounding_box_size[i] and self._crazyflies[name].ready:
                    self.emergencyLand(name, "IS OUTSIDE OF THE BOUNDING BOX " + str(pos))


    def emergencyLand(self, drone_name, message):
        self.operation_timer.destroy()
        print("EMERGENCY, DRONE " + drone_name + " " + message + "! LANDING!!")
        self.moveAll([0.0, 0.0, - self._crazyflies[drone_name].position[2] + self.landing_height])
        time.sleep(5.0)
        self.shutdown()


        

def main(args=None):
    rclpy.init(args=args)
    try:
        controller = Controller()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    controller.shutdown()
    #rclpy.shutdown()
    exit()


if __name__ == '__main__':
    main()

