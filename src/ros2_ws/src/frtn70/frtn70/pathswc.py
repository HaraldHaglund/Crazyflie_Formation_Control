# Needed to use ros and create nodes
from copy import deepcopy
import rclpy
import rclpy.duration
from rclpy.node import Node

# Fullstate control
from crazyflie_interfaces.msg import FullState
import rowan

# For initial delay in startup
import time

# Needed for calculations
import numpy as np
import math

# Import our Crazyflie
from Crazyflie import Crazyflie

# Import our graphics handler
from MultiPathGraphicsHandler import GraphicsHandler

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

        # Try to perform operations at 100 Hz
        operation_interval = 0.01

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

        # Tuning parameters for force model
        self.wcoh = 0.2#0.5
        self.walign = 0.1#0.02
        self.wsep = 0.3#0.05
        self.wgoal = 1.0
        self.boidDistance = 0.8 #How far apart the drones can be to be affected by boidForces
        self.maxForce = 0.1 #* operation_interval # How fast do we allow the drones to move per cycle?

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
            Operation("Wait 10 s",       type="Delay",   countTo=10.0/operation_interval),
            Operation("Land",           type="Land"),
            ]
        

        self.circle_op_start = [
            Operation("Takeoff",        type="Takeoff", force=[0.0, 0.0, 1.0]),
            Operation("Wait 1 s",       type="Delay",   countTo=1.0/operation_interval),            
            ]

        self.operations = {}
        for (num, name) in enumerate(self._drones):
            #self.operations[name] = deepcopy(self.simpleOpList)
            self.operations[name] = deepcopy(self.circle_op_start)
            for i in range(180, -181, -12):
                v = math.radians(i)
                self.operations[name].append(Operation("Move", type="Goal", goal=[math.cos(v)*0.5 + 0.5*num, math.sin(v)*0.5 - 0.5*num, 1.0 + math.sin(v) / 4]))
            self.operations[name].append(Operation("Wait 10 s", type="Delay", countTo=10.0/operation_interval))
            self.operations[name].append(Operation("Land", type="Land"))

        # We have now created our operation - so call the method for rendering them (pathplanner should do this after every updated path)
        self.graphics.displayWaypoints()

        # Variables used to decrease computation time
        self.distances = None
        self.positions = None

        # Call performOperations function every operation_interval seconds
        self.operation_timer = self.create_timer(operation_interval, self.performPathOp)

        # Print debug info every 0.5 seconds, 2Hz
        #self.debug_print_timer = self.create_timer(0.5, self.debugPrint)

        # Draw avgPoint marker every 0.01 seconds, 100Hz
        self.marker_timer = self.create_timer(0.01, self.graphics.displayAvgPoint)

        # Run safety checks at double operation_interval seconds
        self.safety_timer = self.create_timer(operation_interval / 2, self.checkSafety)


    def debugPrint(self):
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
        print("Average position: ", self.getAvgPosition(list(self.getPositions().values())))


    def createDrones(self):
        for cf_name in self._drones:
            print("Created drone " + cf_name)
            # Create crazyflie node, with a goal tolerance of goal_tolerance
            self._crazyflies[cf_name] = Crazyflie(cf_name, self.goal_tolerance)

    
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
            return list(np.array(positions).mean(axis=0))
        else:
            return [0, 0, 0]


    def allReady(self):
        return all([cf.ready for cf in self._crazyflies.values()])


    def performPathOp(self):
        self.positions = self.getPositions()
        self.distances = self.getDistances()
        if not self.allReady():
            return
        
        waitForTakeoff = any([cf.current_op_index == 0 for cf in self._crazyflies.values()])
        boidForces = self.boidforce()
        
        # If any drone has not yet taken off, do so
        for (name, cf) in self._crazyflies.items():
            op = self.operations[name][cf.current_op_index]
            if  op.type == "Takeoff":
                if not op.inProgress:
                    print("Executing op " + op.name + " on " + name)
                    self.applyForce(cf, op.force)
                    op.inProgress = True
                elif cf.taken_off:
                    op.completed = True
                    cf.current_op_index += 1
            elif waitForTakeoff:
                if not op.waiting:
                    print("Drone " + name + " is waiting for others to takeoff")
                    op.waiting = True
                # Do NOT do anything else since we are waiting for takeoff still

            elif any([cf2.current_op_index < cf.current_op_index for cf2 in self._crazyflies.values()]):
                if not op.waiting:
                    print("Drone " + name + " is waiting for others to come to the same step")
                    op.waiting = True
            
            elif op.type == "Goal":
                if not op.inProgress:
                    print("Executing op " + op.name + " on " + name)
                    op.inProgress = True
                else:
                    gf = self.getForceToGoal(cf, op.goal)
                    #print("Drone " + name + " has gf " + str(gf))
                    bf = boidForces[name]
                    tf = self.limitForce(np.array(gf) + np.array(bf))
                    #print("Drone " + name + " has tf " + str(tf))
                    self.applyForce(cf, tf)
                    # Check if we are at our goal, whilst considering boidforces as well
                    if self.droneAtGoal(cf, op.goal + boidForces[name]):
                        op.completed = True
                        cf.current_op_index += 1
            
            elif op.type == "Delay":
                if not op.inProgress:
                    print("Executing op " + op.name + " on " + name)
                    op.inProgress = True
                else:
                    op.counter += 1
                    # Check if we are done counting
                    if op.counter < op.countTo:
                        return
                    op.completed = True
                    cf.current_op_index += 1

            elif op.type == "Land" and not op.inProgress:
                print("Executing op " + op.name + " on " + name)
                origoForce = self.getForceToGoal(cf, [0.0, 0.0, self.landing_height])
                self.applyForce(cf, [0.0, 0.0, origoForce[2]])
                op.inProgress = True
                    
    
    def getForceToGoal(self, cf, goal):
        return (goal - self.positions[cf._drone]) * self.wgoal

    
    def droneAtGoal(self, cf, goal):
        return np.linalg.norm(goal - self.positions[cf._drone]) < self.goal_tolerance
    
    
    def limitForce(self, force):
        #return force
        norm = np.linalg.norm(force)
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
                if self.distances[(n, n2)] < self.boidDistance:
                    neighbours[n].append(n2)

        # Loop through all crazyflies and calculate the boidforces for them
        for (name, cf) in self._crazyflies.items():
            if len(neighbours[name]) == 0:
                bforce[name] = 0.0
                continue
           # 1/|N|
            nweight = 1/(len(neighbours[name]))
           # Fsep
            fsep = -sum([self.distances[(name, n2)]
                        / pow(np.linalg.norm(
                            self.positions[n2] - self.positions[name]), 2) 
                            for n2 in neighbours[name]])

            valign = nweight * sum([np.array(self._crazyflies[n2].velocity) - np.array(cf.velocity) 
                        for n2 in neighbours[name]])
    
            pcoh = nweight * sum([self.distances[(name, n2)] 
                        for n2 in neighbours[name]])
    
            bforce[name] = self.wsep * fsep + self.walign * valign + self.wcoh * pcoh
            #print("Drone " + name + " has boidforce " + str(bforce[name]))
        return bforce
    

    def applyForce(self, cf, force, rotation=0):
        startPoint = cf.position
        goal = np.array(startPoint) + force
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
                    self.emergencyLand(name, "IS OUTSIDE OF THE BOUNDING BOX")


    def emergencyLand(self, drone_name, message):
        self.operation_timer.destroy()
        print("EMERGENCY, DRONE " + drone_name + " " + message + "! LANDING!!")
        self.moveAll([0.0, 0.0, - self._crazyflies[drone_name].position[2] + self.landing_height])
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

