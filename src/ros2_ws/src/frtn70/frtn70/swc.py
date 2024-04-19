# Needed to use ros and create nodes
import rclpy
import rclpy.duration
from rclpy.node import Node

# Needed to display points
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

# Fullstate control
from crazyflie_interfaces.msg import FullState
import rowan

# For initial delay in startup
import time

# Needed for calculations
import numpy as np
import math

# Import our FrameListener
from FrameListener import FrameListener

# Import our graphics handler
from GraphicsHandler import GraphicsHandler

class Operation:
    def __init__(self, name, goal=[0.0, 0.0, 0.0], force=[0.0, 0.0, 0.0], type="Move", countTo=0):
        self.name = name
        self.force = force
        self.goal = goal
        self.inProgress = False
        self.completed = False
        # For waiting until time has passed
        self.counter = 0
        self.countTo = countTo
        self.type = type


# Contains code for controlling the group as a whole, and specific manuvers
# Is a node
class Controller(Node):
    def __init__(self):
        # Run constructor
        super().__init__('Swarm_controller')

        # How far away from the goal we can be to be considered "there"
        self.goal_tolerance = 0.1

        # How big our area is
        self.bounding_box_size = [4.0/2, 3.0/2, 2.0]

        # Tuning parameters for force model
        self.wcoh = 0.5
        self.walign = 0.02
        self.wsep = 0.05

        # Initial goal
        self.goal = [0.0, 0.0, 0.0]

        # The height at which to consider us landed
        self.landing_height = 0.05

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

        # Try to perform operations at 100 Hz
        operation_interval = 0.01
        
        self.operations = [
            Operation("Takeoff",        type="Takeoff", force=[0.0, 0.0, 1.0]),
            Operation("Wait 10 s",       type="Delay",   countTo=10.0/operation_interval),
            Operation("Move forward",   type="Goal",    goal=[0.0, 0.3, 1.0]),
            Operation("Wait 1 s",       type="Delay",   countTo=1.0/operation_interval),
            Operation("Move up",        type="Goal",    goal=[0.0, 0.3, 1.5]),
            Operation("Wait 1 s",       type="Delay",   countTo=1.0/operation_interval),
            Operation("Move back",      type="Goal",    goal=[0.0, 0.0, 1.5]),
            Operation("Wait 1 s",       type="Delay",   countTo=1.0/operation_interval),
            Operation("Move right",     type="Goal",    goal=[0.3, 0.0, 1.5]),
            Operation("Wait 1 s",       type="Delay",   countTo=1.0/operation_interval),
            Operation("Move left",      type="Goal",    goal=[0.0, 0.0, 1.5]),
            #Operation("Leave bounding box",      type="Goal",    goal=[0.0, 0.0, self.bounding_box_size[2] + 1.0]),
            Operation("Wait 10 s",      type="Delay",   countTo=10.0/operation_interval),
            Operation("Land",           type="Land"),
            ]
        

        self.circle_op = [
            Operation("Takeoff",        type="Takeoff", force=[0.0, 0.0, 1.0]),
            Operation("Wait 1 s",       type="Delay",   countTo=1.0/operation_interval),            
            ]
        
        for i in range(180, -181, -12):
            v = math.radians(i)
            self.circle_op.append(Operation("Move", type="Goal", goal=[math.cos(v)*0.5 + 0.5, math.sin(v)*0.5, 1.0 + math.sin(v) / 4]))

        self.circle_op.append(Operation("Wait 10 s", type="Delay", countTo=10.0/operation_interval))
        self.circle_op.append(Operation("Land", type="Land"))
        
        self.operations = self.circle_op

        # We have now created our operation - so call the method for rendering them (pathplanner should do this after every updated path)
        self.graphics.displayWaypoints()

        # Call performOperations function every operation_interval seconds
        self.operation_timer = self.create_timer(operation_interval, self.performOperations)

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
            self._crazyflies[cf_name] = FrameListener(cf_name, self.goal_tolerance)

    
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
            valid_positions[cf_name] = pos

        return valid_positions


    def getAvgPosition(self, positions):
        # Create an average of all the points in the list positions, or send origo
        if len(positions) > 0:
            return list(np.array(positions).mean(axis=0))
        else:
            return [0, 0, 0]


    def allAtPositions(self):
        atPos = [cf.goal_reached for cf in self._crazyflies.values()]
        return all(atPos)
    
    
    def avgAtGoal(self):
        return np.linalg.norm(
                np.array(self.goal) - 
                np.array(self.getAvgPosition(list(self.getPositions().values())))) < self.goal_tolerance


    def allReady(self):
        return all([cf.ready for cf in self._crazyflies.values()])


    def performOperations(self):
        #print("Trying to perform operation")
        if not (self.allReady() and self.allAtPositions()):
            #print("All drones were not ready, goal:", self.allAtPositions(), " ready: ", self.allReady(), "Dists to goal:", [np.linalg.norm(np.array(cf._goal_pos) - np.array(cf.position)) for cf in self._crazyflies.values()])
            return 

        #Ready for next move
        current_op = -1
        for (num, op) in enumerate(self.operations):
            #print("Trying op ", num)
            if not op.completed:
                current_op = num
                #print("Op found")
                break

        if current_op == -1:
            # Could not find operation
            return
        
        op = self.operations[current_op]
        if not op.inProgress:
            print("Executing op " + op.name)
            # Operation is not in progress, mark it as such and execute it
            if op.type == "Move":
                self.moveAll(op.force)
            elif op.type == "Takeoff":
                self.moveAll(op.force)
            elif op.type == "Land":
                print("Landing")
                self.goToGoal([0.0, 0.0, self.landing_height])
                #self.shutdown()
            elif op.type == "Goal":
                self.goToGoal(op.goal)
                self.goal = op.goal
            op.inProgress = True
        else:                
            if op.type == "Delay":
                op.counter += 1
                # Check if we are done counting
                if op.counter < op.countTo:
                    return
            elif op.type == "Goal":
                if not self.avgAtGoal():
                    # We are not yet at the correct avg point, keep waiting
                    return
            # We are done with operation, mark it as such
            op.completed = True

            
    
    #TODO Generate paths dynamically
    #TODO Extend safety system with battery level
    #TODO Extend safety system with drone distance checks - if too close, apply fsep on all drones and land
    #TODO Add obstacles
    #TODO Add obstacleForce = sum(1/(pos - obstaclePos)) for O, O=Obstacles within distance

    def boidforce(self):
        bforce = {}
        # Loop through all crazyflies and calculate the boidforces for them
        for (name, cf) in self._crazyflies.items():
           # 1/|N|
            nweight = 1/(len(self._crazyflies) - 1)
           # Fsep
            fsep = -sum([(np.array(cf2.position) - np.array(cf.position)) /
                        pow(np.linalg.norm(
                            np.array(cf2.position) - np.array(cf.position)), 2) 
                            for cf2 in self._crazyflies.values() if cf2 != cf])

            valign = nweight * sum([np.array(cf2.velocity) - np.array(cf.velocity) 
                        for cf2 in self._crazyflies.values() if cf2 != cf])
    
            pcoh = nweight * sum([np.array(cf2.position) - np.array(cf.position) 
                        for cf2 in self._crazyflies.values() if cf2 != cf])
    
            bforce[name] = self.wsep * fsep + self.walign * valign + self.wcoh * pcoh
        return bforce
    

    def goalforce(self, goal, forces):
        # Calculate new average position based on boidforces
        newpos = []
        for (name, force) in forces.items():
            newpos.append(np.array(self._crazyflies[name].position) + np.array(force))
        avgpos = self.getAvgPosition(newpos)
        # Return the movement vector to get to the goal
        return (np.array(goal) - np.array(avgpos))
    

    def totalForce(self, goal):
        forces = {}
        # Create dict with boidforces and apply the vector to follow to go to the goal on each
        boidforce = self.boidforce()
        goalforce = self.goalforce(goal, boidforce)
        for (name, cf) in self._crazyflies.items():
            forces[name] = np.array(boidforce[name]) + goalforce
        return forces
    

    def applyForce(self, cf, force, rotation=0):
        startPoint = cf.position
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
    

    def goToGoal(self, goal):
        # Get the forces to apply to each drone to follow Reynold's boid and go to the goal
        f = self.totalForce(goal)
        for (name, cf) in self._crazyflies.items():
            self.applyForce(cf, f[name])


    # Runs faster than everything else, and shuts down if something is not within the bounding box
    def checkSafety(self):
        positions = self.getPositions()
        for (name, pos) in positions.items():
            for i in range(3):
                if abs(pos[i]) > self.bounding_box_size[i] and self._crazyflies[name].ready:
                    print("EMERGENCY, DRONE " + name + " IS OUTSIDE THE BOUNDING BOX! LANDING!!", pos)
                    self.moveAll([0.0, 0.0, - pos[2] + self.landing_height])
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

