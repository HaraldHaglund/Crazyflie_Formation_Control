# Needed to use ros and create nodes
import rclpy
import rclpy.duration
from rclpy.node import Node

# Needed to display points
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

# Needed for transform listeners and publishers
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# Fullstate control
from crazyflie_interfaces.msg import FullState
import rowan

# For initial delay in startup
import time

# Needed for calculations
import numpy as np
import math

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
        self.wcoh = 0.02
        self.walign = 0.02
        self.wsep = 0.01

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
        self.createMarkerPublisher()

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
            self.circle_op.append(Operation("Move", type="Goal", goal=[math.cos(v) + 1.0, math.sin(v), 1.0 + math.sin(v) / 2]))

        self.circle_op.append(Operation("Wait 10 s", type="Delay", countTo=10.0/operation_interval))
        self.circle_op.append(Operation("Land", type="Land"))
        
        #self.operations = self.circle_op

        # Call performOperations function every operation_interval seconds
        self.operation_timer = self.create_timer(operation_interval, self.performOperations)

        # Print debug info every 0.5 seconds, 2Hz
        #self.debug_print_timer = self.create_timer(0.5, self.debugPrint)

        # Draw markers every 0.1 seconds, 10Hz
        self.marker_timer = self.create_timer(0.1, self.displayMarkers)

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

    
    def moveAll(self, force, rotation=0):
        #TODO display waypoint when setting new goal
        for (name, cf) in self._crazyflies.items():
            startPoint = self.getPositions()[name]
            goal = np.array(startPoint) + np.array(force)
            #if any([abs(list(goal)[i]) > self.bounding_box_size[i] for i in range(len(self.bounding_box_size))]):
            #    print("GOAL OUTSIDE OF BOUNDING BOX")
            #    return
            cf.setGoal(goal)
            #msg = cf.stateMsg
            msg = cf.getNewStateMsg()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x = goal[0] #startPoint[0] + force[0]
            msg.pose.position.y = goal[1] #startPoint[1] + force[1]
            msg.pose.position.z = goal[2] #startPoint[2] + force[2]
            #q = rowan.from_euler(0, 0, yaw)
            q = rowan.from_euler(0, 0, rotation)
            msg.pose.orientation.w = q[0]
            msg.pose.orientation.x = q[1]
            msg.pose.orientation.y = q[2]
            msg.pose.orientation.z = q[3]
            
            # Send message
            cf.stateMsg = msg
            #cf.controlPublisher.publish(msg)


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
        #TODO display waypoint when setting new goal
        startPoint = cf.position
        goal = np.array(startPoint) + np.array(force)
        #if any([abs(list(goal)[i]) > self.bounding_box_size[i] for i in range(len(self.bounding_box_size))]):
        #    print("GOAL OUTSIDE OF BOUNDING BOX")
        #    return
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
        #cf.controlPublisher.publish(msg)
    

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



    def shutdown(self):
        for cf in self._crazyflies.values():
            cf.shutdown()
        self.destroy_node()
        exit()


    def createMarkerPublisher(self):
        self.markerPublisher = self.create_publisher(MarkerArray, '/markers', 5)

    
    # TODO comment this
    def displayMarkers(self):
        markers = MarkerArray()
        bb=self.bounding_box_size
        corners = [
            [bb[0], bb[1], 0.0],
            [bb[0], bb[1], bb[2]],
            [-bb[0], bb[1], 0.0],
            [-bb[0], bb[1], bb[2]],
            [bb[0], -bb[1], 0.0],
            [bb[0], -bb[1], bb[2]],
            [-bb[0], -bb[1], 0.0],
            [-bb[0], -bb[1], bb[2]],
        ]

        box_lines = [
            (corners[0], corners[1]),
            (corners[2], corners[3]),
            (corners[4], corners[5]),
            (corners[6], corners[7]),
            (corners[0], corners[2]),
            (corners[0], corners[4]),
            (corners[2], corners[6]),
            (corners[4], corners[6]),
            (corners[1], corners[3]),
            (corners[1], corners[5]),
            (corners[3], corners[7]),
            (corners[5], corners[7]),
        ]
        
        #for (i, c) in enumerate(corners):
        for (i, l) in enumerate(box_lines):
            m = Marker()
            m.header.frame_id = "world"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "corner"
            m.type = Marker.LINE_STRIP 
            m.action = Marker.ADD
            m.id = i
            m.pose.position.x = 0.0
            m.pose.position.y = 0.0
            m.pose.position.z = 0.0
            m.pose.orientation.x = 0.0
            m.pose.orientation.y = 0.0
            m.pose.orientation.z = 0.0
            m.pose.orientation.w = 1.0
            m.color.r = 255.0
            m.color.g = 240.0
            m.color.b = 0.0
            m.color.a = 1.0
            #m.lifetime = rclpy.duration.Duration()
            m.scale.x = 0.01
            m.scale.y = 0.01
            m.scale.z = 0.01
            p1 = Point()
            p2 = Point()
            p1.x = l[0][0]
            p1.y = l[0][1]
            p1.z = l[0][2]
            p2.x = l[1][0]
            p2.y = l[1][1]
            p2.z = l[1][2]
            m.points = [p1, p2]
            
            markers.markers.append(m)

        for (i, op) in enumerate(self.operations):
            # Do not create waypoints for moves or delays
            if op.type in ["Delay", "Move"]:
                continue
            m = Marker()
            m.header.frame_id = "world"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "waypoint"
            m.type = Marker.SPHERE 
            m.action = Marker.ADD
            m.id = i + len(box_lines)
            m.pose.position.x = op.goal[0]
            m.pose.position.y = op.goal[1]
            m.pose.position.z = op.goal[2] - self.goal_tolerance / 2
            m.pose.orientation.x = 0.0
            m.pose.orientation.y = 0.0
            m.pose.orientation.z = 0.0
            m.pose.orientation.w = 1.0
            m.color.r = min(200.0 + i, 255.0)
            m.color.g = min(102.0 + i, 255.0)
            m.color.b = 0.0
            m.color.a = 1.0
            #m.lifetime = rclpy.duration.Duration()
            m.scale.x = self.goal_tolerance
            m.scale.y = self.goal_tolerance
            m.scale.z = self.goal_tolerance

            markers.markers.append(m)

        m = Marker()
        avgPos = self.getAvgPosition(list(self.getPositions().values()))
        m.header.frame_id = "world"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "average_position"
        m.type = Marker.SPHERE 
        m.action = Marker.ADD
        m.id = i + len(box_lines) + len(self.operations)
        m.pose.position.x = avgPos[0]
        m.pose.position.y = avgPos[1]
        m.pose.position.z = avgPos[2]
        m.pose.orientation.x = 0.0
        m.pose.orientation.y = 0.0
        m.pose.orientation.z = 0.0
        m.pose.orientation.w = 1.0
        m.color.r = 255.0
        m.color.g = 0.0
        m.color.b = 0.0
        m.color.a = 1.0
        m.text = "AvgPos"
        #m.lifetime = rclpy.duration.Duration()
        m.scale.x = self.goal_tolerance / 2
        m.scale.y = self.goal_tolerance / 2
        m.scale.z = self.goal_tolerance / 2

        markers.markers.append(m)
            
        self.markerPublisher.publish(markers)




class FrameListener(Node):

    def __init__(self, drone, goal_tolerance):
        super().__init__(drone + '_frame_listener')
        self._drone = drone

        # Declare and acquire `target_frame` parameter
        self.target_frame = self.declare_parameter(
          'target_frame', 'world').get_parameter_value().string_value
        
        # Buffer for receiving messages
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # Call on_timer function every 0.01 seconds, 100Hz
        self.timer = self.create_timer(0.01, self.on_timer)
        
        # Store last known position
        self.position = [0, 0, -1000]
        self.rotation = [0, 0, 0]

        # Store velocity - really deltaPosition/deltaTime
        self.velocity = [0.0, 0.0, 0.0]
        
        # Store last_update_time for velocity calculations
        self._last_update_time = 0
        
        # Set our goal, and say that we have reached it. 
        # Might have to set this to initial position after startup.
        self._goal_pos = [0, 0, 0]
        self._goal_rot = [0, 0, 0, 0]
        self.goal_reached = True
        self._goal_tolerance = goal_tolerance
        
        # Check if the connection to ROS is working and we are ready
        self.ready = False
        self.__started = False

        # Create control publisher
        self.createControlPublisher()

        # Needed since we need to continously send signals to the drones to get them to fly
        self.shouldStream = False
        self.stream_timer = self.create_timer(0.1, self.sendPosStream)


    def createControlPublisher(self): 
        self.stateMsg = self.getNewStateMsg()
        self.controlPublisher = self.create_publisher(
                    FullState, self._drone + '/cmd_full_state', 5)
        
    
    def getNewStateMsg(self):
        msg = FullState()
        msg.header.frame_id = '/world'
        msg.twist.angular.x = 0.00
        msg.twist.angular.y = 0.00
        msg.twist.angular.z = 0.0
        msg.twist.linear.x = 0.00
        msg.twist.linear.y = 0.00
        msg.twist.linear.z = 0.0
        msg.acc.x = 0.00
        msg.acc.y = 0.00
        msg.acc.z = 0.0
        return msg

    
    def sendPosStream(self):
        if not self.shouldStream:
            return
        self.stateMsg.header.stamp = self.get_clock().now().to_msg() #rclpy.time.Time().to_msg() #self.get_clock().now().to_msg()
        self.controlPublisher.publish(self.stateMsg)
        
    
    def shutdown(self):
        self.destroy_node()

    
    def setGoal(self, pos, rot=[0, 0, 0, 0]):
        self.shouldStream = True
        self._goal_pos = pos
        self._goal_rot = rot
        self.goal_reached = False


    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = self._drone
        to_frame_rel = self.target_frame

        try:
            # If the transform does not yet exist, skip running the method
            can_transform = self.tf_buffer.can_transform(
                                to_frame_rel, 
                                from_frame_rel,
                                rclpy.time.Time())
                                #self.get_clock().now())
            if not can_transform:
                print("Could not transform frames", to_frame_rel, from_frame_rel)
                self.ready = False
                return

            

            # Get the drone position relative to the world
            tform = self.tf_buffer.lookup_transform(
                    to_frame_rel,
                    from_frame_rel,
                    rclpy.time.Time())
                    #self.get_clock().now())
            
            self.ready = True
        except TransformException as ex:
            self.get_logger().info(
                    f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        
        xyz = tform.transform.translation
        rot = tform.transform.rotation

        # Store variables for calculating velocity
        oldtime = self._last_update_time
        self._last_update_time = self.get_clock().now().nanoseconds
        oldposition = self.position

        self.position = [xyz.x, xyz.y, xyz.z]
        self.rotation = [rot.x, rot.y, rot.z, rot.w]

        # Calculate velocity as deltapos / deltatime (convert nanoseconds to seconds as well)
        self.velocity = list((np.array(self.position) - np.array(oldposition))/
                             ((self._last_update_time - oldtime)/1000000000)) 

        # Check if we have started. If we have, our current goal is our current position
        if not self.__started:
                self.__started = True
                self._goal_pos = self.position
        
        # Check if we have reached goal
        self.goal_reached = np.linalg.norm(
                np.array(self._goal_pos) - 
                np.array(self.position)) < self._goal_tolerance
        

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

