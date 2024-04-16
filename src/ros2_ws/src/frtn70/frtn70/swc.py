# Needed to use ros and create nodes
import rclpy
from rclpy.node import Node

# Needed for transform listeners and publishers
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

#Fullstate
from crazyflie_interfaces.msg import FullState
import rowan

# For initial delay in startup
import time


# Needed for calculations
import numpy as np

class Operation:
    def __init__(self, name, force=[0.0, 0.0, 0.0], type="Move", countTo=0):
        self.name = name
        self.force = force
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
        self.goal_tolerance = 0.2

        # How big our area is
        self.bounding_box_size = [3.0/2, 5.0/2, 2.0]

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
        #self.debug_print_timer = self.create_timer(0.2, self.debugPrint)

        print("CREATING LISTENERS")
        self.createDrones()

        # Try to perform operations at 100 Hz
        operation_interval = 0.01
        self.operations = [
            Operation("Takeoff",                force=[0.0, 0.0, 0.5]),
            Operation("Wait 1 s", type="Delay", countTo=1.0/operation_interval),
            Operation("Move forward",           force=[0.0, 0.5, 0.0]),
            Operation("Wait 1 s", type="Delay", countTo=1.0/operation_interval),
            Operation("Move up",                force=[0.0, 0.0, 0.5]),
            Operation("Wait 1 s", type="Delay", countTo=1.0/operation_interval),
            Operation("Move back",              force=[0.0, -0.5, 0.0]),
            Operation("Wait 1 s", type="Delay", countTo=1.0/operation_interval),
            Operation("Move right",             force=[0.5, 0.0, 0.0]),
            Operation("Wait 1 s", type="Delay", countTo=1.0/operation_interval),
            Operation("Move left",              force=[-0.5, 0.0, 0.0]),
            Operation("Wait 10 s", type="Delay", countTo=10.0/operation_interval),
            ]

        # Call performOperations function every operation_interval seconds
        self.operation_timer = self.create_timer(operation_interval, self.performOperations)


    def debugPrint(self):
        for cf in self._crazyflies.values():
            print("Crazyflie: " + cf._drone,
                "X: ", "{:.8f}".format(cf.position[0])[0:6],
                "Y: ", "{:.8f}".format(cf.position[1])[0:6],
                "Z: ", "{:.8f}".format(cf.position[2])[0:6],
                "X-rot: ", "{:.4f}".format(cf.rotation[0])[0:4],
                "Y-rot: ", "{:.4f}".format(cf.rotation[1])[0:4],
                "Z-rot: ", "{:.4f}".format(cf.rotation[2])[0:4],
                "W-rot: ", "{:.4f}".format(cf.rotation[3])[0:4])
        print("Average position: ", self.getAvgPosition())


    def createDrones(self):
        for cf in self._drones:
            print("Created drone " + cf)
            # Create crazyflie node, with a goal tolerance of goal_tolerance
            self._crazyflies[cf] = FrameListener(cf, self.goal_tolerance)


    def getPositions(self):
        # Filter outliars
        valid_positions = {}
        for (cf, listener) in self._crazyflies.items():
            pos = listener.position
            # Filter?
            valid_positions[cf] = pos

        return valid_positions


    def getAvgPosition(self):
        pos = list(self.getPositions().values())
        if len(pos) > 0:
            return list(np.array(pos).mean(axis=0))
        else:
            return [0, 0, 0]

    
    def moveAll(self, force):
        #TODO display waypoint when setting new goal
        for (name, cf) in self._crazyflies.items():
            startPoint = self.getPositions()[name]
            goal = np.array(startPoint) + np.array(force)
            if any([abs(list(goal)[i]) > self.bounding_box_size[i] for i in range(len(self.bounding_box_size))]):
                print("GOAL OUTSIDE OF BOUNDING BOX")
                return
            cf.setGoal(goal)
            msg = cf.stateMsg
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x = startPoint[0] + force[0]
            msg.pose.position.y = startPoint[1] + force[1]
            msg.pose.position.z = startPoint[2] + force[2]
            #q = rowan.from_euler(0, 0, yaw)
            q = rowan.from_euler(0, 0, 0)
            msg.pose.orientation.w = q[0]
            msg.pose.orientation.x = q[1]
            msg.pose.orientation.y = q[2]
            msg.pose.orientation.z = q[3]
            
            # Send message
            cf.controlPublisher.publish(msg)


    def landAll(self):
        land_height = 0.1
        for (name, cf) in self._crazyflies.items():
            startPoint = self.getPositions()[name]
            goal = np.array([startPoint[0], startPoint[1], land_height])
            cf.setGoal(goal)
            msg = cf.stateMsg
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x = startPoint[0]
            msg.pose.position.y = startPoint[1]
            msg.pose.position.z = land_height
            #q = rowan.from_euler(0, 0, yaw)
            q = rowan.from_euler(0, 0, 0)
            msg.pose.orientation.w = q[0]
            msg.pose.orientation.x = q[1]
            msg.pose.orientation.y = q[2]
            msg.pose.orientation.z = q[3]
            
            # Send message
            cf.controlPublisher.publish(msg)   
        self.shutdown()


    def allAtPositions(self):
        atPos = [cf.goal_reached for cf in self._crazyflies.values()]
        return all(atPos)


    def allReady(self):
        return all([cf.ready for cf in self._crazyflies.values()])


    def performOperations(self):
        #print("Trying to perform operation")
        if not (self.allReady() and self.allAtPositions()):
            #print("All drones were not ready")
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
            print("Landing")
            self.landAll()
            return
        
        op = self.operations[current_op]
        if not op.inProgress:
            print("Executing op " + op.name)
            # Operation is not in progress, mark it as such and execute it
            if op.type == "Move":
                self.moveAll(op.force)
            op.inProgress = True
        else:                
            if op.type == "Delay":
                op.counter += 1
                # Check if we are done counting
                if op.counter < op.countTo:
                    return
            # We are done with operation, mark it as such
            op.completed = True

            


    def shutdown(self):
        for cf in self._crazyflies.values():
            cf.shutdown()
        self.destroy_node()


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


    def createControlPublisher(self): 
        self.stateMsg = FullState()
        self.stateMsg.header.frame_id = '/world'
        self.stateMsg.twist.angular.x = 0.0
        self.stateMsg.twist.angular.y = 0.0
        self.stateMsg.twist.angular.z = 0.0
        self.stateMsg.twist.linear.x = 0.0
        self.stateMsg.twist.linear.y = 0.0
        self.stateMsg.twist.linear.z = 0.0
        self.stateMsg.acc.x = 0.01
        self.stateMsg.acc.y = 0.01
        self.stateMsg.acc.z = 0.01
        self.controlPublisher = self.create_publisher(
                    FullState, self._drone + '/cmd_full_state', 1)
        
    
    def shutdown(self):
        self.destroy_node()

    
    def setGoal(self, pos, rot=[0, 0, 0, 0]):
        #We should check that it is within the bounding box here!
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
            if not can_transform:
                self.ready = False
                return

            self.ready = True

            # Get the drone position relative to the world
            tform = self.tf_buffer.lookup_transform(
                    to_frame_rel,
                    from_frame_rel,
                    rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                    f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        
        xyz = tform.transform.translation
        rot = tform.transform.rotation

        self.position = [xyz.x, xyz.y, xyz.z]
        self.rotation = [rot.x, rot.y, rot.z, rot.w]

        # Check if we have started. If we have, our current goal is our current position
        if not self.__started:
                self.__started = True
                self._goal_pos = self.position
        
        # Check if we have reached goal
        self.goal_reached = np.linalg.norm(
                np.array(self._goal_pos) - 
                np.array(self.position)) < self._goal_tolerance
        
        #TODO: Apply steering force


def main(args=None):
    rclpy.init(args=args)
    try:
        controller = Controller()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    controller.shutdown()
    #rclpy.shutdown()


if __name__ == '__main__':
    main()

