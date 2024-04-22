# Needed to use ros and create nodes
import rclpy
import rclpy.duration
from rclpy.node import Node

# Needed for transform listeners and publishers
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# Fullstate control
from crazyflie_interfaces.msg import FullState, Status

# Needed for calculations
import numpy as np

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

        # Read our status and battery info
        self.statusSubscriber = self.create_subscription(Status, self._drone + '/status', self.statusHandler, 5)

        # Status for supervisor and battery
        self.battery_warn = False
        self.status = None


    def statusHandler(self, msg: Status):
        self.battery_warn = msg.pm_state == Status.PM_STATE_LOW_POWER or msg.pm_state == Status.PM_STATE_SHUTDOWN
        self.status = msg.supervisor_info
        if self.status == Status.SUPERVISOR_INFO_IS_LOCKED:
            print("Drone " + self._drone + " is locked")
            self.shutdown()


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