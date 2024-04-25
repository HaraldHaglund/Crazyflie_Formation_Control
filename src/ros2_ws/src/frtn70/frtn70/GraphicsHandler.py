# Needed to display points
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import numpy as np


class GraphicsHandler:
    def __init__(self, controller):
        self.controller = controller
        self.goal_tolerance = self.controller.goal_tolerance


    def createMarkerPublishers(self):
        self.avgPointPublisher = self.controller.create_publisher(Marker, '/avgPoint', 5)
        self.boundingBoxPublisher = self.controller.create_publisher(MarkerArray, '/boundingBox', 5)
        self.waypointPublisher = self.controller.create_publisher(MarkerArray, '/waypoints', 5)
        self.obstaclePublisher = self.controller.create_publisher(MarkerArray, '/obstacles', 5)
        self.forcePublisher = self.controller.create_publisher(Marker, '/forces', 10)
        self.opPublisher = self.controller.create_publisher(MarkerArray, '/current_op', 5)


    def displayBoundingBox(self):
        markers = MarkerArray()
        bb=self.controller.bounding_box_size
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
            m.header.stamp = self.controller.get_clock().now().to_msg()
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

        self.boundingBoxPublisher.publish(markers)


        
    def displayWaypoints(self):
        markers = MarkerArray()
        for (i, op) in enumerate(self.controller.operations):
            # Do not create waypoints for moves or delays
            if op.type in ["Delay", "Move"]:
                continue
            m = Marker()
            m.header.frame_id = "world"
            m.header.stamp = self.controller.get_clock().now().to_msg()
            m.ns = "waypoint"
            m.type = Marker.SPHERE 
            m.action = Marker.ADD
            m.id = i + 12 # 12 since that is the number of edges in the bounding box
            m.pose.position.x = op.goal[0]
            m.pose.position.y = op.goal[1]
            m.pose.position.z = op.goal[2] - self.goal_tolerance / 2
            m.pose.orientation.x = 0.0
            m.pose.orientation.y = 0.0
            m.pose.orientation.z = 0.0
            m.pose.orientation.w = 1.0
            m.color.r = min(100.0 + i, 255.0)
            m.color.g = min(102.0 + i, 255.0)
            m.color.b = 0.0
            m.color.a = 1.0
            #m.lifetime = rclpy.duration.Duration()
            m.scale.x = self.goal_tolerance
            m.scale.y = self.goal_tolerance
            m.scale.z = self.goal_tolerance

            markers.markers.append(m)

        self.waypointPublisher.publish(markers)


    def displayObstacles(self):
        markers = MarkerArray()
        for (i, ob) in enumerate(self.controller.obstacles):
            m = Marker()
            m.header.frame_id = "world"
            m.header.stamp = self.controller.get_clock().now().to_msg()
            m.ns = "obstacle"
            m.type = Marker.CUBE 
            m.action = Marker.ADD
            m.id = ob.id 
            m.pose.position.x = ob.location[0]
            m.pose.position.y = ob.location[1]
            m.pose.position.z = ob.location[2]
            m.pose.orientation.x = 0.0
            m.pose.orientation.y = 0.0
            m.pose.orientation.z = 0.0
            m.pose.orientation.w = 1.0
            m.color.r = 0.0
            m.color.g = 255.0
            m.color.b = 0.0
            m.color.a = 1.0
            #m.lifetime = rclpy.duration.Duration()
            m.scale.x = ob.size[0]
            m.scale.y = ob.size[1]
            m.scale.z = ob.size[2]

            markers.markers.append(m)

        self.obstaclePublisher.publish(markers)
    

    def displayAvgPoint(self):        
        m = Marker()
        p = self.controller.avgPos
        if p is None:
            p = [0.0, 0.0, 0.0]
        avgPos = p
        m.header.frame_id = "world"
        m.header.stamp = self.controller.get_clock().now().to_msg()
        m.ns = "average_position"
        m.type = Marker.SPHERE 
        m.action = Marker.ADD
        m.id = 1# + len(self.controller.operations) + 12 # 12 since that is the number of edges in the bounding box
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
            
        self.avgPointPublisher.publish(m)

    def displayOp(self, op):        
        ma = MarkerArray()
        m = Marker()
        m1 = Marker()
        p = op.goal
        if p is None:
            p = [0.0, 0.0, 0.0]
        avgPosMax = p + np.array([self.goal_tolerance for i in range(3)])
        m.header.frame_id = "world"
        m.header.stamp = self.controller.get_clock().now().to_msg()
        m.ns = "current_op"
        m.type = Marker.SPHERE 
        m.action = Marker.ADD
        m.id = 1# + len(self.controller.operations) + 12 # 12 since that is the number of edges in the bounding box
        m.pose.position.x = avgPosMax[0]
        m.pose.position.y = avgPosMax[1]
        m.pose.position.z = avgPosMax[2]
        m.pose.orientation.x = 0.0
        m.pose.orientation.y = 0.0
        m.pose.orientation.z = 0.0
        m.pose.orientation.w = 1.0
        m.color.r = 128.0
        m.color.g = 0.0
        m.color.b = 120.0
        m.color.a = 1.0
        m.text = "AvgPosMax"
        #m.lifetime = rclpy.duration.Duration()
        m.scale.x = self.goal_tolerance / 2
        m.scale.y = self.goal_tolerance / 2
        m.scale.z = self.goal_tolerance / 2

        avgPosMin = p - np.array([self.goal_tolerance for i in range(3)])
        m1.header.frame_id = "world"
        m1.header.stamp = self.controller.get_clock().now().to_msg()
        m1.ns = "current_op"
        m1.type = Marker.SPHERE 
        m1.action = Marker.ADD
        m1.id = 1# + len(self.controller.operations) + 12 # 12 since that is the number of edges in the bounding box
        m1.pose.position.x = avgPosMin[0]
        m1.pose.position.y = avgPosMin[1]
        m1.pose.position.z = avgPosMin[2]
        m1.pose.orientation.x = 0.0
        m1.pose.orientation.y = 0.0
        m1.pose.orientation.z = 0.0
        m1.pose.orientation.w = 1.0
        m1.color.r = 128.0
        m1.color.g = 0.0
        m1.color.b = 120.0
        m1.color.a = 1.0
        m1.text = "AvgPosMin"
        #m.lifetime = rclpy.duration.Duration()
        m1.scale.x = self.goal_tolerance / 2
        m1.scale.y = self.goal_tolerance / 2
        m1.scale.z = self.goal_tolerance / 2

        ma.markers.append(m)
        ma.markers.append(m1)
            
        self.opPublisher.publish(ma)

    def displayForce(self, id, startPos, endPos):
        m = Marker()
        m.header.frame_id = "world"
        m.header.stamp = self.controller.get_clock().now().to_msg()
        m.ns = "force_" + str(id)
        m.type = Marker.ARROW 
        m.action = Marker.ADD
        m.id = id
        m.pose.position.x = 0.0
        m.pose.position.y = 0.0
        m.pose.position.z = 0.0
        m.pose.orientation.x = 0.0
        m.pose.orientation.y = 0.0
        m.pose.orientation.z = 0.0
        m.pose.orientation.w = 1.0
        m.color.r = 0.0
        m.color.g = 0.0
        m.color.b = 255.0
        m.color.a = 1.0
        m.scale.x = 0.01
        m.scale.y = 0.015
        m.scale.z = 0.02

        p1 = Point()
        p2 = Point()
        p1.x = startPos[0]
        p1.y = startPos[1]
        p1.z = startPos[2]
        
        d = np.array(endPos) - np.array(startPos)
        n = np.linalg.norm(d)
        nv = d/n
        nv = nv * (0.25 + n*5) # Lengthen to length 0.25m
        p2.x = float(startPos[0] + nv[0])
        p2.y = float(startPos[1] + nv[1])
        p2.z = float(startPos[2] + nv[2])
        m.points = [p1, p2]
            
        self.forcePublisher.publish(m)