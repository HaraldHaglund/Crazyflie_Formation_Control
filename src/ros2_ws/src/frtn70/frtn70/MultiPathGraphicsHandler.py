# Needed to display points
import random
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


class GraphicsHandler:
    def __init__(self, controller):
        self.controller = controller
        self.goal_tolerance = self.controller.goal_tolerance


    def createMarkerPublishers(self):
        self.avgPointPublisher = self.controller.create_publisher(Marker, '/avgPoint', 5)
        self.boundingBoxPublisher = self.controller.create_publisher(MarkerArray, '/boundingBox', 5)
        self.waypointPublisher = self.controller.create_publisher(MarkerArray, '/waypoints', 5)
        self.obstaclePublisher = self.controller.create_publisher(MarkerArray, '/obstacles', 5)


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
        colors = [[255.0, 128.0, 0.0], [0.0, 128.0, 255.0], [0.0, 0.0, 255.0], [0.0, 255.0, 255.0], [255.0, 0.0, 255.0], [255.0, 255.0, 0.0]]
        id = 0
        for (i, name) in enumerate(self.controller.operations):
            r, g, b = colors[i]
            for (j, op) in enumerate(self.controller.operations[name]):
                # Do not create waypoints for moves or delays
                if op.type not in ["Delay", "Move", "Landing", "Takeoff"]:
                    m = Marker()
                    m.header.frame_id = "world"
                    m.header.stamp = self.controller.get_clock().now().to_msg()
                    m.ns = "waypoint"
                    m.type = Marker.SPHERE 
                    m.action = Marker.ADD
                    m.id = id
                    m.pose.position.x = op.goal[0]
                    m.pose.position.y = op.goal[1]
                    m.pose.position.z = op.goal[2] - self.goal_tolerance / 2
                    m.pose.orientation.x = 0.0
                    m.pose.orientation.y = 0.0
                    m.pose.orientation.z = 0.0
                    m.pose.orientation.w = 1.0
                    m.color.r = float(r)
                    m.color.g = float(g)
                    m.color.b = float(b)
                    m.color.a = 1.0
                    #m.lifetime = rclpy.duration.Duration()
                    m.scale.x = self.goal_tolerance
                    m.scale.y = self.goal_tolerance
                    m.scale.z = self.goal_tolerance

                    markers.markers.append(m)
                    id += 1

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
        avgPos = self.controller.getAvgPosition(list(self.controller.getPositions().values()))
        m.header.frame_id = "world"
        m.header.stamp = self.controller.get_clock().now().to_msg()
        m.ns = "average_position"
        m.type = Marker.SPHERE 
        m.action = Marker.ADD
        m.id = 1 + len(self.controller.operations) + 12 # 12 since that is the number of edges in the bounding box
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