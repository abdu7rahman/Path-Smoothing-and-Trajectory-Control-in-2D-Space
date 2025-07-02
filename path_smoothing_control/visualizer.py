from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from rclpy.clock import Clock


def create_path_marker(path, frame="odom", marker_id=0, color=(0.0, 0.5, 1.0), ns="path", z=0.05):
    marker = Marker()
    marker.header.frame_id = frame
    marker.header.stamp = Clock().now().to_msg()
    marker.ns = ns
    marker.id = marker_id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.03
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = *color, 1.0
    marker.lifetime.sec = 0

    marker.points = [Point(x=x, y=y, z=z) for x, y in path]

    return marker


def create_obstacle_markers(obstacles, frame="odom", start_id=100, ns="obstacles", z=0.05):
    markers = MarkerArray()
    now = Clock().now().to_msg()

    for i, (x, y, radius) in enumerate(obstacles):
        marker = Marker()
        marker.header.frame_id = frame
        marker.header.stamp = now
        marker.ns = ns
        marker.id = start_id + i
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = radius * 2
        marker.scale.y = radius * 2
        marker.scale.z = 0.05
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.2
        marker.color.a = 0.7
        marker.lifetime.sec = 0

        markers.markers.append(marker)

    return markers


def create_robot_marker(x, y, z=0.05, frame="odom", marker_id=999, radius=0.08, color=(0.0, 1.0, 0.0), ns="robot"):
    marker = Marker()
    marker.header.frame_id = frame
    marker.header.stamp = Clock().now().to_msg()
    marker.ns = ns
    marker.id = marker_id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    marker.pose.orientation.w = 1.0
    marker.scale.x = radius * 2
    marker.scale.y = radius * 2
    marker.scale.z = radius * 2
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = *color, 1.0
    marker.lifetime.sec = 0

    return marker
