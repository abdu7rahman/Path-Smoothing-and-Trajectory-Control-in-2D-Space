import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import math

from .smooth_bsplines import bspline_path
from .smooth_cubic import cubic_spline_path

class PathMarkerPublisher(Node):
    def __init__(self):
        super().__init__('rviz_marker_publisher')
        self.publisher_ = self.create_publisher(MarkerArray, '/trajectory_markers', 10)

        waypoints = [(0, 0), (1, 0), (1, 1), (0.5, 2), (0, 2)]
        smoothed = bspline_path(waypoints)

        self.marker_array = MarkerArray()
        self.create_timer(1.0, self.publish_markers)
        self.build_path_marker(smoothed)

    def build_path_marker(self, path):
        line_marker = Marker()
        line_marker.header.frame_id = "odom"
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.03
        line_marker.color = ColorRGBA(r=0.0, g=0.7, b=1.0, a=1.0)
        line_marker.id = 0

        for x, y in path:
            pt = Point()
            pt.x = x
            pt.y = y
            pt.z = 0.01
            line_marker.points.append(pt)

        self.marker_array.markers.append(line_marker)

        for i, (x, y) in enumerate(path):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.id = i + 1
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.02
            self.marker_array.markers.append(marker)

    def publish_markers(self):
        self.publisher_.publish(self.marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = PathMarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
