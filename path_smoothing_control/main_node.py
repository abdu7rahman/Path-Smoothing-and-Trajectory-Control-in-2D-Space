import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

from visualization_msgs.msg import Marker, MarkerArray
from .visualizer import create_path_marker, create_obstacle_markers, create_robot_marker

from .smooth_bsplines import bspline_path
from .smooth_cubic import cubic_spline_path
from .obstacle_utils import filter_path
from .trajectory_gen import generate_trajectory
from .controller import pure_pursuit

import math

class TrajectoryFollower(Node):
    def __init__(self):
        super().__init__('trajectory_follower')

        self.declare_parameter('spline_type', 'bspline')
        self.declare_parameter('start', False)

        spline_type = self.get_parameter('spline_type').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscriber_ = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.obstacle_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)

        self.waypoints = [(0, 0), (1, 0), (1, 1), (0.5, 2), (0, 2)]
        self.obstacles = [(0.5, 1.5, 0.3), (0.9, 0.5, 0.2)]

        if spline_type == "cubic":
            self.get_logger().info("⚙️ Using Cubic Spline")
            raw_path = cubic_spline_path(self.waypoints)
        else:
            self.get_logger().info("⚙️ Using B-Spline")
            raw_path = bspline_path(self.waypoints)

        safe_path = filter_path(raw_path, self.obstacles)

        self.trajectory = generate_trajectory(safe_path)

        self.safe_path = safe_path
        self.current_pose = (0.0, 0.0, 0.0)
        self.marker_timer = self.create_timer(0.3, self.publish_markers)

        self.current_index = 0
        self.lookahead_dist = 0.3
        self.reach_threshold = 0.3
        self.logged_wait_message = False
        self.timer = self.create_timer(0.1, self.control_loop)

    def publish_markers(self):
        self.marker_pub.publish(create_path_marker(self.safe_path))
        self.obstacle_pub.publish(create_obstacle_markers(self.obstacles))

        x, y, _ = self.current_pose
        robot_marker = create_robot_marker(x, y)
        self.marker_pub.publish(robot_marker)

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        q = [ori.x, ori.y, ori.z, ori.w]
        _, _, yaw = euler_from_quaternion(q)
        self.current_pose = (pos.x, pos.y, yaw)

    def control_loop(self):
        if not self.get_parameter('start').get_parameter_value().bool_value:
            if not self.logged_wait_message:
                self.get_logger().info("⏸ Waiting for parameter 'start' to be set to true...")
                self.logged_wait_message = True
            return

        if self.current_index >= len(self.trajectory):
            self.get_logger().info("🎯 Trajectory complete!")
            self.publisher_.publish(Twist())
            return

        x, y, theta = self.current_pose
        goal_x, goal_y, _ = self.trajectory[self.current_index]
        dist = math.hypot(goal_x - x, goal_y - y)

        self.get_logger().info(
            f"📍 Pose: ({x:.2f}, {y:.2f}, θ={math.degrees(theta):.2f}°) | 🎯 Goal: ({goal_x:.2f}, {goal_y:.2f}) | 📏 Dist: {dist:.2f}"
        )

        if dist < self.reach_threshold:
            self.get_logger().info(f"✅ Reached waypoint {self.current_index + 1}/{len(self.trajectory)}")
            self.current_index += 1
            return

        lin, ang = pure_pursuit(self.current_pose, self.trajectory[self.current_index:], self.lookahead_dist)

        twist = Twist()
        twist.linear.x = max(0.05, min(0.22, lin))
        twist.angular.z = max(-2.0, min(2.0, ang))
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
