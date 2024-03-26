import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.logging import LoggingSeverity
from nav_msgs.msg import Odometry
from rvc_msgs.msg import RvcOdom
from math import sqrt


class PoseComparisonNode(Node):
    def __init__(self):
        super().__init__('pose_comparison_node')

        qos_reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        # If the publisher uses BEST_EFFORT, then set reliability to ReliabilityPolicy.BEST_EFFORT
        
        self.subscription_rvc_odom = self.create_subscription(
            RvcOdom,
            '/rvc_odom',
            self.callback_rvc_odom,
            qos_reliable)
        self.get_logger().info("Subscribed to /rvc_odom with BEST_EFFORT policy")

        self.subscription_custom_odom = self.create_subscription(
            Odometry,
            '/custom_odom',
            self.callback_custom_odom,
            10)
        self.get_logger().info("Subscribed to /custom_odom")

        self.subscription_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.callback_odom,
            10)
        self.get_logger().info("Subscribed to /odom")

        self.pose_rvc_odom = None
        self.pose_custom_odom = None
        self.pose_odom = None  # Considered as ground truth
        
        # Timer to periodically call compare_poses()
        self.timer = self.create_timer(0.1, self.compare_poses)  # 0.1 second interval
        self.get_logger().info("Node and timer initialized successfully")

    def callback_rvc_odom(self, msg):
        self.pose_rvc_odom = msg.rvc_pose
        self.get_logger().debug(f"Received pose on /rvc_odom: {msg.rvc_pose}")

    def callback_custom_odom(self, msg):
        self.pose_custom_odom = msg.pose.pose
        self.get_logger().debug(f"Received pose on /custom_odom: {msg.pose.pose}")

    def callback_odom(self, msg):
        self.pose_odom = msg.pose.pose
        self.get_logger().debug(f"Received pose on /odom: {msg.pose.pose}")

    def calculate_distance(self, pose1, pose2):
        dx = pose1.x - pose2.position.x
        dy = pose1.y - pose2.position.y
        self.get_logger().debug(f'Calculating distance: pose1.x = {pose1.x}, pose2.x = {pose2.position.x}')
        self.get_logger().debug(f'Calculating distance: pose1.y = {pose1.y}, pose2.y = {pose2.position.y}')
        return sqrt(dx*dx + dy*dy)

    def compare_poses(self):
        if self.pose_rvc_odom and self.pose_odom:
            discrepancy_rvc = self.calculate_distance(self.pose_rvc_odom, self.pose_odom)
            self.get_logger().info('Discrepancy between /rvc_odom and /odom: %f' % discrepancy_rvc)
        elif self.pose_rvc_odom and self.pose_custom_odom:
            discrepancy_custom = self.calculate_distance(self.pose_rvc_odom, self.pose_custom_odom)
            self.get_logger().info('Discrepancy between /rvc_odom and /custom_odom: %f' % discrepancy_custom)
        else:
            self.get_logger().debug("Waiting for all poses to be received...")

def main(args=None):
    rclpy.init(args=args)
    pose_comparison_node = PoseComparisonNode()
    # rclpy.logging.set_logger_level(pose_comparison_node.get_name(), LoggingSeverity.DEBUG)
    try:
        rclpy.spin(pose_comparison_node)  # Changed to spin the node continuously
    except KeyboardInterrupt:
        pass
    finally:
        pose_comparison_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
