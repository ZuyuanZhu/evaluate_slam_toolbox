import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import tf_transformations
import math
import tf2_ros
from geometry_msgs.msg import PoseStamped, Quaternion


class PoseErrorCalculator(Node):
    def __init__(self):
        super().__init__('pose_error_calculator')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.goal_pose = None

    def send_goal_and_wait(self, goal_pose):
        self.goal_pose = goal_pose
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.action_client.wait_for_server()
        self.future = self.action_client.send_goal_async(goal_msg)
        self.future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Goal reached!')
        self.calculate_pose_error()

    def calculate_pose_error(self):
        if self.goal_pose is None:
            self.get_logger().error('Goal pose not set.')
            return

        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            actual_position = trans.transform.translation
            actual_orientation = trans.transform.rotation

            goal_position = self.goal_pose.pose.position
            goal_orientation = self.goal_pose.pose.orientation

            # Positional Error
            positional_error = math.sqrt(
                (goal_position.x - actual_position.x) ** 2 +
                (goal_position.y - actual_position.y) ** 2 +
                (goal_position.z - actual_position.z) ** 2)

            # Orientation Error
            goal_euler = tf_transformations.euler_from_quaternion([
                goal_orientation.x,
                goal_orientation.y,
                goal_orientation.z,
                goal_orientation.w])

            actual_euler = tf_transformations.euler_from_quaternion([
                actual_orientation.x,
                actual_orientation.y,
                actual_orientation.z,
                actual_orientation.w])

            orientation_error = math.sqrt(
                (goal_euler[0] - actual_euler[0]) ** 2 +
                (goal_euler[1] - actual_euler[1]) ** 2 +
                (goal_euler[2] - actual_euler[2]) ** 2)

            self.get_logger().info(f'Positional Error: {positional_error:.4f}, Orientation Error: {orientation_error:.4f}')

        except Exception as e:
            self.get_logger().error(f'Failed to calculate pose error: {e}')

def main(args=None):
    rclpy.init(args=args)
    pose_error_calculator = PoseErrorCalculator()

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.pose.position.x = 0.2  # Example goal position
    goal_pose.pose.position.y = 0.0
    goal_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

    pose_error_calculator.send_goal_and_wait(goal_pose)

    try:
        rclpy.spin(pose_error_calculator)
    except KeyboardInterrupt:
        pass

    pose_error_calculator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

