import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
# from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time
from rclpy.duration import Duration
from nav2_msgs.action import NavigateToPose
import tf_transformations
import math
import tf2_ros
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion


class PoseErrorCalculator(Node):
    def __init__(self):
        super().__init__('pose_error_calculator')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.goal_poses = self.initialize_goal_poses()
        self.goal_pose = None
        self.current_goal_index = 0

    def initialize_goal_poses(self):
        # Define the goal poses as provided
        goal_poses = [
            PoseStamped(header=Header(frame_id='map'), pose=Pose(position=Point(x=1.8551847441369187, y=0.940469060964706, z=0.00875448964744601),
                              orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=0.5419295886150594))),
            PoseStamped(header=Header(frame_id='map'), pose=Pose(position=Point(x=1.7534640732259952, y=-0.2394405433869599, z=0.00871776494059019),
                              orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=0.7841612996486618))),
            PoseStamped(header=Header(frame_id='map'), pose=Pose(position=Point(x=1.421376528983878, y=-1.5226155433894593, z=0.008732245356602558),
                              orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=0.20313778818436523))),
            PoseStamped(header=Header(frame_id='map'), pose=Pose(position=Point(x=0.5536792346192545, y=-1.2509020209256785, z=0.008758151025220779),
                              orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=-0.6516467919741615))),
            PoseStamped(header=Header(frame_id='map'), pose=Pose(position=Point(x=0.4358200267885457, y=0.455400673655191, z=0.008756493873767923),
                              orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=-0.7707024125271144))),
            PoseStamped(header=Header(frame_id='map'), pose=Pose(position=Point(x=-0.19518927613405285, y=1.8786164076508312, z=0.008725528479063535),
                              orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=-0.2786073797082674))),
            PoseStamped(header=Header(frame_id='map'), pose=Pose(position=Point(x=-1.279226046180035, y=-0.5348333154030177, z=0.008739992326590708),
                              orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=0.5757394004414378))),
            PoseStamped(header=Header(frame_id='map'), pose=Pose(position=Point(x=-2.406261406350424, y=-0.006286499769925097, z=0.008731338133976986),
                              orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=-0.9935824178087028))),
            PoseStamped(header=Header(frame_id='map'), pose=Pose(position=Point(x=-1.2039159027100623, y=0.5485480333908036, z=0.008729860856414171),
                              orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=-0.9999830441306548))),
            PoseStamped(header=Header(frame_id='map'), pose=Pose(position=Point(x=-1.71424489032583, y=-1.4708795582505652, z=0.008727764144929784),
                              orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=-0.9773999332188458)))
        ]
        return goal_poses

    def send_next_goal(self):
        if self.current_goal_index < len(self.goal_poses):
            self.send_goal_and_wait(self.goal_poses[self.current_goal_index])
            self.current_goal_index += 1
        else:
            self.get_logger().info('All goals have been processed.')

    def send_goal_and_wait(self, goal_pose):
        self.goal_pose = goal_pose  # Set the current goal pose
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.action_client.wait_for_server()
        send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Goal reached, calculating pose error...')
        self.calculate_pose_error()

        # Create a one-shot timer for a 3-second pause
        self.pause_timer = self.create_timer(3, self.send_next_goal_once)


    def send_next_goal_once(self):
        self.send_next_goal()
        # Cancel the timer to simulate a one-shot timer
        self.pause_timer.cancel()

    # Define calculate_pose_error, feedback_callback, and other necessary methods here
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

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # Assuming feedback includes the current pose, you can log it or perform other operations
        # self.get_logger().info(f"Current Pose: {feedback.current_pose}")


def main(args=None):
    rclpy.init(args=args)
    pose_error_calculator = PoseErrorCalculator()
    pose_error_calculator.send_next_goal()  # Start processing the first goal

    try:
        rclpy.spin(pose_error_calculator)
    except KeyboardInterrupt:
        pass
    finally:
        pose_error_calculator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
