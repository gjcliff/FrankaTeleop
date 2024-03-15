from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from franka_teleop.srv import PlanPath

from visualization_msgs.msg import Marker
from hand_interfaces.msg import Pinch, FingerData

from std_srvs.srv import Empty
from std_msgs.msg import String

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import PoseStamped
import tf2_ros
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from franka_msgs.action import Homing, Grasp

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import numpy as np

class CvFrankaBridge(Node):

    def __init__(self):
        super().__init__('cv_franka_bridge')

        # create callback groups
        self.waypoint_callback_group = MutuallyExclusiveCallbackGroup()
        self.gesture_callback_group = MutuallyExclusiveCallbackGroup()

        # create subscribers
        self.waypoint_subscriber = self.create_subscription(PoseStamped, 'waypoint', self.waypoint_callback, 10, callback_group=self.waypoint_callback_group)
        # self.left_gesture_subscriber = self.create_subscription(String, 'left_gesture', self.left_gesture_callback, 10, callback_group=self.gesture_callback_group)
        self.right_gesture_subscriber = self.create_subscription(String, 'right_gesture', self.right_gesture_callback, 10, callback_group=self.gesture_callback_group)
        self.pinch_data_subscriber = self.create_subscription(Pinch, 'pinch_data', self.pinch_data_callback, 10)

        # create publishers
        self.text_marker_publisher = self.create_publisher(Marker, 'text_marker', 10)

        # create clients
        # self.plan_and_execute_client = self.create_client(PlanPath, 'plan_and_execute_path')
        self.waypoint_client = self.create_client(PlanPath, 'robot_waypoints')
        self.waypoint_client.wait_for_service(timeout_sec=2.0)

        self.text_marker = self.create_text_marker("Thumbs up to begin teleoperation")
        self.gripper_ready = True
        self.gripper_status = "Open"
        self.gripper_homed = False
        self.gripper_force_control = False
        self.gripper_force = 0.001
        self.max_gripper_force = 10.0

        # create action clients
        self.gripper_homing_client = ActionClient(
                self, Homing, 'panda_gripper/homing')
        self.gripper_grasping_client = ActionClient(
                self, Grasp, 'panda_gripper/grasp')

        self.gripper_grasping_client.wait_for_server(timeout_sec=1.0)
        # with a fake gripper, the homing server will not be created
        if not self.gripper_homing_client.wait_for_server(
                timeout_sec=1):
            self.gripper_ready = False
            self.gripper_homed = True

        # create tf buffer and listener
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        # create timer
        self.timer = self.create_timer(0.04, self.timer_callback)

        # create class variables
        self.offset = Pose()
        self.current_waypoint = None
        self.previous_waypoint = None
        self.offset = None
        self.desired_ee_pose = None
        self.initial_ee_pose = None
        self.waypoints = []
        self.move_robot = False
        self.start_time = self.get_clock().now()

        self.lower_distance_threshold = 3.0
        self.upper_distance_threshold = 10.0

        # self.kp_coarse = 1.5
        # self.ki_coarse = 0.01
        # self.kd_coarse = 0.01
        # self.kp_fine = 1.5
        # self.ki_fine = 0.01
        # self.kd_fine = 0.01
        # self.max_output_coarse = 0.2
        # self.max_output_fine = 0.05
        # set the robot in coarse mode to start
        self.kp = 5.0
        self.ki = 0.0
        self.kd = 0.01
        self.kp_angle = 1.0
        self.ki_angle = 0.0
        self.kd_angle = 0.01
        self.max_output = 0.5
        self.integral_prior = 0
        self.position_error_prior = 0
        self.roll_error_prior = 0
        self.pitch_error_prior = 0
        self.yaw_error_prior = 0

        self.pinch_data = None
        self.count = 0

    def pinch_data_callback(self, msg):
        self.pinch_data = msg

    def create_text_marker(self, text):
        marker = Marker()
        marker.header.frame_id = "panda_link0"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = marker.TEXT_VIEW_FACING
        marker.text = text
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 1.0
        marker.scale.z = 0.1
        marker.scale.x = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        return marker

    def gripper_homing_callback(self, request, response):
        goal = Homing.Goal()
        self.gripper_homing_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        return response

    def get_transform(self, target_frame, source_frame):
        try:
            trans = self.buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            translation = trans.transform.translation
            rotation = trans.transform.rotation
            return translation, rotation

        except tf2_ros.LookupException as e:
            # the frames don't exist yet
            self.get_logger().info(f"Lookup exception: {e}")
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]
        except tf2_ros.ConnectivityException as e:
            # the tf tree has a disconnection
            self.get_logger().info(f"Connectivity exception: {e}")
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]
        except tf2_ros.ExtrapolationException as e:
            # the times are two far apart to extrapolate
            self.get_logger().info(f"Extrapolation exception: {e}")
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]

    def get_ee_pose(self):
        ee_home_pos, ee_home_rot = self.get_transform("panda_link0", "panda_hand_tcp")
        ee_pose = Pose()
        ee_pose.position.x = ee_home_pos.x
        ee_pose.position.y = ee_home_pos.y
        ee_pose.position.z = ee_home_pos.z
        ee_pose.orientation.x = ee_home_rot.x
        ee_pose.orientation.y = ee_home_rot.y
        ee_pose.orientation.z = ee_home_rot.z
        ee_pose.orientation.w = ee_home_rot.w
        return ee_pose

    def waypoint_callback(self, msg):
        if self.current_waypoint is None or self.previous_waypoint is None:
            self.current_waypoint = msg.pose
            self.previous_waypoint = Pose()
            return
        distance = np.linalg.norm(np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]) -
                                  np.array([self.current_waypoint.position.x, self.current_waypoint.position.y, self.current_waypoint.position.z]))
        if distance < self.lower_distance_threshold and distance > self.upper_distance_threshold:
            self.current_waypoint = msg.pose
            return
        else:
            self.previous_waypoint = self.current_waypoint
            self.current_waypoint = msg.pose

    def right_gesture_callback(self, msg):
        if msg.data == "Thumb_Up":
            self.text_marker = self.create_text_marker("Paused")
            self.move_robot = True
            self.offset = self.current_waypoint
            if self.count == 0:
                self.initial_ee_pose = self.get_ee_pose()
                self.desired_ee_pose = self.get_ee_pose()
                phi = np.arctan2(self.desired_ee_pose.position.y, self.desired_ee_pose.position.x)
                quat = quaternion_from_euler(-np.pi, 0.0, 0.0)
                self.desired_ee_pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
                self.count += 1
        elif msg.data == "Thumb_Down":
            self.move_robot = False
            self.desired_ee_pose = self.get_ee_pose()
            phi = np.arctan2(self.desired_ee_pose.position.y, self.desired_ee_pose.position.x)
            quat = quaternion_from_euler(-np.pi, 0.0, phi)
            self.desired_ee_pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
            robot_move = PoseStamped()
            robot_move.header.frame_id = "panda_link0"
            robot_move.header.stamp = self.get_clock().now().to_msg()
            robot_move.pose.position.x = 0.0
            robot_move.pose.position.y = 0.0
            robot_move.pose.position.z = 0.001
            robot_move.pose.orientation.x = 1.0

            planpath_request = PlanPath.Request()
            planpath_request.waypoint = robot_move
            future = self.waypoint_client.call_async(planpath_request)

        elif msg.data == "Closed_Fist" and self.gripper_ready and self.gripper_status == "Open":
            self.text_marker = self.create_text_marker("Gripper Closed")
            self.gripper_ready = False
            self.gripper_status = "Closed"
            self.gripper_force_control = False
            self.gripper_force = 0.001
            grasp_goal = Grasp.Goal()
            grasp_goal.width = 0.01
            grasp_goal.speed = 0.1
            grasp_goal.epsilon.inner = 0.05
            grasp_goal.epsilon.outer = 0.05
            grasp_goal.force = self.gripper_force
            future = self.gripper_grasping_client.send_goal_async(grasp_goal, feedback_callback=self.feedback_callback)
            future.add_done_callback(self.grasp_response_callback)

        elif msg.data == "Open_Palm" and self.gripper_ready and self.gripper_status == "Closed":
            self.text_marker = self.create_text_marker("Tracking Right Hand")
            self.gripper_force = 3.0
            grasp_goal = Grasp.Goal()
            grasp_goal.width = 0.075
            grasp_goal.speed = 0.2
            grasp_goal.epsilon.inner = 0.001
            grasp_goal.epsilon.outer = 0.001
            grasp_goal.force = self.gripper_force
            future = self.gripper_grasping_client.send_goal_async(grasp_goal, feedback_callback=self.feedback_callback)
            future.add_done_callback(self.grasp_response_callback)
            self.gripper_ready = False
            self.gripper_status = "Open"
            self.gripper_force_control = False

        elif msg.data == "Pointing_Up" and self.gripper_ready and self.gripper_status == "Closed":
            self.gripper_force_control = True
            self.text_marker = self.create_text_marker("Increasing gripper force")

        if msg.data != "Thumb_Up":
            self.count = 0

    def grasp_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        self.gripper_ready = True

    def feedback_callback(self, feedback):
        self.get_logger().info(f"Feedback: {feedback}")

    async def home_gripper(self):
        await self.gripper_homing_client.send_goal_async(Homing.Goal(), feedback_callback=self.feedback_callback)
        self.gripper_homed = True

    async def timer_callback(self):
        # if not self.gripper_homed:
        #      await self.home_gripper()
        # self.text_marker_publisher.publish(self.text_marker)
        if self.move_robot:
            if self.gripper_ready and self.gripper_status == "Closed" and self.gripper_force_control:
                if self.gripper_force < self.max_gripper_force:
                    self.gripper_force += 1.0
                self.get_logger().info(f"Setting gripper force to: {self.gripper_force}")
                self.gripper_ready = False
                self.gripper_status = "Closed"
                grasp_goal = Grasp.Goal()
                grasp_goal.width = 0.01
                grasp_goal.speed = 0.1
                grasp_goal.epsilon.inner = 0.08
                grasp_goal.epsilon.outer = 0.08
                grasp_goal.force = self.gripper_force
                future = self.gripper_grasping_client.send_goal_async(grasp_goal, feedback_callback=self.feedback_callback)
                future.add_done_callback(self.grasp_response_callback)

            delta = Pose()
            delta.position.x = (self.current_waypoint.position.x - self.offset.position.x) / 1000 # convert to meters
            delta.position.y = (self.current_waypoint.position.y - self.offset.position.y) / 1000 # convert to meters
            delta.position.z = (self.current_waypoint.position.z - self.offset.position.z) / 1000 # convert to meters

            # Get the current and desired positions and orientations of the end-effector
            ee_pose = self.get_ee_pose()
            self.desired_ee_pose.position.x = delta.position.z + self.initial_ee_pose.position.x
            self.desired_ee_pose.position.y = delta.position.x + self.initial_ee_pose.position.y
            self.desired_ee_pose.position.z = -delta.position.y + self.initial_ee_pose.position.z
            current_euler = list(euler_from_quaternion([ee_pose.orientation.x, ee_pose.orientation.y, ee_pose.orientation.z, ee_pose.orientation.w]))
            desired_euler = list(euler_from_quaternion([self.desired_ee_pose.orientation.x, self.desired_ee_pose.orientation.y, self.desired_ee_pose.orientation.z, self.desired_ee_pose.orientation.w]))

            # Orientation PID loops
            if current_euler[0] < 0:
                current_euler[0] += 2 * np.pi
            if desired_euler[0] < 0:
                desired_euler[0] += 2 * np.pi
            roll_error = desired_euler[0] - current_euler[0]
            pitch_error = desired_euler[1] - current_euler[1]
            yaw_error = desired_euler[2] - current_euler[2]

            # self.get_logger().info(f"current_euler: {current_euler}")
            # self.get_logger().info(f"desired_euler: {desired_euler}")

            roll_derivative = (roll_error - self.roll_error_prior)
            pitch_derivative = (pitch_error - self.pitch_error_prior)
            yaw_derivative = (yaw_error - self.yaw_error_prior)

            roll_output = self.kp_angle * roll_error - self.kd_angle * roll_derivative
            pitch_output = self.kp_angle * pitch_error + self.kd_angle * pitch_derivative
            yaw_output = self.kp_angle * yaw_error + self.kd_angle * yaw_derivative

            euler_output = [roll_output, -pitch_output, -yaw_output]
            # euler_output = [0.0, 0.0, 0.0]

            # self.get_logger().info(f"roll_output: {roll_output}, pitch_output: {pitch_output}, yaw_output: {yaw_output}")

            self.roll_error_prior = roll_error
            self.pitch_error_prior = pitch_error
            self.yaw_error_prior = yaw_error

            # Position PID loop
            position_error = np.linalg.norm(np.array([self.desired_ee_pose.position.x, self.desired_ee_pose.position.y, self.desired_ee_pose.position.z]) -
                                   np.array([ee_pose.position.x, ee_pose.position.y, ee_pose.position.z]))

            derivative = (position_error - self.position_error_prior)
            output = self.kp * position_error + self.kd * derivative
            self.position_error_prior = position_error

            if output > self.max_output:
                output = self.max_output

            robot_move = PoseStamped()
            robot_move.header.frame_id = "panda_link0"
            robot_move.header.stamp = self.get_clock().now().to_msg()
            robot_move.pose.position.x = output * (self.desired_ee_pose.position.x - ee_pose.position.x)
            robot_move.pose.position.y = -output * (self.desired_ee_pose.position.y - ee_pose.position.y)
            robot_move.pose.position.z = -output * (self.desired_ee_pose.position.z - ee_pose.position.z)

            planpath_request = PlanPath.Request()
            planpath_request.waypoint = robot_move
            planpath_request.angles = euler_output
            future = self.waypoint_client.call_async(planpath_request)
        else:
            if self.desired_ee_pose is not None:
                # Get the current and desired positions and orientations of the end-effector
                ee_pose = self.get_ee_pose()
                current_euler = euler_from_quaternion([ee_pose.orientation.x, ee_pose.orientation.y, ee_pose.orientation.z, ee_pose.orientation.w])
                desired_euler = euler_from_quaternion([self.desired_ee_pose.orientation.x, self.desired_ee_pose.orientation.y, self.desired_ee_pose.orientation.z, self.desired_ee_pose.orientation.w])

                # Orientation PID loops
                if current_euler[0] < -np.pi:
                    current_euler[0] += 2 * np.pi
                if desired_euler[0] < -np.pi:
                    desired_euler[0] += 2 * np.pi
                roll_error = desired_euler[0] - current_euler[0]
                pitch_error = desired_euler[1] - current_euler[1]
                yaw_error = desired_euler[2] - current_euler[2]

                roll_derivative = (roll_error - self.roll_error_prior)
                pitch_derivative = (pitch_error - self.pitch_error_prior)
                yaw_derivative = (yaw_error - self.yaw_error_prior)

                self.roll_error_prior = roll_error
                self.pitch_error_prior = pitch_error
                self.yaw_error_prior = yaw_error

                roll_output = self.kp * roll_error + self.kd * roll_derivative
                pitch_output = self.kp * pitch_error + self.kd * pitch_derivative
                yaw_output = self.kp * yaw_error + self.kd * yaw_derivative

                euler_output = [roll_output, pitch_output, yaw_output]

                # Position PID loop
                ee_pose = self.get_ee_pose()
                error = np.linalg.norm(np.array([self.desired_ee_pose.position.x, self.desired_ee_pose.position.y, self.desired_ee_pose.position.z]) -
                                       np.array([ee_pose.position.x, ee_pose.position.y, ee_pose.position.z]))

                derivative = (error - self.position_error_prior)
                output = self.kp * error + self.kd * derivative
                self.position_error_prior = error

                if output > self.max_output:
                    output = self.max_output

                robot_move = PoseStamped()
                robot_move.header.frame_id = "panda_link0"
                robot_move.header.stamp = self.get_clock().now().to_msg()
                # robot_move.pose.position.x = output * (self.desired_ee_pose.position.x - ee_pose.position.x)
                # robot_move.pose.position.y = -output * (self.desired_ee_pose.position.y - ee_pose.position.y)
                # robot_move.pose.position.z = -output * (self.desired_ee_pose.position.z - ee_pose.position.z)
                robot_move_x = (self.desired_ee_pose.position.x - ee_pose.position.x)
                robot_move_y = -(self.desired_ee_pose.position.y - ee_pose.position.y)
                robot_move_z = -(self.desired_ee_pose.position.z - ee_pose.position.z)
                robot_move_norm = np.linalg.norm(np.array([robot_move_x, robot_move_y, robot_move_z]))
                robot_move.pose.position.x = robot_move_x/robot_move_norm * output
                robot_move.pose.position.y = robot_move_y/robot_move_norm * output
                robot_move.pose.position.z = robot_move_z/robot_move_norm * output
                self.get_logger().info(f"linear move: {np.linalg.norm(np.array([robot_move.pose.position.x, robot_move.pose.position.y, robot_move.pose.position.z]) - np.array([0.0, 0.0, 0.0]))}")

                planpath_request = PlanPath.Request()
                planpath_request.waypoint = robot_move
                planpath_request.angles = euler_output
                future = self.waypoint_client.call_async(planpath_request)

def main(args=None):
    rclpy.init(args=args)

    cv_franka_bridge = CvFrankaBridge()

    rclpy.spin(cv_franka_bridge)


if __name__ == '__main__':
    main()





















