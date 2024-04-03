"""
Interpret gestures from the user, and convert waypoints into robot motion.

This node interprets four gestures from a human user: thumbs up, thumbs down,
closed fist, and open palm. These gestures are used to control whether or not
the robot tracks the position of the users hand and to control the gripper.
Waypoints received are transformed into the robot base link's frame. Two
PD loops, one for position and one for orientation, are used to control the robot.

SUBSCRIBERS:
  + /waypoint (PoseStamped) - The 3D location of the hand's pose.
  + /right_gesture (String) - The gesture that the right hand is making.
PUBLISHERS:
  + /text_marker (Marker) - The text marker that is published to the RViz.
SERVICE CLIENTS:
  + /robot_waypoints (PlanPath) - The service that plans and executes the robot's
    motion.
ACTION CLIENTS:
  + /panda_gripper/homing (Homing) - The action server that homes the gripper.
  + /panda_gripper/grasp (Grasp) - The action server that controls the gripper.

"""
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from franka_teleop.srv import PlanPath

from visualization_msgs.msg import Marker

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

        # create publishers
        self.text_marker_publisher = self.create_publisher(Marker, 'text_marker', 10)
        self.bounding_box_publisher = self.create_publisher(Marker, 'bounding_box', 10)

        # create clients
        self.waypoint_client = self.create_client(PlanPath, 'robot_waypoints')
        self.waypoint_client.wait_for_service(timeout_sec=2.0)

        # create timer
        self.timer = self.create_timer(0.04, self.timer_callback)

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

        # create class variables
        self.text_marker = self.create_text_marker("Thumbs_up_to_begin_teleoperation")
        self.gripper_ready = True
        self.gripper_status = "Open"
        self.gripper_homed = False
        self.gripper_force_control = False
        self.gripper_force = 0.001
        self.max_gripper_force = 10.0

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

        # bounding box variables
        self.x_limits = [0.2, 0.6]
        self.y_limits = [-0.25, 0.25]
        self.z_limits = [0.15, 0.7]
        self.bounding_box_marker = self.create_box_marker()

        self.count = 0

    def create_text_marker(self, text):
        """Create a text marker."""
        marker = Marker()
        marker.header.frame_id = "panda_link0"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = marker.TEXT_VIEW_FACING
        marker.action = marker.ADD
        marker.text = text
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 1.0
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        return marker

    def create_box_marker(self):
        """Create a line strip that represents the bounding box."""
        marker = Marker()
        marker.header.frame_id = "panda_link0"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        marker.points = [
                Point(x=self.x_limits[0], y=self.y_limits[0], z=self.z_limits[0]),
                Point(x=self.x_limits[1], y=self.y_limits[0], z=self.z_limits[0]),
                Point(x=self.x_limits[1], y=self.y_limits[1], z=self.z_limits[0]),
                Point(x=self.x_limits[0], y=self.y_limits[1], z=self.z_limits[0]),
                Point(x=self.x_limits[0], y=self.y_limits[0], z=self.z_limits[0]),
                Point(x=self.x_limits[0], y=self.y_limits[0], z=self.z_limits[1]),
                Point(x=self.x_limits[1], y=self.y_limits[0], z=self.z_limits[1]),
                Point(x=self.x_limits[1], y=self.y_limits[1], z=self.z_limits[1]),
                Point(x=self.x_limits[0], y=self.y_limits[1], z=self.z_limits[1]),
                Point(x=self.x_limits[0], y=self.y_limits[0], z=self.z_limits[1]),
                Point(x=self.x_limits[1], y=self.y_limits[0], z=self.z_limits[1]),
                Point(x=self.x_limits[1], y=self.y_limits[0], z=self.z_limits[0]),
                Point(x=self.x_limits[1], y=self.y_limits[1], z=self.z_limits[0]),
                Point(x=self.x_limits[1], y=self.y_limits[1], z=self.z_limits[1]),
                Point(x=self.x_limits[0], y=self.y_limits[1], z=self.z_limits[1]),
                Point(x=self.x_limits[0], y=self.y_limits[1], z=self.z_limits[0])
                ]
        marker.scale.x = 0.01
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        return marker

    def gripper_homing_callback(self, request, response):
        """Callback for the gripper homing service."""
        goal = Homing.Goal()
        self.gripper_homing_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        return response

    def get_transform(self, target_frame, source_frame):
        """Get the transform between two frames."""
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
        """Get the current pose of the end-effector."""
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
        """Callback for the waypoint subscriber."""
        if self.current_waypoint is None:
            self.current_waypoint = msg.pose
            return
        distance = np.linalg.norm(np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]) -
                                  np.array([self.current_waypoint.position.x, self.current_waypoint.position.y, self.current_waypoint.position.z]))

        # filter out tiny movements to reduce jitter, and large errors from 
        # camera
        if distance < self.lower_distance_threshold and distance > self.upper_distance_threshold:
            # self.current_waypoint = msg.pose
            return
        else:
            self.current_waypoint = msg.pose

    def right_gesture_callback(self, msg):
        """
        Callback for the right gesture subscriber.

        The right gesture is used to control the robot's motion and the gripper.
        
        Args:
        ----
        msg (String): The gesture that the right hand is making.

        Returns:
        -------
        None

        """
        if msg.data == "Thumb_Up":
            # if thumbs up, start tracking the user's hand
            self.text_marker = self.create_text_marker("Thumbs_Up")
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
            # if thumbs down, stop tracking the user's hand
            self.text_marker = self.create_text_marker("Thumbs_Down")
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
            # if closed fist, close the gripper
            self.text_marker = self.create_text_marker("Closed_Fist")
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
            # if open palm, open the gripper
            self.text_marker = self.create_text_marker("Open_Palm")
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

        if msg.data != "Thumb_Up":
            self.count = 0

    def grasp_response_callback(self, future):
        """Callback for the grasp response."""

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Callback for the grasp result."""

        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        self.gripper_ready = True

    def feedback_callback(self, feedback):
        """Callback for the feedback from the gripper action server."""

        self.get_logger().info(f"Feedback: {feedback}")

    async def home_gripper(self):
        """Home the gripper."""

        await self.gripper_homing_client.send_goal_async(Homing.Goal(), feedback_callback=self.feedback_callback)
        self.gripper_homed = True

    async def timer_callback(self):
        """Callback for the timer."""
        # publish a text marker with the current gesture
        self.text_marker_publisher.publish(self.text_marker)
        self.bounding_box_publisher.publish(self.bounding_box_marker)
        if self.move_robot:
            # find the end-effector's position relative to the offset, which was
            # set the last time the user made a thumbs up gesture
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

            roll_derivative = (roll_error - self.roll_error_prior)
            pitch_derivative = (pitch_error - self.pitch_error_prior)
            yaw_derivative = (yaw_error - self.yaw_error_prior)

            roll_output = self.kp_angle * roll_error - self.kd_angle * roll_derivative
            pitch_output = self.kp_angle * pitch_error + self.kd_angle * pitch_derivative
            yaw_output = self.kp_angle * yaw_error + self.kd_angle * yaw_derivative

            euler_output = [roll_output, -pitch_output, -yaw_output]

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
            # even if the robot is not tracking the hand, we need to enforce
            # that it stays in the same place
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





















