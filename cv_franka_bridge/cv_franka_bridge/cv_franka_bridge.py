from geometry_msgs.msg import Pose, PoseStamped, Point
from franka_teleop.srv import PlanPath

from std_srvs.srv import Empty

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import PoseStamped
import tf2_ros

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import numpy as np

class CvFrankaBridge(Node):

    def __init__(self):
        super().__init__('cv_franka_bridge')

        # create callback groups
        self.waypoint_callback_group = MutuallyExclusiveCallbackGroup()

        # create subscribers
        self.waypoint_subscriber = self.create_subscription(PoseStamped, 'waypoint', self.waypoint_callback, 10, callback_group=self.waypoint_callback_group)

        # create clients
        self.plan_and_execute_client = self.create_client(PlanPath, 'plan_and_execute_path')
        self.waypoint_client = self.create_client(PlanPath, 'robot_waypoints')

        # create services
        self.begin_teleoperation_service = self.create_service(Empty, 'begin_teleop', self.begin_teleoperation_callback)

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

        self.threshold = 3.0

        self.kp = 1.0
        self.ki = 0.01
        self.kd = 0.01
        self.integral_prior = 0
        self.error_prior = 0

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


    def begin_teleoperation_callback(self, request, response):
        self.move_robot = True
        self.desired_ee_pose = self.get_ee_pose()
        self.initial_ee_pose = self.get_ee_pose()
        self.offset = self.current_waypoint
        return response

    def waypoint_callback(self, msg):
        if self.current_waypoint is None or self.previous_waypoint is None:
            self.current_waypoint = msg.pose
            self.previous_waypoint = Pose()
            return
        distance = np.linalg.norm(np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]) -
                                  np.array([self.current_waypoint.position.x, self.current_waypoint.position.y, self.current_waypoint.position.z]))
        if distance > self.threshold:
            self.previous_waypoint = self.current_waypoint
            self.current_waypoint = msg.pose

    def timer_callback(self):
        if self.move_robot:
            delta = Pose()
            delta.position.x = (self.current_waypoint.position.x - self.offset.position.x) / 1000 # convert to meters
            delta.position.y = (self.current_waypoint.position.y - self.offset.position.y) / 1000 # convert to meters
            delta.position.z = (self.current_waypoint.position.z - self.offset.position.z) / 1000 # convert to meters
        
            # self.get_logger().info(f"delta: {delta.position.x}, {delta.position.y}, {delta.position.z}")

            ee_pose = self.get_ee_pose()
            self.desired_ee_pose.position.x = delta.position.z + self.initial_ee_pose.position.x
            self.desired_ee_pose.position.y = delta.position.x + self.initial_ee_pose.position.y
            self.desired_ee_pose.position.z = -delta.position.y + self.initial_ee_pose.position.z

            self.get_logger().info(f"ee_pose: {ee_pose.position.x}, {ee_pose.position.y}, {ee_pose.position.z}")
            self.get_logger().info(f"desired_ee_pose: {self.desired_ee_pose.position.x}, {self.desired_ee_pose.position.y}, {self.desired_ee_pose.position.z}")

            # PID loop
            error = np.linalg.norm(np.array([self.desired_ee_pose.position.x, self.desired_ee_pose.position.y, self.desired_ee_pose.position.z]) -
                                   np.array([ee_pose.position.x, ee_pose.position.y, ee_pose.position.z]))

            derivative = (error - self.error_prior)
            output = self.kp * error + self.kd * derivative
            self.error_prior = error

            # self.get_logger().info(f"output: {output}")
            # self.get_logger().info(f"error: {error}")

            robot_move = PoseStamped()
            robot_move.header.frame_id = "panda_link0"
            robot_move.header.stamp = self.get_clock().now().to_msg()
            robot_move.pose.position.x = output * (self.desired_ee_pose.position.x - ee_pose.position.x)
            robot_move.pose.position.y = -output * (self.desired_ee_pose.position.y - ee_pose.position.y)
            robot_move.pose.position.z = -output * (self.desired_ee_pose.position.z - ee_pose.position.z)
            robot_move.pose.orientation.x = 1.0

            self.get_logger().info(f"robot_move: {robot_move.pose.position.x}, {robot_move.pose.position.y}, {robot_move.pose.position.z}\n")

            planpath_request = PlanPath.Request()
            planpath_request.waypoint = robot_move
            future = self.waypoint_client.call_async(planpath_request)

def main(args=None):
    rclpy.init(args=args)

    cv_franka_bridge = CvFrankaBridge()

    rclpy.spin(cv_franka_bridge)


if __name__ == '__main__':
    main()





















