from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from franka_teleop.srv import PlanPath

from std_srvs.srv import Empty

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import PoseStamped
import tf2_ros

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class CvFrankaBridge(Node):

    def __init__(self):
        super().__init__('cv_franka_bridge')

        # create callback groups
        self.waypoint_callback_group = MutuallyExclusiveCallbackGroup()

        # create subscribers
        self.waypoint_subscriber = self.create_subscription(PoseStamped, 'waypoint', self.waypoint_callback, 10, callback_group=self.waypoint_callback_group)

        # create clients
        self.plan_and_execute_client = self.create_client(PlanPath, 'plan_and_execute_path')
        self.waypoint_client = self.create_client(PlanPath, 'teleop_service')

        # create services
        self.begin_teleoperation_service = self.create_service(Empty, 'begin_teleop', self.begin_teleoperation_callback)

        # create tf buffer and listener
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        # create class variables
        self.offset = Pose()
        self.current_waypoint = PoseStamped()
        self.waypoints = []
        self.move_robot = False
        self.start_time = self.get_clock().now()

    def get_transform(self, target_frame, source_frame):
        # i need to transform the points in the camera frame to points in the 
        # space frame of the robot. What I really want I think is that when the
        # robot first sees a specific hand gesture, it sets the transform at
        # that point in time. This will entail rotating the camera frame 180
        # degrees, i believe, and then setting the transform's translation
        # to where the hand's point is currently at. I could do this with a service
        # call, or by making some logic where when it sees a second hand it will
        # snag the current position of my hand and make the transform. I think
        # a service call is simpler for now.
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

    # def transform_waypoint(self, waypoint):

    def begin_teleoperation_callback(self, request, response):
        # set the pose offset to the hand's current pose
        self.offset = self.current_waypoint.pose
        self.move_robot = True

        # get the initial position of the end effector
        self.ee_home_pos, self.ee_home_rot = self.get_transform("panda_link0", "panda_hand_tcp")
        self.ee_home = PoseStamped()
        self.ee_home.pose.position.x = self.ee_home_pos.x
        self.ee_home.pose.position.y = self.ee_home_pos.y
        self.ee_home.pose.position.z = self.ee_home_pos.z
        self.ee_home.pose.orientation.x = self.ee_home_rot.x
        self.ee_home.pose.orientation.y = self.ee_home_rot.y
        self.ee_home.pose.orientation.z = self.ee_home_rot.z
        self.ee_home.pose.orientation.w = self.ee_home_rot.w

        return response

    def waypoint_callback(self, msg):
        if self.move_robot:
            time_now = self.get_clock().now() - self.start_time
            # self.get_logger().info(f"num of waypoints: {len(self.waypoints)}")
            # self.get_logger().info(f"time: {time_now.to_msg().sec}.{time_now.to_msg().nanosec}")
            # take the average of the last batch of waypoints
            waypoint = PoseStamped()
            waypoint.header.frame_id = "panda_link0"
            waypoint.header.stamp = self.get_clock().now().to_msg()

            waypoint.pose.position.x = msg.pose.position.x / 1000 # convert to meters
            waypoint.pose.position.y = msg.pose.position.y / 1000 # convert to meters
            waypoint.pose.position.z = msg.pose.position.z / 1000 # convert to meters
            waypoint.pose.orientation.x = msg.pose.orientation.x
            waypoint.pose.orientation.y = msg.pose.orientation.y
            waypoint.pose.orientation.z = msg.pose.orientation.z
            waypoint.pose.orientation.w = msg.pose.orientation.w

            # self.get_logger().info(f"waypoint: {waypoint.pose.position.x}, {waypoint.pose.position.y}, {waypoint.pose.position.z}")
            # self.get_logger().info(f"offset: {self.offset.position.x}, {self.offset.position.y}, {self.offset.position.z}")
            

            # subtract the offset from the waypoint to get the relative movement
            waypoint.pose.position.x -= self.offset.position.x / 1000
            waypoint.pose.position.y -= self.offset.position.y / 1000
            waypoint.pose.position.z -= self.offset.position.z / 1000
            waypoint.pose.orientation.x += (waypoint.pose.orientation.x - self.offset.orientation.x)
            waypoint.pose.orientation.y += (waypoint.pose.orientation.y - self.offset.orientation.y)
            waypoint.pose.orientation.z += (waypoint.pose.orientation.z - self.offset.orientation.z)
            waypoint.pose.orientation.w += (waypoint.pose.orientation.w - self.offset.orientation.w)

            # self.get_logger().info(f"post-offset waypoint: {waypoint.pose.position.x}, {waypoint.pose.position.y}, {waypoint.pose.position.z}")

            # figure out what the waypoint is in the robot's space frame
            robot_waypoint = PoseStamped()
            robot_waypoint.header.frame_id = "panda_link0"
            robot_waypoint.header.stamp = self.get_clock().now().to_msg()
            robot_waypoint.pose.position.x = waypoint.pose.position.z #+ self.ee_home.pose.position.x
            robot_waypoint.pose.position.y = -waypoint.pose.position.x #+ self.ee_home.pose.position.y
            robot_waypoint.pose.position.z = -waypoint.pose.position.y #+ self.ee_home.pose.position.z
            robot_waypoint.pose.orientation.x = 1.0
            robot_waypoint.pose.orientation.w = 0.0

            # orientation was calculated earlier when calculating the waypoint
            planpath_request = PlanPath.Request()
            planpath_request.waypoint = robot_waypoint
            future = self.waypoint_client.call_async(planpath_request)
        else:
            self.current_waypoint = msg



def main(args=None):
    rclpy.init(args=args)

    cv_franka_bridge = CvFrankaBridge()

    rclpy.spin(cv_franka_bridge)


if __name__ == '__main__':
    main()





















