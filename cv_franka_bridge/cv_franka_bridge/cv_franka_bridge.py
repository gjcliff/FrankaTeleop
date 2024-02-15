from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from franka_teleop.srv import PlanPath

from std_srvs.srv import Empty

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
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
        self.waypoint_subscriber = self.create_subscription(PoseStamped, 'hand_pose', self.waypoint_callback, 10, callback_group=self.waypoint_callback_group)

        # create clients
        self.plan_and_execute_client = self.create_client(PlanPath, 'plan_and_execute_path')

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
        self.ee_home = Pose()
        self.ee_home.position.x = 0.306891
        self.ee_home.position.y = 0.0
        self.ee_home.position.z = 0.486882
        self.ee_home.orientation.x = 1.0

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
            trans = self.buffer.lookup_transform(target_frame, source_frame, self.get_clock().now())
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
        return response

    async def waypoint_callback(self, msg):
        if self.move_robot:
            self.waypoints.append(msg.pose)
            if(len(self.waypoint) >= 6):
                self.get_logger().info("Received 6 waypoints")
                # take the average of the last batch of waypoints
                waypoint = PoseStamped()
                waypoint.header.frame_id = "panda_link0"
                waypoint.header.stamp = self.get_clock().now().to_msg()

                for i in range(len(self.waypoints)):
                    waypoint.pose.position.x += self.waypoints[i].position.x
                    waypoint.pose.position.y += self.waypoints[i].position.y
                    waypoint.pose.position.z += self.waypoints[i].position.z
                    waypoint.pose.orientation.x += self.waypoints[i].orientation.x
                    waypoint.pose.orientation.y += self.waypoints[i].orientation.y
                    waypoint.pose.orientation.z += self.waypoints[i].orientation.z
                    waypoint.pose.orientation.w += self.waypoints[i].orientation.w
                waypoint.pose.position.x /= len(self.waypoints)
                waypoint.pose.position.y /= len(self.waypoints)
                waypoint.pose.position.z /= len(self.waypoints)
                waypoint.pose.orientation.x /= len(self.waypoints)
                waypoint.pose.orientation.y /= len(self.waypoints)
                waypoint.pose.orientation.z /= len(self.waypoints)
                waypoint.pose.orientation.w /= len(self.waypoints)

                # subtract the offset from the waypoint
                waypoint.pose.position.x -= self.offset.position.x
                waypoint.pose.position.y -= self.offset.position.y
                waypoint.pose.position.z -= self.offset.position.z
                waypoint.pose.orientation.x -= self.offset.orientation.x
                waypoint.pose.orientation.y -= self.offset.orientation.y
                waypoint.pose.orientation.z -= self.offset.orientation.z
                waypoint.pose.orientation.w -= self.offset.orientation.w

                # figure out what the waypoint is in the robot's space frame
                robot_waypoint = PoseStamped()
                robot_waypoint = waypoint
                robot_waypoint.pose.position.x += self.ee_home.position.x
                robot_waypoint.pose.position.y += self.ee_home.position.y
                robot_waypoint.pose.position.z += self.ee_home.position.z
                # orientation was calculated earlier when calculating the waypoint
                        
                # send the waypoint to the robot to execute
                planpath_request = PlanPath.Request()
                planpath_request.waypoint = waypoint
                await self.plan_and_execute_client.call_async(planpath_request)
            else:
                self.current_waypoint = msg



def main(args=None):
    rclpy.init(args=args)

    cv_franka_bridge = CvFrankaBridge()

    rclpy.spin(cv_franka_bridge)


if __name__ == '__main__':
    main()





















