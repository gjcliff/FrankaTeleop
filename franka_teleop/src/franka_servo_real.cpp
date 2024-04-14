// load robot_model and robot_state
#include <Eigen/Geometry>
#include <Eigen/src/Geometry/Quaternion.h>
#include <Eigen/src/Geometry/Transform.h>
#include <atomic>
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <moveit_servo/servo.hpp>
#include <moveit_servo/utils/common.hpp>
#include <mutex>
#include <std_srvs/srv/empty.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <franka_teleop/srv/plan_path.hpp>

using std::placeholders::_1, std::placeholders::_2;
using namespace moveit_servo;

static Eigen::Vector3d linear_step_size{0.00, 0.00, 0.00};
static Eigen::AngleAxisd x_step_size(0.00, Eigen::Vector3d::UnitX());
static Eigen::AngleAxisd y_step_size(0.00, Eigen::Vector3d::UnitY());
static Eigen::AngleAxisd z_step_size(0.00, Eigen::Vector3d::UnitZ());
bool move_robot = false;

/// \brief Callback for the robot_waypoints_service. This service sets the
/// global variables for linear and rotational step size.
void waypoint_callback(const std::shared_ptr<franka_teleop::srv::PlanPath::Request> request,
                       std::shared_ptr<franka_teleop::srv::PlanPath::Response>)
{
  linear_step_size = Eigen::Vector3d{
    request->waypoint.pose.position.x,
    request->waypoint.pose.position.y,
    request->waypoint.pose.position.z};
  // RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "linear_step_size: \n" << linear_step_size);
  x_step_size = Eigen::AngleAxisd(request->angles[0], Eigen::Vector3d::UnitX());
  y_step_size = Eigen::AngleAxisd(request->angles[1], Eigen::Vector3d::UnitY());
  z_step_size = Eigen::AngleAxisd(request->angles[2], Eigen::Vector3d::UnitZ());
}

int main(int argc, char* argv[])
{
  std::this_thread::sleep_for(std::chrono::milliseconds(750));
  rclcpp::init(argc, argv);

  // The servo object expects to get a ROS node.
  const rclcpp::Node::SharedPtr demo_node = std::make_shared<rclcpp::Node>("franka_servo");

  // create services
  rclcpp::Service<franka_teleop::srv::PlanPath>::SharedPtr service = demo_node->create_service<franka_teleop::srv::PlanPath>(
      "robot_waypoints", &waypoint_callback);

  // Get the servo parameters.
  const std::string param_namespace = "moveit_servo";
  const std::shared_ptr<const servo::ParamListener> servo_param_listener =
      std::make_shared<const servo::ParamListener>(demo_node, param_namespace);
  const servo::Params servo_params = servo_param_listener->get_params();

  // The publisher to send trajectory message to the robot controller.
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_outgoing_cmd_pub =
      demo_node->create_publisher<trajectory_msgs::msg::JointTrajectory>(servo_params.command_out_topic,
                                                                         rclcpp::SystemDefaultsQoS());
  // Create the servo object
  const planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor =
      createPlanningSceneMonitor(demo_node, servo_params);
  Servo servo = Servo(demo_node, servo_param_listener, planning_scene_monitor);

  // For syncing pose tracking thread and main thread.
  std::mutex pose_guard;

  // Set the command type for servo.
  servo.setCommandType(CommandType::POSE);

  // The dynamically updated target pose.
  PoseCommand target_pose;
  target_pose.frame_id = servo_params.planning_frame;
  // Initializing the target pose as end effector pose, this can be any pose.
  target_pose.pose = servo.getEndEffectorPose();

  // The pose tracking lambda that will be run in a separate thread.
  auto pose_tracker = [&]() {
    KinematicState joint_state;
    rclcpp::WallRate tracking_rate(1 / servo_params.publish_period);
    while (rclcpp::ok())
    {
      {
        std::lock_guard<std::mutex> pguard(pose_guard);
        joint_state = servo.getNextJointState(target_pose);
      }
      StatusCode status = servo.getStatus();
      if (status != StatusCode::INVALID)
        trajectory_outgoing_cmd_pub->publish(composeTrajectoryMessage(servo_params, joint_state));

      tracking_rate.sleep();
    }
  };

  std::thread tracker_thread(pose_tracker);
  tracker_thread.detach();

  // Frequency at which commands will be sent to the robot controller.
  rclcpp::WallRate command_rate(50);
  RCLCPP_INFO_STREAM(demo_node->get_logger(), servo.getStatusMessage());

  while (rclcpp::ok())
  {
    {
      std::lock_guard<std::mutex> pguard(pose_guard);
      target_pose.pose = servo.getEndEffectorPose();
      target_pose.pose.translate(linear_step_size);
      target_pose.pose.rotate(x_step_size);
      target_pose.pose.rotate(y_step_size);
      target_pose.pose.rotate(z_step_size);
      rclcpp::spin_some(demo_node);
    }
    command_rate.sleep();
  }

  if (tracker_thread.joinable())
    tracker_thread.join();

  rclcpp::shutdown();
}
