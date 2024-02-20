// load robot_model and robot_state
#include <Eigen/Geometry>
#include <atomic>
#include "rclcpp/rclcpp.hpp"
#include <moveit_servo/moveit_servo/servo.hpp>
#include <moveit_servo_lib_parameters/moveit_servo_lib_parameters.hpp>
#include <moveit_servo/moveit_servo/utils/common.hpp>
#include <mutex>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>

using std::placeholders::_1, std::placeholders::_2;
using namespace moveit_servo;

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  // The servo object expects to get a ROS node.
  const rclcpp::Node::SharedPtr demo_node = std::make_shared<rclcpp::Node>("franka_servo");

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

  // Wait for some time, so that the planning scene is loaded in rviz.
  // This is just for convenience, should not be used for sync in real application.
  // std::this_thread::sleep_for(std::chrono::seconds(3));

  // For syncing pose tracking thread and main thread.
  std::mutex pose_guard;
  std::atomic<bool> stop_tracking = false;

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
    while (!stop_tracking && rclcpp::ok())
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

  // Pose tracking thread will exit upon reaching this pose.
  Eigen::Isometry3d terminal_pose = target_pose.pose;
  terminal_pose.rotate(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()));
  terminal_pose.translate(Eigen::Vector3d(0.0, 0.0, -0.1));

  std::thread tracker_thread(pose_tracker);
  tracker_thread.detach();

  // The target pose (frame being tracked) moves by this step size each iteration.
  Eigen::Vector3d linear_step_size{ 0.0, 0.0, -0.002 };
  Eigen::AngleAxisd angular_step_size(0.01, Eigen::Vector3d::UnitZ());

  // Frequency at which commands will be sent to the robot controller.
  rclcpp::WallRate command_rate(50);
  RCLCPP_INFO_STREAM(demo_node->get_logger(), servo.getStatusMessage());

  while (rclcpp::ok())
  {
    {
      std::lock_guard<std::mutex> pguard(pose_guard);
      target_pose.pose = servo.getEndEffectorPose();
      const bool satisfies_linear_tolerance = target_pose.pose.translation().isApprox(
          terminal_pose.translation(), servo_params.pose_tracking.linear_tolerance);
      const bool satisfies_angular_tolerance =
          target_pose.pose.rotation().isApprox(terminal_pose.rotation(), servo_params.pose_tracking.angular_tolerance);
      stop_tracking = satisfies_linear_tolerance && satisfies_angular_tolerance;
      // Dynamically update the target pose.
      if (!satisfies_linear_tolerance)
        target_pose.pose.translate(linear_step_size);
      if (!satisfies_angular_tolerance)
        target_pose.pose.rotate(angular_step_size);
    }

    command_rate.sleep();
  }

  RCLCPP_INFO_STREAM(demo_node->get_logger(), "REACHED : " << stop_tracking);
  stop_tracking = true;

  if (tracker_thread.joinable())
    tracker_thread.join();

  RCLCPP_INFO(demo_node->get_logger(), "Exiting demo.");
  rclcpp::shutdown();
}

