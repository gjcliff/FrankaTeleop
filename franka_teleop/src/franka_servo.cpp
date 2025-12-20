/// \file
/// \brief Implement moveit servo for the franka robot. This node receives an
/// incremental amount to move linearly and rotationally around the x, y, and
/// z axis and commands motion from the robot.
///
/// SERVERS:
///     robot_waypoints (franka_teleop::srv::PlanPath): Set the current waypoint
///     for the robot incrementally
#include <Eigen/Geometry>
#include <chrono>
#include <deque>
#include <franka_teleop/srv/plan_path.hpp>
#include <moveit_servo/servo.hpp>
#include <moveit_servo/utils/common.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

using namespace moveit_servo;

namespace {
constexpr auto K_BASE_FRAME = "panda_link0";
constexpr auto K_TIP_FRAME = "panda_link8";
} // namespace

class FrankaServoNode {
public:
  FrankaServoNode(const rclcpp::Node::SharedPtr &node) : node_(node) {
    using std::placeholders::_1;
    using std::placeholders::_2;

    node_->declare_parameter("use_fake_hardware", true);
    bool use_fake_hardware =
        node_->get_parameter("use_fake_hardware").as_bool();

    moveit::setNodeLoggerName(node_->get_name());

    const std::string param_ns = "moveit_servo";
    servo_param_listener_ =
        std::make_shared<const servo::ParamListener>(node_, param_ns);
    servo_params_ = servo_param_listener_->get_params();

    trajectory_pub_ =
        node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            servo_params_.command_out_topic, rclcpp::SystemDefaultsQoS());

    planning_scene_monitor_ = createPlanningSceneMonitor(node_, servo_params_);
    servo_ = std::make_unique<Servo>(node_, servo_param_listener_,
                                     planning_scene_monitor_);

    waypoint_srv_ = node_->create_service<franka_teleop::srv::PlanPath>(
        "robot_waypoints",
        std::bind(&FrankaServoNode::waypointCallback, this, _1, _2));

    if (use_fake_hardware) {
      std::this_thread::sleep_for(std::chrono::seconds(3));
    }

    robot_state_ =
        planning_scene_monitor_->getStateMonitor()->getCurrentState();
    joint_model_group_ =
        robot_state_->getJointModelGroup(servo_params_.move_group_name);

    servo_->setCommandType(CommandType::POSE);
    target_pose_.frame_id = K_BASE_FRAME;
    target_pose_.pose = robot_state_->getGlobalLinkTransform(K_TIP_FRAME);

    loop();
  }

private:
  void waypointCallback(
      const std::shared_ptr<franka_teleop::srv::PlanPath::Request> req,
      std::shared_ptr<franka_teleop::srv::PlanPath::Response>) {
    linear_step_ = Eigen::Vector3d(req->waypoint.pose.position.x,
                                   req->waypoint.pose.position.y,
                                   req->waypoint.pose.position.z);

    x_step_ = Eigen::AngleAxisd(req->angles[0], Eigen::Vector3d::UnitX());
    y_step_ = Eigen::AngleAxisd(req->angles[1], Eigen::Vector3d::UnitY());
    z_step_ = Eigen::AngleAxisd(req->angles[2], Eigen::Vector3d::UnitZ());
  }

  Eigen::Isometry3d
  get_current_pose(const std::string &target_frame,
                   const moveit::core::RobotStatePtr &robot_state) {
    return robot_state->getGlobalLinkTransform(target_frame);
  }

  void loop() {
    rclcpp::WallRate servo_rate(1 / servo_params_.publish_period);
    std::deque<KinematicState> joint_cmd_window;
    KinematicState current_state = servo_->getCurrentRobotState(true);
    updateSlidingWindow(current_state, joint_cmd_window,
                        servo_params_.max_expected_latency, node_->now());

    RCLCPP_INFO_STREAM(node_->get_logger(), servo_->getStatusMessage());

    while (rclcpp::ok()) {
      target_pose_.pose = get_current_pose(K_TIP_FRAME, robot_state_);
      target_pose_.pose.translate(linear_step_);
      target_pose_.pose.rotate(x_step_);
      target_pose_.pose.rotate(y_step_);
      target_pose_.pose.rotate(z_step_);

      KinematicState joint_state =
          servo_->getNextJointState(robot_state_, target_pose_);
      StatusCode status = servo_->getStatus();

      if (status != StatusCode::INVALID) {
        updateSlidingWindow(joint_state, joint_cmd_window,
                            servo_params_.max_expected_latency, node_->now());
        if (const auto msg =
                composeTrajectoryMessage(servo_params_, joint_cmd_window)) {
          trajectory_pub_->publish(msg.value());
        }

        if (!joint_cmd_window.empty()) {
          robot_state_->setJointGroupPositions(
              joint_model_group_, joint_cmd_window.back().positions);
          robot_state_->setJointGroupVelocities(
              joint_model_group_, joint_cmd_window.back().velocities);
        }
      }

      rclcpp::spin_some(node_);
      servo_rate.sleep();
    }

    RCLCPP_INFO(node_->get_logger(), "exiting servo loop");
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<const servo::ParamListener> servo_param_listener_;
  servo::Params servo_params_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  std::unique_ptr<Servo> servo_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
      trajectory_pub_;
  rclcpp::Service<franka_teleop::srv::PlanPath>::SharedPtr waypoint_srv_;

  Eigen::Vector3d linear_step_{0.0, 0.0, 0.0};
  Eigen::AngleAxisd x_step_{0.0, Eigen::Vector3d::UnitX()};
  Eigen::AngleAxisd y_step_{0.0, Eigen::Vector3d::UnitY()};
  Eigen::AngleAxisd z_step_{0.0, Eigen::Vector3d::UnitZ()};

  PoseCommand target_pose_;
  moveit::core::RobotStatePtr robot_state_;
  const moveit::core::JointModelGroup *joint_model_group_{nullptr};
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("franka_servo");
  FrankaServoNode servo_node(node);
  rclcpp::shutdown();
  return 0;
}
