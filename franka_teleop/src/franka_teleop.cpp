#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"

// load robot_model and robot_state
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

using namespace std::chrono_literals;
using std::placeholders::_1, std::placeholders::_2;


class FrankaTeleop : public rclcpp::Node
{
public:
  FrankaTeleop()
  : Node("franka_teleop", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
        true)),
    count_(0)
  {

    // moveit configuration


    // create ROS service
    service_ =
      create_service<std_srvs::srv::Empty>(
      "path_plan1",
      std::bind(&FrankaTeleop::path_plan1_service, this, _1, _2));

    test =
      create_service<std_srvs::srv::Empty>(
      "test_service",
      std::bind(&FrankaTeleop::test_service_callback, this, _1, _2));

    // create ROS timer
    // timer_ = create_wall_timer(
    // 500ms, std::bind(&FrankaTeleop::timer_callback, this));

    // this->initialize_moveit();
  }

  void initialize_moveit()
  {
    // node_ptr_ = rclcpp::Node::shared_from_this();
    moveit_cpp_ptr_ = std::make_shared<moveit_cpp::MoveItCpp>(
      shared_from_this());

    static const rclcpp::Logger LOGGER = rclcpp::get_logger("franka_moveit_cpp");
    static const std::string planning_group = "panda_arm";
    static const std::string logname = "franka_moveit_cpp";

    // ros2_controllers
    static const std::vector<std::string> CONTROLLERS(1, "panda_arm_controller");

    // why am I doing this? I want to comment it out so bad
    rclcpp::sleep_for(std::chrono::seconds(1));

    // provide the planning scene service, rviz needs this
    // https://moveit.picknik.ai/main/doc/examples/planning_scene_monitor/planning_scene_monitor_tutorial.html

    moveit_cpp_ptr_->getPlanningSceneMonitorNonConst()->providePlanningSceneService();

    // finish initializing objects
    planning_components_ = std::make_shared<moveit_cpp::PlanningComponent>(
      "panda_arm",
      moveit_cpp_ptr_);
    robot_model_ptr_ = moveit_cpp_ptr_->getRobotModel();
    RCLCPP_INFO(LOGGER, "loaded robot model hi");
    robot_start_state_ = planning_components_->getStartState();
    RCLCPP_INFO(LOGGER, "loaded planning components into start state hi");
    joint_model_group_ptr_ = robot_model_ptr_->getJointModelGroup(planning_group);
    RCLCPP_INFO(LOGGER, "loaded jiont model group pointer");

    planning_pipeline_ = std::make_unique<planning_pipeline::PlanningPipelinePtr>(
      new planning_pipeline::PlanningPipeline(robot_model_ptr_, shared_from_this(), "ompl"));

    // setup rviz visualization tools
    // "franka_moveit_cpp" is the marker topic
    visual_tools_ = std::make_unique<moveit_visual_tools::MoveItVisualTools>(
      shared_from_this(), "panda_link0",
      "moveit_cpp_tutorial",
      moveit_cpp_ptr_->getPlanningSceneMonitorNonConst());

    visual_tools_->deleteAllMarkers();
    visual_tools_->loadRemoteControl();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 0.5;
    visual_tools_->publishText(
      text_pose, "Po", rviz_visual_tools::WHITE,
      rviz_visual_tools::XLARGE);
    visual_tools_->trigger();

    visual_tools_->prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
  }

private:
  // void timer_callback()
  // {

  // }

  void test_service_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response)
  {
    RCLCPP_INFO(get_logger(), "help me");
  }


  void path_plan1_service(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response)
  {
    (void) request;
    (void) response;

    // ### PLANNING ###
    // there are multiple ways to set the start and goals states of plans in moveit_cpp
    // example 1:

    // set the start state to the current state of the robot
    planning_components_->setStartStateToCurrentState();

    // construct the goal of the plan using a PoseStamped message
    geometry_msgs::msg::PoseStamped target_pose1;
    target_pose1.header.frame_id = "panda_link0";
    target_pose1.pose.orientation.w = 1.0;
    target_pose1.pose.position.x = 0.28;
    target_pose1.pose.position.y = -0.2;
    target_pose1.pose.position.z = 0.5;
    planning_components_->setGoal(target_pose1, "panda_link8");

    // call PlanningComponents to compute the plan and visualize it
    const planning_interface::MotionPlanResponse plan_solution1 = planning_components_->plan();

    // check if PlanningComponents succeeded
    // robot_start_state->getGlobalLinkTransform() is performing FK
    // "start_pose" is the axis label
    if (plan_solution1) {
      // visualize the start pose in Rviz
      visual_tools_->publishAxisLabeled(
        robot_start_state_->getGlobalLinkTransform(
          "panda_link8"), "start_pose");

      // visualize the goal pose in Rviz
      visual_tools_->publishAxisLabeled(target_pose1.pose, "target_pose");
      visual_tools_->publishText(
        text_pose_, "setStartStateToCurrentState", rviz_visual_tools::WHITE,
        rviz_visual_tools::XLARGE);

      // visualize the trajectory
      visual_tools_->publishTrajectoryLine(plan_solution1.trajectory, joint_model_group_ptr_);
      visual_tools_->trigger();

      // uncomment the lines below if you want to execute the plan!
      // bool blocking = true;
      // moveit_controller_manager::ExecutionStatus result = moveit_cpp_ptr_->execute(
      //   plan_solution1.trajectory, blocking, CONTROLLERS);

      visual_tools_->prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
      visual_tools_->deleteAllMarkers();
      visual_tools_->trigger();
    }
  }
  // rclcpp::Node::SharedPtr node_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr test;
  size_t count_;

  // franka variables
  std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_ptr_;
  std::shared_ptr<moveit_cpp::PlanningComponent> planning_components_;
  std::unique_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
  Eigen::Isometry3d text_pose_;
  moveit::core::RobotModelConstPtr robot_model_ptr_;
  moveit::core::RobotStatePtr robot_start_state_;
  const moveit::core::JointModelGroup * joint_model_group_ptr_;
  std::unique_ptr<planning_pipeline::PlanningPipelinePtr> planning_pipeline_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto franka = std::make_shared<FrankaTeleop>();
  franka->initialize_moveit();
  rclcpp::spin(franka);
  rclcpp::shutdown();
  return 0;
}
