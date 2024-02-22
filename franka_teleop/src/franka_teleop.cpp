#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/logging.hpp>
#include <string>
#include <vector>
#include <eigen3/Eigen/Geometry>

#include "franka_teleop/srv/detail/plan_path__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "franka_teleop/srv/plan_path.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"

// load robot_model and robot_state
// #include <moveit/robot_model_loader/robot_model_loader.h>
// #include <moveit/robot_state/robot_state.h>
// #include <moveit/robot_state/conversions.h>

#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/moveit_cpp/moveit_cpp.h>

using namespace std::chrono_literals;
using std::placeholders::_1, std::placeholders::_2;

class FrankaTeleop : public rclcpp::Node
{
public:
  FrankaTeleop()
  : Node("franka_teleop", rclcpp::NodeOptions().
      automatically_declare_parameters_from_overrides(true))
  {

    // create services
    plan_path_service_ =
      create_service<franka_teleop::srv::PlanPath>(
      "plan_path",
      std::bind(&FrankaTeleop::plan_path_callback, this, _1, _2));

    execute_path_service = create_service<std_srvs::srv::Empty>(
      "execute_path",
      std::bind(&FrankaTeleop::execute_path_callback, this, _1, _2));

    plan_and_execute_path_service_ = create_service<franka_teleop::srv::PlanPath>(
      "plan_and_execute_path",
      std::bind(&FrankaTeleop::plan_and_execute_path_callback, this, _1, _2));

    ready_service_ = create_service<std_srvs::srv::Empty>(
      "ready",
      std::bind(&FrankaTeleop::ready_callback, this, _1, _2));

    get_ee_position_service_ = create_service<std_srvs::srv::Empty>(
      "get_ee_position",
      std::bind(&FrankaTeleop::get_ee_position_callback, this, _1, _2));

    // create ROS timer
    timer_ = create_wall_timer(
    500ms, std::bind(&FrankaTeleop::timer_callback, this));

  }

  void initialize_moveit()
  {
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
      planning_group,
      moveit_cpp_ptr_);
    robot_model_ptr_ = moveit_cpp_ptr_->getRobotModel();
    robot_start_state_ = planning_components_->getStartState();
    robot_current_state_ = moveit_cpp_ptr_->getCurrentState();
    joint_model_group_ptr_ = robot_model_ptr_->getJointModelGroup(planning_group);
  }

private:
  void get_ee_position_callback(
      const std::shared_ptr<std_srvs::srv::Empty::Request>,
      std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    // get the current state of the robot
    robot_current_state_ = moveit_cpp_ptr_->getCurrentState();
    // get the end effector position using FK
    Eigen::Isometry3d ee_pose = robot_current_state_->getGlobalLinkTransform("panda_hand_tcp");
    RCLCPP_INFO_STREAM(get_logger(), "End effector position: " << ee_pose.translation());
  }
  void ready_callback(
      const std::shared_ptr<std_srvs::srv::Empty::Request>,
      std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    planning_components_->setGoal("ready");
    
    auto plan_solution = planning_components_->plan();
    if(plan_solution)
    {
      moveit::core::RobotState robot_state(robot_model_ptr_);
      moveit::core::robotStateMsgToRobotState(plan_solution.start_state, robot_state);
      moveit_cpp_ptr_->execute(plan_solution.trajectory, CONTROLLERS);
    }
  }
  std::shared_ptr<franka_teleop::srv::PlanPath::Response> plan_and_execute_path_callback(
      const std::shared_ptr<franka_teleop::srv::PlanPath::Request> request,
      std::shared_ptr<franka_teleop::srv::PlanPath::Response> response)
  {
    planning_components_->setStartStateToCurrentState();

    // construct the goal of the plan using a PoseStamped message
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "panda_link0";
    target_pose = request->waypoint;
    RCLCPP_INFO_STREAM(get_logger(), "target pose: " << target_pose.pose.position.x << " " << target_pose.pose.position.y << " " << target_pose.pose.position.z << " " << target_pose.pose.orientation.x << " " << target_pose.pose.orientation.y << " " << target_pose.pose.orientation.z << " " << target_pose.pose.orientation.w);
    planning_components_->setGoal(target_pose, "panda_hand_tcp");

    // call PlanningComponents to compute the plan and visualize it
    plan_solution_ = planning_components_->plan();

    // check if PlanningComponents succeeded
    // robot_start_state->getGlobalLinkTransform() is performing FK
    // "start_pose" is the axis label
    if (plan_solution_) {
      // execute the plan
      moveit_controller_manager::ExecutionStatus result = moveit_cpp_ptr_->execute(
        plan_solution_.trajectory, CONTROLLERS);
      
      RCLCPP_INFO_STREAM(get_logger(), "Execution status: " << result.asString());

    }

    return response;
    
  }
  void execute_path_callback(
      const std::shared_ptr<std_srvs::srv::Empty::Request>,
      std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    // ### EXECUTION ###

    moveit_controller_manager::ExecutionStatus result = moveit_cpp_ptr_->execute(
      plan_solution_.trajectory, CONTROLLERS);
    
    RCLCPP_INFO_STREAM(get_logger(), "Execution status: " << result.asString());
  }

  void plan_path_callback(
      const std::shared_ptr<franka_teleop::srv::PlanPath::Request> request,
      std::shared_ptr<franka_teleop::srv::PlanPath::Response>)
  {
    // ### PLANNING ###
    // there are multiple ways to set the start and goals states of plans in moveit_cpp
    // example 1:

    // set the start state to the current state of the robot
    planning_components_->setStartStateToCurrentState();

    // construct the goal of the plan using a PoseStamped message
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "panda_link0";
    target_pose = request->waypoint;
    planning_components_->setGoal(target_pose, "panda_hand_tcp");

    // call PlanningComponents to compute the plan and visualize it
    plan_solution_ = planning_components_->plan();

    // check if PlanningComponents succeeded
    // robot_start_state->getGlobalLinkTransform() is performing FK
    // "start_pose" is the axis label
    if (plan_solution_) {
    }
  }

  void timer_callback()
  {
    // RCLCPP_INFO(get_logger(), "help me");
  }

  // rclcpp::Node::SharedPtr node_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<franka_teleop::srv::PlanPath>::SharedPtr plan_path_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr execute_path_service;
  rclcpp::Service<franka_teleop::srv::PlanPath>::SharedPtr plan_and_execute_path_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr ready_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr get_ee_position_service_;

  // franka variables
  std::vector<std::string> CONTROLLERS{1, "panda_arm_controller"};
  std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_ptr_;
  std::shared_ptr<moveit_cpp::PlanningComponent> planning_components_;
  Eigen::Isometry3d text_pose_;
  moveit::core::RobotModelConstPtr robot_model_ptr_;
  moveit::core::RobotStatePtr robot_start_state_;
  moveit::core::RobotStatePtr robot_current_state_;
  const moveit::core::JointModelGroup * joint_model_group_ptr_;
  planning_interface::MotionPlanResponse plan_solution_;
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
