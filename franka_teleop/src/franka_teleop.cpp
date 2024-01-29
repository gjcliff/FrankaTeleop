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
    // service_ = create_service<std_srvs::srv::Empty>("plan_path_service",
    // std::bind(&FrankaTeleop::plan_path_service, this, _1, _2));

    // create ROS timer
    // timer_ = create_wall_timer(
    // 500ms, std::bind(&FrankaTeleop::timer_callback, this));

  }

  void initialize_moveit()
  {
    node_ptr_ = rclcpp::Node::shared_from_this();

    static const rclcpp::Logger LOGGER = rclcpp::get_logger("franka_moveit_cpp");
    static const std::string planning_group = "panda_arm";
    static const std::string logname = "franka_moveit_cpp";

    // ros2_controllers
    static const std::vector<std::string> CONTROLLERS(1, "panda_arm_controller");

    // why am I doing this? I want to comment it out so bad
    rclcpp::sleep_for(std::chrono::seconds(1));

    // provide the planning scene service, rviz needs this
    // https://moveit.picknik.ai/main/doc/examples/planning_scene_monitor/planning_scene_monitor_tutorial.html
    auto moveit_cpp_ptr = std::make_shared<moveit_cpp::MoveItCpp>(
      node_ptr_);
    moveit_cpp_ptr->getPlanningSceneMonitorNonConst()->providePlanningSceneService();

    // finish initializing objects
    auto planning_components = std::make_shared<moveit_cpp::PlanningComponent>(
      "panda_arm",
      moveit_cpp_ptr);
    auto robot_model_ptr = moveit_cpp_ptr->getRobotModel();
    RCLCPP_INFO(LOGGER, "loaded robot model hi");
    auto robot_start_state = planning_components->getStartState();
    RCLCPP_INFO(LOGGER, "loaded planning components into start state hi");
    auto joint_model_group_ptr = robot_model_ptr->getJointModelGroup(planning_group);
    RCLCPP_INFO(LOGGER, "loaded jiont model group pointer");

    // setup rviz visualization tools
    // "franka_moveit_cpp" is the marker topic
    moveit_visual_tools::MoveItVisualTools visual_tools(node_ptr_, "panda_link0",
      "moveit_cpp_tutorial",
      moveit_cpp_ptr->getPlanningSceneMonitorNonConst());
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(
      text_pose, "thing", rviz_visual_tools::WHITE,
      rviz_visual_tools::XLARGE);
    visual_tools.trigger();

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    // ### PLANNING ###
    // there are multiple ways to set the start and goals states of plans in moveit_cpp
    // example 1:

    // set the start state to the current state of the robot
    planning_components->setStartStateToCurrentState();

    // construct the goal of the plan using a PoseStamped message
    geometry_msgs::msg::PoseStamped target_pose1;
    target_pose1.header.frame_id = "panda_link0";
    target_pose1.pose.orientation.w = 1.0;
    target_pose1.pose.position.x = 0.28;
    target_pose1.pose.position.y = -0.2;
    target_pose1.pose.position.z = 0.5;
    planning_components->setGoal(target_pose1, "panda_link8");

    // call PlanningComponents to compute the plan and visualize it
    const planning_interface::MotionPlanResponse plan_solution1 = planning_components->plan();

    // check if PlanningComponents succeeded
    // fun fact: robot_start_state->getGlobalLinkTransform() is performing FK
    // "start_pose" is the axis label
    if (plan_solution1) {
      // visualize the start pose in Rviz
      visual_tools.publishAxisLabeled(
        robot_start_state->getGlobalLinkTransform(
          "panda_link8"), "start_pose");

      // visualize the goal pose in Rviz
      visual_tools.publishAxisLabeled(target_pose1.pose, "target_pose");
      visual_tools.publishText(
        text_pose, "setStartStateToCurrentState", rviz_visual_tools::WHITE,
        rviz_visual_tools::XLARGE);

      // visualize the trajectory
      visual_tools.publishTrajectoryLine(plan_solution1.trajectory, joint_model_group_ptr);
      visual_tools.trigger();

      // uncomment the lines below if you want to execute the plan!
      // bool blocking = true;
      // moveit_controller_manager::ExecutionStatus result = moveit_cpp_ptr->execute(
      //   plan_solution1.trajectory, blocking, CONTROLLERS);

      visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
      visual_tools.deleteAllMarkers();
      visual_tools.trigger();
    }

    ////////////////////////////////////

    // instantiate RobotModelLoader object
    // node_ptr_ = rclcpp::Node::shared_from_this();
    // RCLCPP_INFO(get_logger(), "loaded node pointer");
    // robot_model_loader::RobotModelLoader robot_model_loader(node_ptr_);
    // RCLCPP_INFO(get_logger(), "loaded robot model");
    // const moveit::core::RobotModelPtr & kinematic_model = robot_model_loader.getModel();
    // RCLCPP_INFO(get_logger(), "Model frame: %s", kinematic_model->getModelFrame().c_str());

    // // Construct a RobotState using RobotModel that maintains the configuration
    // // of the robot
    // moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(kinematic_model));
    // robot_state->setToDefaultValues();
    // const moveit::core::JointModelGroup * joint_model_group = kinematic_model->getJointModelGroup(
    //   "panda_arm");
    // const std::vector<std::string> & joint_names = joint_model_group->getVariableNames();

    // // Get joint values
    // std::vector<double> joint_values;
    // robot_state->copyJointGroupPositions(joint_model_group, joint_values);
    // for (std::size_t i = 0; i < joint_names.size(); ++i) {
    //   RCLCPP_INFO(get_logger(), "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    // }

    // // Get joint limits
    // /* Set one joint in the Panda arm outside its joint limit */
    // joint_values[0] = 5.57;
    // robot_state->setJointGroupPositions(joint_model_group, joint_values);


    // /* Check whether any joint is outside its joint limits */
    // RCLCPP_INFO_STREAM(
    //   get_logger(),
    //   "Current state is " << (robot_state->satisfiesBounds() ? "valid" : "not valid"));

    // /* Enforce the joint limits for this state and check again*/
    // robot_state->enforceBounds();
    // RCLCPP_INFO_STREAM(
    //   get_logger(),
    //   "Current state is " << (robot_state->satisfiesBounds() ? "valid" : "not valid"));

    // // Get the Forward Kinematics
    // robot_state->setToRandomPositions(joint_model_group);
    // const Eigen::Isometry3d & end_effector_state =
    //   robot_state->getGlobalLinkTransform("panda_link8");

    // /* Print end-effector pose. Remember that this is in the model frame */
    // RCLCPP_INFO_STREAM(get_logger(), "Translation: \n" << end_effector_state.translation() << "\n");
    // RCLCPP_INFO_STREAM(get_logger(), "Rotation: \n" << end_effector_state.rotation() << "\n");

    // // Get the Inverse Kinematics
    // double timeout = 1.0;
    // bool found_ik = robot_state->setFromIK(joint_model_group, end_effector_state, timeout);
    // // Now, we can print out the IK solution (if found):
    // if (found_ik) {
    //   robot_state->copyJointGroupPositions(joint_model_group, joint_values);
    //   for (std::size_t i = 0; i < joint_names.size(); ++i) {
    //     RCLCPP_INFO(get_logger(), "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    //   }
    // } else {
    //   RCLCPP_INFO(get_logger(), "Did not find IK solution");
    // }

    // // Get the Jacobian
    // Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
    // Eigen::MatrixXd jacobian;
    // robot_state->getJacobian(
    //   joint_model_group, robot_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
    //   reference_point_position, jacobian);
    // RCLCPP_INFO_STREAM(get_logger(), "Jacobian: \n" << jacobian << "\n");
  }

private:
  // void timer_callback()
  // {

  // }
  rclcpp::Node::SharedPtr node_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_;
  size_t count_;
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
