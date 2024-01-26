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
namespace rvt = rviz_visual_tools;

/* blah blah blah cstring */

class FrankaTeleop : public rclcpp::Node
{
  public:
    FrankaTeleop() : Node("franka_teleop"),
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
      // instantiate RobotModelLoader object
      node_ptr_ = rclcpp::Node::shared_from_this();
      robot_model_loader::RobotModelLoader robot_model_loader(node_ptr_);
      const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
      RCLCPP_INFO(get_logger(), "Model frame: %s", kinematic_model->getModelFrame().c_str());

      // Construct a RobotState using RobotModel that maintains the configuration
      // of the robot
      moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(kinematic_model));
      robot_state->setToDefaultValues();
      const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("panda_arm");
      const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

      // Get joint values
      std::vector<double> joint_values;
      robot_state->copyJointGroupPositions(joint_model_group, joint_values);
      for (std::size_t i = 0; i < joint_names.size(); ++i)
      {
        RCLCPP_INFO(get_logger(), "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
      }

      // Get joint limits
      /* Set one joint in the Panda arm outside its joint limit */
      joint_values[0] = 5.57;
      robot_state->setJointGroupPositions(joint_model_group, joint_values);

      /* Check whether any joint is outside its joint limits */
      RCLCPP_INFO_STREAM(get_logger(), "Current state is " << (robot_state->satisfiesBounds() ? "valid" : "not valid"));

      /* Enforce the joint limits for this state and check again*/
      robot_state->enforceBounds();
      RCLCPP_INFO_STREAM(get_logger(), "Current state is " << (robot_state->satisfiesBounds() ? "valid" : "not valid"));

      // Get the Forward Kinematics
      robot_state->setToRandomPositions(joint_model_group);
      const Eigen::Isometry3d& end_effector_state = robot_state->getGlobalLinkTransform("panda_link8");

      /* Print end-effector pose. Remember that this is in the model frame */
      RCLCPP_INFO_STREAM(get_logger(), "Translation: \n" << end_effector_state.translation() << "\n");
      RCLCPP_INFO_STREAM(get_logger(), "Rotation: \n" << end_effector_state.rotation() << "\n");
      
      // Get the Inverse Kinematics
      double timeout = 0.1;
      bool found_ik = robot_state->setFromIK(joint_model_group, end_effector_state, timeout);
      // Now, we can print out the IK solution (if found):
      if (found_ik)
      {
        robot_state->copyJointGroupPositions(joint_model_group, joint_values);
        for (std::size_t i = 0; i < joint_names.size(); ++i)
        {
          RCLCPP_INFO(get_logger(), "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
        }
      }
      else
      {
        RCLCPP_INFO(get_logger(), "Did not find IK solution");
      }

      // Get the Jacobian
      Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
      Eigen::MatrixXd jacobian;
      robot_state->getJacobian(joint_model_group, robot_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                              reference_point_position, jacobian);
      RCLCPP_INFO_STREAM(get_logger(), "Jacobian: \n" << jacobian << "\n");
    }

  private:


    // void timer_callback()
    // {
      
    // }
    rclcpp::Node::SharedPtr node_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_;
    geometry_msgs::msg::Pose target_pose;
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