#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"

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
      
      PLANNING_GROUP_ = "panda_arm";
      LOGNAME_ = "moveit_cpp_panda";
      CONTROLLERS_ = {"panda_arm_controller"};

      // create ROS service
      service_ = create_service<std_srvs::srv::Empty>("plan_path_service",
      std::bind(&FrankaTeleop::plan_path_service, this, _1, _2));

      // create ROS timer
      // timer_ = create_wall_timer(
      // 500ms, std::bind(&FrankaTeleop::timer_callback, this));

    }

    void initialize_moveit()
    {
      node_ptr_ = rclcpp::Node::shared_from_this();
      moveit_cpp_ptr_ = std::make_shared<moveit_cpp::MoveItCpp>(node_ptr_);
      moveit_cpp_ptr_->getPlanningSceneMonitorNonConst()->providePlanningSceneService();
      planning_components_ = std::make_shared<moveit_cpp::PlanningComponent>(PLANNING_GROUP_, moveit_cpp_ptr_);
      robot_model_ptr_ = moveit_cpp_ptr_->getRobotModel();
      robot_start_state_ = planning_components_->getStartState();
      joint_model_group_ptr_ = robot_model_ptr_->getJointModelGroup(PLANNING_GROUP_);
    }

  private:
    
    void plan_path_service(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                            std::shared_ptr<std_srvs::srv::Empty::Response> response)
    {
      (void) request;
      (void) response;
      const geometry_msgs::msg::PoseStamped msg = [&]{
        geometry_msgs::msg::PoseStamped msg;
        msg.header.frame_id = "panda_link0";
        msg.pose.orientation.w = 1.0;
        msg.pose.position.x = 0.28;
        msg.pose.position.y = -0.2;
        msg.pose.position.z = 0.5;
        return msg;
      }();

      planning_components_->setGoal(msg, "panda_link8");
      const planning_interface::MotionPlanResponse plan_solution = planning_components_->plan();
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "i planned something!!");
    }

    // void timer_callback()
    // {
      
    // }
    rclcpp::Node::SharedPtr node_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_;
    geometry_msgs::msg::Pose target_pose;
    size_t count_;

    std::string PLANNING_GROUP_;
    std::string LOGNAME_;
    std::vector<std::string> CONTROLLERS_;

    std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_ptr_;
    std::shared_ptr<moveit_cpp::PlanningComponent> planning_components_;
    moveit::core::RobotModelConstPtr robot_model_ptr_;
    moveit::core::RobotStatePtr robot_start_state_;
    const moveit::core::JointModelGroup * joint_model_group_ptr_;
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