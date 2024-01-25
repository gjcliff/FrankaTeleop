#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

#include "geometry_msgs/msg/pose.hpp"

#include "moveit/move_group_interface/move_group_interface.h"

using namespace std::chrono_literals;
using std::placeholders::_1, std::placeholders::_2;
using moveit::planning_interface::MoveGroupInterface;

/* blah blah blah cstring */

class FrankaTeleop : public rclcpp::Node, public moveit::planning_interface::MoveGroupInterface
{
  public:
    FrankaTeleop()
    : Node("franka_teleop", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
    move_group_interface_(MoveGroupInterface(rclcpp::Node::make_shared(this), "teleop")),
    count_(0)
    {
      // lambda function to create and initialize a pose msg and set it equal to target_pose
      move_group_interface_.setPoseTarget(target_pose);
      service_ = this->create_service<std_srvs::srv::Empty>("set_and_execute_pose", std::bind(&FrankaTeleop::set_target_pose, this, _1, _2));
      timer_ = this->create_wall_timer(
      500ms, std::bind(&FrankaTeleop::timer_callback, this));
    }

  private:

    void set_target_pose(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                         std::shared_ptr<std_srvs::srv::Empty::Response> response)
    {
      (void) request;
      (void) response;
      target_pose = []{
        geometry_msgs::msg::Pose msg;
        msg.orientation.w = 1.0;
        msg.position.x = 0.28;
        msg.position.y = -0.2;
        msg.position.z = 0.5;
        return msg;
        }();
        plan_target_pose();
    }

    void plan_target_pose()
    {
      auto const [success,plan] = [&]{
          moveit::planning_interface::MoveGroupInterface::Plan msg;
          auto const ok = static_cast<bool>(move_group_interface_.plan(msg));
          return std::make_pair(ok, msg);
        }();

      if (success) {
        execute_target_pose(plan);
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("question-mark"), "planning failed!");
      }
    }

    void execute_target_pose(moveit::planning_interface::MoveGroupInterface::Plan plan)
    {
      move_group_interface_.execute(plan);
    }

    void timer_callback()
    {
      
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_;
    // MoveGroupInterface(std::make_unique<FrankaTeleop>(), "teleop");
    // technically not what the tutorial is doing, but whatever. I can't use auto to define class members
    moveit::planning_interface::MoveGroupInterface move_group_interface_; 
    geometry_msgs::msg::Pose target_pose;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrankaTeleop>());
  rclcpp::shutdown();
  return 0;
}