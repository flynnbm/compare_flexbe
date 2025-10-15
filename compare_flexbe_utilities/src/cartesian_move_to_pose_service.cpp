#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include "compare_flexbe_utilities/srv/cartesian_move_to_pose.hpp"

class CartesianPathPlanner : public rclcpp::Node
{
public:
  CartesianPathPlanner()
    : Node("cartesian_path_planner")
  {
    this->declare_parameter<std::string>("planning_group", "arm");
    this->declare_parameter<double>("eef_step", 0.005);

    std::string group_name;
    this->get_parameter("planning_group", group_name);

    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){}), group_name);

    service_ = this->create_service<compare_flexbe_utilities::srv::CartesianMoveToPose>(
      "plan_cartesian_path",
      std::bind(&CartesianPathPlanner::plan_callback, this, std::placeholders::_1, std::placeholders::_2)
    );

    RCLCPP_INFO(this->get_logger(), "Cartesian path planning service ready.");
  }

private:
  rclcpp::Service<compare_flexbe_utilities::srv::CartesianMoveToPose>::SharedPtr service_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  void plan_callback(
    const std::shared_ptr<compare_flexbe_utilities::srv::CartesianMoveToPose::Request> req,
    std::shared_ptr<compare_flexbe_utilities::srv::CartesianMoveToPose::Response> res)
  {
    // double jump_threshold = this->get_parameter("jump_threshold").as_double();
    double eef_step = this->get_parameter("eef_step").as_double();
    // move_group_interface_->setPoseReferenceFrame("panda_link0");
    // move_group_interface_->setStartStateToCurrentState();

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(move_group_interface_->getCurrentPose().pose);
    waypoints.insert(waypoints.end(), req->waypoints.begin(), req->waypoints.end());

    // debugging messages
    // RCLCPP_INFO(this->get_logger(), "Planning frame: %s | EEF: %s | Group: %s",
    //           move_group_interface_->getPlanningFrame().c_str(),
    //           move_group_interface_->getEndEffectorLink().c_str(),
    //           move_group_interface_->getName().c_str());

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_interface_->computeCartesianPath(
      waypoints, eef_step, trajectory);

    res->percentage_planned = static_cast<float>(fraction * 100.0f);
    res->success = false;

    if (fraction > 0.9)
    {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory = trajectory;

      auto result = move_group_interface_->execute(plan);
      if (result == moveit::core::MoveItErrorCode::SUCCESS)
      {
        res->success = true;
      }
    }

    RCLCPP_INFO(this->get_logger(),
      "Cartesian plan: %.2f%% success | Executed: %s",
      res->percentage_planned,
      res->success ? "true" : "false");
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CartesianPathPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
