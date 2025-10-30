#include <rclcpp/rclcpp.hpp>
#include "compare_flexbe_utilities/mtc_plan_and_execute_pick_action.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto opts = rclcpp::NodeOptions()
                .allow_undeclared_parameters(true)
                .automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<compare_flexbe_utilities::MtcPlanAndExecuteNode>(opts);

  // Multi-threaded so action goal thread, feedback, and other callbacks don’t block each other
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}