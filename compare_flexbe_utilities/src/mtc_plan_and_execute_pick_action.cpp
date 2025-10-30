#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <moveit/task_constructor/task.h>

// stages
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>

// solvers
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <compare_flexbe_utilities/action/mtc_plan_and_execute_pick.hpp>

namespace mtc = moveit::task_constructor;
namespace stages = mtc::stages;
namespace solvers = mtc::solvers;

namespace compare_flexbe_utilities {

using PlanAndExecute = compare_flexbe_utilities::action::MTCPlanAndExecutePick;
using GoalHandle = rclcpp_action::ServerGoalHandle<PlanAndExecute>;
using moveit::task_constructor::Task;
namespace stages = moveit::task_constructor::stages;
namespace solvers = moveit::task_constructor::solvers;

class MtcPlanAndExecuteNode : public rclcpp::Node {
public:
  explicit MtcPlanAndExecuteNode(const rclcpp::NodeOptions& opts)
  : rclcpp::Node("mtc_plan_and_execute_node", opts)
  {
    // Separate callback group so we can run planning/execution without blocking other callbacks
    // cbg_exec_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // rclcpp_action::ServerOptions as_opts;
    // as_opts.callback_group = cbg_exec_;

    action_server_ = rclcpp_action::create_server<PlanAndExecute>(
      this,
      "mtc_plan_and_execute_pick",
      std::bind(&MtcPlanAndExecuteNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MtcPlanAndExecuteNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&MtcPlanAndExecuteNode::handle_accepted, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(), "MTC Plan+Execute action server ready.");
  }

private:
  // rclcpp::CallbackGroup::SharedPtr cbg_exec_;
  rclcpp_action::Server<PlanAndExecute>::SharedPtr action_server_;

  // ---- Action handlers ----
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&,
                                          std::shared_ptr<const PlanAndExecute::Goal> goal)
  {
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
  {
    RCLCPP_WARN(get_logger(), "Cancel requested.");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    // Run in a separate thread so the callback returns immediately (multi-threaded executor will pick it up)
    std::thread([this, goal_handle](){ this->execute_goal(goal_handle); }).detach();
  }

  // ---- Core execution ----
  void execute_goal(const std::shared_ptr<GoalHandle> goal_handle)
  {
    auto goal = goal_handle->get_goal();
    PlanAndExecute::Feedback feedback;
    PlanAndExecute::Result result;

    auto send_feedback = [&](const std::string& phase,
                         const std::string& stage, float prog){
      auto fb = std::make_shared<PlanAndExecute::Feedback>();
      fb->phase = phase;
      fb->stage_name = stage;
      fb->stage_progress = prog;
      goal_handle->publish_feedback(fb);
    };

    // Build task
    send_feedback("planning", "", 0.0f);

    Task task("pick_task");
    try {
      task.loadRobotModel(shared_from_this()); // allow parameters to flow in
    } catch (const std::exception& e) {
      result.success = false; result.error_code = 10;
      result.message = std::string("RobotModel load failed: ") + e.what();
      goal_handle->abort(std::make_shared<PlanAndExecute::Result>(result));
      return;
    }

    // Solvers
    auto ptp = std::make_shared<solvers::PipelinePlanner>(shared_from_this());
    auto cart = std::make_shared<solvers::CartesianPath>();
    auto hand = std::make_shared<solvers::JointInterpolationPlanner>();

    cart->setStepSize(goal->cart_step_size > 0.f ? goal->cart_step_size : 0.005);
    cart->setMaxVelocityScalingFactor(goal->vel_scale > 0.f ? goal->vel_scale : 0.6);
    cart->setMaxAccelerationScalingFactor(goal->acc_scale > 0.f ? goal->acc_scale : 0.6);

    const std::string arm_group   = goal->arm_group.empty()   ? "arm"   : goal->arm_group;
    const std::string hand_group  = goal->hand_group.empty()  ? "hand"  : goal->hand_group;
    const std::string eef_link    = goal->eef_link.empty()    ? "tool0" : goal->eef_link;
    const std::string ik_frame    = goal->ik_frame.empty()    ? "tool0" : goal->ik_frame;
    const std::string open_state  = goal->open_named_state.empty()  ? "open"  : goal->open_named_state;
    const std::string close_state = goal->close_named_state.empty() ? "close" : goal->close_named_state;

    task.setProperty("group", arm_group);
    task.setProperty("eef",   eef_link);
    task.setProperty("ik_frame", ik_frame);

    // Helper: wrap stages that need pose targets with ComputeIK
    auto IK = [&](const std::string& name,
                    std::unique_ptr<mtc::Stage> inner,
                    const geometry_msgs::msg::PoseStamped& target){
        auto ik = std::make_unique<stages::ComputeIK>(name, std::move(inner));
        ik->setProperty("group", arm_group);
        ik->setProperty("eef",   eef_link);
        ik->setProperty("ik_frame", ik_frame);
        ik->setTargetPose(target);
        return ik; // returns unique_ptr<Stage> (ComputeIK derives from Stage)
    };

    // Stages
    task.add(std::make_unique<stages::CurrentState>("current"));

    // to approach (PTP)
    {
      auto move = std::make_unique<stages::MoveTo>("to approach (ptp)", ptp);
      move->setProperty("group", arm_group);
      task.add(IK("ik(approach)", std::move(move), goal->approach));
    }

    // open gripper
    {
      auto open = std::make_unique<stages::MoveTo>("open gripper", hand);
      open->setProperty("group", hand_group);
      open->setGoal(open_state);
      task.add(std::move(open));
    }

    // approach -> grasp (LIN)
    {
      auto lin = std::make_unique<stages::MoveTo>("approach->grasp (lin)", cart);
      lin->setProperty("group", arm_group);
      task.add(IK("ik(grasp)", std::move(lin), goal->grasp));
    }

    // close gripper
    {
      auto close = std::make_unique<stages::MoveTo>("close gripper", hand);
      close->setProperty("group", hand_group);
      close->setGoal(close_state);
      task.add(std::move(close));
    }

    // attach object (if provided)
    if (!goal->object_id.empty()) {
      auto attach = std::make_unique<stages::ModifyPlanningScene>("attach");
      attach->attachObject(goal->object_id, eef_link);
      task.add(std::move(attach));
    }

    // grasp -> retreat (LIN)
    {
      auto lin_retreat = std::make_unique<stages::MoveTo>("grasp->retreat (lin)", cart);
      lin_retreat->setProperty("group", arm_group);
      task.add(IK("ik(retreat)", std::move(lin_retreat), goal->retreat));
    }

    // Enable MTC Introspection
    // task.enableIntrospection(shared_from_this(), rclcpp::Duration::from_seconds(0.3));
    task.enableIntrospection(true);

    // --- Plan ---
    send_feedback("planning", "", 0.33f);

    moveit::core::MoveItErrorCode plan_ec;
    try {
      plan_ec = task.plan(1);
    } catch (const std::exception& e) {
      result.success = false; result.error_code = 11;
      result.message = std::string("Planning threw: ") + e.what();
      goal_handle->abort(std::make_shared<PlanAndExecute::Result>(result));
      return;
    }
    if (plan_ec != moveit::core::MoveItErrorCode::SUCCESS || task.solutions().empty()) {
      result.success = false; result.error_code = 12;
      result.message = "Planning failed: no solutions.";
      goal_handle->abort(std::make_shared<PlanAndExecute::Result>(result));
      return;
    }

    // Allow cancel after planning, before motion
    if (goal_handle->is_canceling()) {
      result.success = false; result.error_code = 20;
      result.message = "Canceled before execution.";
      goal_handle->canceled(std::make_shared<PlanAndExecute::Result>(result));
      return;
    }

    // Execute (coarse feedback)
    send_feedback("executing", "starting", 0.0f);

    moveit::core::MoveItErrorCode exec_ec;
    try {
      auto solution = task.solutions().front();   // get first solution
      // Optional: publish a mid-progress tick
      send_feedback("executing", "running", 0.5f);
      exec_ec = task.execute(*solution);
    } catch (const std::exception& e) {
      result.success = false; result.error_code = 22;
      result.message = std::string("Execution threw: ") + e.what();
      goal_handle->abort(std::make_shared<PlanAndExecute::Result>(result));
      return;
    }

    if (exec_ec != moveit::core::MoveItErrorCode::SUCCESS) {
      result.success = false; result.error_code = 23;
      result.message = "Execution failed.";
      goal_handle->abort(std::make_shared<PlanAndExecute::Result>(result));
      return;
    }

    send_feedback("executing", "done", 1.0f);
    result.success = true; result.error_code = 0; result.message = "Plan+Execute succeeded.";
    goal_handle->succeed(std::make_shared<PlanAndExecute::Result>(result));
  }
};

} // namespace compare_flexbe_utilities

RCLCPP_COMPONENTS_REGISTER_NODE(compare_flexbe_utilities::MtcPlanAndExecuteNode)