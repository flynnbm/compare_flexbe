#!/usr/bin/env python3
# Copyright ...
# License: Apache 2.0 (same header as your other files)

from typing import Optional

import rclpy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.clock import Clock

from compare_flexbe_utilities.action import MTCPlanAndExecutePick


class MTCPlanAndExecutePickActionState(EventState):
    """
    MTC - add detail.

    -- timeout_sec      float                           Seconds to wait for action server discovery (default: 5.0)
    -- action_name      str                             action server name

    ># approach         geometry_msgs/PoseStamped       Stamped pose for approach
    ># grasp            geometry_msgs/PoseStamped       Stamped pose for grasp
    ># retreat          geometry_msgs/PoseStamped       Stamped pose for retreat
    ># object_id        string                          Scene object_id (simulation)

    <= success                                          The action reported success
    <= next                                             The pose planning failed, attempt next set of goals
    <= failed                                           Any failure (no server, reject, error, stalled+no goal reached)
    """

    def __init__(self, 
                 timeout_sec: float = 5.0,
                 action_name = 'mtc_plan_and_execute_pick'):
        super().__init__(
            outcomes=['success', 'next', 'failed'],
            input_keys=['grasp_approach_poses', 'grasp_target_poses', 'grasp_retreat_poses', 'object_id', 'grasp_index'],
            output_keys=['grasp_index'],
            #     # optional overrides:
            #     'arm_group', 'hand_group', 'eef_link', 'ik_frame',
            #     'open_named_state', 'close_named_state',
            #     'cart_step_size', 'vel_scale', 'acc_scale'
            # ],
        )

        # --- configuration / parameters ---
        self._timeout_sec = float(timeout_sec)
        self._action_name = action_name

        # --- internal state ---
        self._ac: Optional[ProxyActionClient] = None
        self._had_error = False
        self._sent_goal = False

        # Create proxy action client in on_enter once we know the action name.
        self._ac = None

    def execute(self, userdata):
        # Execute this method periodically while the state is active.
        # Main purpose is to check state conditions and trigger a corresponding outcome.
        # If no outcome is returned, the state will stay active.

        # Check for error or no response
        if self._had_error or not self._sent_goal:
            return 'failed'

        # If we don’t have a result yet, keep waiting.
        if not self._ac.has_result(self._action_name):
            return None

        # NOTE: ProxyActionClient.get_result() returns the *Result message* directly (not a wrapper)
        try:
            r = self._ac.get_result(self._action_name)  # type: MTCPlanAndExecutePick.Result
        except Exception as e:
            Logger.logerr(f"[{type(self).__name__}] Failed to get result: {e}")
            return 'failed'

        if r is None:
            Logger.logerr(f"[{type(self).__name__}] Empty result from action.")
            return 'failed'

        # Success path
        if getattr(r, 'success', False):
            Logger.loginfo(f"[{type(self).__name__}] Success: "
                        f"{getattr(r, 'message', '(no message)')} "
                        f"(code={getattr(r, 'error_code', -1)})")
            return 'success'

        # Failure path: log and decide whether to iterate
        Logger.logerr(f"[{type(self).__name__}] Failed: "
                    f"{getattr(r, 'message', '(no message)')} "
                    f"(code={getattr(r, 'error_code', -1)})")

        # Try to advance to next candidate if any remain
        try:
            n = min(len(userdata.grasp_approach_poses), len(userdata.grasp_target_poses), len(userdata.grasp_retreat_poses))
            if isinstance(userdata.grasp_index, int) and (userdata.grasp_index + 1) < n:
                userdata.grasp_index += 1
                return 'next'
            else:
                return 'failed'
        except Exception as e:
            Logger.logwarn(f"[{type(self).__name__}] Could not compute remaining candidates ({e}); not iterating.")
            return 'failed'
    
    def on_enter(self, userdata):
        # Call this method a single time when the state becomes active, when a transition from another state to this one is taken.
        # It is primarily used to start actions which are associated with this state.
        
        # reset state
        self._had_error = False
        self._sent_goal = False

        grasp_approach_poses    = userdata.grasp_approach_poses
        grasp_target_poses      = userdata.grasp_target_poses
        grasp_retreat_poses     = userdata.grasp_retreat_poses
        idx                     = int(userdata.grasp_index)

        # Construct goal
        try:
            goal = MTCPlanAndExecutePick.Goal()
            goal.approach = grasp_approach_poses[idx]
            goal.grasp    = grasp_target_poses[idx]
            goal.retreat  = grasp_retreat_poses[idx]
            goal.object_id = getattr(userdata, 'object_id', '') or ''
        except Exception as e:
            Logger.logerr(f"[{type(self).__name__}] Failed to construct goal: {e}")
            self._had_error = True
            return

        # Create / register proxy client for this action
        try:
            self._ac = ProxyActionClient({self._action_name: MTCPlanAndExecutePick})
        except Exception as e:
            Logger.logerr(f"[{type(self).__name__}] Failed to create ProxyActionClient: {e}")
            self._had_error = True
            return

        # Wait for server (ProxyActionClient handles the underlying rclpy client)
        if not self._ac.is_available(self._action_name):
            Logger.logerr(f"[{type(self).__name__}] Action '{self._action_name}' not available after "
                          f"{self._timeout_sec:.1f}s.")
            self._had_error = True
            return

        # Send goal
        try:
            self._ac.send_goal(self._action_name, goal)
            self._sent_goal = True
        except Exception as e:
            Logger.logerr(f"[{type(self).__name__}] Exception while sending goal: {e}")
            self._had_error = True

    def on_exit(self, userdata):
        # Call this method when an outcome is returned and another state gets active.
        # It can be used to stop possibly running processes started by on_enter.

        # You could cancel the goal here if you want to be defensive on early exits:
        try:
            if self._sent_goal and self._ac is not None and self._ac.is_available(self._action_name):
                self._ac.cancel(self._action_name)
        except Exception:
            # Non-fatal cleanup
            pass

    def on_start(self):
        # Call this method when the behavior is instantiated on board.
        # If possible, it is generally better to initialize used resources in the constructor
        #   because if anything failed, the behavior would not even be started.

        # No-op: template hook
        pass

    def on_stop(self):
        # Call this method whenever the behavior stops execution, also if it is cancelled.
        # Use this event to clean up things like claimed resources.

        # No-op: template hook
        pass