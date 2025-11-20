#!/usr/bin/env python3
# Copyright ...
# License: Apache 2.0 (same header as your other files)

import rclpy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from control_msgs.action import GripperCommand


class GripperCommandActionState(EventState):
    """
    Sends a GripperCommand action to a hand controller.

    -- timeout_sec       float   Seconds to wait for action server discovery (default: 5.0)
    -- action_name_fmt   str     Format string for the action name using {robot_name}
                                 (default: '/{robot_name}_hand_controller/gripper_cmd')

    ># robot_name        str     Robot name used to format the action server name
    ># position          float   Desired gripper position
    ># max_effort        float   Maximum effort to apply

    <= success                   Goal succeeded or motion considered successful
    <= failed                    Any failure (no server, reject, error, stalled+no goal reached)
    """

    def __init__(self,
                 timeout_sec: float = 5.0,
                 action_name_fmt: str = '/{robot_name}_hand_controller/gripper_cmd'):
        super().__init__(
            outcomes=['success', 'failed'],
            input_keys=['robot_name', 'position', 'max_effort']
        )

        # --- configuration / parameters ---
        self._timeout_sec = float(timeout_sec)
        self._action_name_fmt = action_name_fmt

        # --- internal state ---
        self._action_name = None
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
            r = self._ac.get_result(self._action_name)  # type: GripperCommand.Result
        except Exception as e:
            Logger.logerr(f"[{type(self).__name__}] Failed to get result: {e}")
            return 'failed'

        if r is None:
            Logger.logerr(f"[{type(self).__name__}] Empty result from action.")
            return 'failed'

        Logger.loginfo(
            f"[{type(self).__name__}] Result: "
            f"position={getattr(r, 'position', float('nan')):.4f}, "
            f"effort={getattr(r, 'effort', float('nan')):.2f}, "
            f"stalled={getattr(r, 'stalled', False)}, "
            f"reached_goal={getattr(r, 'reached_goal', False)}"
        )

        reached = getattr(r, 'reached_goal', False)
        stalled = getattr(r, 'stalled', False)
        pos = getattr(r, 'position', 0.0)

        # Success policy: prefer reached_goal, otherwise allow "close enough & not stalled"
        if reached or (abs(pos - userdata.position) < 1e-3 and not stalled):
            Logger.loginfo(f"[{type(self).__name__}] Gripper goal succeeded.")
            return 'success'

        Logger.logerr(f"[{type(self).__name__}] Gripper goal failed (stalled={stalled}, reached={reached}).")
        return 'failed'
    
    def on_enter(self, userdata):
        # Call this method a single time when the state becomes active, when a transition from another state to this one is taken.
        # It is primarily used to start actions which are associated with this state.

        self._had_error = False
        self._sent_goal = False
        self._action_name = self._action_name_fmt.format(robot_name=userdata.robot_name)

        # Construct goal
        goal = GripperCommand.Goal()
        goal.command.position = float(userdata.position)
        goal.command.max_effort = float(userdata.max_effort)

        # Create / register proxy client for this action
        try:
            self._ac = ProxyActionClient({self._action_name: GripperCommand})
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
            Logger.loginfo(f"[{type(self).__name__}] Sending goal to '{self._action_name}': "
                           f"position={goal.command.position}, max_effort={goal.command.max_effort}")
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