#!/usr/bin/env python3

# Copyright 2023 Christopher Newport University
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from flexbe_core import EventState, Logger

from rclpy.action import ActionClient
from control_msgs.action import GripperCommand


class GripperCommandActionState(EventState):
    """
    Sends a gripper command via action server.

    ># robot_name     str     Prefix for action server name (e.g., 'my_robot' -> '/my_robot_hand_controller/gripper_cmd')
    ># position       float   Desired gripper position
    ># max_effort     float   Maximum effort to apply

    -- timeout_sec    float   Timeout to wait for action server (default: 5.0)

    <= success                Gripper moved or succeeded
    <= failure                Failed to move or action rejected
    """

    def __init__(self, timeout_sec: float = 5.0):
        super().__init__(
            outcomes=['success', 'failure'],
            input_keys=['robot_name', 'position', 'max_effort']
        )
        self._timeout_sec = timeout_sec
        self._client = None
        self._goal_future = None
        self._result_future = None
        self._goal_handle = None
        self._server_available = False
        self._action_name = None
        self._request_sent = False

    def on_start(self):
        if not hasattr(GripperCommandActionState, '_node'):
            raise RuntimeError("This state requires a FlexBE-attached ROS 2 node.")
        self._node = GripperCommandActionState._node

    def on_enter(self, userdata):
        self._request_sent = False
        self._goal_future = None
        self._result_future = None
        self._goal_handle = None
        self._server_available = False

        try:
            self._action_name = f"/{userdata.robot_name}_hand_controller/gripper_cmd"
            self._client = ActionClient(self._node, GripperCommand, self._action_name)

            Logger.loginfo(f"[GripperCommandActionState] Waiting for action server '{self._action_name}'...")
            if not self._client.wait_for_server(timeout_sec=self._timeout_sec):
                Logger.logerr(f"[GripperCommandActionState] Action server '{self._action_name}' not available.")
                return
            self._server_available = True
            Logger.loginfo(f"[GripperCommandActionState] Connected to action server '{self._action_name}'.")

            goal_msg = GripperCommand.Goal()
            goal_msg.command.position = userdata.position
            goal_msg.command.max_effort = userdata.max_effort

            Logger.loginfo(f"[GripperCommandActionState] Sending goal: position={goal_msg.command.position}, "
                           f"max_effort={goal_msg.command.max_effort}")
            self._goal_future = self._client.send_goal_async(goal_msg, feedback_callback=self.feedback_cb)
            self._request_sent = True

        except Exception as e:
            Logger.logerr(f"[GripperCommandActionState] Exception during goal send: {str(e)}")

    def execute(self, userdata):
        if not self._server_available or not self._request_sent:
            return 'failure'

        if self._goal_future and not self._goal_future.done():
            return None  # Still waiting for goal to be accepted

        if self._goal_future and self._goal_handle is None and self._goal_future.done():
            self._goal_handle = self._goal_future.result()
            if not self._goal_handle.accepted:
                Logger.logerr(f"[GripperCommandActionState] Goal was rejected by the server.")
                return 'failure'
            Logger.loginfo(f"[GripperCommandActionState] Goal accepted, waiting for result...")
            self._result_future = self._goal_handle.get_result_async()
            return None

        if self._result_future and self._result_future.done():
            result = self._result_future.result()
            if result is None:
                Logger.logerr("[GripperCommandActionState] No result received.")
                return 'failure'

            result_data = result.result
            Logger.loginfo(f"[GripperCommandActionState] Result: position={result_data.position:.4f}, "
                           f"effort={result_data.effort:.2f}, "
                           f"stalled={result_data.stalled}, "
                           f"reached_goal={result_data.reached_goal}")

            if result_data.position > 0.001:
                Logger.loginfo("[GripperCommandActionState] Gripper moved — treating as success.")
                return 'success'

            if result.status == 0 and result_data.reached_goal:
                Logger.loginfo("[GripperCommandActionState] Gripper goal succeeded cleanly.")
                return 'success'

            Logger.logerr("[GripperCommandActionState] Gripper did not move and goal failed.")
            return 'failure'

        return None  # still waiting

    def feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        Logger.logdebug(f"[GripperCommandActionState] Feedback: {feedback}")

    def on_exit(self, userdata):
        Logger.loginfo("[GripperCommandActionState] Exiting state.")

    def on_stop(self):
        Logger.loginfo("[GripperCommandActionState] Behavior stopped.")