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
from rclpy.task import Future
from rclpy.duration import Duration

import numpy as np
from typing import Iterable

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller

from gpd_ros.srv import ComputeGraspPoses as SrvType
from gpd_ros.msg import GraspConfigList
from std_msgs.msg import Int64
from geometry_msgs.msg import Point, Pose, PoseStamped
from pcl_msgs.msg import PointIndices

class GPDGraspPosesServiceState(EventState):
    """
    Calls the gpd_ros/grasp_pose service with a GraspConfigList input and returns Pose[].

    -- service_timeout          float       Timeout for service call in seconds (default: 5.0)

    ># grasp_configs            gpd_ros/GraspConfigList         The output from the GPD grasp detection service that contains grasp point and orientation data
    <# grasp_target_poses       geometry_msgs/Pose[] poses      A list of grasp pose candidate target poses computed from GPD outputs
    <# grasp_approach_poses     geometry_msgs/Pose[] poses      A list of grasp pose candidate approach poses computed from GPD outputs and target poses

    <= done               Service call succeeded
    <= failed             Service call failed or timed out
    """
    def __init__(self, service_timeout=5.0, service_name='/compute_grasp_poses'):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.

        super().__init__(outcomes=['done', 'failed'],
                            input_keys=['grasp_configs'],
                            output_keys=['grasp_target_poses', 'grasp_approach_poses', 'grasp_waypoints']
        )
        self._service_name = service_name
        self._service_timeout = service_timeout
        self._client = None

        # Create proxy service caller to handle rclpy node
        self._srv = ProxyServiceCaller({self._service_name: SrvType})

        # result storage
        self._res = None
        self._had_error = False

    def execute(self, userdata):
        # Execute this method periodically while the state is active.
        # Main purpose is to check state conditions and trigger a corresponding outcome.
        # If no outcome is returned, the state will stay active.

        # Check for error or no response
        if self._had_error or self._res is None:
            return 'failed'

        try:
            # Keep the raw lists if you still want them
            userdata.grasp_target_poses = self._res.target_poses
            userdata.grasp_approach_poses = self._res.approach_poses

            # Build list-of-sets: [[approach0, target0], [approach1, target1], ...]
            # zip() will truncate to the shorter list if lengths differ.
            userdata.grasp_waypoints = [
                [a, t] for a, t in zip(self._res.approach_poses, self._res.target_poses)
            ]
            Logger.loginfo(f"[{type(self).__name__}] Received grasp poses list with {len(self._res.target_poses)} poses.")
        except Exception as e:
            Logger.logerr(f"[{type(self).__name__}] Service call failed: {str(e)}")
            return 'failed'

        # Return outcome finished
        return 'done'

    def on_enter(self, userdata):
        # Call this method a single time when the state becomes active, when a transition from another state to this one is taken.
        # It is primarily used to start actions which are associated with this state.

        # construct request
        request = SrvType.Request()
        request.grasps = userdata.grasp_configs

        # wait for availability (once per entry)
        if not self._srv.is_available(self._service_name):
            Logger.logerr(f"[{type(self).__name__}] Service '{self._service_name}' not available after {self._service_timeout}s.")
            self._had_error = True
            return

        # send request
        try:
            self._res = self._srv.call(self._service_name, request)
            Logger.loginfo(f"[{type(self).__name__}] Called service '{self._service_name}'.")
        except Exception as e:
            Logger.logerr(f"[{type(self).__name__}] Service call failed: {e}")

    def on_exit(self, userdata):
        # Call this method when an outcome is returned and another state gets active.
        # It can be used to stop possibly running processes started by on_enter.

        # No-op: template hook
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