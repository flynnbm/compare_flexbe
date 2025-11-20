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
from flexbe_core.proxy import ProxyServiceCaller

from compare_flexbe_utilities.srv import PlaneSegmentation as SrvType # request: input, use_voxel, leaf_size, distance_threshold, max_iterations

class PlaneSegmentationServiceState(EventState):
    """
    Calls /plane_segmentation to remove the dominant plane from the ORIGINAL cloud,
    using plane coefficients fit on an optionally voxelized copy for speed.

    -- service_timeout     float   Timeout for service discovery (sec, default: 5.0)
    -- service_name        str     Service name (default: '/plane_segmentation')
    -- use_voxel           bool    Run RANSAC on voxelized copy (default: True)
    -- leaf_size           float   Voxel leaf size in meters (default: 0.01)
    -- distance_threshold  float   RANSAC distance threshold in meters (default: 0.01)
    -- max_iterations      int     Max RANSAC iterations (default: 1000)

    ># cloud_in                   sensor_msgs/PointCloud2
    <# cloud_without_plane        sensor_msgs/PointCloud2
    <# plane_indices              pcl_msgs/PointIndices
    <# plane_coefficients         float[4]      [a,b,c,d]
    <# plane_inlier_count         int

    <= finished
    <= failed
    """
    def __init__(self, service_timeout=5.0, service_name='/plane_segmentation', use_voxel=True, leaf_size=0.01, distance_threshold=0.01, max_iterations=1000):
        super().__init__(outcomes=['finished', 'failed'],
                            input_keys=['cloud_in'],
                            output_keys=['cloud_without_plane', 'plane_indices', 'plane_coefficients', 'plane_inlier_count']
        )
        self._params = dict(
            use_voxel=use_voxel,
            leaf_size=float(leaf_size),
            distance_threshold=float(distance_threshold),
            max_iterations=int(max_iterations)
        )
        self._service_timeout = service_timeout
        self._service_name = service_name
        self._client = None
        self._future = None

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

        # Write userdata
        try:
            userdata.cloud_without_plane = self._res.without_plane
            userdata.plane_indices = self._res.plane_indices
            userdata.plane_coefficients = self._res.plane_coefficients
            userdata.plane_inlier_count = self._res.inlier_count
        except Exception as e:
            Logger.logerr(f"[{type(self).__name__}] Failed to write userdata: {e}")
            return 'failed'

        # Return outcome finished
        return 'finished'

    def on_enter(self, userdata):
        # Call this method a single time when the state becomes active, when a transition from another state to this one is taken.
        # It is primarily used to start actions which are associated with this state.
        
        # construct request
        request = SrvType.Request()
        request.input = userdata.cloud_in
        request.use_voxel = bool(self._params['use_voxel'])
        request.leaf_size = float(self._params['leaf_size'])
        request.distance_threshold = float(self._params['distance_threshold'])
        request.max_iterations = int(self._params['max_iterations'])

        # reset state
        self._res = None
        self._had_error = False

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
            self._res = None
            self._had_error = True

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
