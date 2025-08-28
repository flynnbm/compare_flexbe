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

from flexbe_core import EventState, Logger

from gpd_ros.srv import DetectGrasps as SrvType
from gpd_ros.msg import CloudIndexed, CloudSources
from std_msgs.msg import Int64
from geometry_msgs.msg import Point

class DetectGraspsServiceState(EventState):
    """
    Calls the gpd_ros/detect_grasps service with a CloudIndexed input and returns GraspConfigList.

    -- service_timeout    float       Timeout for service call in seconds (default: 5.0)

    ># cloud              sensor_msgs/PointCloud2   Point Cloud of scene (full/unfiltered cloud is best)
    ># camera_source      std_msgs/Int64[]          For each point in the cloud, the index of the camera that acquired the point
    ># view_points        geometry_msgs/Point[]     A list of camera positions at which the point cloud was acquired
    ># indices            std_msgs/Int64[]          The indices into the point cloud at which to sample grasp candidates
    <# grasp_configs      gpd_ros/GraspConfigList   The output from the service

    <= done               Service call succeeded
    <= failed             Service call failed or timed out
    """

    SERVICE_NAME = '/detect_grasps'

    def __init__(self, service_timeout=5.0, service_name='/detect_grasps'):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.

        super().__init__(outcomes=['done', 'failed'],
                            input_keys=['cloud', 'camera_source', 'view_points', 'indices'],
                            output_keys=['grasp_configs']
        )
        self._service_name = service_name
        self._service_timeout = service_timeout
        self._client = None
        self._future = None

    def execute(self, userdata):
        # Execute this method periodically while the state is active.
        # Main purpose is to check state conditions and trigger a corresponding outcome.
        # If no outcome is returned, the state will stay active.

        if self._future is None:
            return 'failed'

        if self._future.done():
            try:
                result = self._future.result()
                userdata.grasp_configs = result.grasp_configs
                Logger.loginfo(f"[{type(self).__name__}] Received grasp configs with {len(result.grasp_configs.grasps)} grasps.")
                return 'done'
            except Exception as e:
                Logger.logerr(f"[{type(self).__name__}] Service call failed: {str(e)}")
                return 'failed'

        return None  # still waiting

    def on_enter(self, userdata):
        # Call this method a single time when the state becomes active, when a transition from another state to this one is taken.
        # It is primarily used to start actions which are associated with this state.

        # helper: ensure that inputs are std_msgs/msg/Int64[]
        def as_int64_array(seq):
            return [v if isinstance(v, Int64) else Int64(data=int(v)) for v in seq]

        # helper: ensure that inputs are geometry_msgs/msg/Point[]
        def as_points(seq):
            return [p if isinstance(p, Point) else Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in seq]

        # construct cloud_sources
        cloud_sources = CloudSources()
        cloud_sources.cloud = userdata.cloud
        cloud_sources.camera_source = as_int64_array(userdata.camera_source)
        cloud_sources.view_points = as_points(userdata.view_points)

        # construct cloud_indexed
        cloud_indexed = CloudIndexed()
        cloud_indexed.cloud_sources = cloud_sources
        cloud_indexed.indices = as_int64_array(userdata.indices)

        # construct request
        request = SrvType.Request()
        request.cloud_indexed = cloud_indexed

        # send request
        try:
            self._future = self._client.call_async(request)
            Logger.loginfo(f"[{type(self).__name__}] Sent request to {self._service_name} service.")
        except Exception as e:
            Logger.logerr(f"[{type(self).__name__}] Failed to send request: {str(e)}")

    def on_exit(self):
        # Call this method when an outcome is returned and another state gets active.
        # It can be used to stop possibly running processes started by on_enter.

        # No-op: template hook
        pass

    def on_start(self):
        # Call this method when the behavior is instantiated on board.
        # If possible, it is generally better to initialize used resources in the constructor
        #   because if anything failed, the behavior would not even be started.

        # create the service client, andensure that the service server is initialized
        self._client = type(self).create_client(SrvType, self._service_name)
        if not self._client.wait_for_service(timeout_sec=self._service_timeout):
            Logger.logerr(f"[{type(self).__name__}] Service {self._service_name} not available after waiting.")
            return 'failed'

    def on_stop(self):
        # Call this method whenever the behavior stops execution, also if it is cancelled.
        # Use this event to clean up things like claimed resources.

        # make sure the client is destroyed when the behavior ends so it can restart cleanly
        if self._client:
            try:
                self._client.destroy()
            except Exception:
                pass