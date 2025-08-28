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
from flexbe_core.proxy import ProxyPublisher
from sensor_msgs.msg import PointCloud2


class PublishPointCloudState(EventState):
    """
    Calls /get_point_cloud to collect current view point cloud and camera pose.

    -- camera_topic        str      Point Cloud Output Topic (default: '/pub_cloud/points')

    <# cloud_in                     sensor_msgs/PointCloud2     point cloud we want to publish

    <= done
    <= failed
    """
    def __init__(self, pub_topic: str='/pub_cloud/points'):
        super().__init__(outcomes=['done', 'failed'],
                            input_keys=['cloud_in']
        )
        self._pub_topic = pub_topic
        self._pub = ProxyPublisher({self._pub_topic: PointCloud2})
        self._published = False

    def execute(self, userdata):
        # Execute this method periodically while the state is active.
        # Main purpose is to check state conditions and trigger a corresponding outcome.
        # If no outcome is returned, the state will stay active.

        # One-shot state: as soon as we've published, we can leave.
        if self._published:
            return 'done'
        return None
    
    def on_enter(self, userdata):
        # Call this method a single time when the state becomes active, when a transition from another state to this one is taken.
        # It is primarily used to start actions which are associated with this state.

        # Validate input and publish once.
        try:
            cloud: T.Optional[PointCloud2] = userdata.cloud_in
        except Exception:
            Logger.logerr(f"[{type(self).__name__}] Missing 'cloud_in' in userdata.")
            self._published = False
            return

        if cloud is None or not isinstance(cloud, PointCloud2):
            Logger.logerr(f"[{type(self).__name__}] 'cloud_in' must be a sensor_msgs/PointCloud2.")
            self._published = False
            return

        try:
            self._pub.publish(self._pub_topic, cloud)
            Logger.loginfo(f"[{type(self).__name__}] Published point cloud on '{self._pub_topic}'.")
            self._published = True
        except Exception as e:
            Logger.logerr(f"[{type(self).__name__}] Failed to publish: {e}")
            self._published = False

    def on_exit(self, userdata):
        # Call this method when an outcome is returned and another state gets active.
        # It can be used to stop possibly running processes started by on_enter.

        # No-op: template hook
        pass

    def on_start(self):
        # Call this method when the behavior is instantiated on board.
        # If possible, it is generally better to initialize used resources in the constructor
        #   because if anything failed, the behavior would not even be started.

        # Nothing needed; ProxyPublisher already set up in __init__.
        pass

    def on_stop(self):
        # Call this method whenever the behavior stops execution, also if it is cancelled.
        # Use this event to clean up things like claimed resources.

        # No client to destroy; ProxyPublisher is managed by FlexBE.
        pass