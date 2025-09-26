#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2025 Brian Flynn
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

###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

"""
Define EuclideanClusterExtractionPipeine.

A perception pipeline which employs several basic filters such as plane
segmentation and cluster extraction in order to cluster pickable objects from a
scene and choose a target for grasping

Created on Wed Aug 27 2025
@author: Brian Flynn
"""


from compare_flexbe_states.detect_grasps_service_state import DetectGraspsServiceState
from compare_flexbe_states.euclidean_clustering_service_state import EuclideanClusteringServiceState
from compare_flexbe_states.filter_by_indices_service_state import FilterByIndicesServiceState
from compare_flexbe_states.get_point_cloud_service_state import GetPointCloudServiceState
from compare_flexbe_states.publish_point_cloud_state import PublishPointCloudState
from flexbe_core import Autonomy
from flexbe_core import Behavior
from flexbe_core import ConcurrencyContainer
from flexbe_core import Logger
from flexbe_core import OperatableStateMachine
from flexbe_core import PriorityContainer
from flexbe_core import initialize_flexbe_core

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]


# [/MANUAL_IMPORT]


class EuclideanClusterExtractionPipeineSM(Behavior):
    """
    Define EuclideanClusterExtractionPipeine.

    A perception pipeline which employs several basic filters such as plane
    segmentation and cluster extraction in order to cluster pickable objects from a
    scene and choose a target for grasping
    """

    def __init__(self, node):
        super().__init__()
        self.name = 'EuclideanClusterExtractionPipeine'

        # parameters of this behavior

        # Initialize ROS node information
        initialize_flexbe_core(node)

        # references to used behaviors

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]


        # [/MANUAL_INIT]

        # Behavior comments:

    def create(self):
        """Create state machine."""
        # Root state machine
        # x:135 y:388, x:251 y:389
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], output_keys=['target_cluster_indexed', 'scene_pointcloud'])
        _state_machine.userdata.scene_pointcloud = 0
        _state_machine.userdata.camera_pose = 0
        _state_machine.userdata.target_cluster_indexed = 0
        _state_machine.userdata.obstacle_clusters_indexed = 0
        _state_machine.userdata.cluster_count = 0
        _state_machine.userdata.point_cloud_visual = 0
        _state_machine.userdata.camera_source = 0

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]


        # [/MANUAL_CREATE]

        with _state_machine:
            # x:119 y:63
            OperatableStateMachine.add('GetPointCloud',
                                       GetPointCloudServiceState(service_timeout=5.0,
                                                                 service_name='/get_point_cloud',
                                                                 camera_topic='/rgbd_camera/points',
                                                                 target_frame='panda_link0'),
                                       transitions={'finished': 'EuclideanClustering'  # 180 59 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 155 81 228 116 -1 -1
                                                    },
                                       autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'camera_pose': 'camera_pose',
                                                  'cloud_out': 'scene_pointcloud',
                                                  'cloud_frame': 'cloud_frame'})

            # x:593 y:192
            OperatableStateMachine.add('DetectGrasps',
                                       DetectGraspsServiceState(service_timeout=5.0,
                                                                service_name='/detect_grasps'),
                                       transitions={'done': 'FilterByIndices'  # 704 124 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 291 238 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'cloud': 'scene_pointcloud',
                                                  'camera_source': 'camera_source',
                                                  'view_points': 'camera_pose',
                                                  'indices': 'target_cluster_indices',
                                                  'grasp_configs': 'grasp_configs'})

            # x:344 y:61
            OperatableStateMachine.add('EuclideanClustering',
                                       EuclideanClusteringServiceState(service_timeout=5.0,
                                                                       service_name='/euclidean_clustering',
                                                                       cluster_tolerance=0.02,
                                                                       min_cluster_size=100,
                                                                       max_cluster_size=25000),
                                       transitions={'finished': 'DetectGrasps'  # 575 147 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 222 55 406 114 -1 -1
                                                    },
                                       autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'cloud_in': 'scene_pointcloud',
                                                  'camera_pose': 'camera_pose',
                                                  'target_cluster_indices': 'target_cluster_indices',
                                                  'obstacle_cluster_indices': 'obstacle_cluster_indices'})

            # x:823 y:55
            OperatableStateMachine.add('FilterByIndices',
                                       FilterByIndicesServiceState(service_timeout=5.0,
                                                                   service_name='/filter_by_indices'),
                                       transitions={'finished': 'PublishPointCloud'  # 1025 49 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 371 50 -1 -1 -1 -1
                                                    },
                                       autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'cloud_in': 'scene_pointcloud',
                                                  'target_indices': 'target_cluster_indices',
                                                  'cloud_out': 'point_cloud_visual'})

            # x:1051 y:56
            OperatableStateMachine.add('PublishPointCloud',
                                       PublishPointCloudState(pub_topic='/filtered_cloud/target_object'),
                                       transitions={'done': 'finished'  # 516 58 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 500 59 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'cloud_in': 'point_cloud_visual'})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]


    # [/MANUAL_FUNC]
