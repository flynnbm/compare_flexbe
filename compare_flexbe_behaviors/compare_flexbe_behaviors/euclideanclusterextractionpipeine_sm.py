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
from compare_flexbe_states.gpd_grasp_poses_service_state import GPDGraspPosesServiceState
from compare_flexbe_states.move_to_named_pose_service_state import MoveToNamedPoseServiceState
from compare_flexbe_states.move_to_pose_service_state import MoveToPoseServiceState
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
        # x:2399 y:360, x:251 y:389
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], output_keys=['target_cluster_indexed', 'scene_pointcloud'])
        _state_machine.userdata.scene_pointcloud = 0
        _state_machine.userdata.camera_pose = 0
        _state_machine.userdata.target_cluster_indexed = 0
        _state_machine.userdata.obstacle_clusters_indexed = 0
        _state_machine.userdata.cluster_count = 0
        _state_machine.userdata.point_cloud_visual = 0
        _state_machine.userdata.camera_source = 0
        _state_machine.userdata.grasp_poses = []
        _state_machine.userdata.test_indices = []
        _state_machine.userdata.grasp_waypoints = []
        _state_machine.userdata.waypoint_index = 0
        _state_machine.userdata.grasp_target_poses = []
        _state_machine.userdata.grasp_index = 0
        _state_machine.userdata.ready_pose = 'ready'
        _state_machine.userdata.snapshot_pose = 'retracted'
        _state_machine.userdata.start_pose = 'inspect'

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]


        # [/MANUAL_CREATE]

        with _state_machine:
            # x:43 y:183
            OperatableStateMachine.add('MoveForSnapshot',
                                       MoveToNamedPoseServiceState(service_timeout=5.0,
                                                                   service_name='/move_to_named_pose'),
                                       transitions={'finished': 'TakeSnapshot'  # 97 124 -1 -1 -1 -1
                                                    , 'failure': 'failed'  # 119 323 -1 -1 -1 -1
                                                    },
                                       autonomy={'finished': Autonomy.Off, 'failure': Autonomy.Off},
                                       remapping={'target_pose_name': 'snapshot_pose'})

            # x:1351 y:62
            OperatableStateMachine.add('ComputePoses',
                                       GPDGraspPosesServiceState(service_timeout=5.0,
                                                                 service_name='/compute_grasp_poses'),
                                       transitions={'done': 'MoveReady'  # 1548 76 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 1416 263 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'grasp_configs': 'grasp_configs',
                                                  'grasp_target_poses': 'grasp_target_poses',
                                                  'grasp_approach_poses': 'grasp_approach_poses',
                                                  'grasp_waypoints': 'grasp_waypoints'})

            # x:885 y:56
            OperatableStateMachine.add('DetectGrasps',
                                       DetectGraspsServiceState(service_timeout=5.0,
                                                                service_name='/detect_grasps'),
                                       transitions={'done': 'PublishPointCloud'  # 1079 73 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 915 234 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'cloud': 'point_cloud_visual',
                                                  'camera_source': 'camera_source',
                                                  'view_points': 'camera_pose',
                                                  'indices': 'test_indices',
                                                  'grasp_configs': 'grasp_configs'})

            # x:393 y:58
            OperatableStateMachine.add('EuclideanClustering',
                                       EuclideanClusteringServiceState(service_timeout=5.0,
                                                                       service_name='/euclidean_clustering',
                                                                       cluster_tolerance=0.02,
                                                                       min_cluster_size=100,
                                                                       max_cluster_size=25000),
                                       transitions={'finished': 'FilterByIndices'  # 606 71 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 455 228 455 111 -1 -1
                                                    },
                                       autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'cloud_in': 'scene_pointcloud',
                                                  'camera_pose': 'camera_pose',
                                                  'target_cluster_indices': 'target_cluster_indices',
                                                  'obstacle_cluster_indices': 'obstacle_cluster_indices'})

            # x:649 y:58
            OperatableStateMachine.add('FilterByIndices',
                                       FilterByIndicesServiceState(service_timeout=5.0,
                                                                   service_name='/filter_by_indices'),
                                       transitions={'finished': 'DetectGrasps'  # 841 71 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 690 233 -1 -1 -1 -1
                                                    },
                                       autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'cloud_in': 'scene_pointcloud',
                                                  'target_indices': 'target_cluster_indices',
                                                  'cloud_out': 'point_cloud_visual'})

            # x:2098 y:59
            OperatableStateMachine.add('MoveBackToStart',
                                       MoveToNamedPoseServiceState(service_timeout=5.0,
                                                                   service_name='/move_to_named_pose'),
                                       transitions={'finished': 'finished'  # 2405 210 -1 -1 -1 -1
                                                    , 'failure': 'failed'  # 2180 353 -1 -1 -1 -1
                                                    },
                                       autonomy={'finished': Autonomy.Off, 'failure': Autonomy.Off},
                                       remapping={'target_pose_name': 'start_pose'})

            # x:1857 y:63
            OperatableStateMachine.add('MoveOMPL',
                                       MoveToPoseServiceState(timeout_sec=5.0,
                                                              service_name='/move_to_pose'),
                                       transitions={'done': 'MoveBackToStart'  # 2041 77 -1 -1 -1 -1
                                                    , 'next': 'MoveOMPL'  # 1939 240 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 1772 329 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Off,
                                                 'next': Autonomy.Off,
                                                 'failed': Autonomy.Off},
                                       remapping={'grasp_poses': 'grasp_target_poses',
                                                  'grasp_index': 'grasp_index'})

            # x:1591 y:62
            OperatableStateMachine.add('MoveReady',
                                       MoveToNamedPoseServiceState(service_timeout=5.0,
                                                                   service_name='/move_to_named_pose'),
                                       transitions={'finished': 'MoveOMPL'  # 1808 79 -1 -1 -1 -1
                                                    , 'failure': 'failed'  # 1657 267 -1 -1 -1 -1
                                                    },
                                       autonomy={'finished': Autonomy.Off, 'failure': Autonomy.Off},
                                       remapping={'target_pose_name': 'ready_pose'})

            # x:1135 y:60
            OperatableStateMachine.add('PublishPointCloud',
                                       PublishPointCloudState(pub_topic='/filtered_cloud/target_object'),
                                       transitions={'done': 'ComputePoses'  # 1307 74 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 1172 242 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'cloud_in': 'point_cloud_visual'})

            # x:156 y:55
            OperatableStateMachine.add('TakeSnapshot',
                                       GetPointCloudServiceState(service_timeout=5.0,
                                                                 service_name='/get_point_cloud',
                                                                 camera_topic='/rgbd_camera/points',
                                                                 target_frame='simple_pedestal'),
                                       transitions={'finished': 'EuclideanClustering'  # 350 68 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 289 232 265 108 -1 -1
                                                    },
                                       autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'camera_pose': 'camera_pose',
                                                  'cloud_out': 'scene_pointcloud',
                                                  'cloud_frame': 'cloud_frame'})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]


    # [/MANUAL_FUNC]
