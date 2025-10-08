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


from compare_flexbe_states.cartesian_move_to_pose_service_state import CartesianMoveToPoseServiceState
from compare_flexbe_states.detect_grasps_service_state import DetectGraspsServiceState
from compare_flexbe_states.euclidean_clustering_service_state import EuclideanClusteringServiceState
from compare_flexbe_states.filter_by_indices_service_state import FilterByIndicesServiceState
from compare_flexbe_states.get_point_cloud_service_state import GetPointCloudServiceState
from compare_flexbe_states.gpd_grasp_poses_service_state import GPDGraspPosesServiceState
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
        # x:1367 y:375, x:251 y:389
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
                                       transitions={'finished': 'EuclideanClustering'  # 306 84 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 236 225 228 116 -1 -1
                                                    },
                                       autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'camera_pose': 'camera_pose',
                                                  'cloud_out': 'scene_pointcloud',
                                                  'cloud_frame': 'cloud_frame'})

            # x:1275 y:55
            OperatableStateMachine.add('ComputePoses',
                                       GPDGraspPosesServiceState(service_timeout=5.0,
                                                                 service_name='/compute_grasp_poses'),
                                       transitions={'done': 'MoveCartesian'  # 1483 77 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 1351 236 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'grasp_configs': 'grasp_configs',
                                                  'grasp_target_poses': 'grasp_target_poses',
                                                  'grasp_approach_poses': 'grasp_approach_poses',
                                                  'grasp_waypoints': 'grasp_waypoints'})

            # x:833 y:52
            OperatableStateMachine.add('DetectGrasps',
                                       DetectGraspsServiceState(service_timeout=5.0,
                                                                service_name='/detect_grasps'),
                                       transitions={'done': 'PublishPointCloud'  # 1017 68 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 874 238 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'cloud': 'point_cloud_visual',
                                                  'camera_source': 'camera_source',
                                                  'view_points': 'camera_pose',
                                                  'indices': 'test_indices',
                                                  'grasp_configs': 'grasp_configs'})

            # x:344 y:61
            OperatableStateMachine.add('EuclideanClustering',
                                       EuclideanClusteringServiceState(service_timeout=5.0,
                                                                       service_name='/euclidean_clustering',
                                                                       cluster_tolerance=0.02,
                                                                       min_cluster_size=100,
                                                                       max_cluster_size=25000),
                                       transitions={'finished': 'FilterByIndices'  # 557 80 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 400 213 406 114 -1 -1
                                                    },
                                       autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'cloud_in': 'scene_pointcloud',
                                                  'camera_pose': 'camera_pose',
                                                  'target_cluster_indices': 'target_cluster_indices',
                                                  'obstacle_cluster_indices': 'obstacle_cluster_indices'})

            # x:600 y:56
            OperatableStateMachine.add('FilterByIndices',
                                       FilterByIndicesServiceState(service_timeout=5.0,
                                                                   service_name='/filter_by_indices'),
                                       transitions={'finished': 'DetectGrasps'  # 790 71 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 648 229 -1 -1 -1 -1
                                                    },
                                       autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'cloud_in': 'scene_pointcloud',
                                                  'target_indices': 'target_cluster_indices',
                                                  'cloud_out': 'point_cloud_visual'})

            # x:1546 y:68
            OperatableStateMachine.add('MoveCartesian',
                                       CartesianMoveToPoseServiceState(service_timeout=5.0,
                                                                       service_name='/plan_cartesian_path'),
                                       transitions={'done': 'finished'  # 1808 313 -1 -1 -1 -1
                                                    , 'next': 'MoveCartesian'  # 1661 226 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 1374 314 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Off,
                                                 'next': Autonomy.Off,
                                                 'failed': Autonomy.Off},
                                       remapping={'waypoints': 'grasp_waypoints',
                                                  'waypoint_index': 'waypoint_index'})

            # x:1055 y:53
            OperatableStateMachine.add('PublishPointCloud',
                                       PublishPointCloudState(pub_topic='/filtered_cloud/target_object'),
                                       transitions={'done': 'ComputePoses'  # 1230 69 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 1099 242 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'cloud_in': 'point_cloud_visual'})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]


    # [/MANUAL_FUNC]
