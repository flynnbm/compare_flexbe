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
from compare_flexbe_states.gpd_grasp_poses_service_state import GPDGraspPosesServiceState
from compare_flexbe_states.publish_point_cloud_state import PublishPointCloudState
from end_effector_flexbe_states.gripper_command_action_state import GripperCommandActionState
from flexbe_core import Autonomy
from flexbe_core import Behavior
from flexbe_core import ConcurrencyContainer
from flexbe_core import Logger
from flexbe_core import OperatableStateMachine
from flexbe_core import PriorityContainer
from flexbe_core import initialize_flexbe_core
from move_group_flexbe_states.move_to_named_pose_service_state import MoveToNamedPoseServiceState
from mtc_flexbe_states.mtc_approach_and_pick_action_state import MTCApproachAndPickActionState
from mtc_flexbe_states.mtc_retreat_and_place_action_state import MTCRetreatAndPlaceActionState
from pcl_flexbe_states.euclidean_clustering_service_state import EuclideanClusteringServiceState
from pcl_flexbe_states.filter_by_indices_service_state import FilterByIndicesServiceState
from pcl_flexbe_states.get_point_cloud_service_state import GetPointCloudServiceState
from pcl_flexbe_states.passthrough_filter_service_state import PassthroughServiceState

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
        _state_machine.userdata.grasp_index = 0
        _state_machine.userdata.ready_pose = 'ready'
        _state_machine.userdata.snapshot_pose = 'retracted'
        _state_machine.userdata.start_pose = 'inspect'
        _state_machine.userdata.grasp_approach_poses = 0
        _state_machine.userdata.grasp_retreat_poses = 0
        _state_machine.userdata.grasp_target_poses = 0
        _state_machine.userdata.object_id = ''
        _state_machine.userdata.inspect_pose = 'inspect'
        _state_machine.userdata.plane_indices = 0
        _state_machine.userdata.plane_coefficients = 0
        _state_machine.userdata.plane_inlier_count = 0
        _state_machine.userdata.robot_name = 'panda'
        _state_machine.userdata.close = 0.01
        _state_machine.userdata.max_effort = 40.0
        _state_machine.userdata.approach_index = 0
        _state_machine.userdata.retreat_index = 0
        _state_machine.userdata.open = 0.035

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]


        # [/MANUAL_CREATE]

        with _state_machine:
            # x:163 y:59
            OperatableStateMachine.add('pcl_snapshot_test',
                                       GetPointCloudServiceState(service_timeout=5.0,
                                                                 service_name='/get_point_cloud',
                                                                 camera_topic='/rgbd_camera/points',
                                                                 target_frame='panda_link0'),
                                       transitions={'finished': 'PassthroughFilterPCL'  # 394 416 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 266 228 -1 -1 -1 -1
                                                    },
                                       autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'camera_pose': 'camera_pose',
                                                  'cloud_out': 'scene_pointcloud',
                                                  'cloud_frame': 'cloud_frame'})

            # x:2112 y:63
            OperatableStateMachine.add('CloseGripper',
                                       GripperCommandActionState(timeout_sec=5.0,
                                                                 action_name_fmt='/{robot_name}_hand_controller/gripper_cmd'),
                                       transitions={'success': 'MTCRetreatAndPlace'  # 2305 73 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 1889 328 -1 -1 -1 -1
                                                    },
                                       autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'robot_name': 'robot_name',
                                                  'position': 'close',
                                                  'max_effort': 'max_effort'})

            # x:1351 y:62
            OperatableStateMachine.add('ComputePoses',
                                       GPDGraspPosesServiceState(service_timeout=5.0,
                                                                 service_name='/compute_grasp_poses'),
                                       transitions={'done': 'MoveToNamedGoalMGI'  # 1556 74 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 1305 257 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'grasp_configs': 'grasp_configs',
                                                  'grasp_target_poses': 'grasp_target_poses',
                                                  'grasp_approach_poses': 'grasp_approach_poses',
                                                  'grasp_retreat_poses': 'grasp_retreat_poses',
                                                  'grasp_waypoints': 'grasp_waypoints'})

            # x:885 y:56
            OperatableStateMachine.add('DetectGrasps',
                                       DetectGraspsServiceState(service_timeout=5.0,
                                                                service_name='/detect_grasps'),
                                       transitions={'done': 'ComputePoses'  # 1209 27 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 969 234 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'cloud': 'scene_pointcloud',
                                                  'camera_source': 'camera_source',
                                                  'view_points': 'camera_pose',
                                                  'indices': 'test_indices',
                                                  'grasp_configs': 'grasp_configs'})

            # x:398 y:64
            OperatableStateMachine.add('EuclideanClusteringPCL',
                                       EuclideanClusteringServiceState(service_timeout=5.0,
                                                                       service_name='/euclidean_clustering',
                                                                       cluster_tolerance=0.02,
                                                                       min_cluster_size=100,
                                                                       max_cluster_size=25000),
                                       transitions={'finished': 'FilterByIndicesPCL'  # 609 80 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 677 232 -1 -1 -1 -1
                                                    },
                                       autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'cloud_in': 'scene_pointcloud',
                                                  'camera_pose': 'camera_pose',
                                                  'target_cluster_indices': 'target_cluster_indices',
                                                  'obstacle_cluster_indices': 'obstacle_cluster_indices'})

            # x:648 y:59
            OperatableStateMachine.add('FilterByIndicesPCL',
                                       FilterByIndicesServiceState(service_timeout=5.0,
                                                                   service_name='/filter_by_indices'),
                                       transitions={'finished': 'DetectGrasps'  # 839 70 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 807 232 -1 -1 -1 -1
                                                    },
                                       autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'cloud_in': 'scene_pointcloud',
                                                  'target_indices': 'target_cluster_indices',
                                                  'cloud_out': 'point_cloud_visual'})

            # x:1865 y:64
            OperatableStateMachine.add('MTCApproachAndPick',
                                       MTCApproachAndPickActionState(timeout_sec=5.0,
                                                                     action_name='mtc_approach_and_pick'),
                                       transitions={'success': 'CloseGripper'  # 2077 74 -1 -1 -1 -1
                                                    , 'next': 'MTCApproachAndPick'  # 1972 194 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 1620 319 -1 -1 -1 -1
                                                    },
                                       autonomy={'success': Autonomy.Off,
                                                 'next': Autonomy.Off,
                                                 'failed': Autonomy.Off},
                                       remapping={'grasp_approach_poses': 'grasp_approach_poses',
                                                  'grasp_target_poses': 'grasp_target_poses',
                                                  'grasp_retreat_poses': 'grasp_retreat_poses',
                                                  'object_id': 'object_id',
                                                  'grasp_index': 'approach_index'})

            # x:2341 y:61
            OperatableStateMachine.add('MTCRetreatAndPlace',
                                       MTCRetreatAndPlaceActionState(timeout_sec=5.0,
                                                                     action_name='mtc_retreat_and_place'),
                                       transitions={'success': 'MoveToStart'  # 2570 78 -1 -1 -1 -1
                                                    , 'next': 'MTCRetreatAndPlace'  # 2464 193 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 2127 331 -1 -1 -1 -1
                                                    },
                                       autonomy={'success': Autonomy.Off,
                                                 'next': Autonomy.Off,
                                                 'failed': Autonomy.Off},
                                       remapping={'grasp_approach_poses': 'grasp_approach_poses',
                                                  'grasp_target_poses': 'grasp_target_poses',
                                                  'grasp_retreat_poses': 'grasp_retreat_poses',
                                                  'object_id': 'object_id',
                                                  'grasp_index': 'retreat_index'})

            # x:48 y:209
            OperatableStateMachine.add('MoveSnapshot',
                                       MoveToNamedPoseServiceState(service_timeout=5.0,
                                                                   service_name='/move_to_named_pose'),
                                       transitions={'finished': 'pcl_snapshot_test'  # 111 140 -1 -1 -1 -1
                                                    , 'failure': 'failed'  # 120 339 -1 -1 -1 -1
                                                    },
                                       autonomy={'finished': Autonomy.Off, 'failure': Autonomy.Off},
                                       remapping={'target_pose_name': 'snapshot_pose'})

            # x:1592 y:65
            OperatableStateMachine.add('MoveToNamedGoalMGI',
                                       MoveToNamedPoseServiceState(service_timeout=5.0,
                                                                   service_name='/move_to_named_pose'),
                                       transitions={'finished': 'MTCApproachAndPick'  # 1818 81 -1 -1 -1 -1
                                                    , 'failure': 'failed'  # 1482 278 -1 -1 -1 -1
                                                    },
                                       autonomy={'finished': Autonomy.Off, 'failure': Autonomy.Off},
                                       remapping={'target_pose_name': 'inspect_pose'})

            # x:2612 y:98
            OperatableStateMachine.add('MoveToStart',
                                       MoveToNamedPoseServiceState(service_timeout=5.0,
                                                                   service_name='/move_to_named_pose'),
                                       transitions={'finished': 'finished'  # 2657 329 -1 -1 -1 -1
                                                    , 'failure': 'failed'  # 2291 359 -1 -1 -1 -1
                                                    },
                                       autonomy={'finished': Autonomy.Off, 'failure': Autonomy.Off},
                                       remapping={'target_pose_name': 'start_pose'})

            # x:518 y:485
            OperatableStateMachine.add('PassthroughFilterPCL',
                                       PassthroughServiceState(service_timeout=5.0,
                                                               service_name='/passthrough_filter',
                                                               lower_limit=0.13,
                                                               upper_limit=1.0,
                                                               field='z'),
                                       transitions={'finished': 'DetectGrasps'  # 807 433 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 249 488 -1 -1 -1 -1
                                                    },
                                       autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'cloud_in': 'scene_pointcloud',
                                                  'cloud_filtered': 'scene_pointcloud'})

            # x:1135 y:60
            OperatableStateMachine.add('PublishPointCloud',
                                       PublishPointCloudState(pub_topic='/filtered_cloud/target_object'),
                                       transitions={'done': 'ComputePoses'  # 1307 74 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 1145 244 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'cloud_in': 'point_cloud_visual'})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]


    # [/MANUAL_FUNC]
