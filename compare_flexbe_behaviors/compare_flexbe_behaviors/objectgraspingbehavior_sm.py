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
Define ObjectGraspingBehavior.

sub-behavior for approach -> grasp -> retreat objective

Created on Sat Oct 18 2025
@author: Brian Flynn
"""


from compare_flexbe_states.gripper_command_action_state import GripperCommandActionState
from compare_flexbe_states.move_to_named_pose_service_state import MoveToNamedPoseServiceState
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


class ObjectGraspingBehaviorSM(Behavior):
    """
    Define ObjectGraspingBehavior.

    sub-behavior for approach -> grasp -> retreat objective
    """

    def __init__(self, node):
        super().__init__()
        self.name = 'ObjectGraspingBehavior'

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
        # x:1234 y:108, x:311 y:400
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['open_position', 'closed_position', 'max_effort', 'robot_name'])
        _state_machine.userdata.open_position = 0.035
        _state_machine.userdata.closed_position = 0.0
        _state_machine.userdata.max_effort = 50.0
        _state_machine.userdata.robot_name = 'panda'
        _state_machine.userdata.inspect_pose = 'inspect'
        _state_machine.userdata.ready_pose = 'ready'

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]


        # [/MANUAL_CREATE]

        with _state_machine:
            # x:124 y:80
            OperatableStateMachine.add('MoveToInspect',
                                       MoveToNamedPoseServiceState(service_timeout=5.0,
                                                                   service_name='/move_to_named_pose'),
                                       transitions={'finished': 'OpenGripper'  # 329 102 -1 -1 -1 -1
                                                    , 'failure': 'failed'  # 203 306 -1 -1 -1 -1
                                                    },
                                       autonomy={'finished': Autonomy.Off, 'failure': Autonomy.Off},
                                       remapping={'target_pose_name': 'inspect_pose'})

            # x:791 y:81
            OperatableStateMachine.add('CloseGripper',
                                       GripperCommandActionState(timeout_sec=5.0,
                                                                 action_name_fmt='/{robot_name}_hand_controller/gripper_cmd'),
                                       transitions={'success': 'finished'  # 1067 110 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 878 291 -1 -1 -1 -1
                                                    },
                                       autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'robot_name': 'robot_name',
                                                  'position': 'closed_position',
                                                  'max_effort': 'max_effort'})

            # x:554 y:80
            OperatableStateMachine.add('MoveToReady',
                                       MoveToNamedPoseServiceState(service_timeout=5.0,
                                                                   service_name='/move_to_named_pose'),
                                       transitions={'finished': 'CloseGripper'  # 759 96 -1 -1 -1 -1
                                                    , 'failure': 'failed'  # 622 287 -1 -1 -1 -1
                                                    },
                                       autonomy={'finished': Autonomy.Off, 'failure': Autonomy.Off},
                                       remapping={'target_pose_name': 'ready_pose'})

            # x:348 y:81
            OperatableStateMachine.add('OpenGripper',
                                       GripperCommandActionState(timeout_sec=5.0,
                                                                 action_name_fmt='/{robot_name}_hand_controller/gripper_cmd'),
                                       transitions={'success': 'MoveToReady'  # 532 100 -1 -1 -1 -1
                                                    , 'failed': 'failed'  # 399 295 -1 -1 -1 -1
                                                    },
                                       autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'robot_name': 'robot_name',
                                                  'position': 'open_position',
                                                  'max_effort': 'max_effort'})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]


    # [/MANUAL_FUNC]
