from unittest import TestCase

import numpy as np
import pytest

import joint_defs
from joint_defs import ALL_JOINTS


class ActuatorTest(TestCase):
    def test_single_actuator_force_to_torque(self):
        single_actuator_joints = list(filter(lambda j: len(j.actuator_origins) == 1, ALL_JOINTS))
        for j in single_actuator_joints:
            j = joint_defs.RL4_KNE_PIT
            j.actuator_forces = np.array(10)
            calculated_torque = j.force_to_torque()
            j.torques = calculated_torque
            calculated_force = j.torque_to_force()
            ...
            assert calculated_force == pytest.approx(10)
