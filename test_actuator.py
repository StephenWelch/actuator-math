from unittest import TestCase

import numpy as np
import pytest

import joint_defs
from joint_defs import ALL_JOINTS


class ActuatorTest(TestCase):

    def test_single_actuator_force_to_torque(self):
        for j in joint_defs.get_n_actuator_joints(1):
            j.actuator_forces = np.array(10)
            calculated_torque = j.force_to_torque()
            j.torques = calculated_torque
            calculated_force = j.torque_to_force()
            assert calculated_force == pytest.approx(10)

    def test_dual_actuator_force_to_torque(self):
        for j in joint_defs.get_n_actuator_joints(2):
            j.actuator_forces = np.array([10, 10])
            calculated_torque = j.force_to_torque()
            j.torques = calculated_torque
            calculated_force = j.torque_to_force()
            assert np.allclose(np.array([10, 10]), calculated_force)
