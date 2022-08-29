import joint_defs
from joint_defs import ALL_JOINTS
import numpy as np

if __name__ == '__main__':
    # [pitch, yaw, roll]

    # j = joint_defs.RL4_KNE_PIT
    # j.actuator_forces = np.array(10)
    # j.torques = j.force_to_torque()
    # calculated_force = j.torque_to_force()
    # print(calculated_force)
    # print(j.torques)
    knee = joint_defs.RL4_KNE_PIT
    knee.torques = np.array([10, 0, 0])
    knee.actuator_forces = knee.torque_to_force()
    print(f"Knee @ 0: {knee.actuator_forces}")

    knee.angles[0] = -np.pi / 4
    knee.actuator_forces = knee.torque_to_force()
    print(f"Knee @ -pi/4 {knee.actuator_forces}")

    hip = joint_defs.RL3_HIP_PIT
    hip.torques = np.array([10, 0, 0])
    hip.actuator_forces = hip.torque_to_force()
    print(f"Hip @ 0: {hip.actuator_forces}")

    hip.angles[0] = -np.pi / 4
    hip.actuator_forces = hip.torque_to_force()
    print(f"Hip @ -pi/4: {hip.actuator_forces}")


    ankle = joint_defs.RL5_ANK
    ankle.torques = np.array([10, 0, 0])
    ankle.actuator_forces = ankle.torque_to_force()
    print(f"Ankle: {ankle.actuator_forces}")