from dataclasses import dataclass
import numpy as np
from typing import List

@dataclass
class JointData:
    joint_origin: np.ndarray
    dof: np.ndarray
    actuator_origins: List[np.ndarray]

    actuator_forces: List[float] 
    joint_torques: np.ndarray

    def __post_init__(self):
        self.actuator_forces = [0.0] * len(self.actuator_origins)
        self.joint_torques = np.zeros(self.dof.shape)

RL1_HIP_YAW_ROL = JointData(
    np.array([-119, 0, -275]),
    np.array([0, 0, 1]),
    [
        np.array([-47, 60, -54], [-162, 59, -263]),
        np.array([-47, -60, -54], [-160, -62, -263])
    ]
)

RL3_HIP_PIT = JointData(
    np.array([-119, 1.7, -330])
    np.array([0, 1, 0]),
    [
        np.array([-87, -44, -343], [-87, 87, -638])
    ]
)

RL4_KNE_PIT = JointData(
    np.array([-119, 2.5, -778]), 
    np.array([0, 1, 0]),
    [
        np.array([-119, -44, -408], [-119, -57, -834])
    ]
)

RL5_ANK = JointData(
    np.array([-119, 6, -1213]),
    np.array([1, 1, 0]),
    [  
        np.array([-63, 48, -866], [-65, -67, -1188]),
        np.array([-175, 48, -866], [-175, -67, -1188])
    ]
)

def force_to_torque(joint: JointData):
    ...

def torque_to_force(joint: JointData):
    ...