from __future__ import annotations
from dataclasses import dataclass
import math
import matplotlib
import numpy as np
from typing import List
from scipy.spatial.transform import Rotation as R
from mpl_toolkits.mplot3d.axes3d import Axes3D

def rotate(origin, point, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in radians.
    """
    ox, oy = origin
    px, py = point

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return qx, qy

@dataclass
class JointData:
    parent: JointData
    origin: np.ndarray
    dof: np.ndarray
    actuator_origins: np.ndarray

    actuator_forces: np.ndarray = np.array([])
    torques: np.ndarray = np.array([])
    angles: np.ndarray = np.array([])

    def __post_init__(self):
        self.origin = np.array(self.origin)
        self.dof = np.array(self.dof)
        self.actuator_origins = np.array(self.actuator_origins)

        self.actuator_forces = np.zeros(len(self.actuator_origins))
        self.torques = np.zeros(self.dof.shape)
        self.angles = np.zeros(self.dof.shape)

    def num_dof(self) -> int:
        return self.dof.count_nonzero()

    def plot(self, ax: Axes3D) -> List:
        origin = self.rotated_origin()
        ax.scatter3D(origin[0], origin[1], origin[2])
        actuator_origins = self.rotated_actuator_origins()
        for i in range(len(actuator_origins)):
            ax.plot3D(
                actuator_origins[i, :, 0].flatten(),
                actuator_origins[i, :, 1].flatten(), 
                actuator_origins[i, :, 2].flatten())

    def joint_rot_matrix(self) -> R:
        return R.from_euler('XYZ', self.angles)
        
    def rotated_origin(self) -> np.ndarray:
        # Rotate origin based on parent joint angle
        if self.parent:
            return self.parent.joint_rot_matrix().apply(self.origin - self.parent.rotated_origin()) + self.parent.rotated_origin()

        return self.origin

    def rotated_actuator_origins(self, chain: bool = True) -> np.ndarray:
        actuator_origins = self.actuator_origins

        actuator_origins = self.actuator_origins - self.origin
        for r in range(actuator_origins.shape[0]):
            actuator_origins[r][1] = self.joint_rot_matrix().apply(actuator_origins[r][1])
        actuator_origins = actuator_origins + self.origin
        
        if self.parent and chain:
            actuator_origins = actuator_origins - self.parent.rotated_origin()
            for r in range(actuator_origins.shape[0]):
                actuator_origins[r, :] = self.parent.joint_rot_matrix().apply(actuator_origins[r, :])
            actuator_origins = actuator_origins + self.parent.rotated_origin()

        return actuator_origins



RL1_HIP_YAW_ROL = JointData(
    None,
    [-119, 0, -275],
    [0, 0, 1],
    [
        [[-47, 60, -54], [-162, 59, -263]],
        [[-47, -60, -54], [-160, -62, -263]]
    ]
)

RL3_HIP_PIT = JointData(
    RL1_HIP_YAW_ROL,
    [-119, 1.7, -330],
    [0, 1, 0],
    [
        [[-87, -44, -343], [-87, 87, -638]]
    ]
)

RL4_KNE_PIT = JointData(
    RL3_HIP_PIT,
    [-119, 2.5, -778], 
    [0, 1, 0],
    [
        [[-119, -44, -408], [-119, -57, -834]]
    ]
)

RL5_ANK = JointData(
    RL4_KNE_PIT,
    [-119, 6, -1213],
    [1, 1, 0],
    [  
        [ [-63, 48, -866], [-65, -67, -1188] ],
        [ [-175, 48, -866], [-175, -67, -1188] ]
    ]
)

ALL_JOINTS = [RL1_HIP_YAW_ROL, RL3_HIP_PIT, RL4_KNE_PIT, RL5_ANK]

def force_to_torque(joint: JointData):
    # Make origins relative to joint origin.
    rotated_actuator_origins = joint.actuator_origins - joint.origin
    # For each row (actuator) in actuator_origins, rotate the actuator end point by the joint angle.
    for r in range(rotated_actuator_origins.shape[0]):
        rotated_actuator_origins[r][1] = joint.joint_rot_matrix().apply(rotated_actuator_origins[r][1])

    # Get vector from start of actuator to rotated end of actuator.
    rotated_actuator_vecs = rotated_actuator_origins[:, 1] - rotated_actuator_origins[:, 0]
    # Get direction of each actuator
    rotated_actuator_dirs = rotated_actuator_vecs / np.linalg.norm(rotated_actuator_vecs, axis=1)
    # Get force applied to each actuator
    actuator_forces = rotated_actuator_dirs * joint.actuator_forces
    # Get torque applied by each actuator to the joint
    torque = np.cross(rotated_actuator_origins[:, 1], actuator_forces)
    return torque


def torque_to_force(joint: JointData):
    # Make origins relative to joint origin.
    rotated_actuator_origins = joint.actuator_origins - joint.origin
    # For each row (actuator) in actuator_origins, rotate the actuator end point by the joint angle.
    for r in range(rotated_actuator_origins.shape[0]):
        rotated_actuator_origins[r][1] = joint.joint_rot_matrix().apply(rotated_actuator_origins[r][1])

    # Get vector from start of actuator to rotated end of actuator.
    rotated_actuator_vecs = rotated_actuator_origins[:, 1] - rotated_actuator_origins[:, 0]
    # Get direction of each actuator
    rotated_actuator_dirs = rotated_actuator_vecs / np.linalg.norm(rotated_actuator_vecs, axis=1)

    # Rotated end position x torque
    cross = np.cross(rotated_actuator_origins[:, 1], joint.torques)
    cross = cross / rotated_actuator_origins[:, 1].dot(rotated_actuator_origins[:, 1].T)
    # print(rotated_actuator_origins[:, 1])
    # print(rotated_actuator_origins[:, 1].dot(rotated_actuator_origins[:, 1].T))
    return np.linalg.norm(cross) / (cross / np.linalg.norm(cross)).dot(rotated_actuator_dirs.T)