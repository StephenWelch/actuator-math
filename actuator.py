from __future__ import annotations
from dataclasses import dataclass
import math
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

    def force_to_torque(self):
        # Make origins relative to joint origin.
        rotated_actuator_origins = self.actuator_origins - self.origin
        # For each row (actuator) in actuator_origins, rotate the actuator end point by the joint angle.
        for r in range(rotated_actuator_origins.shape[0]):
            rotated_actuator_origins[r][1] = self.joint_rot_matrix().apply(rotated_actuator_origins[r][1])

        # Get vector from start of actuator to rotated end of actuator.
        rotated_actuator_vecs = rotated_actuator_origins[:, 1] - rotated_actuator_origins[:, 0]
        # Get direction of each actuator
        rotated_actuator_dirs = rotated_actuator_vecs / np.linalg.norm(rotated_actuator_vecs, axis=1)[:, np.newaxis]
        # Get direction of torques and scale by forces
        torque = np.cross(rotated_actuator_dirs, rotated_actuator_origins[:, 1]).T.dot(self.actuator_forces.T)

        return torque

    def torque_to_force(self):
        # Make origins relative to joint origin.
        rotated_actuator_origins = self.actuator_origins - self.origin
        # For each row (actuator) in actuator_origins, rotate the actuator end point by the joint angle.
        for r in range(rotated_actuator_origins.shape[0]):
            rotated_actuator_origins[r][1] = self.joint_rot_matrix().apply(rotated_actuator_origins[r][1])

        # Get vector from start of actuator to rotated end of actuator.
        rotated_actuator_vecs = rotated_actuator_origins[:, 1] - rotated_actuator_origins[:, 0]
        # Get direction of each actuator
        rotated_actuator_dirs = rotated_actuator_vecs / np.linalg.norm(rotated_actuator_vecs, axis=1)[:, np.newaxis]

        torque_dirs = np.cross(rotated_actuator_dirs, rotated_actuator_origins[:, 1]).T
        force = np.linalg.lstsq(torque_dirs, self.torques)

        return force[0]