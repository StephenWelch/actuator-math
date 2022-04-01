from __future__ import annotations
from dataclasses import dataclass
import math
import numpy as np
from typing import List

import pandas as pd
from scipy.spatial.transform import Rotation as R
from mpl_toolkits.mplot3d.axes3d import Axes3D
import plotly.graph_objects as go
import plotly.express as px

@dataclass
class JointData:
    name: str
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

    def plot_matplotlib(self, ax: Axes3D) -> List:
        origin = self.rotated_origin()
        ax.scatter3D(origin[0], origin[1], origin[2])
        actuator_origins = self.rotated_actuator_origins()
        for i in range(len(actuator_origins)):
            ax.plot3D(
                actuator_origins[i, :, 0].flatten(),
                actuator_origins[i, :, 1].flatten(),
                actuator_origins[i, :, 2].flatten())

    def plot_origins_plotly(self, fig: go.Figure):
        origin = self.rotated_origin()
        fig.add_scatter3d(
            name=self.name + ' Origin',
            x=origin[0:1],
            y=origin[1:2],
            z=origin[2:3],
            marker=dict(size=6, color='red')
        )
        return fig

    def plot_actuators_plotly(self, fig: go.Figure):
        actuator_origins = self.rotated_actuator_origins()
        for i in range(len(actuator_origins)):
            fig.add_scatter3d(
                name=self.name + ' Actuator',
                x=actuator_origins[i, :, 0].flatten(),
                y=actuator_origins[i, :, 1].flatten(),
                z=actuator_origins[i, :, 2].flatten(),
                marker=dict(size=4),
                line=dict(color='blue')
            )

    def joint_rot_matrix(self) -> R:
        return R.from_euler('XYZ', self.angles)

    def rotated_origin(self) -> np.ndarray:
        # Rotate origin based on parent joint angle
        if self.parent:
            return self.parent.joint_rot_matrix().apply(
                self.origin - self.parent.rotated_origin()) + self.parent.rotated_origin()

        return self.origin

    def rotated_actuator_origins(self, chain: bool = True) -> np.ndarray:
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

        # Remove torques from unused DoFs
        # torque_setpt = self.torques
        torque_dirs = np.delete(torque_dirs, self.dof == 0, axis=0)
        torque_setpt = np.delete(self.torques, self.dof == 0, axis=0)

        # force = np.linalg.lstsq(torque_dirs, torque_setpt, rcond=None)
        force = np.linalg.inv(torque_dirs) @ torque_setpt

        return force[0]
