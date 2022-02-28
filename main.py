import math
from typing import Tuple
import numpy as np
import matplotlib.pyplot as plt
from actuator import ALL_JOINTS, RL4_KNE_PIT, JointData, force_to_torque, torque_to_force
from mpl_toolkits import mplot3d
from matplotlib.widgets import Slider, Button

    

def main():
    ax = plt.axes(projection='3d')
    plt.subplots_adjust(bottom=0.25)

    def plot_all():
        ax.axes.set_xlim3d(left=-500, right=500) 
        ax.axes.set_ylim3d(bottom=-500, top=500) 
        ax.axes.set_zlim3d(bottom=-1500, top=0) 
        for joint in ALL_JOINTS:
            joint.plot(ax)

        limits = np.array([getattr(ax, f'get_{axis}lim')() for axis in 'xyz'])
        ax.set_box_aspect(np.ptp(limits, axis = 1))

    def add_angle_slider(ax, joint: JointData, label: str, range: Tuple[float, float], axis: int):
        slider = Slider(
                ax=plt.axes([0.25, 0.1, 0.65, 0.03]),
                label=label,
                valmin=range[0],
                valmax=range[1],
                valinit=0,
        )

        def update(val):
            joint.angles[axis] = math.radians(val)
            ax.clear()
            plot_all()

            t = force_to_torque(joint)
            joint.torques = t
            f = torque_to_force(joint)
            print(f"Angle: {val}")
            print(f"Torque: {t}")
            print(f"Force: {f}")
            
        slider.on_changed(update)
        return slider

    def add_force_slider(ax, joint: JointData, label: str, range: Tuple[float, float], actuator: int):
        slider = Slider(
                ax=plt.axes([0.1, 0.25, 0.0225, 0.63]),
                label=label,
                valmin=range[0],
                valmax=range[1],
                valinit=0,
                orientation='vertical'
        )

        def update(val):
            joint.actuator_forces[actuator] = val
            force_setpoint = val
            t = force_to_torque(joint)
            joint.torques = t
            f = torque_to_force(joint)
            ax.clear()
            plot_all()

            print(f'Set Force: {force_setpoint}')
            print(f'Torque: {t}')
            print(f'Calculated Force: {f}')
            
        slider.on_changed(update)
        return slider

    knee_angle_slider = add_angle_slider(ax, RL4_KNE_PIT, 'Knee Pitch', (-90, 90), 0)
    knee_force_slider = add_force_slider(ax, RL4_KNE_PIT, 'Knee Force', (0, 100), 0)
    

    plot_all()

    plt.show()

if __name__ == '__main__':
    main()