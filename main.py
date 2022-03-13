import math
from typing import Tuple
import numpy as np
import matplotlib.pyplot as plt
from actuator import JointData
from joint_defs import RL4_KNE_PIT, RL5_ANK, ALL_JOINTS
from mpl_toolkits import mplot3d
from matplotlib.widgets import Slider, Button

ax = plt.axes(projection='3d')
plt.subplots_adjust(bottom=0.25)

horizontal_slider_ct = 1
vertical_slider_ct = 1

def log_calc(joint):
        ...

def plot_all():
    ax.axes.set_xlim3d(left=-500, right=500) 
    ax.axes.set_ylim3d(bottom=-500, top=500) 
    ax.axes.set_zlim3d(bottom=-1500, top=0) 
    for joint in ALL_JOINTS:
        joint.plot(ax)

    limits = np.array([getattr(ax, f'get_{axis}lim')() for axis in 'xyz'])
    ax.set_box_aspect(np.ptp(limits, axis = 1))

def add_angle_slider(ax, joint: JointData, label: str, range: Tuple[float, float], axis: int):
    global horizontal_slider_ct
    slider = Slider(
            ax=plt.axes([0.25, 0.05 * horizontal_slider_ct, 0.65, 0.03]),
            label=label,
            valmin=range[0],
            valmax=range[1],
            valinit=0,
    )

    def update(val):
        joint.angles[axis] = math.radians(val)
        ax.clear()
        plot_all()
        log_calc(joint)
        
    slider.on_changed(update)
    horizontal_slider_ct += 1
    return slider

def add_dual_force_slider(ax, joint: JointData, label: str, magnitude_range: Tuple[float, float], angular_range: Tuple[float, float], actuator_a: int, actuator_b: int) -> Tuple[Slider, Slider]:
    magnitude_slider = Slider(
            ax=plt.axes([0.1, 0.25, 0.0225, 0.63]),
            label=label + ' magnitude',
            valmin=magnitude_range[0],
            valmax=magnitude_range[1],
            valinit=0,
            orientation='vertical'
    ) 
    angular_slider = Slider(
            ax=plt.axes([0.1, 0.25, 0.0225, 0.63]),
            label=label + ' angular',
            valmin=angular_range[0],
            valmax=angular_range[1],
            valinit=0,
            orientation='vertical'
    )

    def magnitude_update(val):
        joint.actuator_forces[actuator_a] = val
        log_calc(joint)
    
    def angular_update(val):
        joint.actuator_forces[actuator_a] = magnitude_slider.val - val
        joint.actuator_forces[actuator_b] = magnitude_slider.val + val
        log_calc(joint)
    
    magnitude_slider.on_changed(magnitude_update)
    angular_slider.on_changed(angular_update)


def add_single_force_slider(ax, joint: JointData, label: str, magnitude_range: Tuple[float, float], actuator: int):
    slider = Slider(
            ax=plt.axes([0.1, 0.25, 0.0225, 0.63]),
            label=label,
            valmin=magnitude_range[0],
            valmax=magnitude_range[1],
            valinit=0,
            orientation='vertical'
    )

    def update(val):
        joint.actuator_forces[actuator] = val
        force_setpoint = val
        t = joint.force_to_torque()
        joint.torques = t
        f = joint.torque_to_force()
        ax.clear()
        plot_all()

        print(f'Set Force: {force_setpoint}')
        print(f'Torque: {t}')
        print(f'Calculated Force: {f}')
        
    slider.on_changed(update)
    return slider

def main():
    knee_angle_slider = add_angle_slider(ax, RL4_KNE_PIT, 'Knee Pitch', (-90, 90), 0)
    knee_force_slider = add_single_force_slider(ax, RL4_KNE_PIT, 'Knee Force', (0, 100), 0)
    ankle_pitch_slider = add_angle_slider(ax, RL5_ANK, 'Ankle Pitch', (-90, 90), 0)

    plot_all()
    plt.show()

if __name__ == '__main__':
    main()