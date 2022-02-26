import math
import numpy as np
import matplotlib.pyplot as plt
from actuator import ALL_JOINTS, RL4_KNE_PIT, JointData, force_to_torque, torque_to_force
from mpl_toolkits import mplot3d
from matplotlib.widgets import Slider, Button



def main():
    ax = plt.axes(projection='3d')
    plt.subplots_adjust(bottom=0.25)

    knee_pitch = Slider(
        ax=plt.axes([0.25, 0.1, 0.65, 0.03]),
        label='Knee Angle',
        valmin=-90,
        valmax=90,
        valinit=0,
    )

    def plot_all():
        ax.axes.set_xlim3d(left=-500, right=500) 
        ax.axes.set_ylim3d(bottom=-500, top=500) 
        ax.axes.set_zlim3d(bottom=-1500, top=0) 
        for joint in ALL_JOINTS:
            joint.plot(ax)

        limits = np.array([getattr(ax, f'get_{axis}lim')() for axis in 'xyz'])
        ax.set_box_aspect(np.ptp(limits, axis = 1))

    def update_knee_angle(val):
        RL4_KNE_PIT.angles[0] = math.radians(knee_pitch.val)
        ax.clear()
        plot_all()

    knee_pitch.on_changed(update_knee_angle)

    plot_all()

    plt.show()

if __name__ == '__main__':
    main()