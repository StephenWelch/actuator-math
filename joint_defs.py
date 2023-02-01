from __future__ import annotations

from typing import List

from actuator import JointData

RL1_HIP_YAW_ROL = JointData(
    'Left Hip Yaw/Roll',
    None,
    [-119, 0, -275],
    [0, 0, 1],
    [
        [[-47, 60, -54], [-162, 59, -263]],
        [[-47, -60, -54], [-160, -62, -263]]
    ]
)
RL3_HIP_PIT = JointData(
    'Left Hip Pitch',
    RL1_HIP_YAW_ROL,
    [-119, 1.7, -330],
    [1, 1, 1],
    [
        [[-87, -44, -343], [-87, 87, -638]]
    ]
)
RL4_KNE_PIT = JointData(
    'Left Knee Pitch',
    RL3_HIP_PIT,
    [-119, 2.5, -778],
    [1, 0, 0],
    [
        [[-119, -44, -408], [-119, -57, -834]]
    ]
)
RL5_ANK = JointData(
    'Left Ankle Pitch/Roll',
    RL4_KNE_PIT,
    [-119, 6, -1213],
    [1, 1, 0],
    [
        [ [-63, 48, -866], [-63, -67, -1188] ],
        [ [-175, 48, -866], [-175, -67, -1188] ]
    ]
)
ALL_JOINTS = [RL1_HIP_YAW_ROL, RL3_HIP_PIT, RL4_KNE_PIT, RL5_ANK]

def get_n_actuator_joints(n: int) -> List[JointData]:
    return list(filter(lambda j: len(j.actuator_origins) == n, ALL_JOINTS))
