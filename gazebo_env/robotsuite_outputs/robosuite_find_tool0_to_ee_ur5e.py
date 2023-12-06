import numpy as np

tool0_to_world = np.array(
    [
        [-1.0, 0, 0.002, 0.255],
        [0.002, 0.0, 1.0, -0.974],
        [0.0, 1.0, 0.0, -0.234],
        [0.000, 0.000, 0.000, 1.000],
    ]
)
world_to_ee = np.array(
    [
        [-1.0, 0, 0.002, 0.257],
        [0.002, 0.0, 1.0, 0.377],
        [0.0, 1.0, 0.0, 0.975],
        [0.000, 0.000, 0.000, 1.000],
    ]
)
print(tool0_to_world @ world_to_ee)