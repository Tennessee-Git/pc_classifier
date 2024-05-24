import numpy as np
from pc_c.Constants import LIDAR_HEIGHT, LIDAR_WIDTH

def convert_to_array(arr, is_rgb=False):
        output = []
        arr = np.array(arr)
        for i in arr:
            output.append(i.p)

        if is_rgb:
            extra_len = (LIDAR_HEIGHT * LIDAR_WIDTH) - len(output)
            for j in range(extra_len):
                output.append([0,0,0])
            return np.reshape(output,(LIDAR_HEIGHT, LIDAR_WIDTH,3))

        return np.array(output)
