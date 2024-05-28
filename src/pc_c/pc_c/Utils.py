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
            return np.reshape(np.array(output).astype(np.uint8),(LIDAR_HEIGHT, LIDAR_WIDTH,3))

        return np.array(output)

def convert_to_type(arr, T,p, is_rgb=False):
        arr = np.nan_to_num(arr)
        output = []
        if is_rgb:
            arr = arr.astype(np.uint8)
        for i in arr:
            el = T()
            setattr(el, p, i)
            output.append(el)
        return output
