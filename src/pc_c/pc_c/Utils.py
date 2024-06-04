import numpy as np
from pc_c.Constants import LIDAR_HEIGHT, LIDAR_WIDTH

def convert_to_array(arr):
        output = []
        arr = np.array(arr)
        for i in arr:
            output.append(i.p)

        return np.array(output)

def convert_to_type(arr, T,p):
        arr = np.nan_to_num(arr)
        output = []
        for i in arr:
            el = T()
            setattr(el, p, i)
            output.append(el)
        return output
