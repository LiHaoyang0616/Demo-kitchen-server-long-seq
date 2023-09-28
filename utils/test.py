import numpy as np
from transforms3d.euler import quat2euler

# Initial and final quaternions
quat_start = np.array([-0.137, -0.238, -0.643, 0.715])
quat_end = np.array([-0.448, -0.509, -0.540, 0.499])

# Time steps to evaluate
steps = [0.1, 0.3, 0.5, 0.7, 0.9, 1]



def slerp_quat(t, quat_start, quat_end, t_start, t_end):
    # Normalize the quaternions just to be safe
    quat_start = quat_start / np.linalg.norm(quat_start)
    quat_end = quat_end / np.linalg.norm(quat_end)

    # Compute the cosine of the angle between the two vectors.
    dot = np.dot(quat_start, quat_end)

    # If the dot product is negative, slerp won't take the shorter path.
    # Note that v1 and -v1 are equivalent when the negation is applied to all four components. 
    if dot < 0.0:
        quat_end = -quat_end
        dot = -dot

    DOT_THRESHOLD = 0.9995
    if dot > DOT_THRESHOLD:
        # If the inputs are too close for comfort, linearly interpolate
        # and normalize the result.
        result = quat_start + t * (quat_end - quat_start)
        return result / np.linalg.norm(result)

    # Since dot is in range [0, DOT_THRESHOLD], acos is safe
    theta_0 = np.arccos(dot)  # theta_0 = angle between input vectors
    theta = theta_0 * t  # theta = angle between v0 and result
    sin_theta = np.sin(theta)  # compute this value only once
    sin_theta_0 = np.sin(theta_0)  # compute this value only once

    s0 = np.cos(theta) - dot * sin_theta / sin_theta_0  # == sin(theta_0 - theta) / sin(theta_0)
    s1 = sin_theta / sin_theta_0
    return (s0 * quat_start) + (s1 * quat_end)

for t in steps:
    arm_rotation = slerp_quat(t, quat_start, quat_end, 0, 1)
    print(f"t = {t}: {arm_rotation}")



point = quat2euler([0.212608, -0.540643, 0.455746, 0.674387])
print(point)

import numpy as np

phase_step = [0.02421990908009605, 0.0, 0.7833338077143464, 1.0, 0.0, 0.3609856411517955, 0.2873391576500961, 0.0, 0.5369905264180647, 0.6640551729499461, 0.9039964574003815, 0.3806308588575755, 0.011439338854525097, 0.0, 0.5737054175466745, 0.0, 0.4585268324188888, 0.040990964228715634, 0.0, 0.30719391215589786, 0.032649779647291065, 0.0]

phase_step = np.array(phase_step)
phase_step = np.where(phase_step == 0, 0.5, phase_step)
result = (500 * phase_step).astype(int)

# 给定列表

phase_steps = np.array([112, 300, 391, 300, 300, 180, 143, 300, 268, 332, 451, 390, 500, 450, 286, 400, 229, 200, 300, 153, 160, 300])

# 初始值
start_value = 5

# 初始化一个空列表来存储每段区间的结尾值
end_values = []

# 计算每段区间的结尾值并添加到列表中
current_value = start_value
for step in phase_steps:
    current_value += step
    end_values.append(current_value)

# 将结果转化为NumPy数组（可选）
end_values = np.array(end_values)

# 打印每段区间的结尾值
print(end_values)


# import imageio.v2 as imageio
# import os
# image_folder = 'images/ability'  
# images = [img for img in os.listdir(image_folder) if img.endswith(".png")]
# images.sort()  
# image_files = [imageio.imread(os.path.join(image_folder, img)) for img in images]
# imageio.mimsave('pickup_sponge_chuanmo.gif', image_files, duration=1000/10) 