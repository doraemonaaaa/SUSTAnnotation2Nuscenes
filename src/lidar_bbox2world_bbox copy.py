import numpy as np
from pyquaternion import Quaternion

# 将 XYZ 欧拉角转换为四元数
def euler_to_quaternion(euler_angles):
    # 欧拉角单位转换 (角度 -> 弧度)
    euler_radians = np.radians(euler_angles)

    # 依次按 'xyz' 顺序旋转
    qx = Quaternion(axis=[1, 0, 0], angle=euler_radians[0])
    qy = Quaternion(axis=[0, 1, 0], angle=euler_radians[1])
    qz = Quaternion(axis=[0, 0, 1], angle=euler_radians[2])
    
    # 欧拉角顺序 'xyz' -> 组合
    return qx * qy * qz

# tested, it is all right for nuscenes
def LidarBbox2WorldBbox(ego_world_translation, ego_world_rotation, box_ego_translation, box_ego_rotation, box_size):
    # 如果是 list，先转换为 Quaternion
    if isinstance(ego_world_rotation, list):
        ego_world_rotation = Quaternion(ego_world_rotation[0], 
                                        ego_world_rotation[1], 
                                        ego_world_rotation[2], 
                                        ego_world_rotation[3])
    box_ego_rotation = euler_to_quaternion(box_ego_rotation)
    # 🚀 计算 box 的世界坐标位置
    box_ego_translation = np.array(box_ego_translation)  # 确保是 numpy 数组
    if box_ego_translation.shape != (3,):
        raise ValueError(f"box_ego_translation 维度错误: {box_ego_translation}")
    
    # 🚗 Ego 坐标系到世界坐标系的数据
    # ego_world_translation = np.array([-0.25, -0.1, 0.0])
    # ego_world_rotation = Quaternion(0.0, 0.0, -0.446, -0.895)  # wxyz
    # box_ego_translation = np.array([-2.028576874592424, 1.74543679954891, 0.16528985829391607])
    # box_ego_rotation = Quaternion(axis=[0, 0, 1], angle=-2.661399808201459 * np.pi / 180)  # 转为弧度

    # 🚀 计算 box 的世界坐标位置
    box_world_translation = ego_world_rotation.rotate(box_ego_translation) + ego_world_translation
    box_world_rotation = ego_world_rotation * box_ego_rotation
    
    # 高度为中心, 不需要这个就是正确的，加上是错的
    #box_world_translation[2] += box_size[2] / 2

    # print("Box World Translation:", box_world_translation)
    # print("Box World Rotation (Quaternion, wxyz):", box_world_rotation)
    
    return box_world_translation, box_world_rotation

# 🚘 世界坐标系回到车辆坐标系
# relative_translation = ego_world_rotation.inverse.rotate(box_world_translation - ego_world_translation)
# relative_rotation = ego_world_rotation.inverse * box_world_rotation

# print("车辆坐标系下 box translation:", relative_translation)
# print("车辆坐标系下 box rotation (pyquaternion):", relative_rotation)

ego_world_translation = np.array([-0.3,  -0.05,  0.  ])
ego_world_rotation = Quaternion(0.000, -0.000, -0.214, -0.977,)  # wxyz
box_ego_translation = [-1.3873980082226454, 0.9474732190356734, 0.13990111370899494]
box_ego_rotation =(0.036062884343860534, -0.012542445859294874, 3.0687047448861198)
box_size = [0.75, 0.53, 1.59]
print(LidarBbox2WorldBbox(ego_world_translation, ego_world_rotation, box_ego_translation, box_ego_rotation, box_size))