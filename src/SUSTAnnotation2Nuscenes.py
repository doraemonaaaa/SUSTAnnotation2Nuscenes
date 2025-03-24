import os
import re
import json
import hashlib
import sys
from typing import Optional, List, Dict
import numpy as np
from scipy.spatial.transform import Rotation as R

# 将 src 添加到模块搜索路径
sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))
from utils import *
from lidar_bbox2world_bbox import *

epoch_name = "NuscenesData"
dataset_foldet = r'data/Dataset/' + epoch_name  # 根路径，可修改
sust_annotations_folder = r'data/SUSTAnnotation/' + epoch_name  # 根路径，可修改

lidar_folder = dataset_foldet + r'/samples/LIDAR_TOP'
sample_data_path = dataset_foldet + r'/sample_data.json'
sample_json_path = dataset_foldet + r'/sample.json'
ego_pose_path = r'data/Dataset/ego_pose.json'
attribute_define_json = r'data/attribute.json'


sample_annotation_output_path = r'data/sample_annotation.json'
instance_output_path = r'data/instance.json'

sample_annotation_prev_next_sliders = {}  # 缓存生成annotation的prev next
instance_jsons_cache = {}  # 缓存instance记录然后一并生成

# 映射表
mapping = {
    "sitting,static": "sitting_lying_down",
    "static,sitting": "sitting_lying_down",
    "sitting": "sitting_lying_down",
    "static": "standing"
}

# 获取所有雷达文件名
lidar_file_names = os.listdir(lidar_folder)
# 加载 sample_data.json 文件
with open(sample_data_path, 'r', encoding='utf-8') as f:
    sample_data_json_data = json.load(f)
# 加载 sample.json 文件
with open(sample_json_path, 'r', encoding='utf-8') as f:
    sample_json_data = json.load(f)
# 加载 ego_pose.json 文件
with open(ego_pose_path, 'r', encoding='utf-8') as f:
    ego_pose_json_data = json.load(f)
# 加载 attribute.json 文件
with open(attribute_define_json, 'r', encoding='utf-8') as f:
    attrubute_json_data = json.load(f)
    
# 排序雷达文件名
lidar_file_names_sorted = sorted(  # 提取时间戳并排序（按主时间戳和次时间戳组成的元组排序）
    lidar_file_names,
    key=lambda x: tuple(map(int, x.split("__")[-1].split(".")[0].split("_")))
)
# for idx, filename in enumerate(lidar_file_names_sorted):  # 打印排序结果（序号从0开始）
#     print(f"Index {idx}: {filename}")

# 获取所有顺序排好的标注JSON 文件名
sust_annotation_file_names_sorted = sorted(os.listdir(sust_annotations_folder), key=lambda x: int(x.split('.')[0]))  # 按文件名排序
# for idx, filename in enumerate(sust_annotation_file_names_sorted):  # 打印排序结果（序号从0开始）
#     print(f"Index {idx}: {filename}")

for idx, filename in enumerate(sust_annotation_file_names_sorted):
    sust_annotations_file_path = os.path.join(sust_annotations_folder, filename)
    print(f"Processing file: {sust_annotations_file_path}")  # 打印当前处理的文件路径

    try:
        with open(sust_annotations_file_path, 'r', encoding='utf-8') as f:
            annotation_json_datas = json.load(f)
    except json.JSONDecodeError as e:
        print(f"Error in file: {sust_annotations_file_path}")
        print(f"Error details: {e}")
        with open(sust_annotations_file_path, 'r', encoding='utf-8') as f:
            print(f"File content: {f.read()}")
        raise  # 重新抛出异常，终止程序
        
    # 提取 objects 列表
    objects = annotation_json_datas.get('objects', [])
    if not objects:
        continue
    else:
        for obj in objects:
            obj_attr = obj.get('obj_attr')  # 提取 obj_attr
            obj_id = obj.get('obj_id')
            obj_type = obj.get('obj_type')
            psr = obj.get('psr', {})  # 提取 psr 字段
            position = psr.get('position')  # 提取 position
            rotation = psr.get('rotation')  # 提取 rotation
            scale = psr.get('scale')  # 提取 scale
            #visibility = obj.get('visibility')  # 提取 visibility
            num_lidar_pts = obj.get('num_lidar_pts')  # 提取 num_lidar_pts
            
            #if num_lidar_pts == 0 or num_lidar_pts is None:
            num_lidar_pts = 9999999
            num_radar_pts = 9999999  # 固定值为 9999999
            
            obj_type = obj_type.lower()  # 字符串全部小写
            obj_attr = mapping.get(obj_attr, obj_attr)  # 如果匹配，映射SUST属性到nuscenes属性, 如果没找到映射，就保持原值
            
            box_size = convert_to_list(scale)
            box_ego_translation = convert_to_list(position)
            box_ego_rotation = convert_to_list(rotation)
            
            if None in [obj_attr, obj_id, obj_type, box_ego_translation, box_ego_rotation, box_size]:
                print(f"Error: obj_attr={obj_attr}, obj_id={obj_id}, obj_type={obj_type}, box_ego_translation={box_ego_translation}, box_ego_rotation={box_ego_rotation}, box_size={box_size}, num_lidar_pts={num_lidar_pts}")
                break
            
            object_name = f"{obj_type}{obj_id}"  # 例如 "Pedestrian1"
            lidar_file_name = "samples/LIDAR_TOP/" + lidar_file_names_sorted[idx]
            
            '''BUG'''
            # 由于数据的bug，将_转换为:,__不被替换
            # 先将 '__' 替换为一个临时标记，比如 '__TEMP__'
            temp_file_name = lidar_file_name.replace("__", "'")
            # 然后将单个 '_' 替换为 ':'
            corrected_file_name = temp_file_name.replace("_", ":")
            # 最后将临时标记 '__TEMP__' 替换回 '__'
            corrected_file_name = corrected_file_name.replace("'", "__") 
            s = corrected_file_name
            # 1. 从后往前查找最后两个 ':'，并替换倒数两个 ':'
            parts = s.rsplit(":", 2)
            if len(parts) >= 3:
                s = ":".join(parts[:-2]) + "_" + parts[-2] + "_" + parts[-1]
            # 2. 然后将第一个 ':' 替换为 '_'
            first_colon_index = s.find(":")
            if first_colon_index != -1:
                s = s[:first_colon_index] + "_" + s[first_colon_index+1:]
            corrected_file_name = s

            # 获得instance token, sample_token
            corr_sample_data_json = find_json_with_key("filename", corrected_file_name, sample_data_json_data)
            #print(annotation_json_obj)
            sample_token = corr_sample_data_json["sample_token"]
            #print(sample_token)
            corr_sample_json = find_json_with_key("token", sample_token, sample_json_data)
            scene_token = corr_sample_json["scene_token"]
            #print(scene_token)
            instance_token = generate_token(object_name=object_name, scene_token=scene_token)
            #print(instance_token)
            
            # 获得attribute_token
            attribute = obj_type + '.' + obj_attr
            atrribute_json = find_json_with_key("name", attribute, attrubute_json_data)
            if atrribute_json is None: 
                print("atrrbute not found", attribute)
                raise ValueError
            attribute_token  = atrribute_json["token"]
            #print(attribute_token)
            
            # 计算世界坐标
            ego_pose_token = corr_sample_data_json["ego_pose_token"]
            ego_pose_json = find_json_with_key("token", ego_pose_token, ego_pose_json_data)
            ego_world_translation = ego_pose_json["translation"]
            ego_world_rotation = ego_pose_json["rotation"]
            box_world_translation, box_world_rotation = LidarBbox2WorldBbox(ego_world_translation, ego_world_rotation, box_ego_translation, box_ego_rotation, box_size)
            box_world_translation = box_world_translation.tolist()
            box_world_rotation = list(box_world_rotation.elements)
            
            # 生成sample_annotation_token
            sample_annotation_token = generate_unique_token()
            
            sample_annotation_json = {
                "attribute_tokens" : [attribute_token],
                "instance_token" : instance_token,
                "num_lidar_pts" : num_lidar_pts,
                "num_radar_pts" : num_radar_pts,
                "sample_token" : sample_token,
                "rotation" : box_world_rotation,
                "translation" : box_world_translation,
                "size" : box_size,
                "visibility_token" : "4",
                "token" : sample_annotation_token,
                "prev" : "",
                "next" : ""
            }
            # 处理sample_annotation的生成
            generate_prev_next_and_write_json_dynamically(sample_annotation_prev_next_sliders, instance_token, sample_annotation_json, sample_annotation_output_path)
            # 处理instance的生成
            update_instance_json(instance_jsons_cache=instance_jsons_cache, instance_token=instance_token, sample_annotation_token=sample_annotation_token)

# 完成后处理sample_annotation和instance的缓存
for key, json_list in sample_annotation_prev_next_sliders.items():
    # 遍历每个 json 列表
    for json_data in json_list:
        # 检查每个 json，如果需要写入，调用 add_key_value_pair_to_json
        add_key_value_pair_to_json(sample_annotation_output_path, json_data, True)

for key, json_data in instance_jsons_cache.items():
    add_key_value_pair_to_json(instance_output_path, json_data, True)


#sample_data_json = find_json_with_key()
