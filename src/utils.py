import os
import re
import json
import hashlib
import sys
from typing import Optional, List, Dict
import numpy as np
from scipy.spatial.transform import Rotation as R
import uuid


# 定义生成 token 的函数
def generate_token(object_name: Optional[str] = None, scene_token: Optional[str] = None) -> str:
    """
    生成唯一令牌，需要物体名和场景名作为额外标识符。
    
    :param object_name: 物体名（可选）
    :param scene_token: 场景名（可选）
    :return: 唯一令牌（SHA256 哈希值）
    """
    unique_string = ""
    # 如果有物体名，则添加到字符串中
    if object_name is not None:
        unique_string += f"_{object_name}"
    # 如果有场景名，则添加到字符串中
    if scene_token is not None:
        unique_string += f"_{scene_token}"
    # 使用 SHA256 生成哈希值
    sha256_hash = hashlib.sha256(unique_string.encode()).hexdigest()
    return sha256_hash

def generate_unique_token() -> str:
    """
    生成一个唯一标识符，不需要任何输入。
    使用 UUID 来生成唯一标识符。
    
    :return: 唯一标识符（UUID）
    """
    unique_token = str(uuid.uuid4())  # 使用 UUID4 生成唯一标识符
    return unique_token


def find_json_with_key(search_key, search_value, json_datas)-> json:

    # 查找包含指定键值对的JSON对象
    result = None
    for obj in json_datas:
        if obj.get(search_key) == search_value:
            result = obj
            break
        
    return result

def convert_to_list(data):
    """将字典格式的坐标或旋转数据转换为列表格式 [x, y, z]，否则返回 None"""
    try:
        if isinstance(data, dict) and all(key in data for key in ['x', 'y', 'z']):
            return [data['x'], data['y'], data['z']]
    except Exception as e:
        print(f"Error processing data: {e}")
    return None


def add_key_value_pair_to_json(json_path, key_value_pairs, is_save):
    """
    将新的键值对添加到 JSON 数组中，并根据需要保存到文件。
    
    :param json_path: JSON 文件的路径
    :param key_value_pairs: 要添加的键值对数据
    :param is_save: 是否保存到文件
    """
    # 检查文件是否存在，如果存在就加载，否则初始化一个空列表
    if os.path.exists(json_path):
        with open(json_path, 'r', encoding='utf-8') as f:
            try:
                # 尝试加载现有的 JSON 数据
                existing_data = json.load(f)
                if not isinstance(existing_data, list):
                    existing_data = []  # 如果数据格式不是列表，则初始化为列表
            except json.JSONDecodeError:
                # 如果文件为空或无效，初始化为空列表
                existing_data = []
    else:
        # 如果文件不存在，则初始化为空列表
        existing_data = []

    # 将新的键值对添加到现有数据列表中
    existing_data.append(key_value_pairs)

    # 如果需要保存，则保存到文件
    if is_save:
        try:
            with open(json_path, 'w', encoding='utf-8') as f:
                json.dump(existing_data, f, ensure_ascii=False, indent=4)
            #print(f"Changes saved to {json_path}")
        except Exception as e:
            print(f"Failed to save changes to {json_path}: {e}")
    else:
        print("Data added but not saved.")

# 示例调用
def generate_prev_next_and_write_json_dynamically(prev_next_sliders, key, cache_json, write_json_path):
    """
    生成 prev 和 next 键，并将更新后的数据写入 JSON 文件
    :param prev_next_sliders: 存储每个键对应的 JSON 数据列表的字典
    :param key: 用于查找或初始化对应slider的键
    :param cache_json: 当前要添加的目标 JSON 数据
    :param write_json_path: 写入数据的目标 JSON 文件路径
    """
    # 获取或初始化与 key 相关的 slider 列表
    this_slider = prev_next_sliders.get(key, [])  # 如果不存在则初始化为空列表

    if len(this_slider) < 2:
        this_slider.append(cache_json)
        
        if len(this_slider) == 2:  # 当 slider 列表大小为 2 时处理
            # 设置第一个元素的 next 键为第二个元素的 token
            this_slider[0]["next"] = this_slider[1]["token"]
            # 设置第二个元素的 prev 键为第一个元素的 token
            this_slider[1]["prev"] = this_slider[0]["token"]
            
            # 将第一个元素写入 JSON 文件
            add_key_value_pair_to_json(write_json_path, this_slider[0], True)
            
            # 将第一个元素替换为第二个元素，并移除最后一个元素
            this_slider[0] = this_slider[1]  
            this_slider.pop()  # 弹出最后一个元素

    # 更新字典中的 prev_next_sliders 对应 key 的值
    prev_next_sliders[key] = this_slider
    
def update_instance_json(instance_jsons_cache, instance_token, sample_annotation_token):
    """
    更新实例的 JSON 数据：设置最后一个注释令牌并增加注释数量。

    :param instance_jsons_cache: 存储所有实例 JSON 数据的缓存（字典）
    :param instance_token: 当前实例的唯一标识符
    :param sample_annotation_token: 样本注释的唯一标识符
    """
    # 获取实例 JSON 数据，如果不存在则初始化
    json_data = instance_jsons_cache.get(instance_token)

    if json_data is None:
        # 如果实例 JSON 数据不存在，初始化一个带默认值的字典
        instance_jsons_cache[instance_token] = {
            "category_token": "1fa93b757fc74fb197cdd60001ad8abf",  # 默认成人类别的 category_token
            "first_annotation_token": sample_annotation_token,  # 第一个注释令牌
            "last_annotation_token": sample_annotation_token,   # 设置为第一个注释令牌
            "nbr_annotations": 1,  # 初始化注释数量为 1
            "token": instance_token  # 实例的唯一标识符
        }
    else:
        # 如果实例 JSON 数据已存在，更新 last_annotation_token 和 nbr_annotations
        json_data["last_annotation_token"] = sample_annotation_token  # 更新最后一个注释令牌
        json_data["nbr_annotations"] += 1  # 增加注释数量