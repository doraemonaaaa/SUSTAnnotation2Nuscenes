import os
import shutil
import json
from collections import defaultdict


# 加载JSON数据的函数
def load_json(file_path):
    with open(file_path, 'r') as f:
        return json.load(f)

# 合并所有JSON文件的函数
def merge_json_files(folders, output_dir):
    merged_data = {
        "sample_annotation": [],
        "instance": [],
    }

    # 遍历每个文件夹
    for folder in folders:
        print(f"正在处理文件夹: {folder}")
        
        # 定义需要合并的文件路径
        sample_annotation_path = os.path.join(folder, 'sample_annotation.json')
        instance_path = os.path.join(folder, 'instance.json')

        # 检查文件是否存在，如果存在则加载并合并
        if os.path.exists(sample_annotation_path):
            merged_data["sample_annotation"].extend(load_json(sample_annotation_path))
        if os.path.exists(instance_path):
            merged_data["instance"].extend(load_json(instance_path))

    return merged_data

# 将合并后的数据保存到指定目录的JSON文件
def save_merged_data(merged_data, output_dir):
    # 确保输出目录存在
    os.makedirs(output_dir, exist_ok=True)

    # 保存sample_annotation数据
    sample_annotation_output_dir = os.path.join(output_dir, 'sample_annotation')
    os.makedirs(sample_annotation_output_dir, exist_ok=True)
    with open(os.path.join(sample_annotation_output_dir, 'sample_annotation.json'), 'w') as f:
        json.dump(merged_data["sample_annotation"], f, indent=4)
    print(f"sample_annotation 数据已保存至: {sample_annotation_output_dir}/sample_annotation.json")

    # 保存instance数据
    instance_output_dir = os.path.join(output_dir, 'instance')
    os.makedirs(instance_output_dir, exist_ok=True)
    with open(os.path.join(instance_output_dir, 'instance.json'), 'w') as f:
        json.dump(merged_data["instance"], f, indent=4)
    print(f"instance 数据已保存至: {instance_output_dir}/instance.json")

# 输入需要合并的文件夹路径（请根据实际情况替换）
data_root = 'data/Dataset/'
folders = [
    data_root + 'AIR_B1_merged_data',
    data_root + 'AIR_F1_merged_data',
    data_root + 'AIR_F11_merged_data',
    data_root + 'AIR_G_merged_data'
    # data_root + '20250123_115904',
    # data_root + '20250123_120013',
    # data_root + '20250123_120119',
    # data_root + '20250123_120533',
    # data_root + '20250123_120637',
    # data_root + '20250123_120804',
    # data_root + '20250123_120911',
    # data_root + 'NuscenesData'
]

# 指定合并后数据保存的输出目录
output_dir = data_root + "merged_data"

# 合并数据并保存
merged_data = merge_json_files(folders, output_dir)
save_merged_data(merged_data, output_dir)
