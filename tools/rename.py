import os
import shutil
import numpy as np

def modify_and_rename_files_in_folders(workspace, folders):
    """ 遍历 workspace 下的指定文件夹，执行以下操作：
        1. 删除第一个文件
        2. 复制最后一个文件并粘贴
        3. 按 原来的80个名字依次赋名
        result： 让所属folder的文件快一帧，但是最后一帧为假数据
    """
    for folder in folders:
        folder_path = os.path.join(workspace, folder)
        if not os.path.exists(folder_path):
            print(f"Warning: {folder_path} does not exist.")
            continue
        
        # 获取所有文件名
        fold_file_names = os.listdir(folder_path)
        # 排序文件名
        fold_file_names_sorted = sorted(  # 提取时间戳并排序（按主时间戳和次时间戳组成的元组排序）
            fold_file_names,
            key=lambda x: tuple(map(int, x.split("__")[-1].split(".")[0].split("_")))
        )
        # for idx, filename in enumerate(lidar_file_names_sorted):  # 打印排序结果（序号从0开始）
        #     print(f"Index {idx}: {filename}")

        files = sorted(os.listdir(folder_path))  # 获取所有文件

        if len(files) == 0:
            print(f"Skipping {folder}: No files found.")
            continue

        # 删除第一个文件
        first_file = os.path.join(folder_path, files[0])
        os.remove(first_file)
        print(f"Deleted: {first_file}")

        # 复制最后一个文件
        files = sorted(os.listdir(folder_path))  # 重新获取文件列表
        if len(files) > 0:
            last_file = os.path.join(folder_path, files[-1])
            last_file_copy = os.path.join(folder_path, f"copy_{files[-1]}")
            shutil.copy(last_file, last_file_copy)
            print(f"Copied: {last_file} -> {last_file_copy}")

        # 重新获取文件列表并重命名
        files = sorted(os.listdir(folder_path))  # 重新获取文件列表
        for idx, filename in enumerate(files, start=1):
            ext = os.path.splitext(filename)[1]  # 获取文件扩展名
            new_name = fold_file_names_sorted[idx-1]  # 01, 02, ..., 99
            old_path = os.path.join(folder_path, filename)
            new_path = os.path.join(folder_path, new_name)
            os.rename(old_path, new_path)
            print(f"Renamed: {old_path} -> {new_path}")


if __name__ == "__main__":
    
    workspace = r"E:\DataSet\实车\dataset_2025_1_22_20.21\AIR_G\20250122_194559\samples"  # 修改为实际的工作空间路径
    cam_folders = ["CAM_FRONT", "CAM_BACK_RIGHT", "CAM_BACK", "CAM_BACK_LEFT", "CAM_FRONT_LEFT", "CAM_FRONT_RIGHT"]
    
    modify_and_rename_files_in_folders(workspace, cam_folders)
   
