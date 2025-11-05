# webcam_image_launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_file_path = os.path.join(
        get_package_share_directory('usb_img_pkg'),
        'config',
        'webcam_image.yaml'
    )

    return LaunchDescription([
        Node(
            package='usb_img_pkg',               # package 名稱
            executable='webcam_image_node',        # 在 setup.py 裡註冊的執行檔或 entry point 名稱
            name='webcam_image_node',              # 對節點重新命名
            output='screen',
            parameters=[config_file_path],         # 指定要讀取的參數檔
        )
    ])
