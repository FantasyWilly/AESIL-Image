#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
File    : webcam_image_node.py
Author  : FantasyWilly
Email   : bc697522h04@gmail.com
SPDX-License-Identifier: Apache-2.0 

功能總覽:
    • 獲取 Webcam 影像來源
    • 透過 Cv_Bridge 將影像傳至 ROS2

遵循:
    • Google Python Style Guide (含區段標題)
    • PEP 8 (行寬 ≤ 88, snake_case, 2 空行區段分隔)
'''

# ------------------------------------------------------------------------------------ #
# Import
# ------------------------------------------------------------------------------------ #
# 標準庫
import time

# 第三方套件
import cv2

# ROS 2 Python API
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


# ------------------------------------------------------------------------------------ #
# USBImgBridge 獲取影像來源並傳至 ROS2
# ------------------------------------------------------------------------------------ #
class USBImgBridge(Node):
    def __init__(self):

        # 1-1. 初始化 Node 節點
        super().__init__('webcam_image_node')

        # 1-2. 宣告 CAMERA DEVICE 參數
        self.declare_parameter(
            'camera_device', 
            '/dev/video0')
        self.camera_device = self.get_parameter('camera_device').get_parameter_value().string_value
        self.get_logger().info(f'[使用]: CAMERA DEVICE: {self.camera_device}')

        # 1-3. 初始化 ROS2 Publisher
        self.publisher_ = self.create_publisher(
            Image, 
            '/webcam/camera/image_raw', 
            qos_profile_sensor_data
        )

        # 1-4. 初始化 CvBridge
        self.bridge = CvBridge()

        # 2. 獲取影像來源
        self.cap = cv2.VideoCapture(self.camera_device)
        if not self.cap.isOpened():
            self.get_logger().error('[失敗]: Failed to open /dev/video0')
            return

        time.sleep(0.5)

        # 3.1 從裝置直接獲取 FPS
        getattr_fps = self.cap.get(cv2.CAP_PROP_FPS)

        if getattr_fps and getattr_fps > 0:
            self.get_logger().info(f'[FPS]: {getattr_fps:.2f}')
            timer_period = 1.0 / getattr_fps
        else:
            fps = 30.0
            self.get_logger().warning(f'[失敗]: 無法從裝置獲取FPS, 採用預設值 {fps} FPS')
            timer_period = 1.0 / fps

        # 3.2 從裝置直接獲取影像大小
        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(f'[解析度]: {width} x {height}')

        # 4. 每秒呼叫 計數器函數
        self.timer = self.create_timer(timer_period, self.timer_callback)

    # -------------------------------- 發布影像至 ROS2 --------------------------------- #
    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('[失敗]: Failed to read frame from /dev/video0')
            return
        
        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(img_msg)
            self.get_logger().debug('發布影像至 ROS2')
        except Exception as e:
            self.get_logger().error(f'[錯誤]: Error converting/publishing image: {e}')

    # ------------------------------ 關閉影像 & 釋放資源 -------------------------------- #
    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


# ------------------------------------------------------------------------------------ #
# Entry point
# ------------------------------------------------------------------------------------ #
def main(args=None):
    rclpy.init(args=args)
    node = USBImgBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()