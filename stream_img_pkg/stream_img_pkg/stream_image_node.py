#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
File    : stream_image_node.py
Author  : FantasyWilly
Email   : bc697522h04@gmail.com
SPDX-License-Identifier: Apache-2.0 

功能總覽:
    • 從 RTSP, RTMP, HTTP 串流獲取影像來源
    • 透過 Cv_Bridge 將影像傳至 ROS2

遵循:
    • Google Python Style Guide
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
# RTSPImgBridge 獲取影像來源並傳至 ROS2
# ------------------------------------------------------------------------------------ #
class RTSPImgBridge(Node):
    def __init__(self):

        # 1-1. 初始化 Node 節點
        super().__init__('stream_image_node')

        # 1-2. 宣告 RTSP URL 參數
        self.declare_parameter(
            'rtsp_url', 
            'rtsp://user:user@192.168.168.108:554/cam/realmonitor?channel=1&subtype=0')
        self.rtsp_url = self.get_parameter('rtsp_url').get_parameter_value().string_value
        self.get_logger().info(f'使用 RTSP URL: {self.rtsp_url}')

        # 1-3. 初始化 ROS2 Publisher
        self.publisher_ = self.create_publisher(
            Image,
            '/stream/camera/image_raw',
            qos_profile_sensor_data
        )

        # 1-4. 初始化 CvBridge
        self.bridge = CvBridge()

        # 2. 開啟 RTSP 串流
        self.open_stream()

        # 3.1 從裝置直接獲取 FPS or 採用固定幀率
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

    # ------------------------------ 打開 RTSP 影像串流 -------------------------------- #
    def open_stream(self):
        self.get_logger().info('[資訊]: 嘗試開啟 RTSP 串流...')
        self.cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
        time.sleep(0.5)
        if not self.cap.isOpened():
            self.get_logger().error(f'[失敗]: 無法開啟 RTSP 串流 {self.rtsp_url}')
        else:
            self.get_logger().info('[成功]: RTSP 串流已開啟')

    # -------------------------------- 發布影像至 ROS2 --------------------------------- #
    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('[警告]: 讀取影格失敗，嘗試重連...')
            self.cap.release()
            time.sleep(1.0)
            self.open_stream()
            return

        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(img_msg)
            self.get_logger().debug('已發布影像至 ROS2 Topic')
        except Exception as e:
            self.get_logger().error(f'[錯誤]: 轉換或發布影像失敗: {e}')

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
    node = RTSPImgBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
