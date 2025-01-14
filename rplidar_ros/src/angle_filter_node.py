#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math


class AngleFilterNode(Node):
    def __init__(self):
        super().__init__('angle_filter_node')

        # 參數設定：保留的角度範圍
        self.declare_parameter('min_angle', 0.0)  # 起始角度 (degrees)
        self.declare_parameter('max_angle', 80.0)  # 終止角度 (degrees)
        # self.declare_parameter('additional_min_angle', 190.0)  # 另一個起始角度 (degrees)
        # self.declare_parameter('additional_max_angle', 360.0)  # 另一個終止角度 (degrees)

        # 獲取參數
        self.min_angle = math.radians(self.get_parameter('min_angle').value)
        self.max_angle = math.radians(self.get_parameter('max_angle').value)
        # self.additional_min_angle = math.radians(self.get_parameter('additional_min_angle').value)
        # self.additional_max_angle = math.radians(self.get_parameter('additional_max_angle').value)

        # 訂閱原始 LaserScan 資料
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan_original',  # 原始數據話題
            self.scan_callback,
            10
        )

        # 發布處理後的 LaserScan 資料
        self.filtered_scan_pub = self.create_publisher(
            LaserScan,
            '/scan',  # 處理後的數據話題
            10
        )

        self.get_logger().info('AngleFilterNode 已啟動，正在篩選指定角度範圍內的數據')

    def scan_callback(self, msg: LaserScan):
        # 建立篩選後的 LaserScan 消息
        filtered_scan = LaserScan()
        filtered_scan.header = msg.header
        filtered_scan.angle_min = msg.angle_min
        filtered_scan.angle_max = msg.angle_max
        filtered_scan.angle_increment = msg.angle_increment
        filtered_scan.time_increment = msg.time_increment
        filtered_scan.scan_time = msg.scan_time
        filtered_scan.range_min = msg.range_min
        filtered_scan.range_max = msg.range_max

        # 篩選角度範圍
        filtered_ranges = []
        filtered_intensities = []

        current_angle = msg.angle_min
        for r, i in zip(msg.ranges, msg.intensities):
            # 如果角度在範圍內，保留數據
            if (self.min_angle <= current_angle <= self.max_angle):
            # or \
            #    (self.additional_min_angle <= current_angle <= self.additional_max_angle):
                filtered_ranges.append(r)
                filtered_intensities.append(i)
            else:
                # 否則設為無效數據
                filtered_ranges.append(float('inf'))
                filtered_intensities.append(0.0)

            # 更新當前角度
            current_angle += msg.angle_increment

        # 填充篩選後的數據
        filtered_scan.ranges = filtered_ranges
        filtered_scan.intensities = filtered_intensities

        # 發布篩選後的消息
        self.filtered_scan_pub.publish(filtered_scan)


def main(args=None):
    rclpy.init(args=args)
    node = AngleFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
