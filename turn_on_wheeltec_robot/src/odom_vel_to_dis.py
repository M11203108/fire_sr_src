#!/usr/bin/env python3
from rclpy.node import Node
from zlac8015d import ZLAC8015D
from geometry_msgs.msg import Twist, TransformStamped
from tf_transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
from pymodbus.exceptions import ModbusIOException
from tf2_ros import TransformBroadcaster

import time
import math
import rclpy
import numpy as np

WHEEL_DIAMETER = 180 / 1000      # 馬達直徑（mm）
WHEEL_SPACING = 650 / 1000       # 馬達前後軸距（mm）
WHEEL_AXLESPACING = 661 / 1000   # 馬達左右軸距（mm）


class ReadOdomVelToDis(Node):
    def __init__(self):
        super().__init__('odom_vel_to_dis')

        # 宣告並獲取參數值
        self.declare_parameter('usart_port_name_0', '/dev/motorttyUSB0')
        self.declare_parameter('usart_port_name_1', '/dev/motorttyUSB1')
        self.declare_parameter('serial_baud_rate', 115200)
        self.declare_parameter('robot_frame_id', 'base_footprint')
        self.declare_parameter('odom_frame_id', 'odom_combined')
        self.declare_parameter('cmd_vel', 'cmd_vel')

        self.usart_port_name_0 = self.get_parameter('usart_port_name_0').value
        self.usart_port_name_1 = self.get_parameter('usart_port_name_1').value
        self.serial_baud_rate = self.get_parameter('serial_baud_rate').value
        self.robot_frame_id = self.get_parameter('robot_frame_id').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel').value

        # 訂閱 cmd_vel 話題
        self.subscription = self.create_subscription(Twist, self.cmd_vel_topic, self.move_callback, 10)

        # 創建 odom 話題的發佈器
        self.publisher_ = self.create_publisher(Odometry, 'odom_combined', 10)

        # 初始化控制器
        self.motorAB = ZLAC8015D.Controller(port=self.usart_port_name_0)
        self.motorCD = ZLAC8015D.Controller(port=self.usart_port_name_1)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info("TransformBroadcaster initialized")

        # 初始化位置和速度
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.Vx, self.Vy, self.Vz = 0.0, 0.0, 0.0
        self.prev_time = time.time()

        # 初始化計時器
        self.timer_period = 0.033
        self.timer = self.create_timer(self.timer_period, self.Vel_to_Dis)

        # 設置加速度和減速度時間
        self.motorAB.set_accel_time(500, 500)
        self.motorAB.set_decel_time(1000, 1000)
        self.motorCD.set_accel_time(500, 500)
        self.motorCD.set_decel_time(1000, 1000)

        # 設置模式並啟用電機
        self.motorAB.set_mode(3)
        self.motorCD.set_mode(3)
        self.motorAB.enable_motor()
        self.motorCD.enable_motor()

    def get_Vel_From_Encoder(self):
        try:
            self.MOTOR_A_Encoder, self.MOTOR_B_Encoder = self.motorAB.get_linear_velocities()
            self.MOTOR_C_Encoder, self.MOTOR_D_Encoder = self.motorCD.get_linear_velocities()
        except ModbusIOException as e:
            self.get_logger().error(f"Failed to read motor velocities: {e}")
            return

        # 更新編碼器方向
        self.MOTOR_A_Encoder = -self.MOTOR_A_Encoder
        self.MOTOR_B_Encoder = -self.MOTOR_B_Encoder
        self.MOTOR_C_Encoder = -self.MOTOR_C_Encoder
        self.MOTOR_D_Encoder = -self.MOTOR_D_Encoder

        # 計算速度
        self.Vx = -(self.MOTOR_A_Encoder + self.MOTOR_B_Encoder + self.MOTOR_C_Encoder + self.MOTOR_D_Encoder) / 4#-
        self.Vy = (-self.MOTOR_A_Encoder + self.MOTOR_B_Encoder + self.MOTOR_C_Encoder - self.MOTOR_D_Encoder) / 4
        self.Vz = (-self.MOTOR_A_Encoder + self.MOTOR_B_Encoder - self.MOTOR_C_Encoder + self.MOTOR_D_Encoder) / (2 * (WHEEL_AXLESPACING + WHEEL_SPACING))

    def Vel_to_Dis(self):
        self.get_Vel_From_Encoder()
        current_time = time.time()
        dt = current_time - self.prev_time

        delta_x = self.Vx * dt
        delta_y = self.Vy * dt
        delta_th = self.Vz * dt

        self.x += delta_x * math.cos(self.theta) - delta_y * math.sin(self.theta)
        self.y += delta_x * math.sin(self.theta) + delta_y * math.cos(self.theta)
        self.theta += delta_th

        self.prev_time = current_time

        # 回傳位置資訊
        self.publish_odometry()
        self.publish_tf()
        self.get_logger().info(f"Position updated: x={self.x}, y={self.y}, theta={self.theta}")

    def publish_odometry(self):
        quat = quaternion_from_euler(0, 0, self.theta)
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.robot_frame_id

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        odom_msg.twist.twist.linear.x = self.Vx
        odom_msg.twist.twist.linear.y = self.Vy
        odom_msg.twist.twist.angular.z = self.Vz

        self.publisher_.publish(odom_msg)

    def publish_tf(self):
        quat = quaternion_from_euler(0, 0, self.theta)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_frame_id
        t.child_frame_id = self.robot_frame_id
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(t)

    def move_callback(self, msg: Twist):
        vx, vy, vz = msg.linear.x, msg.linear.y, msg.angular.z
        self.set_speeds(vx, vy, vz)

    def set_speeds(self, vx, vy, vz): # 速度轉換為轉速
        #vy = 0
        # vz = 0
        # if abs(vx) < 0.01: vx = 0
        #if abs(vz) < 0.01: vz = 0
        vz_linear = float(vz * ((WHEEL_SPACING + WHEEL_AXLESPACING) / 2))
        speed_A = int((vx - vy - vz_linear) * (30 / np.pi / self.motorAB.R_Wheel))
        speed_B = int((vx + vy + vz_linear) * (30 / np.pi / self.motorAB.R_Wheel))
        speed_D = int((vx - vy + vz_linear) * (30 / np.pi / self.motorCD.R_Wheel))
        speed_C = int((vx + vy - vz_linear) * (30 / np.pi / self.motorCD.R_Wheel))
        self.motorAB.set_rpm(speed_B, -speed_A)
        self.motorCD.set_rpm(speed_D, -speed_C)
        print("cmd=",vx,vy,vz)
        print(speed_A,speed_B,speed_C,speed_D)
    def on_shutdown(self):
        self.motorAB.disable_motor()
        self.motorCD.disable_motor()


def main(args=None):
    rclpy.init(args=args)
    node = ReadOdomVelToDis()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
