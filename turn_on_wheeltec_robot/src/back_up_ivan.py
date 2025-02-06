#!/usr/bin/env python3
from rclpy.node import Node
from zlac8015d import ZLAC8015D
from geometry_msgs.msg import Twist, TransformStamped
from tf_transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
from pymodbus.exceptions import ModbusIOException
from tf2_ros import TransformBroadcaster
from wheeltec_robot_msg.msg import Data


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
        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)

        # 初始化控制器
        self.motorAB = ZLAC8015D.Controller(port=self.usart_port_name_0)
        self.motorCD = ZLAC8015D.Controller(port=self.usart_port_name_1)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info("TransformBroadcaster initialized")

        # 初始化位置和速度
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.Vx, self.Vy, self.Vz = 0.0, 0.0, 0.0
        # self.prev_time = time.time()
        self.prev_time = self.get_clock().now().nanoseconds * 1e-9  # 使用 ROS 時間

        # 初始化計時器
        self.timer_period = 0.033
        self.timer = self.create_timer(self.timer_period, self.Vel_to_Dis)

        # 設置加速度和減速度時間
        self.motorAB.set_accel_time(500, 500)
        self.motorAB.set_decel_time(1000, 1000)
        self.motorCD.set_accel_time(500, 500)
        self.motorCD.set_decel_time(1000, 1000)

        #Data
        self.robotpose_publisher = self.create_publisher(Data, 'robotpose', 10)
        self.robotvel_publisher = self.create_publisher(Data, 'robotvel', 10)

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
        current_time = self.get_clock().now().nanoseconds * 1e-9 # ROS 時間，轉換為秒
        # current_time = time.time()
        dt = current_time - self.prev_time

        self.get_Vel_From_Encoder()

        delta_x = (self.Vx * math.cos(self.theta) - self.Vy * math.sin(self.theta)) * dt
        delta_y = (self.Vx * math.sin(self.theta) + self.Vy * math.cos(self.theta)) * dt
        delta_theta = self.Vz * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta)) #ivan change

        self.prev_time = current_time

        # 回傳位置資訊
        self.publish_odometry()
        self.publish_tf()
        # self.get_logger().info(f"Position updated: x={self.x}, y={self.y}, theta={self.theta}")

    def get_quaternion_from_yaw(self, yaw):
        # qw = math.cos(yaw / 2.0)
        # qx = 0.0
        # qy = 0.0
        # qz = math.sin(yaw / 2.0)
        return quaternion_from_euler(0, 0, yaw)

    def publish_odometry(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.robot_frame_id

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        qw, qx, qy, qz = self.get_quaternion_from_yaw(self.theta)
        odom_msg.pose.pose.orientation.w = qw
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz


        odom_msg.twist.twist.linear.x = self.Vx
        odom_msg.twist.twist.linear.y = self.Vy
        odom_msg.twist.twist.angular.z = self.Vz

        # 發布 robotpose 消息
        pose_msg = Data()
        pose_msg.x = self.x
        pose_msg.y = self.y
        pose_msg.z = self.theta
        self.robotpose_publisher.publish(pose_msg)

        # 發布 robotvel 消息
        vel_msg = Data()
        vel_msg.x = self.Vx
        vel_msg.y = self.Vy
        vel_msg.z = self.Vz
        self.robotvel_publisher.publish(vel_msg)

        self.publisher_.publish(odom_msg)

    

    def publish_tf(self):
        # 創建 TransformStamped 消息
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.odom_frame_id  # 父座標系
        tf_msg.child_frame_id = self.robot_frame_id  # 子座標系

        # 設定位置
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0

        # 設定旋轉 (Quaternion)
        qw, qx, qy, qz = self.get_quaternion_from_yaw(self.theta)
        tf_msg.transform.rotation.w = qw
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz

        # 發布 TF
        self.tf_broadcaster.sendTransform(tf_msg)

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
