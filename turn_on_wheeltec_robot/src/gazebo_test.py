#!/usr/bin/env python3
from rclpy.node import Node
from zlac8015d import ZLAC8015D
from geometry_msgs.msg import Twist, TransformStamped
from tf_transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
from pymodbus.exceptions import ModbusIOException
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import JointState

import time
import math
import rclpy
import numpy as np

WHEEL_DIAMETER = 180 / 1000      # 馬達直徑（mm）
WHEEL_SPACING = 650 / 1000       # 馬達前後軸距（mm）
WHEEL_AXLESPACING = 661 / 1000   # 馬達左右軸距（mm）


class ReadOdomVelToDis(Node):
    def __init__(self):
        super().__init__('gazebo_test')

        self.declare_parameter('robot_frame_id', 'base_link')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('cmd_vel', 'cmd_vel')

        self.robot_frame_id = self.get_parameter('robot_frame_id').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel').value

        # 訂閱 cmd_vel 話題
        self.subscription = self.create_subscription(Twist, self.cmd_vel_topic, self.move_callback, 10)

        #訂閱 /joint_states 話題
        self.subscription = self.create_subscription(JointState, 'joint_states', self.get_Vel_From_Encoder, 10)

        # 創建 odom 話題的發佈器
        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info("TransformBroadcaster initialized")

        # 初始化位置和速度
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.Vx, self.Vy, self.Vz = 0.0, 0.0, 0.0
        self.prev_time = time.time()

        # 初始化計時器
        self.timer_period = 1.0 / 10.0
        self.timer = self.create_timer(self.timer_period, self.Vel_to_Dis)

    def get_Vel_From_Encoder(self, msg: JointState):
        joint_names = msg.name
        joint_velocities = msg.velocity

        if len(joint_velocities) < 4:
            return  # 確保我們獲取了足夠的數據

        # 根據 Gazebo 的 Joint 順序對應速度
        self.MOTOR_A_Encoder = joint_velocities[joint_names.index("left_wheel_joint")]
        self.MOTOR_B_Encoder = joint_velocities[joint_names.index("right_wheel_joint")]
        self.MOTOR_C_Encoder = joint_velocities[joint_names.index("left_front_joint")]
        self.MOTOR_D_Encoder = joint_velocities[joint_names.index("right_front_joint")]

        # 計算機器人的 Vx, Vy, Vz
        self.Vx = -(self.MOTOR_A_Encoder + self.MOTOR_B_Encoder + self.MOTOR_C_Encoder + self.MOTOR_D_Encoder) / 4
        self.Vy = (-self.MOTOR_A_Encoder + self.MOTOR_B_Encoder + self.MOTOR_C_Encoder - self.MOTOR_D_Encoder) / 4
        self.Vz = (-self.MOTOR_A_Encoder + self.MOTOR_B_Encoder - self.MOTOR_C_Encoder + self.MOTOR_D_Encoder) / (2 * (WHEEL_AXLESPACING + WHEEL_SPACING))

    def Vel_to_Dis(self):
        # 這裡不應該直接呼叫 get_Vel_From_Encoder()
        # 因為 get_Vel_From_Encoder() 只應該被 /joint_states 訂閱時觸發

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
        odom_msg.pose.pose.orientation.w = quat[3]
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]

        self.get_logger().info(f"Orientation Quaternion - x: {quat[0]}, y: {quat[1]}, z: {quat[2]}, w: {quat[3]}")
        self.get_logger().info(f"Orientation: {odom_msg.pose.pose.orientation}")



        odom_msg.twist.twist.linear.x = self.Vx
        odom_msg.twist.twist.linear.y = 0.0 #self.Vy
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
        pass

    # def set_speeds(self, vx, vy, vz): # 速度轉換為轉速
    #     #vy = 0
    #     # vz = 0
    #     # if abs(vx) < 0.01: vx = 0
    #     #if abs(vz) < 0.01: vz = 0
    #     vz_linear = float(vz * ((WHEEL_SPACING + WHEEL_AXLESPACING) / 2))
    #     speed_A = int((vx - vy - vz_linear) * (30 / np.pi / self.motorAB.R_Wheel))
    #     speed_B = int((vx + vy + vz_linear) * (30 / np.pi / self.motorAB.R_Wheel))
    #     speed_D = int((vx - vy + vz_linear) * (30 / np.pi / self.motorCD.R_Wheel))
    #     speed_C = int((vx + vy - vz_linear) * (30 / np.pi / self.motorCD.R_Wheel))
    #     self.motorAB.set_rpm(speed_B, -speed_A)
    #     self.motorCD.set_rpm(speed_D, -speed_C)
    #     print("cmd=",vx,vy,vz)
    #     print(speed_A,speed_B,speed_C,speed_D)
    def on_shutdown(self):
        self.get_logger().info("Shutting down odometry node.")


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