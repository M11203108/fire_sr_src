from rclpy.node import Node
from zlac8015d import ZLAC8015D
from geometry_msgs.msg import Twist , Quaternion
from tf_transformations import quaternion_from_euler
from nav_msgs.msg import Odometry

import time
import numpy as np
import math
import rclpy

class ReadOdomVelToDis(Node):
    def __init__(self):
        super().__init__('odom_vel_to_dis')
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.move_callback, 10)
        self.subscription
        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)

        self.motorAB = ZLAC8015D.Controller(port="/dev/ttyUSB0")
        self.motorCD = ZLAC8015D.Controller(port="/dev/ttyUSB1")
        self.Wheel_axlespacing  = 6.5 #前後軸距離
        self.Wheel_spacing = 6.6#左右輪距離

        self.cpr = 16385  #每转脉冲数，示例值
        self.travel_in_one_rev = 0.09 * 2 * math.pi

        self.prev_time = time.time()

      
    def get_wheels_travelled(self):
        registers = self.motorAB.modbus_fail_read_handler(self.motorAB.L_FB_POS_HI, 4)
        l_pul_hi = registers[0]
        l_pul_lo = registers[1]
        r_pul_hi = registers[2]
        r_pul_lo = registers[3]

        l_pulse = np.int32(((l_pul_hi & 0xFFFF) << 16) | (l_pul_lo & 0xFFFF))
        r_pulse = np.int32(((r_pul_hi & 0xFFFF) << 16) | (r_pul_lo & 0xFFFF))

        l_travelled = (float(l_pulse) / self.cpr) * self.travel_in_one_rev
        r_travelled = (float(r_pulse) / self.cpr) * self.travel_in_one_rev

        return l_travelled, r_travelled
    
    def get_linearV_travelled(self):
        SL_init,SR_init=self.get_wheels_travelled()#起始位置
        st=time.time()
        time.sleep(0.01)
        SL,SR=self.get_wheels_travelled()#檢測位置
        VL = (SL-SL_init)/(st-time.time())
        VR = (SR-SR_init)/(st-time.time())
        return VL, VR
    
    def get_Vel_From_Encoder(self): #取得速度
        self.MOTOR_A_Encoder, self.MOTOR_B_Encoder = self.motorAB.get_linear_velocities()#取線速度
        self.MOTOR_C_Encoder, self.MOTOR_D_Encoder = self.motorCD.get_linear_velocities()#取線速度
        self.MOTOR_A_Encoder = -self.MOTOR_A_Encoder#馬達安裝方向相反故改為負號#輪編號不同需更改
        self.MOTOR_B_Encoder = -self.MOTOR_B_Encoder#輪編號不同需更改
        self.MOTOR_C_Encoder = -self.MOTOR_C_Encoder#輪編號不同需更改
        self.MOTOR_D_Encoder = -self.MOTOR_D_Encoder#輪編號不同需更改
        print('V=',self.MOTOR_A_Encoder,self.MOTOR_B_Encoder,self.MOTOR_C_Encoder,self.MOTOR_D_Encoder)

        self.Vx = (self.MOTOR_A_Encoder + self.MOTOR_B_Encoder + self.MOTOR_C_Encoder + self.MOTOR_D_Encoder) / 4
        self.Vy = (-self.MOTOR_A_Encoder + self.MOTOR_B_Encoder + self.MOTOR_C_Encoder - self.MOTOR_D_Encoder) / 4
        self.Vz = (-self.MOTOR_A_Encoder + self.MOTOR_B_Encoder - self.MOTOR_C_Encoder + self.MOTOR_D_Encoder) / (4 * (self.Wheel_axlespacing + self.Wheel_spacing))

    def Vel_to_Dis(self):
        current_time = time.time()
        dt = current_time - self.prev_time
        delta_x = self.Vx * dt
        delta_y = self.Vy * dt
        delta_th = self.Vz * dt
        self.x += delta_x * math.cos(self.theta) - delta_y * math.sin(self.theta)
        self.y += delta_x * math.sin(self.theta) + delta_y * math.cos(self.theta)
        self.theta += delta_th
        
        self.prev_time = current_time

        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        quat = quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation = Quaternion(*quat)
        
        odom_msg.twist.twist.linear.x = self.Vx
        odom_msg.twist.twist.linear.y = self.Vy
        odom_msg.twist.twist.angular.z = self.Vz

        
        self.publisher_.publish(odom_msg)
        
        print(f'Position: x={self.x}, y={self.y}, theta={self.theta}')

    def move_callback(self, msg: Twist):
        # 在這裡更新你的速度值，然後可以調用 `Vel_to_Dis()` 計算里程並發布
        self.get_Vel_From_Encoder()
        self.Vel_to_Dis()

def main(args=None):
    rclpy.init(args=args)
    node = ReadOdomVelToDis()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()