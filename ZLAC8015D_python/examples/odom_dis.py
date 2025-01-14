import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from zlac8015d import ZLAC8015D
import numpy as np
import time

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # 初始化ZLAC8015D马达控制器
        self.motorAB = ZLAC8015D.Controller(port="/dev/ttyUSB0")  # 馬達AB
        self.motorCD = ZLAC8015D.Controller(port="/dev/ttyUSB1")  # 馬達CD
        
        # 设定轮子的参数
        self.cpr = 16385  # 每转脉冲数，示例值
        self.travel_in_one_rev = 0.56548667  # 轮子一周的行进距离，单位米，示例值

    def get_wheels_travelled(self):
        # 使用 ZLAC8015D 的方法获取左右轮的行进距离
        registers = self.motorAB.modbus_fail_read_handler(self.motorAB.L_FB_POS_HI, 4)
        l_pul_hi = registers[0]
        l_pul_lo = registers[1]
        r_pul_hi = registers[2]
        r_pul_lo = registers[3]

        # 将高位和低位组合成32位的脉冲数
        l_pulse = np.int32(((l_pul_hi & 0xFFFF) << 16) | (l_pul_lo & 0xFFFF))
        r_pulse = np.int32(((r_pul_hi & 0xFFFF) << 16) | (r_pul_lo & 0xFFFF))

        # 计算行进距离，单位为米
        l_travelled = (float(l_pulse) / self.cpr) * self.travel_in_one_rev
        r_travelled = (float(r_pulse) / self.cpr) * self.travel_in_one_rev

        print("l_travelled",l_travelled,", r_travelled",r_travelled)

        return l_travelled, r_travelled

    def timer_callback(self):
        l_travelled, r_travelled = self.get_wheels_travelled()

        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'

        # 设置机器人的位置和方向
        odom_msg.pose.pose.position.x = l_travelled
        odom_msg.pose.pose.position.y = 0.0
        odom_msg.pose.pose.position.z = 0.0

        # 假设机器人没有旋转
        odom_msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # 设置机器人的速度
        odom_msg.child_frame_id = 'base_link'
        odom_msg.twist.twist.linear.x = (l_travelled + r_travelled) / 2.0
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0

        self.publisher_.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
