
from rclpy.node import Node
from zlac8015d import ZLAC8015D
from geometry_msgs.msg import Twist
import time
import math
import rclpy


WHEEL_DIAMETER = 180 /1000      # 馬達直徑（mm）
WHEEL_SPACING = 650 / 1000      # 馬達前後軸距（mm）
WHEEL_AXLESPACING = 661 / 1000  # 馬達左右軸距（mm）

# 初始化兩個 ZLAC8015D 控制器，分別連接到不同的端口
motorAB = ZLAC8015D.Controller(port="/dev/ttyUSB0")  # 馬達AB
motorCD = ZLAC8015D.Controller(port="/dev/ttyUSB1")  # 馬達CD

# 設置加速度和減速度時間
a_time = 500
b_time = 1000
motorAB.set_accel_time(a_time, a_time)
motorAB.set_decel_time(b_time, b_time)
motorCD.set_accel_time(a_time, a_time)
motorCD.set_decel_time(b_time, b_time)

# 設置馬達模式
motorAB.set_mode(3)
motorCD.set_mode(3)

def speed_rpm(speed):# 根據車輛輪胎直徑將速度轉換為轉速（rpm）

    rpm = int(speed * 60 / (WHEEL_DIAMETER * math.pi))  # 將速度轉換為轉速
    return rpm

def set_speeds(vx, vy, vz):# 根據車輛的整體速度指令，計算每個馬達的轉速

    vz_linear = float(vz * ((WHEEL_SPACING + WHEEL_AXLESPACING) / 2))
    speed_A = int(vx - vy - vz_linear )
    speed_B = int(vx + vy + vz_linear)
    speed_D = int(vx + vy - vz_linear)
    speed_C = int(vx - vy + vz_linear)

    print("speed_A",speed_A,", :",speed_B,", speed_C",speed_C,"speed D",speed_D)
    # 設置兩個馬達的速度指令speed_B
    cmdsAB = [ speed_A, -speed_B]  # AB馬達速度指令
    cmdsCD = [ speed_C, -speed_D]  # CD馬達速度指令
    # cmdsAB = [ 0, 0]  # AB馬達速度指令
    # cmdsCD = [0, speed_C]  # CD馬達速度指令
    # 設置馬達速度
    motorAB.set_rpm(cmdsAB[0], cmdsAB[1])
    motorCD.set_rpm(cmdsCD[0], cmdsCD[1])

class TeleopTwistKeyboardNode(Node):

    def __init__(self):
        super().__init__('teleop_twist_keyboard_node')    # 初始化 Node 類別
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.move_callback, 10)
        self.subscription  # prevent unused variable warning
        # motorAB.enable_motor()
        # motorCD.enable_motor()

    def move_callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        vz = msg.angular.z
        set_speeds(vx, vy, vz)

    # def on_shutdown(self):
    #     motorAB.disable_motor()
    #     motorCD.disable_motor()

def main(args=None):    # 主程式
    rclpy.init(args=args)
    teleop_twist_keyboard_node = TeleopTwistKeyboardNode()
    motorAB.enable_motor()
    motorCD.enable_motor()

    try:
        rclpy.spin(teleop_twist_keyboard_node)
    except KeyboardInterrupt:
        pass

    teleop_twist_keyboard_node.destroy_node()
    rclpy.shutdown()

    motorAB.disable_motor()
    motorCD.disable_motor()
    print("程序已退出。")

if __name__ == '__main__':    # 程式進入點
    main()    # 呼叫主程式
