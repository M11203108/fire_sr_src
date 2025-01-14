from zlac8015d import ZLAC8015D
import time
import math

# 马达直径（mm）
WHEEL_DIAMETER = 180
WHEEL_SPACING = 650
WHEEL_AXLESPACING = 661

# 初始化兩個 ZLAC8015D 控制器，分別連接到不同的端口
motorAB = ZLAC8015D.Controller(port="/dev/ttyUSB0")#馬達AB
motorDC = ZLAC8015D.Controller(port="/dev/ttyUSB1")#馬達DC

# 设置加速度和减速度时间
a_time = 500
b_time = 1000
motorAB.set_accel_time(a_time, a_time)
motorAB.set_decel_time(b_time, b_time)
motorDC.set_accel_time(a_time, a_time)
motorDC.set_decel_time(b_time, b_time)

# 设置马达模式
motorAB.set_mode(3)
motorDC.set_mode(3)


# 定义速度转换函数
def speed_to_rpm(speed):
    # 根据车辆轮胎直径等信息将速度转换为转速（rpm）
    wheel_diameter = 180  # 轮胎直径，单位：mm
    rpm = int(speed * 60 / (wheel_diameter * math.pi))  # 将速度转换为转速
    return rpm

# 定义速度指令函数
def set_speeds(vx, vy, omega):
    # 根据车辆的整体速度指令，计算每个马达的转速
    speed_A = int(vx - vy + omega * ((WHEEL_SPACING+WHEEL_AXLESPACING)/2))
    speed_B = int(vx + vy + omega * ((WHEEL_SPACING+WHEEL_AXLESPACING)/2))
    speed_D = int(vx + vy - omega * ((WHEEL_SPACING+WHEEL_AXLESPACING)/2))
    speed_C = int(vx - vy - omega * ((WHEEL_SPACING+WHEEL_AXLESPACING)/2))

    # 设置两个马达的速度指令
    cmdsAB = [-speed_A, speed_B]  # AB馬達速度指令
    cmdsDC = [-speed_D, speed_C]  # DC馬達速度指令

    # 设置马达速度
    motorAB.set_rpm(cmdsAB[0], cmdsAB[1])
    motorDC.set_rpm(cmdsDC[0], cmdsDC[1])


# 主程序
def main():
    print("请输入车辆的速度指令：")
    print("格式：vx vy omega")
    print("其中，vx 表示前后速度，vy 表示左右速度，omega 表示旋转速度")
    print("示例：30 0 0 表示前进，-30 0 0 表示后退")
    
    while True:
        try:
            # 获取用户输入的速度指令
            vx, vy, omega = map(float, input("速度指令：").replace(',', ' ').split())
            # 启用两个马达
            motorAB.enable_motor()
            motorDC.enable_motor()
            # 设置速度指令
            set_speeds(vx, vy, omega)

            # # 启用两个马达
            # motorAB.enable_motor()
            # motorDC.enable_motor()

        except KeyboardInterrupt:
            # 捕获 Ctrl+C 键盘事件，停止运动并退出程序
            motorAB.disable_motor()
            motorDC.disable_motor()
            print("\n程序已退出。")
            motorAB.clear_alarm()
            motorDC.clear_alarm()
            break
        except ValueError:
            print("输入格式错误，请重新输入。")

if __name__ == "__main__":
    main()
