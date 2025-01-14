from zlac8015d import ZLAC8015D
import time
import math


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

# 定義速度轉換函數
def speed_to_rpm(speed):
    # 根據車輛輪胎直徑將速度轉換為轉速（rpm）
    rpm = int(speed * 60 / (WHEEL_DIAMETER * math.pi))  # 將速度轉換為轉速
    return rpm
 
# 定義速度指令函數
def set_speeds(vx, vy, vz):
    # 根據車輛的整體速度指令，計算每個馬達的轉速
    vz_linear = float(vz * ((WHEEL_SPACING + WHEEL_AXLESPACING) / 2))
    speed_A = int(vx - vy - vz_linear )
    speed_B = int(vx + vy + vz_linear)
    speed_D = int(vx + vy - vz_linear)
    speed_C = int(vx - vy + vz_linear)

    print("speed_A",speed_A,", speed_B:",speed_B,", speed_C",speed_C,"speed D",speed_D)
    # 設置兩個馬達的速度指令
    cmdsAB = [ -speed_A, speed_B]  # AB馬達速度指令
    cmdsCD = [ speed_C, -speed_D]  # CD馬達速度指令
    # cmdsAB = [ 0, 0]  # AB馬達速度指令
    # cmdsCD = [0, speed_C]  # CD馬達速度指令
    # 設置馬達速度
    motorAB.set_rpm(cmdsAB[0], cmdsAB[1])
    motorCD.set_rpm(cmdsCD[0], cmdsCD[1])

# 主程序
def main():
    # print("請輸入車輛的速度指令：")
    # print("格式：vx vy vz")
    # print("其中，vx 表示前後速度，vy 表示左右速度，vz 表示旋轉速度")
    # print("示例：30 0 0 表示前進，-30 0 0 表示後退")
    
    last_time = time.time()
    i = 0
    period = 0.0
    # 獲取用戶輸入的速度指令
    # vx, vy, vz = map(float, input("速度指令：").replace(',', ' ').split())
    while True:
        try:
            # # 獲取用戶輸入的速度指令
            vx, vy, vz = map(float, input("速度指令：").replace(',', ' ').split())
            # 啟用兩個馬達
            motorAB.enable_motor()
            motorCD.enable_motor()
            # 設置速度指令
            
            set_speeds(vx, vy, vz)
            
            rpmA, rpmB = motorAB.get_rpm()
            rpmD, rpmC = motorCD.get_rpm()
            print("period: {:.4f} rpmA: {:.1f} | rpmB: {:.1f} | rpmC: {:.1f} | rpmD: {:.1f}".format(period,rpmA,rpmB,rpmC,rpmD))
            period = time.time() - last_time
            time.sleep(0.05)
            
        except KeyboardInterrupt:
            # 捕獲 Ctrl+C 鍵盤事件，停止運動並退出程序
            motorAB.disable_motor()
            motorCD.disable_motor()
            print("\n程序已退出。")
            ##
            # motorAB.clear_alarm()
            # motorCD.clear_alarm()
            break
        except ValueError:
            print("輸入格式錯誤，請重新輸入。")
        
        last_time = time.time()

if __name__ == "__main__":
    main()
