from zlac8015d import ZLAC8015D
import time

# 初始化兩個 ZLAC8015D 控制器，分別連接到不同的端口
motorAB = ZLAC8015D.Controller(port="/dev/ttyUSB0")#馬達AB
motorDC = ZLAC8015D.Controller(port="/dev/ttyUSB1")#馬達DC

# 停用兩個馬達
motorAB.disable_motor()
motorDC.disable_motor()

# 設置加速度和減速度時間
a_time=500
b_time=1000
motorAB.set_accel_time(a_time, a_time)
motorAB.set_decel_time(b_time, b_time)
motorDC.set_accel_time(a_time, a_time)
motorDC.set_decel_time(b_time,b_time)

# 設置馬達模式
motorAB.set_mode(3)
motorDC.set_mode(3)

# 啟用兩個馬達
motorAB.enable_motor()
motorDC.enable_motor()
#time.sleep(50)

# 設置兩個馬達的速度指令
MotorSpeed_A=30
MotorSpeed_B=30
MotorSpeed_D=30
MotorSpeed_C=30
cmdsAB = [-MotorSpeed_A, MotorSpeed_B]  # AB馬達速度指令
cmdsDC = [-MotorSpeed_D, MotorSpeed_C]  # DC馬達速度指令

# 設置馬達速度
motorAB.set_rpm(cmdsAB[0], cmdsAB[1])
motorDC.set_rpm(cmdsDC[0], cmdsDC[1])

last_time = time.time()
period = 0.0

while True:
    try:
        # 獲取兩個馬達的速度
        rpmL_A, rpmR_B = motorDC.get_rpm()
        rpmL_D, rpmR_C = motorDC.get_rpm()

        # 顯示速度信息
        print("Motor 1: period: {:.4f} rpmL: {:.1f} | rpmR: {:.1f}".format(period, rpmL_A, rpmR_B))
        print("Motor 2: period: {:.4f} rpmL: {:.1f} | rpmR: {:.1f}".format(period, rpmL_D, rpmR_C))

        period = time.time() - last_time
        time.sleep(0.01)

    except KeyboardInterrupt:
        # 當按下 Ctrl+C 時，停用兩個馬達並退出迴圈
        motorAB.disable_motor()
        motorDC.disable_motor()
        break

    last_time = time.time()
