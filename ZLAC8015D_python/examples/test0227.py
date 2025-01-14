import serial

def check_rs485_connection(port="/dev/ttyUSB0", baudrate=9600):
    try:
        # 尝试打开串口
        ser = serial.Serial(port, baudrate, timeout=1)
        
        # 如果成功打开串口，表示连接正常
        print(f"Connected to RS485 on {port} with baudrate {baudrate}")
        
        # 关闭串口
        ser.close()
    except serial.SerialException as e:
        # 发生异常，表示连接失败
        print(f"RS485 connection error: {e}")

# 检查 RS485 连接
check_rs485_connection()
