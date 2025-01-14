
from pymodbus.client.sync import ModbusSerialClient as ModbusClient
import numpy as np
import time
import threading
class Controller(object):#父類別

	def __init__(self, port="/dev/ttyUSB0"):

		self._port = port

		self.client = ModbusClient(method='rtu', port=self._port, baudrate=115200, timeout=1)

		self.client.connect()

		self.ID = 1

		######################
		## Register Address ##
		######################
		## Common
		self.CONTROL_REG = 0x200E
		self.OPR_MODE = 0x200D
		self.L_ACL_TIME = 0x2080
		self.R_ACL_TIME = 0x2081
		self.L_DCL_TIME = 0x2082
		self.R_DCL_TIME = 0x2083

		## Velocity control
		self.L_CMD_RPM = 0x2088
		self.R_CMD_RPM = 0x2089
		self.L_FB_RPM = 0x20AB
		self.R_FB_RPM = 0x20AC

		## Position control
		self.POS_CONTROL_TYPE = 0x200F

		self.L_MAX_RPM_POS = 0x208E
		self.R_MAX_RPM_POS = 0x208F

		self.L_CMD_REL_POS_HI = 0x208A
		self.L_CMD_REL_POS_LO = 0x208B
		self.R_CMD_REL_POS_HI = 0x208C
		self.R_CMD_REL_POS_LO = 0x208D

		self.L_FB_POS_HI = 0x20A7
		self.L_FB_POS_LO = 0x20A8
		self.R_FB_POS_HI = 0x20A9
		self.R_FB_POS_LO = 0x20AA

		## Troubleshooting
		self.L_FAULT = 0x20A5
		self.R_FAULT = 0x20A6

		########################
		## Control CMDs (REG) ##
		########################
		self.EMER_STOP = 0x05
		self.ALRM_CLR = 0x06
		self.DOWN_TIME = 0x07
		self.ENABLE = 0x08
		self.POS_SYNC = 0x10
		self.POS_L_START = 0x11
		self.POS_R_START = 0x12

		####################
		## Operation Mode ##
		####################
		self.POS_REL_CONTROL = 1
		self.POS_ABS_CONTROL = 2
		self.VEL_CONTROL = 3

		self.ASYNC = 0
		self.SYNC = 1

		#################
		## Fault codes ##
		#################
		self.NO_FAULT = 0x0000
		self.OVER_VOLT = 0x0001
		self.UNDER_VOLT = 0x0002
		self.OVER_CURR = 0x0004
		self.OVER_LOAD = 0x0008
		self.CURR_OUT_TOL = 0x0010
		self.ENCOD_OUT_TOL = 0x0020
		self.MOTOR_BAD = 0x0040
		self.REF_VOLT_ERROR = 0x0080
		self.EEPROM_ERROR = 0x0100
		self.WALL_ERROR = 0x0200
		self.HIGH_TEMP = 0x0400
		self.FAULT_LIST = [self.OVER_VOLT, self.UNDER_VOLT, self.OVER_CURR, self.OVER_LOAD, self.CURR_OUT_TOL, self.ENCOD_OUT_TOL, \
					self.MOTOR_BAD, self.REF_VOLT_ERROR, self.EEPROM_ERROR, self.WALL_ERROR, self.HIGH_TEMP]

		##############
		## Odometry ##
		##############
		## 8 inches wheel
		self.cpr = 16385
		self.R_Wheel = 0.09 #meter
		self.travel_in_one_rev = self.R_Wheel * 3.14

	## Some time if read immediatly after write, it would show ModbusIOException when get data from registers
	def modbus_fail_read_handler(self, ADDR, WORD):

		read_success = False
		reg = [None]*WORD
		while not read_success:
			result = self.client.read_holding_registers(ADDR, WORD, unit=self.ID)
			try:
				for i in range(WORD):
					reg[i] = result.registers[i]
				read_success = True
			except AttributeError as e:
				print(e)
				pass

		return reg

	def rpm_to_radPerSec(self, rpm):
		return rpm*2*np.pi/60.0

	def rpm_to_linear(self, rpm):

		W_Wheel = self.rpm_to_radPerSec(rpm)
		V = W_Wheel*self.R_Wheel

		return V

	def set_mode(self, MODE):
		if MODE == 1:
			print("Set relative position control")
		elif MODE == 2:
			print("Set absolute position control")
		elif MODE == 3:
			print("Set speed rpm control")
		else:
			print("set_mode ERROR: set only 1, 2, or 3")
			return 0

		result = self.client.write_register(self.OPR_MODE, MODE, unit=self.ID)
		return result

	def get_mode(self):

		# result = self.client.read_holding_registers(self.OPR_MODE, 1, unit=self.ID)
		registers = self.modbus_fail_read_handler(self.OPR_MODE, 1)

		mode = registers[0]

		return mode

	def enable_motor(self):
		result = self.client.write_register(self.CONTROL_REG, self.ENABLE, unit=self.ID)

	def disable_motor(self):
		result = self.client.write_register(self.CONTROL_REG, self.DOWN_TIME, unit=self.ID)

	def get_fault_code(self):

		fault_codes = self.client.read_holding_registers(self.L_FAULT, 2, unit=self.ID)

		L_fault_code = fault_codes.registers[0]
		R_fault_code = fault_codes.registers[1]

		L_fault_flag = L_fault_code in self.FAULT_LIST
		R_fault_flag = R_fault_code in self.FAULT_LIST

		return (L_fault_flag, L_fault_code), (R_fault_flag, R_fault_code)

	def clear_alarm(self):
		result = self.client.write_register(self.CONTROL_REG, self.ALRM_CLR, unit=self.ID)

	def set_accel_time(self, L_ms, R_ms):

		if L_ms > 32767:
			L_ms = 32767
		elif L_ms < 0:
			L_ms = 0

		if R_ms > 32767:
			R_ms = 32767
		elif R_ms < 0:
			R_ms = 0

		result = self.client.write_registers(self.L_ACL_TIME, [int(L_ms),int(R_ms)], unit=self.ID)


	def set_decel_time(self, L_ms, R_ms):

		if L_ms > 32767:
			L_ms = 32767
		elif L_ms < 0:
			L_ms = 0

		if R_ms > 32767:
			R_ms = 32767
		elif R_ms < 0:
			R_ms = 0

		result = self.client.write_registers(self.L_DCL_TIME, [int(L_ms), int(R_ms)], unit=self.ID)

	def int16Dec_to_int16Hex(self,int16):

		lo_byte = (int16 & 0x00FF)
		hi_byte = (int16 & 0xFF00) >> 8

		all_bytes = (hi_byte << 8) | lo_byte

		return all_bytes


	def set_rpm(self, L_rpm, R_rpm):
		if L_rpm > 3000:L_rpm = 3000
		elif L_rpm < -3000:L_rpm = -3000
		if R_rpm > 3000:R_rpm = 3000
		elif R_rpm < -3000:R_rpm = -3000
		left_bytes = self.int16Dec_to_int16Hex(L_rpm)
		right_bytes = self.int16Dec_to_int16Hex(R_rpm)
		result = self.client.write_registers(self.L_CMD_RPM, [left_bytes, right_bytes], unit=self.ID)
		# print('result=',result)

	def get_rpm(self):
		# rpms = self.client.read_holding_registers(self.L_FB_RPM, 2, unit=self.ID)
		# fb_L_rpm = np.int16(rpms.registers[0])/10.0
		# fb_R_rpm = np.int16(rpms.registers[1])/10.0

		registers = self.modbus_fail_read_handler(self.L_FB_RPM, 2)
		fb_L_rpm = np.int16(registers[0])/10.0
		fb_R_rpm = np.int16(registers[1])/10.0

		return fb_L_rpm, fb_R_rpm

	def set_linear_velocities(self,VL,VR):#設定線速度#待測試
		rpmL = int((VL/self.R_Wheel) * (30/np.pi))
		rpmR = int((VR/self.R_Wheel) * (30/np.pi))
		print('rpmL=',rpmL,'rpmR=',rpmR)
		self.set_rpm(rpmL,rpmR)
	
	def get_linear_velocities(self):

		rpmL, rpmR = self.get_rpm()

		VL = self.rpm_to_linear(rpmL)
		VR = self.rpm_to_linear(-rpmR)

		return VL, VR
	
	def get_linearV_travelled(self):#用移動距離回推速度#by mark test#因為直接從控制器拉轉速出來有雜訊
		SL_init,SR_init=self.get_wheels_travelled()#起始位置
		st=time.time()
		time.sleep(0.01)
		SL,SR=self.get_wheels_travelled()#檢測位置
		VL = (SL-SL_init)/(st-time.time())
		VR = (SR-SR_init)/(st-time.time())
		return VL, VR
        
	def map(self, val, in_min, in_max, out_min, out_max):

			return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

	def set_maxRPM_pos(self, max_L_rpm, max_R_rpm):

		if max_L_rpm > 1000:
			max_L_rpm = 1000
		elif max_L_rpm < 1:
			max_L_rpm = 1

		if max_R_rpm > 1000:
			max_R_rpm = 1000
		elif max_R_rpm < 1:
			max_R_rpm = 1

		result = self.client.write_registers(self.L_MAX_RPM_POS, [int(max_L_rpm), int(max_R_rpm)], unit=self.ID)

	def set_position_async_control(self):

		result = self.client.write_register(self.POS_CONTROL_TYPE, self.ASYNC, unit=self.ID)

	def move_left_wheel(self):

		result = self.client.write_register(self.CONTROL_REG, self.POS_L_START, unit=self.ID)

	def move_right_wheel(self):

		result = self.client.write_register(self.CONTROL_REG, self.POS_R_START, unit=self.ID)

	def deg_to_32bitArray(self, deg):

		dec = int(self.map(deg, -1440, 1440, -65536, 65536))
		HI_WORD = (dec & 0xFFFF0000) >> 16
		LO_WORD = dec & 0x0000FFFF

		return [HI_WORD, LO_WORD]

	def set_relative_angle(self, ang_L, ang_R):

		L_array = self.deg_to_32bitArray(ang_L)
		R_array = self.deg_to_32bitArray(ang_R)
		all_cmds_array = L_array + R_array

		result = self.client.write_registers(self.L_CMD_REL_POS_HI, all_cmds_array, unit=self.ID)

	def get_wheels_travelled(self):
		registers = self.modbus_fail_read_handler(self.L_FB_POS_HI, 4)
		l_pul_hi = registers[0]
		l_pul_lo = registers[1]
		r_pul_hi = registers[2]
		r_pul_lo = registers[3]

		l_pulse = np.int32(((l_pul_hi & 0xFFFF) << 16) | (l_pul_lo & 0xFFFF))
		r_pulse = np.int32(((r_pul_hi & 0xFFFF) << 16) | (r_pul_lo & 0xFFFF))
		l_travelled = (float(l_pulse)/self.cpr)*self.travel_in_one_rev  # unit in meter
		r_travelled = (float(r_pulse)/self.cpr)*self.travel_in_one_rev  # unit in meter

		return l_travelled, r_travelled

	def get_wheels_tick(self):

		registers = self.modbus_fail_read_handler(self.L_FB_POS_HI, 4)
		l_pul_hi = registers[0]
		l_pul_lo = registers[1]
		r_pul_hi = registers[2]
		r_pul_lo = registers[3]

		l_tick = np.int32(((l_pul_hi & 0xFFFF) << 16) | (l_pul_lo & 0xFFFF))
		r_tick = np.int32(((r_pul_hi & 0xFFFF) << 16) | (r_pul_lo & 0xFFFF))

		return l_tick, r_tick

class MacRobot(Controller):#子類別
	def __init__(self, portAB="/dev/ttyUSB0",portCD="/dev/ttyUSB1"):
		self.portAB = portAB
		self.portCD = portCD
		# super().__init__(portAB)#繼承
		# super().__init__(portCD)#繼承
		self.motorAB=Controller(portAB)
		self.motorCD=Controller(portCD)
		self.Wheel_Diameter = 0.18#輪徑
		self.Wheel_axlespacing  = 6.5 #前後軸距離
		self.Wheel_spacing = 6.6#左右輪距離
		
        ##速度輸入
		self.inputVx=0
		self.inputVy=0
		self.inputVz=0
		#目前速度
		self.Vx,self.Vy,self.Vz=0,0,0
        ##速度平滑處理
		self.smooth_VX=0
		self.smooth_VY=0
		self.smooth_VZ=0
		##PI參數
		self.Last_bias_A=0
		self.Last_bias_B=0
		self.Last_bias_C=0
		self.Last_bias_D=0
		self.Velocity_KP=2
		self.Velocity_KI=1
		#速度輸出
		self.OutputA=0
		self.OutputB=0
		self.OutputC=0
		self.OutputD=0
		self.outputMax=1#輸出上限
		##
		self.Odometry_POS=[0,0,0]#里程計位姿
        ##
		self.running_flag=False
		self.feedback_flag=False
		##宣告多執行緒##
		self.runningThread=threading.Thread(target=self.vmode_move_task,args=(0.1))
		self.feedbackThread=threading.Thread(target=self.feedback_task,args=(0.1))

	def vmode_move_task(self,dt):#速度移動任務#機器人速度輸出
		while(self.running_flag):
			st=time.time()
			try:
				self.Get_Velocity_Form_Encoder()#從編碼器取得回授線速度
				self.Drive_Motor(self.inputVx,self.inputVy,self.inputVz)#輸出
			except KeyboardInterrupt:
				self.close()
				break
			if dt-(time.time()-st)>0:time.sleep(dt-(time.time()-st))#dt=執行間格
		
	def Get_Velocity_Form_Encoder(self):#從編碼器取得回授線速度
		self.MOTOR_A_Encoder, self.MOTOR_B_Encoder = self.motorAB.get_linear_velocities()#取線速度
		self.MOTOR_D_Encoder, self.MOTOR_C_Encoder = self.motorCD.get_linear_velocities()
		# print('VA=',self.MOTOR_A_Encoder,self.MOTOR_B_Encoder,self.MOTOR_C_Encoder,self.MOTOR_D_Encoder)
		self.MOTOR_A_Encoder = -self.MOTOR_A_Encoder#馬達安裝方向相反故改為負號#輪編號不同需更改
		self.MOTOR_B_Encoder = -self.MOTOR_B_Encoder#輪編號不同需更改
		self.MOTOR_C_Encoder = -self.MOTOR_C_Encoder#輪編號不同需更改
		self.MOTOR_D_Encoder = -self.MOTOR_D_Encoder#輪編號不同需更改
		print('VA=',self.MOTOR_A_Encoder,self.MOTOR_B_Encoder,self.MOTOR_C_Encoder,self.MOTOR_D_Encoder)
		#self.Vx = (self.MOTOR_A_Encoder + self.MOTOR_B_Encoder + self.MOTOR_C_Encoder + self.MOTOR_D_Encoder)/4
		#self.Vy = (self.MOTOR_A_Encoder-self.MOTOR_B_Encoder+self.MOTOR_C_Encoder-self.MOTOR_D_Encoder)/4
		#self.Vz = (-self.MOTOR_A_Encoder-self.MOTOR_B_Encoder+self.MOTOR_C_Encoder+self.MOTOR_D_Encoder)/(4*(self.Wheel_axlespacing+self.Wheel_spacing))
		
		# 根据逆运动学计算机器人的线速度和角速度
		self.Vx = (self.MOTOR_A_Encoder + self.MOTOR_B_Encoder + self.MOTOR_C_Encoder + self.MOTOR_D_Encoder) / 4
		self.Vy = (-self.MOTOR_A_Encoder + self.MOTOR_B_Encoder - self.MOTOR_C_Encoder + self.MOTOR_D_Encoder) / 4
		self.Vz = (-self.MOTOR_A_Encoder - self.MOTOR_B_Encoder + self.MOTOR_C_Encoder + self.MOTOR_D_Encoder) / (4 * (self.Wheel_axlespacing + self.Wheel_spacing))

	def getPos(self):#計算機器人的里程計位姿TODO
		self.Odometry_POS=[0,0,0]
	def setPos(self,x,y,z):#更新機器人的里程計位姿TODO
		self.Odometry_POS=[x,y,z]
	def Drive_Motor(self,Vx,Vy,Vz):
		#將目標速度進行平滑處理
		Vx,Vy,Vz=self.smooth(Vx,Vy,Vz)
		#逆向運動學#從目標移動速度推到各馬達轉速
		MOTOR_A_Target = +Vx-Vy-Vz*(self.Wheel_axlespacing +self.Wheel_spacing)#輪編號不同需更改
		MOTOR_B_Target = +Vx+Vy+Vz*(self.Wheel_axlespacing+self.Wheel_spacing)#輪編號不同需更改
		MOTOR_C_Target = +Vx-Vy+Vz*(self.Wheel_axlespacing+self.Wheel_spacing)#輪編號不同需更改
		MOTOR_D_Target = +Vx+Vy-Vz*(self.Wheel_axlespacing+self.Wheel_spacing)#輪編號不同需更改
		
        #馬達轉速PI校正
		self.Incremental_PI_MotorA(MOTOR_A_Target,self.MOTOR_A_Encoder)
		self.Incremental_PI_MotorB(MOTOR_B_Target,self.MOTOR_B_Encoder)
		self.Incremental_PI_MotorC(MOTOR_C_Target,self.MOTOR_C_Encoder)
		self.Incremental_PI_MotorD(MOTOR_D_Target,self.MOTOR_D_Encoder)
		#輸出
		self.motorAB.set_linear_velocities(-MOTOR_A_Target, MOTOR_B_Target)
		self.motorCD.set_linear_velocities(MOTOR_C_Target, -MOTOR_D_Target)
		
	def smooth(self,vx,vy,vz):#平滑速度輸出
		step=0.01
		if  (vx>self.smooth_VX): self.smooth_VX+=step
		elif(vx<self.smooth_VX): self.smooth_VX-=step
		else                   : self.smooth_VX =vx
		if  (vy>self.smooth_VY): self.smooth_VY+=step
		elif(vy<self.smooth_VY): self.smooth_VY-=step
		else                   : self.smooth_VY =vy
		if  (vz>self.smooth_VZ): self.smooth_VZ+=step
		elif(vz<self.smooth_VZ): self.smooth_VZ-=step
		else                   : self.smooth_VZ =vz
	    #不移動時的移動雜訊濾除
		if(vx==0 and self.smooth_VX<0.02 and self.smooth_VX>-0.02): self.smooth_VX=0
		if(vy==0 and self.smooth_VY<0.02 and self.smooth_VY>-0.02): self.smooth_VY=0
		if(vz==0 and self.smooth_VZ<0.02 and self.smooth_VZ>-0.02): self.smooth_VZ=0
	
		return self.smooth_VX,self.smooth_VY,self.smooth_VZ
	def Incremental_PI_MotorA(self,Target,Encoder):
		Bias = Target-Encoder #Calculate the deviation //计算偏差
		self.OutputA += self.Velocity_KP*(Bias-self.Last_bias_A)+self.Velocity_KI*Bias
		if(self.OutputA >  self.outputMax):self.OutputA =  self.outputMax
		if(self.OutputA < -self.outputMax):self.OutputA = -self.outputMax
		self.Last_bias_A = Bias #Save the last deviation //保存上一次偏差
		print('EncoderA=',Encoder,'TargetA=',Target)#test
	def Incremental_PI_MotorB(self,Target,Encoder):
		Bias = Target-Encoder #Calculate the deviation //计算偏差
		self.OutputB += self.Velocity_KP*(Bias-self.Last_bias_B)+self.Velocity_KI*Bias
		if(self.OutputB >  self.outputMax):self.OutputB =  self.outputMax
		if(self.OutputB < -self.outputMax):self.OutputB = -self.outputMax
		self.Last_bias_B = Bias #Save the last deviation //保存上一次偏差
	def Incremental_PI_MotorC(self,Target,Encoder):
		Bias=Target-Encoder #Calculate the deviation //计算偏差
		self.OutputC += self.Velocity_KP*(Bias-self.Last_bias_C) + self.Velocity_KI*Bias
		if(self.OutputC >  self.outputMax):self.OutputC =  self.outputMax
		if(self.OutputC < -self.outputMax):self.OutputC = -self.outputMax
		self.Last_bias_C = Bias #Save the last deviation //保存上一次偏差
	def Incremental_PI_MotorD(self,Target,Encoder):
		Bias=Target-Encoder #Calculate the deviation //计算偏差
		self.OutputD += self.Velocity_KP*(Bias-self.Last_bias_D) + self.Velocity_KI*Bias
		if(self.OutputD > self.outputMax):self.OutputD  =  self.outputMax
		if(self.OutputD < -self.outputMax):self.OutputD = -self.outputMax
		self.Last_bias_D = Bias #Save the last deviation //保存上一次偏差
	
	def feedback_task(self,dt):#回授#傳給ROS系統的資料
		while(self.feedback_flag):
			st=time.time()
            #need to do
            #把實際V回傳給ROS
			print('V=',self.Vx, self.Vy, self.Vz)
			if dt-(time.time()-st)>0:time.sleep(dt-(time.time()-st))#dt=執行間格
	def set_V_task(self,Vx,Vy,Vz):#獲得速度指令
		self.inputVx, self.inputVy, self.inputVz = Vx, Vy, Vz
		print('get',Vx,Vy,Vz)
	def startTask(self):
        ##
		self.motorAB.enable_motor()
		self.motorCD.enable_motor()
		self.motorAB.set_mode(3)#預設速度控制，速度控制=3
		self.motorCD.set_mode(3)
		##
		self.running_flag=True
		self.feedback_flag=True
		self.runningThread=threading.Thread(target=self.vmode_move_task,args=(1,))
		self.feedbackThread=threading.Thread(target=self.feedback_task,args=(1,))
		self.runningThread.start()
		self.feedbackThread.start()
	def close(self):
		self.running_flag=False
		self.feedback_flag=False
		self.runningThread.join()
		self.feedbackThread.join()
		self.motorAB.disable_motor()
		self.motorCD.disable_motor()
		
import threading
import os
if __name__ == '__main__':
	os.system("gnome-terminal -e 'bash -c \"ls; exec bash\"'")
	robot=MacRobot('/dev/ttyUSB0','/dev/ttyUSB1')
	robot.set_V_task(0.1,0,0)
	robot.startTask()
	time.sleep(3)
	robot.set_V_task(-0.1,0,0)
	
	time.sleep(3)
	robot.set_V_task(0,0.1,0)
	
	time.sleep(3)
	robot.set_V_task(0,-0.1,0)

	time.sleep(3)
	robot.set_V_task(0,0,0.01)

	time.sleep(3)
	robot.set_V_task(0,0,-0.01)

	time.sleep(3)
	robot.set_V_task(0,0,0)
	robot.close()
	# while(1):
	# 	try:
	# 		Vx,Vy,Vz = map(int,input().split())
	# 		robot.set_V_task(Vx,Vy,Vz)
	# 	except KeyboardInterrupt:
	# 		robot.set_V_task(0,0,0)
	# 		robot.close()
	# 		break
			
	
