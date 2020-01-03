#!/usr/bin/python3.4
# -*-coding:utf-8 -*

# Common constants

U_alim = 12.2	# Voltage of the alim in V


# physical dimentions of the vehicule
dxLeft = 0.042*3.141/48.0
dxRight= 0.042*3.141/48.0		# Perimeter of the wheel / nbre of encoder tilt per rotation
wheels_width = 0.128   			# distance beetwin wheels
radius_G = 0.14					# Heigh of the weightPoint relative to the wheels axis

# motor constants
Kv_left = 30.9 * dxLeft			# Speed / U_motor  (m/s /V)
Kv_right= 30.9 * dxRight
KVr	= Kv_right* U_alim / 1200.0	# Speed / %pwm
KVl	= Kv_left * U_alim / 1200.0
R_intern = 17.1				# Calculated from stall torque, stall current und kv

# Pins Encoders
pinLeftA = 6
pinLeftB = 13
pinRightA= 19
pinRightB= 26



# PI inclinaison, the value proposed are are the robust ones (half the limit ones) by 12V
kva = 0.0      	#30.0
kpa = 4000.     	#1000
tpa = 1000.000004  	#0.00005
ksa = 100.		# % geschwindigkeitsvorsteuerung
kda = 0.0     		#10.0
kaa = .0		#5.0
sata= 150.0    		#200
#t_filter_a = 0.002 # 0.05

# PID orientation
kvo = 2. 	#1.2
kpo = 500. # 400
tpo = 0.000017 #  0.0006
kdo = 10.1# 0.001
sato= 0
t_filter_o = 0.001

# PID position
kvw = 0.2	# 0.5
kpw = 0.2
tpw = 1.6
ksw = 100.0		# %
kdw = 0.001
satw= .0
t_filter_w = 0.1

# loop time, seconds. Must be more than the actual time it takes to free CPU use for other process
loop_time = 0.002


