#!/usr/bin/env python3

import math
import Arm_Lib
import time

if __name__ == '__main__':
	sbus = Arm_Lib.Arm_Device()
	angle = 90
	servo = -1
	while servo != 0:
		servo = int(input(">> servo : "))
		if servo == 0: break
		angle_r = float(input(">> angle(RA) : "))
		angle_d = math.degrees(angle_r)
		print(f'Degree == {angle_d:.4f}')
		sbus.Arm_serial_servo_write(servo, angle_d, 100)
		print()