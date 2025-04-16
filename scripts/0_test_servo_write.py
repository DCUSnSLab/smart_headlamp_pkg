#!/usr/bin/env python3

import math
import rospy
import Arm_Lib
import time
from sensor_msgs.msg import JointState


MIN_D = -90
MAX_D = 90
ANGLE_LIST_D = [0, 0, 0, 0, 0, 0]
SERVO2 = 1
SERVO3 = 2


def move_only_one_servo() -> None:

	sbus = Arm_Lib.Arm_Device()
	id = -1
	print("*\tIf you want to finish, set SERVO ID 0.")

	while True:
		id = int(input(">>\tSERVO ID : "))
		if id == 0: break
		ang_r = float(input(">>\tANGLE(RA) : "))
		ang_d = math.degrees(ang_r) + 90
		print(f'*\tANGLE(DE) : {ang_d:.0f}')
		t = int(input(">>\tTIME(ms) : "))
		print(f"*\tArm_serial_servo_write({id}, {ang_d:.0f}, {t}) , START")
		sbus.Arm_serial_servo_write(id, ang_d, t)
		print()

	return


def move_servos_continuously(rate: int):

	global ANGLE_LIST_D
	sbus = Arm_Lib.Arm_Device()
	current_ang_d = MIN_D
	t_ms = int(1000 / rate)
	slp_s = rate / 1000.00
	step = 1
	
	try:
		print("*\tIf you want to finish, Press Ctrl + C.")
		print(f"*\tSTART ANGLE : {current_ang_d} | STEP : {step}")
		print(f"*\tTIME : {t_ms} ms | SLEEP : {slp_s}")
		while True:
			ang_d = current_ang_d + 90
			print(f"*\tArm_serial_servo_write([2,3], {ang_d}, {t_ms})")
			#sbus.Arm_serial_servo_write(2, ang_d, t_ms)
			#sbus.Arm_serial_servo_write(3, ang_d, t_ms)
			ANGLE_LIST_D[SERVO2] = ang_d
			ANGLE_LIST_D[SERVO3] = ang_d
			sbus.Arm_serial_servo_write6_array(ANGLE_LIST_D, t_ms)
			current_ang_d += step
			if current_ang_d > MAX_D:
				step = -1
				current_ang_d = MAX_D + step
			elif current_ang_d < MIN_D:
				step = 1
				current_ang_d = MIN_D + step
			time.sleep(slp_s)

	except KeyboardInterrupt:
		print("*\tFINISH !")

	finally:
		print("*\tTrying to reset servos...", flush=True)
		try:
			time.sleep(1)
			sbus.Arm_serial_servo_write6_array([0, 90, 90, 0, 0, 0], t_ms)
			time.sleep(1)
			print("*\tServos reset to 90 degrees.", flush=True)
		except Exception as e:
			print(f"!\tFailed to reset servos: {e}", flush=True)

	return


if __name__ == '__main__':
	move_servos_continuously(10)
	# move_only_one_servo()

	# rospy.init_node('servo_test')
	# sbus = Arm_Lib.Arm_Device()
	# angle = 90
	# servo = -1
	# while servo != 0:
	# 	servo = int(input(">> servo : "))
	# 	if servo == 0: break
	# 	angle_r = float(input(">> angle(RA) : "))
	# 	angle_d = math.degrees(angle_r) + 90
	# 	print(f'Degree == {angle_d:.4f}')
	# 	sbus.Arm_serial_servo_write(servo, angle_d, 100)
	# 	print()

	# 	rate = rospy.Rate(10)
	# 	ang = JointState()
	# 	ang_pub = rospy.Publisher('joint_states', JointState, queue_size=1)

	# 	ang.header.frame_id = ''
	# 	ang.header.stamp = rospy.Time.now()
	# 	ang.velocity = []
	# 	ang.effort = []
	# 	ang.position = [angle_r]
	# 	jnt_name = "joint" + str(servo)
	# 	ang.name = [jnt_name]

	# 	ang_pub.publish(ang)
	# 	rate.sleep()