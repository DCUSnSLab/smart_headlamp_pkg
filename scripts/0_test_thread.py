#!/usr/bin/env python3

import math
import time
import threading
import Arm_Lib


MIN_D = -90
MAX_D = 90


def servo_worker(servo_id: int, rate: int, stop_event: threading.Event):
    sbus = Arm_Lib.Arm_Device()
    t_ms = int(1000 / rate)
    sleep_s = 1.0 / rate
    step = 5
    current_ang_d = MIN_D

    print(f"*\t[Servo {servo_id}] START")

    while not stop_event.is_set():
        ang_d = current_ang_d + 90
        print(f"*\t[Servo {servo_id}] → {ang_d:.1f}° | t = {t_ms} ms")
        sbus.Arm_serial_servo_write(servo_id, ang_d, t_ms)

        current_ang_d += step
        if current_ang_d > MAX_D:
            step = -5
            current_ang_d = MAX_D + step
        elif current_ang_d < MIN_D:
            step = 5
            current_ang_d = MIN_D + step

        time.sleep(sleep_s)

    print(f"*\t[Servo {servo_id}] STOPPED")


def move_servos_continuously(rate: int):
    stop_event = threading.Event()

    try:
        print("*\tIf you want to finish, Press Ctrl + C.")
        t1 = threading.Thread(target=servo_worker, args=(2, rate, stop_event))
        t2 = threading.Thread(target=servo_worker, args=(3, rate, stop_event))

        t1.start()
        t2.start()

        t1.join()
        t2.join()

    except KeyboardInterrupt:
        print("\n*\tInterrupt received. Stopping threads...")
        stop_event.set()
        t1.join()
        t2.join()

    finally:
        print("*\tTrying to reset servos...", flush=True)
        sbus = Arm_Lib.Arm_Device()
        try:
            time.sleep(1)
            sbus.Arm_serial_servo_write(2, 90, 100)
            time.sleep(1)
            sbus.Arm_serial_servo_write(3, 90, 100)
            time.sleep(1)
            print("*\tServos reset to 90 degrees.", flush=True)
        except Exception as e:
            print(f"!\tFailed to reset servos: {e}", flush=True)

    return


if __name__ == '__main__':
    move_servos_continuously(10)
