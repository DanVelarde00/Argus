#!/usr/bin/env python3
"""
Bench motor test — spins each motor briefly via MAV_CMD_DO_MOTOR_TEST.
No arming for flight, no GPS needed. REMOVE PROPS BEFORE RUNNING.

Usage:  python3 motor_test.py [throttle_pct] [seconds] [num_motors]
        defaults: 10 % for 2 s on 4 motors
"""

import sys
import time
from pymavlink import mavutil

FC = "tcp:127.0.0.1:5760"

throttle = int(sys.argv[1]) if len(sys.argv) > 1 else 10
duration = int(sys.argv[2]) if len(sys.argv) > 2 else 2
n_motors = int(sys.argv[3]) if len(sys.argv) > 3 else 4

print(f"!!! REMOVE PROPS !!!  throttle={throttle}%  dur={duration}s  motors={n_motors}")
input("Press Enter when props are off and area is clear...")

m = mavutil.mavlink_connection(FC, source_system=255)
m.mav.heartbeat_send(6, 8, 192, 0, 4)
m.wait_heartbeat(timeout=30)
print(f"FC heartbeat sys={m.target_system}")

for motor in range(1, n_motors + 1):
    print(f"Motor {motor} -> {throttle}% for {duration}s")
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST, 0,
        motor,                                        # p1: motor instance (1..N)
        mavutil.mavlink.MOTOR_TEST_THROTTLE_PERCENT,  # p2: throttle type
        throttle,                                     # p3: throttle value
        duration,                                     # p4: timeout seconds
        0, 0, 0)
    time.sleep(duration + 1)

print("Done.")
