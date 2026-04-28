#!/usr/bin/env python3
"""
Emergency kill — force-disarm the FC over MAVLink immediately.
Use ONLY if the drone needs to come down NOW. It will fall.
For a soft abort, prefer kill.py rtl   (switches mode to RTL instead).

Usage:
  python3 kill.py            # force disarm (motors off, drone falls)
  python3 kill.py rtl        # mode -> RTL (returns to launch and lands)
  python3 kill.py land       # mode -> LAND (descends in place)
"""

import sys
import time
from pymavlink import mavutil

FC = "tcp:127.0.0.1:5760"
MODE_LAND = 9
MODE_RTL  = 6

def connect():
    m = mavutil.mavlink_connection(FC, source_system=255)
    m.mav.heartbeat_send(6, 8, 192, 0, 4)
    m.wait_heartbeat(timeout=10)
    return m

def force_disarm(m):
    # p1=0 disarm, p2=21196 = magic "force" value to disarm even mid-flight
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        0, 21196, 0, 0, 0, 0, 0)

def set_mode(m, mode):
    m.mav.set_mode_send(
        m.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode)

def main():
    action = sys.argv[1] if len(sys.argv) > 1 else "disarm"
    m = connect()
    if action == "rtl":
        print("RTL")
        set_mode(m, MODE_RTL)
    elif action == "land":
        print("LAND")
        set_mode(m, MODE_LAND)
    else:
        print("FORCE DISARM")
        for _ in range(5):
            force_disarm(m)
            time.sleep(0.1)

if __name__ == "__main__":
    main()
