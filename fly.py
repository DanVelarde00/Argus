#!/usr/bin/env python3
"""
Minimal ARGUS waypoint flight.
Usage:  python3 fly.py <target_lat> <target_lon> [altitude_m]

Sequence:  arm -> takeoff -> fly to waypoint -> hover 30s -> RTL
"""

import math
import sys
import time
from pymavlink import mavutil

FC          = "tcp:127.0.0.1:5760"
ALT_M       = 20.0
HOVER_S     = 30
WP_RADIUS_M = 2.0

MODE_GUIDED, MODE_LOITER, MODE_RTL = 4, 5, 6


def haversine(lat1, lon1, lat2, lon2):
    R = 6_371_000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = math.radians(lat2 - lat1), math.radians(lon2 - lon1)
    a = math.sin(dp/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dl/2)**2
    return R * 2 * math.asin(math.sqrt(a))


def log(tag, msg):
    print(f"[{time.strftime('%H:%M:%S')}][{tag}] {msg}", flush=True)


class Drone:
    def __init__(self):
        log("FC", f"Connecting to {FC}")
        self.m = mavutil.mavlink_connection(FC)
        self.m.wait_heartbeat(timeout=30)
        log("FC", f"Heartbeat from sys={self.m.target_system}")
        for s in (mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                  mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
                  mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS):
            self.m.mav.request_data_stream_send(
                self.m.target_system, self.m.target_component, s, 5, 1)

    def telem(self):
        lat = lon = alt = hdg = 0.0
        armed = False
        fix = 0
        for _ in range(20):
            msg = self.m.recv_match(blocking=False)
            if msg is None:
                break
            t = msg.get_type()
            if t == "GLOBAL_POSITION_INT":
                lat = msg.lat / 1e7
                lon = msg.lon / 1e7
                alt = msg.relative_alt / 1000.0
                hdg = msg.hdg / 100.0
            elif t == "GPS_RAW_INT":
                fix = msg.fix_type
            elif t == "HEARTBEAT":
                armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        return lat, lon, alt, hdg, armed, fix

    def set_mode(self, mode):
        self.m.mav.set_mode_send(self.m.target_system,
                                 mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                 mode)

    def cmd(self, c, **p):
        self.m.mav.command_long_send(
            self.m.target_system, self.m.target_component, c, 0,
            p.get("p1", 0), p.get("p2", 0), p.get("p3", 0), p.get("p4", 0),
            p.get("p5", 0), p.get("p6", 0), p.get("p7", 0))

    def wait_gps(self, timeout=60):
        log("GPS", "Waiting for 3D fix...")
        t0 = time.time()
        while time.time() - t0 < timeout:
            *_, fix = self.telem()
            if fix >= 3:
                lat, lon, *_ = self.telem()
                log("GPS", f"Fix: {lat:.6f},{lon:.6f}")
                return True
            time.sleep(1)
        return False

    def arm_takeoff(self, alt):
        log("FLT", "GUIDED")
        self.set_mode(MODE_GUIDED); time.sleep(1)
        log("FLT", "Arming")
        self.cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, p1=1)
        for _ in range(30):
            *_, armed, _ = self.telem()
            if armed:
                break
            time.sleep(0.5)
        else:
            log("FLT", "Arm timeout"); return False
        log("FLT", f"Takeoff -> {alt}m")
        self.cmd(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, p7=alt)
        for _ in range(60):
            _, _, a, *_ = self.telem()
            log("FLT", f"  alt={a:.1f}/{alt}")
            if a >= alt * 0.95:
                return True
            time.sleep(1)
        return False

    def goto(self, lat, lon, alt):
        log("NAV", f"-> {lat:.7f},{lon:.7f} @ {alt}m")
        MASK = 0b0000_111111111000
        self.m.mav.set_position_target_global_int_send(
            0, self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            MASK, int(lat*1e7), int(lon*1e7), alt,
            0, 0, 0, 0, 0, 0, 0, 0)
        while True:
            cl, cn, ca, *_ = self.telem()
            d = haversine(cl, cn, lat, lon)
            log("NAV", f"  dist={d:.1f}m alt={ca:.1f}m")
            if d <= WP_RADIUS_M and abs(ca - alt) < 3:
                return True
            time.sleep(2)

    def hover(self, sec):
        log("HOV", f"LOITER {sec}s")
        self.set_mode(MODE_LOITER)
        time.sleep(sec)

    def rtl(self):
        log("RTL", "Returning")
        self.set_mode(MODE_RTL)
        while True:
            *_, armed, _ = self.telem()
            if not armed:
                log("RTL", "Landed/disarmed")
                return
            time.sleep(2)


def main():
    if len(sys.argv) < 3:
        print("Usage: fly.py <lat> <lon> [alt_m]"); sys.exit(1)
    lat, lon = float(sys.argv[1]), float(sys.argv[2])
    alt = float(sys.argv[3]) if len(sys.argv) > 3 else ALT_M

    d = Drone()
    if not d.wait_gps():            sys.exit("No GPS")
    if not d.arm_takeoff(alt):      sys.exit("Takeoff failed")
    if not d.goto(lat, lon, alt):   sys.exit("Nav failed")
    d.hover(HOVER_S)
    d.rtl()


if __name__ == "__main__":
    main()
