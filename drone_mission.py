#!/usr/bin/env python3
"""
ARGUS Drone Mission Controller
Raspberry Pi Zero 2W | ArduPilot via MAVLink

Architecture:
  ESP32 Gateway ──UART──> Pi ──MAVLink──> ArduPilot FC
  Pi ──UART──> ESP32 (drone GPS/heading telemetry pushes)

Behavior:
  1. Continuously streams drone GPS + heading to ESP32 (~1 Hz)
  2. Listens for threat_confirmed waypoints from ESP32
  3. On threat: arm → takeoff → fly to waypoint → hover 30 s → RTL
  4. Reports LAUNCHED / ENGAGING / COMPLETE / FAILED back to ESP32

Requirements (install on Pi):
  pip3 install pymavlink pyserial

MAVLink connection:
  Requires mavlink-router running with a TcpServerEndpoint on port 5760.
  See mavlink-router/main.conf for the required addition.

Serial ports (adjust if needed):
  FC  (ArduPilot) : via mavlink-router TCP (no direct serial conflict)
  ESP32 Gateway   : /dev/ttyUSB0   (USB) or /dev/ttyAMA1 (GPIO UART)
"""

import json
import math
import sys
import threading
import time
from datetime import datetime

import serial
from pymavlink import mavutil

# ============================================================================
# CONFIGURATION  — edit these to match your hardware
# ============================================================================

# MAVLink connection string (via mavlink-router TCP server)
FC_CONNECTION = "tcp:127.0.0.1:5760"

# ESP32 serial port.  USB adapter: /dev/ttyUSB0   GPIO UART: /dev/ttyAMA1
ESP32_PORT = "/dev/ttyUSB0"
ESP32_BAUD = 115200

# Mission parameters
CRUISE_ALTITUDE_M  = 20.0   # AGL metres for the intercept flight
HOVER_DURATION_S   = 10     # Seconds to loiter at waypoint
WAYPOINT_RADIUS_M  = 2.0    # Acceptance radius (metres, ground distance)
GPS_TIMEOUT_S      = 60     # Seconds to wait for 3-D fix before aborting
ARM_TIMEOUT_S      = 30     # Seconds to wait for arming confirmation
TAKEOFF_TIMEOUT_S  = 60     # Seconds to reach cruise altitude
TELEMETRY_HZ       = 1.0    # How often to push GPS/heading to ESP32

# ArduPilot custom mode numbers (Copter firmware)
MODE_STABILIZE = 0
MODE_GUIDED    = 4
MODE_LOITER    = 5
MODE_RTL       = 6


# ============================================================================
# HELPERS
# ============================================================================

def log(tag: str, msg: str) -> None:
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    print(f"[{ts}][{tag:12s}] {msg}", flush=True)


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Ground distance in metres between two WGS-84 coordinates."""
    R = 6_371_000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi   = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    return R * 2 * math.asin(math.sqrt(a))


# ============================================================================
# MAIN CONTROLLER
# ============================================================================

class DroneMissionController:

    def __init__(self):
        self.mav: mavutil.mavfile | None = None
        self.esp: serial.Serial | None = None

        # Shared telemetry (written by MAVLink reader thread)
        self._lock   = threading.Lock()
        self._lat    = 0.0
        self._lon    = 0.0
        self._alt    = 0.0   # metres AGL
        self._hdg    = 0.0   # degrees 0-359
        self._armed  = False
        self._mode   = 0     # ArduPilot custom mode
        self._fix    = 0     # GPS fix type (0=none, 3=3D)

        self._stop        = threading.Event()
        self._mission_active = False
        self._esp_buf     = ""

    # -------------------------------------------------------------------------
    # CONNECT
    # -------------------------------------------------------------------------

    def connect_fc(self) -> None:
        """Connect to the flight controller through mavlink-router."""
        log("FC", f"Connecting: {FC_CONNECTION}")
        self.mav = mavutil.mavlink_connection(FC_CONNECTION, source_system=255)
        # Announce ourselves as a GCS so mavlink-router starts forwarding to us.
        self.mav.mav.heartbeat_send(6, 8, 192, 0, 4)
        self.mav.wait_heartbeat(timeout=30)
        log("FC", f"Heartbeat OK  system={self.mav.target_system} "
                   f"component={self.mav.target_component}")
        self._request_streams()

    def _request_streams(self) -> None:
        """Ask ArduPilot to stream the messages the mission controller needs."""
        streams = [
            # (stream_id, rate_hz)
            (mavutil.mavlink.MAV_DATA_STREAM_POSITION,   5),  # GLOBAL_POSITION_INT
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,     5),  # ATTITUDE (heading backup)
            (mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS, 2), # GPS_RAW_INT (fix type)
        ]
        for stream_id, rate in streams:
            self.mav.mav.request_data_stream_send(
                self.mav.target_system,
                self.mav.target_component,
                stream_id,
                rate,
                1,  # start
            )
        log("FC", "Data streams requested")

    def connect_esp32(self) -> None:
        """Open UART to the ESP32 gateway."""
        log("ESP32", f"Opening {ESP32_PORT} @ {ESP32_BAUD}")
        self.esp = serial.Serial(ESP32_PORT, ESP32_BAUD, timeout=0.1)
        time.sleep(1.5)
        log("ESP32", "Serial ready")

    # -------------------------------------------------------------------------
    # MAVLINK READER THREAD
    # -------------------------------------------------------------------------

    def _mavlink_reader(self) -> None:
        """Background thread: parse incoming MAVLink and update shared telemetry."""
        while not self._stop.is_set():
            try:
                msg = self.mav.recv_match(blocking=True, timeout=0.5)
                if msg is None:
                    continue
                t = msg.get_type()
                with self._lock:
                    if t == "GLOBAL_POSITION_INT":
                        self._lat = msg.lat / 1e7
                        self._lon = msg.lon / 1e7
                        self._alt = msg.relative_alt / 1000.0  # mm → m AGL
                        self._hdg = msg.hdg / 100.0            # cdeg → deg
                    elif t == "GPS_RAW_INT":
                        self._fix = msg.fix_type
                    elif t == "HEARTBEAT":
                        self._armed = bool(
                            msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                        )
                        self._mode = msg.custom_mode
            except Exception as exc:
                log("MAV_READER", f"Error: {exc}")

    def _telemetry(self) -> tuple:
        """Return (lat, lon, alt, hdg, armed, fix) snapshot."""
        with self._lock:
            return (self._lat, self._lon, self._alt,
                    self._hdg, self._armed, self._fix)

    # -------------------------------------------------------------------------
    # ESP32 COMMUNICATION
    # -------------------------------------------------------------------------

    def _esp_send(self, data: dict) -> None:
        """Write a JSON line to the ESP32."""
        try:
            self.esp.write((json.dumps(data) + "\n").encode())
        except Exception as exc:
            log("ESP32_TX", f"Error: {exc}")

    def push_telemetry(self) -> None:
        """Send current drone GPS + heading to ESP32 (called at TELEMETRY_HZ)."""
        lat, lon, alt, hdg, armed, fix = self._telemetry()
        self._esp_send({
            "type":  "drone_telemetry",
            "lat":   round(lat, 7),
            "lon":   round(lon, 7),
            "alt":   round(alt, 1),
            "hdg":   round(hdg, 1),
            "armed": armed,
            "fix":   fix,
            "ts":    int(time.time()),
        })

    def report_status(self, status: str) -> None:
        """Send DRONE_STATUS:<status> text + JSON to ESP32."""
        self.esp.write(f"DRONE_STATUS:{status}\n".encode())
        self._esp_send({"type": "drone_status", "status": status})

    def _read_esp32_lines(self) -> list[str]:
        """Drain ESP32 serial buffer; return list of complete text lines."""
        lines = []
        try:
            waiting = self.esp.in_waiting
            if waiting:
                self._esp_buf += self.esp.read(waiting).decode("utf-8", errors="ignore")
            while "\n" in self._esp_buf:
                line, self._esp_buf = self._esp_buf.split("\n", 1)
                line = line.strip()
                if line:
                    lines.append(line)
        except Exception as exc:
            log("ESP32_RX", f"Error: {exc}")
        return lines

    # -------------------------------------------------------------------------
    # MAVLINK COMMAND HELPERS
    # -------------------------------------------------------------------------

    def _set_mode(self, mode_id: int) -> None:
        self.mav.mav.set_mode_send(
            self.mav.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )

    def _command_long(self, cmd, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0) -> None:
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            cmd, 0,
            p1, p2, p3, p4, p5, p6, p7,
        )

    # -------------------------------------------------------------------------
    # PRE-FLIGHT
    # -------------------------------------------------------------------------

    def wait_for_gps(self) -> bool:
        """Block until 3-D GPS fix (or timeout)."""
        log("GPS", "Waiting for 3-D fix…")
        deadline = time.time() + GPS_TIMEOUT_S
        while time.time() < deadline:
            *_, fix = self._telemetry()
            if fix >= 3:
                lat, lon, *_ = self._telemetry()
                log("GPS", f"Fix acquired  home={lat:.6f},{lon:.6f}")
                return True
            time.sleep(1)
        log("GPS", "TIMEOUT — no GPS fix")
        return False

    # -------------------------------------------------------------------------
    # FLIGHT SEQUENCE
    # -------------------------------------------------------------------------

    def arm_and_takeoff(self, target_alt: float) -> bool:
        """Switch GUIDED → arm → takeoff to target_alt metres AGL."""
        log("FLIGHT", "Setting GUIDED mode")
        self._set_mode(MODE_GUIDED)
        time.sleep(1)

        log("FLIGHT", "Arming motors")
        self._command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, p1=1)

        deadline = time.time() + ARM_TIMEOUT_S
        while time.time() < deadline:
            *_, armed, _ = self._telemetry()
            if armed:
                log("FLIGHT", "Armed")
                break
            time.sleep(0.5)
        else:
            log("FLIGHT", "ERROR: arm timeout")
            return False

        log("FLIGHT", f"Takeoff → {target_alt:.0f} m AGL")
        self._command_long(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, p7=target_alt)

        deadline = time.time() + TAKEOFF_TIMEOUT_S
        while time.time() < deadline:
            _, _, alt, _, _, _ = self._telemetry()
            log("FLIGHT", f"  alt={alt:.1f}/{target_alt:.1f} m")
            if alt >= target_alt * 0.95:
                log("FLIGHT", "Cruise altitude reached")
                return True
            time.sleep(1)

        log("FLIGHT", "ERROR: takeoff timeout")
        return False

    def goto_waypoint(self, lat: float, lon: float, alt: float) -> bool:
        """Command GUIDED flight to lat/lon/alt; block until within acceptance radius."""
        log("NAV", f"Heading to {lat:.7f}, {lon:.7f} @ {alt:.0f} m")

        # Mask: use position only (ignore velocity, acceleration, yaw)
        TYPE_MASK_POS_ONLY = 0b0000_111111111000

        self.mav.mav.set_position_target_global_int_send(
            0,                          # time_boot_ms (ignored)
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            TYPE_MASK_POS_ONLY,
            int(lat * 1e7),             # lat_int (1e-7 deg)
            int(lon * 1e7),             # lon_int (1e-7 deg)
            alt,                        # alt (m AGL)
            0, 0, 0,                    # vx, vy, vz
            0, 0, 0,                    # afx, afy, afz
            0, 0,                       # yaw, yaw_rate
        )

        while True:
            curr_lat, curr_lon, curr_alt, _, _, _ = self._telemetry()
            dist = haversine_m(curr_lat, curr_lon, lat, lon)
            log("NAV", f"  dist={dist:.1f} m  alt={curr_alt:.1f} m")
            if dist <= WAYPOINT_RADIUS_M and abs(curr_alt - alt) < 3.0:
                log("NAV", "Waypoint reached")
                return True
            time.sleep(2)

    def hover(self, duration: int) -> None:
        """Switch to LOITER and hold for `duration` seconds."""
        log("HOVER", f"LOITER for {duration} s")
        self._set_mode(MODE_LOITER)
        for remaining in range(duration, 0, -1):
            if remaining % 5 == 0:
                _, _, alt, hdg, _, _ = self._telemetry()
                log("HOVER", f"  {remaining:3d} s left | alt={alt:.1f} m | hdg={hdg:.0f}°")
                self.push_telemetry()   # keep ESP32 updated during hover
            time.sleep(1)
        log("HOVER", "Hover complete")

    def return_to_launch(self) -> None:
        """Set RTL mode and wait until the drone lands and disarms."""
        log("RTL", "RTL initiated")
        self._set_mode(MODE_RTL)
        while True:
            _, _, alt, _, armed, _ = self._telemetry()
            log("RTL", f"  alt={alt:.1f} m  armed={armed}")
            if not armed:
                log("RTL", "Landed and disarmed")
                return
            time.sleep(2)

    # -------------------------------------------------------------------------
    # FULL MISSION
    # -------------------------------------------------------------------------

    def execute_mission(self, target_lat: float, target_lon: float) -> None:
        """
        Complete intercept mission:
          arm → takeoff → fly to waypoint → hover 30 s → RTL
        Runs in its own thread so the main loop keeps streaming telemetry.
        """
        if self._mission_active:
            log("MISSION", "Already active — ignoring duplicate waypoint")
            return

        self._mission_active = True
        log("MISSION", "=" * 55)
        log("MISSION", f"START  target={target_lat:.7f}, {target_lon:.7f}")
        log("MISSION", "=" * 55)

        try:
            if not self.wait_for_gps():
                raise RuntimeError("No GPS fix — aborting")

            self.report_status("LAUNCHED")
            if not self.arm_and_takeoff(CRUISE_ALTITUDE_M):
                raise RuntimeError("Takeoff failed")

            self.report_status("ENGAGING")
            if not self.goto_waypoint(target_lat, target_lon, CRUISE_ALTITUDE_M):
                raise RuntimeError("Navigation failed")

            self.hover(HOVER_DURATION_S)

            self.report_status("COMPLETE")
            self.return_to_launch()

        except Exception as exc:
            log("MISSION", f"ERROR: {exc}")
            self.report_status("FAILED")
            try:
                self.return_to_launch()
            except Exception:
                pass
        finally:
            self._mission_active = False
            log("MISSION", "Mission ended")

    # -------------------------------------------------------------------------
    # MAIN LOOP
    # -------------------------------------------------------------------------

    def run(self) -> None:
        # Start background MAVLink reader
        reader = threading.Thread(target=self._mavlink_reader, daemon=True)
        reader.start()

        log("ARGUS", "System ready — streaming telemetry, listening for threats")
        log("ARGUS", "Ctrl-C to exit")

        last_telem = 0.0
        telem_interval = 1.0 / TELEMETRY_HZ

        try:
            while True:
                now = time.time()

                # Push GPS/heading to ESP32 at configured rate
                if now - last_telem >= telem_interval:
                    self.push_telemetry()
                    last_telem = now

                # Read ESP32 → handle messages
                for raw in self._read_esp32_lines():
                    if not raw.startswith("{"):
                        continue
                    try:
                        data = json.loads(raw)
                    except json.JSONDecodeError:
                        log("ESP32_RX", f"Bad JSON: {raw}")
                        continue

                    msg_type = data.get("type", "")

                    if msg_type == "threat_confirmed" and not self._mission_active:
                        t_lat = data.get("target_lat", 0.0)
                        t_lon = data.get("target_lon", 0.0)
                        conf  = data.get("conf", 0)
                        node  = data.get("node", "?")
                        log("THREAT", f"Node {node} | conf={conf}% | WP={t_lat:.7f},{t_lon:.7f}")
                        threading.Thread(
                            target=self.execute_mission,
                            args=(t_lat, t_lon),
                            daemon=True,
                        ).start()

                    elif msg_type == "status":
                        log("GATEWAY", f"State={data.get('state','?')}  "
                                       f"nodes={data.get('nodes',0)}  "
                                       f"threats={data.get('threats',0)}")

                time.sleep(0.05)

        except KeyboardInterrupt:
            log("ARGUS", "Shutting down…")
        finally:
            self._stop.set()
            if self.esp and self.esp.is_open:
                self.esp.close()
            log("ARGUS", "Done")


# ============================================================================
# ENTRY POINT
# ============================================================================

def main() -> None:
    print("=" * 60)
    print("  ARGUS Drone Mission Controller")
    print("  Raspberry Pi Zero 2W  |  ArduPilot MAVLink")
    print("=" * 60)

    ctrl = DroneMissionController()
    try:
        ctrl.connect_esp32()
        ctrl.connect_fc()
        ctrl.run()
    except Exception as exc:
        log("MAIN", f"Fatal: {exc}")
        sys.exit(1)


if __name__ == "__main__":
    main()
