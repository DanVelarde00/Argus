# ARGUS — System Report

**Autonomous Radar-Gated Unmanned Sentry**
A drone-based area-denial platform that combines distributed ground radar pucks, an aggregating gateway, and an autonomous quadcopter to detect, confirm, and intercept ground-level intrusions inside a defined zone.

---

## 1. System Overview

ARGUS has three layers, each running independently and communicating across well-defined interfaces:

```
   ┌───────────────┐    ESP-NOW     ┌───────────────┐    USB-CDC     ┌────────────────┐    MAVLink     ┌──────────────────┐
   │ Detection     │ ───── 2.4 GHz─►│ Gateway       │ ──── 115200 ──►│ Raspberry Pi   │ ── TCP 5760 ──►│ ArduPilot Flight │
   │ Pucks (N×)    │   (channel 1)  │ ESP32-S3      │   JSON / line  │ Zero 2W        │  pymavlink     │ Controller       │
   │ FireBeetle    │                │ FireBeetle    │                │ (mission ctrl) │                │ (Copter 4.x)     │
   └───────────────┘                └───────────────┘                └────────────────┘                └──────────────────┘
        │                                                                     │                                  │
        ├── DFRobot C4001 24 GHz radar                                        ├── mavlink-router (UART⇆TCP)     ├── GPS+compass
        ├── BMI160 IMU                                                         └── /dev/ttyUSB0 to gateway       ├── ESCs / motors
        └── BN-880 GPS                                                                                            └── Battery / FC
```

The pipeline is **detection → confirmation → mission**. Any one stage failing fails closed (no flight), but each stage is independently testable.

---

## 2. Hardware

### 2.1 Detection Pucks (`repo/detection_node/`)
Each puck is a sealed unit with one job: detect movement, geo-locate it, and shout it to the gateway.

| Component | Role |
|---|---|
| **DFRobot FireBeetle 2 ESP32-S3** | MCU + Wi-Fi/ESP-NOW radio |
| **DFRobot C4001 mmWave radar** | 24 GHz Doppler + range, ~25 m max range |
| **BMI160 IMU** | Heading (compass calibration) and self-orientation |
| **BN-880 GPS** | Self-locates the puck on first power-up; cached to NVS for instant fix afterwards |

Pucks are fully autonomous — once flashed, drop them in the field and they begin reporting.

### 2.2 Gateway (`repo/gateway_controller_usb/`)
A second FireBeetle 2 ESP32 acting as the data concentrator.

- **ESP-NOW receiver**: catches detection messages from up to N pucks
- **USB-Serial bridge**: forwards distilled state to the Pi at 115200 baud as one JSON object per line
- **Threat assessor**: runs a 2-second persistence filter before declaring a threat confirmed
- **Pause/resume controller**: tells pucks to stand down while the drone is engaging

### 2.3 Companion Computer
**Raspberry Pi Zero 2W** running Raspberry Pi OS (HAMR custom image, ARM64).

- Hosts `mavlink-router` (UART ⇆ TCP bridge to the FC)
- Runs `drone_mission.py` (the mission orchestrator)
- Runs a Wi-Fi hotspot (`HAMR-...`) for ground-station connections
- Single point of MAVLink mediation — every command to the FC goes through here

### 2.4 Drone
- Generic ArduPilot-compatible quadcopter
- Flight controller running ArduCopter 4.x
- GPS+compass module (combo puck, separate from the detection pucks)
- 4S battery (~14.5 V observed)
- Flight controller talks to the Pi over UART (GPIO14/15, 921600 baud)
- **No RC receiver currently bound** — drone is fully MAVLink-controlled

---

## 3. Software Architecture

### 3.1 Puck Firmware (`detection_node.ino`)

Loop runs at ~5 Hz idle, ~10 Hz while a target is being tracked.

**Key state machine:**
1. Idle: sample radar + IMU + GPS at low rate
2. Possible-target: radar reports a hit. Start sample-counter.
3. Confirmed: target persists ≥ `REQUIRED_STRONG_SAMPLES` *and* duration ≥ `CONFIRMED_DETECTION_MS` *and* energy above the rolling noise floor + margin.
4. Send: build a `DetectionMessage` (node ID, sequence, lat/lon, range, speed, heading, energy, confidence, sat count) and `esp_now_send` to the gateway.
5. Hold-time: continue tracking until target lost > `DETECTION_HOLD_MS`, then drop back to idle.

**Confidence calculation** combines radar SNR, GPS quality, and IMU stability into a 0-100% score. The gateway uses it as a filter floor.

**Manual diagnostics** via Serial Monitor:
- `t` — fire a test ESP-NOW ping bypassing detection logic (sequence=0)
- `s` — print status summary
- `c` / `x` — IMU calibration start / cancel

**ESP-NOW pairing fix** (the bug we debugged): `esp_now_peer_info_t` is now zero-initialized with `ifidx = WIFI_IF_STA` and an explicit channel 1 lock on both ends. Without this, `add_peer` succeeds but `esp_now_send` silently fails.

### 3.2 Gateway Firmware (`gateway_controller_usb.ino`)

Runs two parallel responsibilities:

**ESP-NOW receive callback** (separate task on the Wi-Fi core):
- Validates message size
- Auto-registers any new MAC (no pre-shared peer list needed — pucks just appear)
- Updates per-node state (last seen, total detections, last known position)
- Forwards a JSON `{"type":"detection",...}` line to the Pi

**Main loop** (CPU0):
- Threat assessor: tracks the active threat, requires ≥`THREAT_CONFIRMATION_MS` (2 s) of sustained detections from the same node above `MIN_CONFIDENCE_LEVEL` (40 %).
- On confirm: emits `{"type":"threat_confirmed",...}` JSON, broadcasts `P` (pause) command to all pucks, transitions to `DRONE_DEPLOYING`.
- Listens for Pi commands (`STATUS`, `RESUME`, `DRONE_STATUS:<state>`).
- Periodic `{"type":"status",...}` heartbeat every ~10 s.

**JSON atomicity** (the second bug we hit): all three Serial outputs (`sendDetectionToPi`, `sendThreatToPi`, `sendStatusToPi`) build into a `char` buffer and write with a single `Serial.println` so concurrent prints from the main loop can't fragment a JSON line.

### 3.3 Pi Mission Controller (`drone_mission.py`)

A Python orchestrator with two threads:

**Thread 1 — Telemetry pump (`TELEMETRY_HZ` = 1 Hz):**
- Streams drone GPS + heading down to the gateway over UART so pucks can know where the drone is. Currently informational only.

**Thread 2 — Main mission loop:**
- Reads JSON lines from `/dev/ttyUSB0` (gateway)
- On `threat_confirmed`: starts a mission state machine

**Mission state machine:**
```
START → WAIT_GPS_FIX (60 s) → SET_GUIDED → ARM (30 s) → TAKEOFF (60 s)
      → GOTO waypoint → HOVER (10 s) → RTL → wait disarmed → END
```

Any timeout in any state aborts to RTL. The `RTL` is sent unconditionally even when not armed, which is harmless (no-op).

**MAVLink details that matter:**
- `mavlink-router` gates TCP forwarding until the client sends a heartbeat first. `drone_mission.py` (and `fly.py`) explicitly emit `heartbeat_send(6, 8, 192, 0, 4)` (type=GCS, autopilot=Invalid, base_mode=192, custom_mode=0, system_status=4) before `wait_heartbeat()`. Without this, MAVLink times out silently.
- Waypoints are sent with `MAV_FRAME_GLOBAL_RELATIVE_ALT_INT` so altitude is interpreted as **above launch point**, not above the target ground. Currently 2 m AGL.
- Acceptance radius is 5 m so the drone calls a waypoint "reached" without splitting hairs — once inside, it transitions straight to hover instead of fighting GPS jitter.

### 3.4 mavlink-router Configuration

```
[UartEndpoint alpha]
Device = /dev/serial0
Baud = 921600

[TcpServerEndpoint local]
Port = 5760
```

Single TCP endpoint on port 5760. **Only one client can hold this at a time** — Mission Planner and `drone_mission.py` are mutually exclusive. To run both simultaneously a second `[TcpServerEndpoint]` block on a different port would be required.

---

## 4. End-to-End Data Flow

A complete intrusion → response cycle:

```
T+0.0 s   Walker enters radar arc, ~1 m range, energy spikes from noise (≈25 %) to peak (≈55 %)
T+0.5 s   Puck logs successive samples meeting energy + range + speed criteria
T+2.0 s   strongSampleCount ≥ REQUIRED_STRONG_SAMPLES AND duration ≥ CONFIRMED_DETECTION_MS
          → puck builds DetectionMessage, esp_now_send to gateway MAC
T+2.0 s   Gateway recv callback fires (different core, no blocking)
          → registers node MAC if new
          → starts threat assessor
T+4.0 s   Persistent detection ≥ THREAT_CONFIRMATION_MS at conf ≥ 40%
          → gateway prints {"type":"threat_confirmed", ...} to Pi
          → broadcasts 'P' to pucks (pause)
T+4.1 s   Pi reads JSON line, parses, hands lat/lon to mission state machine
T+4.1 s   drone_mission.py: WAIT_GPS_FIX
T+4.5 s   FC has 3-D fix → SET_GUIDED → arm command → motor spin
T+8.0 s   Takeoff to 2 m AGL
T+15-60 s Cruise to waypoint (3 m/s in current params)
T+...     Inside 5 m radius → hover 10 s
T+...+10  RTL → descend → disarm → mission done
```

If any radar puck loses target during the threat window the gateway lets the threat decay to IDLE and resumes pucks.

---

## 5. Key Parameters & Their Tradeoffs

| Param | Where | Value | Effect of changing |
|---|---|---|---|
| `MIN_CONFIDENCE_LEVEL` | gateway | 40 | Lower = trips on weaker / noisier signals; higher = misses real but marginal targets |
| `THREAT_CONFIRMATION_MS` | gateway | 2000 | How long a target must persist before it's "real". Lower = faster response, more false launches |
| `NODE_TIMEOUT_MS` | gateway | 30000 | How long without a packet before a puck is dropped from the active list |
| `CONFIRMED_DETECTION_MS` | puck | per puck | Anti-flicker for the radar — too low = jumps fire on twitches, too high = misses fast walkers |
| `MOCK_GPS_ENABLED` | puck | `false` | If true, all pucks report identical hardcoded coords. Useful for indoor demo, dangerous for live flight |
| `CRUISE_ALTITUDE_M` | Pi | 2.0 | Above launch, not above target |
| `WAYPOINT_RADIUS_M` | Pi | 5.0 | Acceptance bubble |
| `HOVER_DURATION_S` | Pi | 10 | How long the drone loiters before RTL |
| `FS_THR_ENABLE` | FC | 0 | RC failsafe — must be off because there's no RC bound |
| `ARMING_CHECK` | FC | 1 | Full pre-arm checks, do not disable |

---

## 6. Safety Architecture

**Currently weak. Until an RC receiver is bound, the only kill paths are:**

1. **Mission Planner connected to TCP 5760** — operator's finger on Disarm/RTL/Land buttons. Has to be running on a laptop on the Pi's Wi-Fi the whole flight.
2. **`kill.py` (when present on Pi)** — `kill.py rtl` switches mode to RTL, `kill.py land` to LAND, no-arg force-disarms with magic value `21196` (which works mid-flight, but the drone *will fall*).
3. **GCS Failsafe** (`FS_GCS_ENABLE = 2`) — if MAVLink heartbeats stop arriving, FC auto-LANDs. Dead-man switch for Pi crashes.
4. **Geofence** (when configured) — ALT_MAX, FENCE_RADIUS, with FENCE_ACTION = RTL.
5. **Battery failsafes** — RTL on low, LAND on critical.

The single point of failure is the Pi. If it freezes mid-flight without crashing the MAVLink connection, the drone could continue executing its last GUIDED command. GCS Failsafe mitigates this only if the heartbeat actually stops.

**Mitigation plan**: bind any cheap RC receiver (ELRS, FrSky, etc.), assign a switch to "motor disarm" or mode-flip to STABILIZE, restore `FS_THR_ENABLE = 1`. This gives a hardware-level kill that doesn't depend on software.

---

## 7. Known Issues & Gotchas

1. **mavlink-router heartbeat gating** — clients must send a heartbeat *before* `wait_heartbeat()` or they'll wait forever. Already fixed in all scripts.
2. **ESP-NOW peer struct must be zeroed** — uninitialized `ifidx` makes `esp_now_send` silently fail. Already fixed.
3. **JSON line corruption** — gateway must build JSON in a buffer, not via multi-call `Serial.print`, because the ESP-NOW recv callback runs on a different task and interleaves prints. Already fixed.
4. **Puck `WiFi.macAddress()` returns 00:00:00:00:00:00 at boot** — Arduino-ESP32 quirk if called too early. Cosmetic only; ESP-NOW uses the real MAC from the radio.
5. **Brownout detector** — pucks reset under cheap USB cables / weak USB sources. Use a beefier cable and battery for field deployment.
6. **GPS HDOP indoors / near buildings** — easily 2.5-3.5 with 7-8 sats; ArduPilot needs <1.4 by default for `Position Estimate Ready`. Open sky required, not just "outside."
7. **Compass mag-field error** — fires after FC reset or when battery wires/metal are too close. Recalibrate via `MAV_CMD_DO_START_MAG_CAL` (custom Python script does this without Mission Planner needing the heavy MAG_CAL_PROGRESS stream).
8. **Single TCP endpoint** — Mission Planner and `drone_mission.py` cannot both connect at once. Add a second `TcpServerEndpoint` to mavlink-router config if both are needed simultaneously.

---

## 8. Repository Layout

```
repo/
├── detection_node/
│   └── detection_node.ino             ← puck firmware (Arduino, ESP32-S3)
├── gateway_controller_usb/
│   └── gateway_controller_usb.ino     ← gateway firmware (USB-to-Pi variant)
├── gateway_controller/
│   └── gateway_controller.ino         ← gateway firmware (UART variant, unused currently)
├── drone_mission.py                   ← Pi mission orchestrator (the brain)
├── fly.py                             ← Standalone flight test (no pucks needed)
├── kill.py                            ← Emergency disarm / RTL / LAND
├── motor_test.py                      ← Bench motor verification (props OFF)
├── monitor.py                         ← Telemetry inspector
├── raspberry_pi_example.py            ← Older reference / example
├── mavlink-router-main.conf           ← UART⇆TCP router config
└── ARGUS_REPORT.md                    ← This document
```

---

## 9. Operational Status (as of 2026-04-28)

- ✅ Full pipeline verified end-to-end on the bench (puck → gateway → Pi → MAVLink → FC → motor command).
- ✅ Outdoor GPS lock confirmed at 7-8 sats / HDOP ~2.8 (open sky needed for tighter HDOP).
- ✅ Compass calibration completed via `calmag.py`.
- ✅ Puck firmware switched to live GPS, gateway threshold raised to 40 % for outdoor use.
- ⏳ Pending: first autonomous outdoor flight.
- ⏳ Pending: RC receiver bind for hardware kill.
- ⏳ Pending: geofence + GCS-failsafe parameter set finalized in MP.

---

## 10. Glossary

| Term | Meaning |
|---|---|
| ESP-NOW | Espressif's connectionless 2.4 GHz protocol; lower latency than Wi-Fi, no AP needed |
| MAVLink | The lingua franca of ArduPilot; binary message protocol over serial/TCP/UDP |
| EKF | Extended Kalman Filter — fuses GPS, IMU, compass into a position+velocity estimate |
| HDOP | Horizontal Dilution of Precision; lower = better GPS geometry |
| GUIDED | ArduCopter mode where waypoints come from MAVLink commands |
| RTL | Return To Launch — autonomous mode, drone flies back home and lands |
| GCS | Ground Control Station (e.g., Mission Planner) |
| Pre-arm check | ArduPilot's safety gate before motors can spin |
