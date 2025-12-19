# Argus System Documentation

## Table of Contents

1. [System Overview](#system-overview)
2. [Detection Node](#detection-node)
3. [Gateway Controller](#gateway-controller)
4. [Raspberry Pi Integration](#raspberry-pi-integration)
5. [Data Flow](#data-flow)
6. [Hardware Configuration](#hardware-configuration)
7. [Configuration Reference](#configuration-reference)
8. [Troubleshooting](#troubleshooting)
9. [Future Enhancements](#future-enhancements)

---

## System Overview

The Argus system consists of three tiers:

1. **Detection Nodes** - Distributed ESP32 units with radar, GPS, and IMU sensors
2. **Gateway Controller** - Central ESP32 hub that manages nodes and validates threats
3. **Raspberry Pi** - Mission controller for drone deployment and response logic

```
Detection Nodes              Gateway                    Raspberry Pi
+-----------------+         +------------------+        +------------------+
| C4001 Radar     |         |                  |        |                  |
| GPS Module      |--ESP-NOW-->| Node Registry  |--UART-->| Threat Handler  |
| LSM303 IMU      |         | Threat Validator |        | Drone Controller |
+-----------------+         +------------------+        +------------------+
    (1 to 10)
```

---

## Detection Node

### Firmware: `detection_node.ino`

The detection node is a standalone sensor unit that monitors for targets and reports detections to the gateway.

### Hardware Requirements

| Component | Model | Interface |
|-----------|-------|-----------|
| Microcontroller | FireBeetle 2 ESP32-E | - |
| Radar | DFRobot C4001 mmWave | UART (Serial1) |
| GPS | Adafruit Ultimate GPS v3 | UART (Serial2) |
| IMU | LSM303DLHC | I2C |

### Pin Configuration

```
Radar C4001:
  TX -> D2 (GPIO RX1)
  RX -> D3 (GPIO TX1)

GPS Module:
  TX -> GPIO17 (RX2)
  RX -> GPIO16 (TX2)

IMU LSM303DLHC:
  SDA -> GPIO21
  SCL -> GPIO22
```

### Detection Algorithm

High-confidence detection requires all of the following:

1. **Energy threshold**: Smoothed energy >= 20% above dynamic noise floor
2. **Sample count**: 5+ consecutive strong samples
3. **Duration**: Detection persists for 1+ second
4. **Range**: Target within 23 meters
5. **Speed**: Target speed <= 20 m/s

**Confidence Calculation:**
- 60% weight: Energy level above noise floor
- 40% weight: Detection duration (50% per second, capped at 100%)
- Minimum 70% required for gateway threat confirmation

### Power Optimization

Dynamic sampling rates balance responsiveness with power consumption:

| Mode | Radar | IMU | GPS | Trigger |
|------|-------|-----|-----|---------|
| Idle | 200ms | 100ms | 1000ms | No active detection |
| Active | 100ms | 50ms | 500ms | Target being tracked |

Additional optimizations:
- WiFi modem sleep enabled (ESP-NOW remains functional)
- I2C timeout reduced to 5ms
- Reduced serial output when paused

**Power Consumption:**
- Idle: ~120mA
- Active: ~150mA
- Peak (ESP-NOW TX): ~180mA
- Battery life (3000mAh LiPo): 20-25 hours

### ESP-NOW Message Format

```cpp
struct DetectionMessage {
  uint8_t nodeId;              // Unique node identifier (1-255)
  uint32_t sequenceNumber;     // Incremental detection counter
  unsigned long timestamp;     // Milliseconds since boot
  float latitude;              // GPS coordinates
  float longitude;
  float altitude;
  float range_m;               // Radar range measurement
  float speed_mps;             // Target speed
  float heading_deg;           // IMU compass heading
  float energyLevel;           // Normalized 0.0-1.0
  uint8_t satellites;          // GPS quality indicator
  uint8_t confidenceLevel;     // 0-100 detection confidence
};
```

### Serial Commands

| Command | Action |
|---------|--------|
| `c` | Start 15-second IMU calibration |
| `x` | Cancel calibration |

**Calibration Procedure:**
1. Send `c` command
2. Rotate device through all orientations for 15 seconds
3. Calibration completes automatically and stores offsets

### CSV Output Format

```
time_ms,targets,range_m,speed_mps,energy_pct,heading_deg,gps_valid,lat,lon,sats
12450,1,15.234,2.150,45.23,287.45,1,37.774929,-122.419416,8
```

---

## Gateway Controller

### Firmware: `gateway_controller.ino`

The gateway aggregates detections from all nodes, validates threats, and coordinates with the Raspberry Pi.

### Hardware Requirements

| Component | Model | Interface |
|-----------|-------|-----------|
| Microcontroller | FireBeetle 2 ESP32-E | - |
| Pi Connection | UART | Serial2 |

### Pin Configuration

```
Raspberry Pi Connection:
  GPIO16 (Gateway RX) -> Pi TX
  GPIO17 (Gateway TX) -> Pi RX
  GND -> Pi GND

Baud Rate: 115200
```

### Features

**Node Management:**
- Auto-registers nodes on first detection message
- Supports up to 10 simultaneous nodes
- Monitors node health (30-second timeout)
- Stores last known GPS position per node

**Threat Validation:**
- Requires >= 70% confidence from detection node
- Threat must persist >= 2 seconds before confirmation
- Calculates target GPS position from node position + bearing + range
- Single active threat at a time

**Detection Pause System:**
- On threat confirmation: broadcasts PAUSE to all nodes
- Prevents repeated detections for same target
- Auto-resumes after 2-minute timeout if Pi unresponsive

### Mission State Machine

```
IDLE
  -> THREAT_DETECTED (high-confidence detection received)
     -> DRONE_DEPLOYING (threat confirmed, PAUSE sent, Pi notified)
        -> DRONE_EN_ROUTE (Pi confirms launch)
           -> DRONE_ENGAGING (drone at target)
              -> MISSION_COMPLETE (Pi reports done)
                 -> IDLE (RESUME sent to nodes)
```

### Messages to Raspberry Pi

**Detection Event** (every detection):
```json
{
  "type": "detection",
  "node": 1,
  "seq": 5,
  "ts": 12450,
  "lat": 37.774929,
  "lon": -122.419416,
  "alt": 10.5,
  "range": 15.2,
  "speed": 2.1,
  "heading": 287.4,
  "energy": 0.45,
  "conf": 85,
  "sats": 8
}
```

**Threat Confirmed** (when validation passes):
```json
{
  "type": "threat_confirmed",
  "node": 1,
  "conf": 87,
  "target_lat": 37.775123,
  "target_lon": -122.419587,
  "range": 15.2,
  "speed": 2.1,
  "heading": 287.4
}
```

**Status Update** (every 10 seconds):
```json
{
  "type": "status",
  "state": "IDLE",
  "nodes": 3,
  "detections": 127,
  "threats": 5
}
```

### Commands from Raspberry Pi

| Command | Action |
|---------|--------|
| `DRONE_STATUS:LAUNCHED` | Drone has launched |
| `DRONE_STATUS:ENGAGING` | Drone engaging target |
| `DRONE_STATUS:COMPLETE` | Mission complete, resume detection |
| `DRONE_STATUS:FAILED` | Mission failed, resume detection |
| `STATUS` | Request status update |
| `RESUME` | Manual resume (emergency override) |

---

## Raspberry Pi Integration

### Example Script: `raspberry_pi_example.py`

The example script demonstrates receiving detection data and controlling the gateway.

### Setup

```bash
# Enable serial hardware
sudo raspi-config
# Interface Options -> Serial Port -> Login shell: NO, Hardware: YES

# Install dependencies
pip3 install pyserial

# Test connection
python3 raspberry_pi_example.py
```

### Integration Point

Replace the placeholder in `handle_threat()` with drone deployment logic:

```python
def handle_threat(self, data):
    target_lat = data['target_lat']
    target_lon = data['target_lon']

    try:
        self.report_drone_status("LAUNCHED")

        # Your navigation code here
        # navigate_to(target_lat, target_lon)

        self.report_drone_status("ENGAGING")

        # Your investigation code here
        # investigate_target()

        self.report_drone_status("COMPLETE")

    except Exception as e:
        print(f"Mission failed: {e}")
        self.report_drone_status("FAILED")
```

---

## Data Flow

```
DETECTION NODE                 GATEWAY                    RASPBERRY PI

1. Radar detects target
   at 15m, bearing 287

2. GPS fixes node position

3. IMU measures heading

4. Build confidence over
   1+ second

5. Send ESP-NOW message -----> Receive detection
   - Node ID                   Register/update node
   - GPS coords                Track health
   - Range/speed
   - Heading                   Calculate target GPS
   - Confidence                from node + bearing

                               Validate confidence
                               and persistence

                               If confirmed:
                                 Send JSON -----------> Receive threat_confirmed
                                 threat_confirmed       {target_lat, target_lon}

                                 Broadcast PAUSE
                                 to all nodes          Execute drone mission

6. Stop sending     <--------- Send PAUSE command
   detections                                          Send DRONE_STATUS:LAUNCHED
   (paused)
                                                       Drone navigates

                                                       Send DRONE_STATUS:COMPLETE

7. Resume           <--------- Broadcast RESUME  <---- Gateway resumes on
   normal ops                  command                 Pi command
```

---

## Hardware Configuration

### Detection Node Wiring

| Component | Pin | ESP32 Pin | Notes |
|-----------|-----|-----------|-------|
| Radar TX | TX | D2 | Serial1 RX |
| Radar RX | RX | D3 | Serial1 TX |
| GPS TX | TX | GPIO17 | Serial2 RX |
| GPS RX | RX | GPIO16 | Serial2 TX |
| IMU SDA | SDA | GPIO21 | I2C Data |
| IMU SCL | SCL | GPIO22 | I2C Clock |
| All VCC | - | 3.3V | Shared power |
| All GND | - | GND | Common ground |

### Gateway Wiring

| Component | Pin | ESP32 Pin | Notes |
|-----------|-----|-----------|-------|
| Pi TX | TX | GPIO16 | Serial2 RX |
| Pi RX | RX | GPIO17 | Serial2 TX |
| Pi GND | GND | GND | Common ground |

### Raspberry Pi Serial

```
Device: /dev/serial0
Baud: 115200
Data bits: 8
Stop bits: 1
Parity: None
```

---

## Configuration Reference

### Detection Node Parameters

```cpp
// detection_node.ino

// Line 33: Gateway MAC address
uint8_t gatewayAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Line 48: Magnetic declination for your location
static const float MAG_DECLINATION_DEG = -13.0f;

// Line 83: Unique node identifier
static const uint8_t NODE_ID = 1;

// Detection thresholds
static const float MIN_STRONG_ENERGY_NORM = 0.20f;     // Energy threshold
static const uint8_t REQUIRED_STRONG_SAMPLES = 5;      // Consecutive samples
static const uint32_t CONFIRMED_DETECTION_MS = 1000;   // Persistence time
static const float MAX_RANGE_METERS = 23.0f;           // Max valid range
```

### Gateway Parameters

```cpp
// gateway_controller.ino

static const uint8_t MIN_CONFIDENCE_LEVEL = 70;        // Min confidence %
static const uint32_t THREAT_CONFIRMATION_MS = 2000;   // Threat persistence
static const uint32_t MISSION_TIMEOUT_MS = 120000;     // 2-min auto-resume
static const uint32_t NODE_TIMEOUT_MS = 30000;         // Node offline timeout
static const uint8_t MAX_NODES = 10;                   // Maximum nodes
```

### Magnetic Declination

Find your location's magnetic declination at:
https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml

---

## Troubleshooting

### Detection Node Issues

**Radar initialization failure:**
- Verify Serial1 wiring (D2/D3)
- Confirm C4001 is powered
- Radar auto-retries every 1 second

**IMU initialization failure:**
- Check I2C wiring (SDA=21, SCL=22)
- Try alternate accelerometer address (0x18 vs 0x19)
- IMU auto-retries every 750ms

**GPS not acquiring fix:**
- Requires outdoor placement or window proximity
- Check Serial2 wiring (GPIO16/17)
- Cold start may take 1-5 minutes
- Verify GPS TX connects to ESP32 RX

**ESP-NOW transmission failure:**
- Verify gateway MAC address is correct
- Confirm both devices on same WiFi channel
- Check range (250m outdoor maximum, less indoors)

### Gateway Issues

**Node slot exhausted:**
- Increase MAX_NODES constant if using more than 10 nodes
- Restart gateway to clear stale entries

**Node timeout detected:**
- Node has not transmitted in 30 seconds
- Check node power and operation
- Verify ESP-NOW range

**Mission timeout:**
- Auto-resumes after 2 minutes if Pi unresponsive
- Send DRONE_STATUS:COMPLETE to clear manually

### Raspberry Pi Issues

**Serial connection failure:**
- Verify serial hardware enabled in raspi-config
- Check wiring connections
- Confirm correct device path (/dev/serial0)

**JSON parse errors:**
- Verify baud rate matches (115200)
- Check for loose connections causing noise

---

## Future Enhancements

### Near-Term Improvements

**Battery Monitoring:**
ADC-based voltage monitoring with low-battery alerts to gateway.

**EEPROM Calibration Storage:**
Save IMU calibration across reboots for faster startup.

**Zone-Based Alerts:**
Define geographic zones with different risk levels and response protocols.

**Web Dashboard:**
Pi-hosted interface for live monitoring, detection history, and node status.

### Mid-Term Additions

**LLM Integration:**
AI-driven threat reasoning that analyzes detection context and recommends actions.

**Multi-Node Triangulation:**
Calculate precise target position from bearing intersection when multiple nodes detect the same target.

**Target Tracking:**
Track moving targets and predict future positions for efficient interception.

**OTA Firmware Updates:**
Wireless firmware updates from gateway to all nodes.

### Long-Term Research

**Multi-Modal Sensor Fusion:**
Combine radar, GPS, vision (YOLO), and voice (ASR) with LLM reasoning.

**Collaborative Patrol:**
Multiple drones coordinating coverage and response.

**Behavioral Learning:**
Build models of normal activity patterns and flag anomalies.

**Operator Cognitive Load Study:**
Measure effectiveness of autonomous vs manual threat assessment.

---

## Current Implementation Status

### Implemented Features

- Multiple detection nodes operating simultaneously
- ESP-NOW wireless communication (tested to 250m)
- GPS position tracking
- IMU heading with calibration
- Radar detection with confidence scoring
- Gateway aggregation and node tracking
- Serial JSON communication to Pi
- Detection pause/resume coordination
- Auto-recovery from node failures
- Mission timeout safety

### Not Yet Implemented

- LLM integration
- Vision classification (YOLO)
- Multi-node triangulation
- Threat scoring system
- Zone-based alerts
- Battery monitoring
- OTA updates
- Web dashboard
- Target tracking/prediction

---

## References

**Component Documentation:**
- DFRobot C4001 mmWave sensor datasheet
- Espressif ESP-NOW protocol documentation
- LSM303DLHC accelerometer/magnetometer datasheet

**Libraries:**
- DFRobot C4001 Arduino library
- Adafruit LSM303DLHC library
- TinyGPS++ library

---

**Version:** 1.0
**Last Updated:** 2025-01-17
