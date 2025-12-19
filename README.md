# Argus

Distributed Detection and Response System

## Overview

Argus is an autonomous perimeter monitoring system that integrates mmWave radar, GPS, and IMU sensors with a centralized gateway for threat detection and response coordination.

**Status:** Under Active Development

Current development focus:
- Drone deployment and autonomous navigation
- LLM integration for intelligent decision-making
- Multi-node sensor fusion and triangulation
- Full system integration testing

## Architecture

```
Detection Nodes (ESP32)              Gateway (ESP32)              Raspberry Pi
+-------------------+                +------------------+         +----------------+
| C4001 mmWave      |                |                  |         |                |
| GPS Module        | -- ESP-NOW --> | Node Management  | --UART->| Drone Control  |
| IMU (LSM303)      |                | Threat Validation|         | Mission Logic  |
+-------------------+                +------------------+         +----------------+
     (Multiple)
```

## Components

### Detection Nodes
- mmWave radar (C4001) for target detection at 0-23m range
- GPS for precise location tagging
- IMU magnetometer for target bearing calculation
- Dynamic power management (idle: 200ms, active: 100ms sampling)
- Multi-stage detection filtering with confidence scoring
- ESP-NOW wireless communication

### Gateway Controller
- Manages up to 10 detection nodes
- Validates detections before triggering response
- Coordinates detection pause during active missions
- Serial JSON interface for Raspberry Pi integration
- Node health monitoring with timeout detection

## Hardware Requirements

**Detection Node:**
- FireBeetle 2 ESP32-E
- DFRobot C4001 mmWave Radar
- Adafruit Ultimate GPS v3
- LSM303DLHC IMU

**Gateway:**
- FireBeetle 2 ESP32-E
- UART connection to Raspberry Pi

## Quick Start

### 1. Gateway Setup

Upload `gateway_controller.ino` to the gateway ESP32. Note the MAC address from Serial Monitor output.

### 2. Detection Node Setup

For each detection node:

```cpp
// Edit detection_node.ino
// Line 33: Set gateway MAC address
uint8_t gatewayAddress[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};

// Line 83: Set unique node ID
static const uint8_t NODE_ID = 1;
```

Upload to each node ESP32.

### 3. Raspberry Pi Setup

```bash
# Enable serial hardware
sudo raspi-config
# Interface Options > Serial Port > Login shell: NO, Hardware: YES

# Install dependencies
pip3 install pyserial

# Run integration example
python3 raspberry_pi_example.py
```

## Communication Protocol

### ESP-NOW (Node to Gateway)
Binary struct containing detection data including GPS coordinates, radar measurements, and confidence scores.

### Serial UART (Gateway to Pi)
JSON messages at 115200 baud:

```json
{"type":"threat_confirmed","node":1,"conf":87,"target_lat":37.775123,"target_lon":-122.419587,"range":15.2}
```

### Pi Commands to Gateway
```
DRONE_STATUS:LAUNCHED
DRONE_STATUS:ENGAGING
DRONE_STATUS:COMPLETE
```

## Key Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| Detection Range | 0-23m | Radar effective range |
| Min Confidence | 70% | Threat confirmation threshold |
| Confirmation Time | 1 second | Required detection persistence |
| Mission Timeout | 2 minutes | Auto-resume if no Pi response |
| Node Timeout | 30 seconds | Offline detection threshold |

## Power Consumption

| Component | Idle | Active |
|-----------|------|--------|
| Detection Node | 120mA | 150mA |
| Gateway | 100mA | 130mA |

Estimated battery life with 3000mAh LiPo: 20-25 hours

## Files

| File | Description |
|------|-------------|
| `detection_node.ino` | Detection node firmware |
| `gateway_controller.ino` | Gateway firmware |
| `raspberry_pi_example.py` | Pi integration example |
| `SYSTEM_DOCUMENTATION.md` | Technical reference |

## Documentation

See [SYSTEM_DOCUMENTATION.md](SYSTEM_DOCUMENTATION.md) for complete technical documentation including:
- Pin configurations
- Detection algorithms
- Communication protocols
- Troubleshooting guide
- Future enhancements

## License

MIT License
