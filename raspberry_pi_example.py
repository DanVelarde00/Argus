#!/usr/bin/env python3
"""
ARGUS Raspberry Pi Integration Example
Receives detection data from ESP32 Gateway and manages drone deployment

Usage:
    python3 raspberry_pi_example.py [serial_port]

Default serial port: /dev/serial0 (Pi hardware UART)
"""

import serial
import json
import time
import sys
from datetime import datetime

class ArgusGatewayInterface:
    def __init__(self, port='/dev/ttyUSB0', baud=115200):
        """Initialize serial connection to ESP32 Gateway

        Default port: /dev/ttyUSB0 (USB connection to FireBeetle)
        Alternate ports to try: /dev/ttyACM0, /dev/ttyUSB1
        For GPIO UART: /dev/serial0
        """
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            time.sleep(2)  # Wait for connection to stabilize
            print(f"[ARGUS] Connected to gateway on {port} @ {baud} baud")
        except serial.SerialException as e:
            print(f"[ERROR] Could not open serial port {port}: {e}")
            sys.exit(1)

        self.detection_count = 0
        self.threat_count = 0
        self.active_mission = False

    def send_command(self, command):
        """Send command to gateway"""
        msg = f"{command}\n"
        self.ser.write(msg.encode('utf-8'))
        print(f"[TX] {command}")

    def request_status(self):
        """Request status update from gateway"""
        self.send_command("STATUS")

    def report_drone_status(self, status):
        """
        Report drone mission status to gateway
        Status: LAUNCHED, ENGAGING, COMPLETE, FAILED
        """
        self.send_command(f"DRONE_STATUS:{status}")

    def manual_resume(self):
        """Manually resume detection (emergency override)"""
        self.send_command("RESUME")
        self.active_mission = False

    def handle_detection(self, data):
        """Process detection message from node"""
        self.detection_count += 1

        node_id = data.get('node', '?')
        lat = data.get('lat', 0.0)
        lon = data.get('lon', 0.0)
        range_m = data.get('range', 0.0)
        conf = data.get('conf', 0)

        print(f"\n[DETECTION #{self.detection_count}]")
        print(f"  Node: {node_id}")
        print(f"  Location: {lat:.6f}, {lon:.6f}")
        print(f"  Range: {range_m:.1f}m")
        print(f"  Confidence: {conf}%")
        print(f"  Timestamp: {datetime.now().strftime('%H:%M:%S.%f')[:-3]}")

        # Optional: Log to file
        with open('argus_detections.log', 'a') as f:
            f.write(f"{datetime.now().isoformat()},{json.dumps(data)}\n")

    def handle_threat(self, data):
        """Process confirmed threat - deploy drone"""
        self.threat_count += 1

        target_lat = data.get('target_lat', 0.0)
        target_lon = data.get('target_lon', 0.0)
        confidence = data.get('conf', 0)
        node_id = data.get('node', '?')
        speed = data.get('speed', 0.0)
        heading = data.get('heading', 0.0)

        print("\n" + "="*60)
        print(f"  THREAT CONFIRMED - DEPLOYING DRONE")
        print("="*60)
        print(f"  Source Node: {node_id}")
        print(f"  Confidence: {confidence}%")
        print(f"  Target Position: {target_lat:.6f}, {target_lon:.6f}")
        print(f"  Target Speed: {speed:.1f} m/s @ {heading:.1f}°")
        print(f"  Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print("="*60 + "\n")

        # Mark mission active
        self.active_mission = True

        # Log threat
        with open('argus_threats.log', 'a') as f:
            f.write(f"{datetime.now().isoformat()},{json.dumps(data)}\n")

        # ========================================
        # YOUR DRONE DEPLOYMENT CODE HERE
        # ========================================

        # Example drone deployment (replace with your actual code):
        try:
            print("[DRONE] Initializing deployment...")
            self.report_drone_status("LAUNCHED")

            # Simulate drone flight
            print(f"[DRONE] Flying to target: {target_lat:.6f}, {target_lon:.6f}")
            time.sleep(2)  # Replace with actual drone control

            print("[DRONE] Arrived at target location")
            self.report_drone_status("ENGAGING")

            # Simulate engagement
            time.sleep(3)  # Replace with actual engagement logic

            print("[DRONE] Mission complete - returning to base")
            self.report_drone_status("COMPLETE")

            self.active_mission = False

        except Exception as e:
            print(f"[ERROR] Drone deployment failed: {e}")
            self.report_drone_status("FAILED")
            self.active_mission = False

    def handle_status(self, data):
        """Process gateway status update"""
        state = data.get('state', 'UNKNOWN')
        nodes = data.get('nodes', 0)
        detections = data.get('detections', 0)
        threats = data.get('threats', 0)

        print(f"\n[GATEWAY STATUS]")
        print(f"  State: {state}")
        print(f"  Active Nodes: {nodes}")
        print(f"  Total Detections: {detections}")
        print(f"  Total Threats: {threats}")

    def process_message(self, line):
        """Parse and handle incoming JSON message"""
        try:
            data = json.loads(line)
            msg_type = data.get('type', '')

            if msg_type == 'detection':
                self.handle_detection(data)
            elif msg_type == 'threat_confirmed':
                self.handle_threat(data)
            elif msg_type == 'status':
                self.handle_status(data)
            else:
                print(f"[UNKNOWN] Message type: {msg_type}")

        except json.JSONDecodeError:
            print(f"[WARN] Invalid JSON: {line}")
        except Exception as e:
            print(f"[ERROR] Processing message: {e}")

    def run(self):
        """Main processing loop"""
        print("\n[ARGUS] System ready - waiting for detections...")
        print("[INFO] Press Ctrl+C to exit\n")

        # Request initial status
        time.sleep(1)
        self.request_status()

        buffer = ""

        try:
            while True:
                # Read from gateway
                if self.ser.in_waiting > 0:
                    chunk = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                    buffer += chunk

                    # Process complete lines
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()

                        if line and line.startswith('{'):
                            self.process_message(line)

                # Small delay to prevent CPU spinning
                time.sleep(0.01)

        except KeyboardInterrupt:
            print("\n\n[ARGUS] Shutting down...")
            if self.active_mission:
                print("[WARN] Mission active - sending resume command")
                self.manual_resume()
            self.ser.close()
            print("[ARGUS] Goodbye!")

def main():
    """Main entry point"""
    # Get serial port from command line or use default
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'

    print("="*60)
    print("  ARGUS Raspberry Pi Gateway Interface")
    print("="*60)
    print(f"Connecting to gateway on {port}")
    print("To use GPIO UART instead: python3 raspberry_pi_example.py /dev/serial0")
    print("="*60)

    # Create interface and run
    gateway = ArgusGatewayInterface(port)
    gateway.run()

if __name__ == '__main__':
    main()
