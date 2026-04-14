#!/usr/bin/env python3
"""
ARGUS Multi-Device Serial Monitor
Auto-detects all connected ESP32 devices (pucks + gateway) and shows
their output simultaneously with color coding.

Usage:
    python monitor.py                    # auto-detect all ports
    python monitor.py COM3 COM4 COM5     # specify ports manually

Commands (type and hit Enter):
    s       -> send status request to ALL pucks
    t       -> send test ESP-NOW ping to ALL pucks
    1s      -> send status to device #1 only
    2t      -> send test ping to device #2 only
    q       -> quit

Requirements:
    pip install pyserial
"""

import serial
import serial.tools.list_ports
import threading
import sys
import time
from datetime import datetime

# Colors per device
COLORS = [
    '\033[92m',   # green
    '\033[94m',   # blue
    '\033[93m',   # yellow
    '\033[95m',   # magenta
    '\033[96m',   # cyan
]
RESET   = '\033[0m'
BOLD    = '\033[1m'
RED     = '\033[91m'
WHITE   = '\033[97m'

BAUD = 115200

ports       = []
port_labels = []
port_colors = []

# Keywords that flag a line as high-priority (printed bold white)
ALERT_KEYWORDS = [
    'THREAT', 'DETECTION CONFIRMED', 'DRONE', 'CONFIRMED',
    'FAILED', 'TIMEOUT', 'ERROR', 'PAUSED', 'RESUMED',
    'Live fix acquired', 'Position saved', 'Loaded saved'
]


def is_alert(line: str) -> bool:
    return any(k.lower() in line.lower() for k in ALERT_KEYWORDS)


def find_esp32_ports():
    keywords = ['cp210', 'ch340', 'ch341', 'usb serial', 'usb-serial', 'ftdi', 'esp32']
    found = []
    for p in serial.tools.list_ports.comports():
        desc = (p.description or '').lower()
        hwid = (p.hwid or '').lower()
        if any(k in desc or k in hwid for k in keywords):
            found.append(p.device)
    return sorted(found)


def reader_thread(ser, label, color):
    buf = ''
    while True:
        try:
            waiting = ser.in_waiting
            if waiting:
                chunk = ser.read(waiting).decode('utf-8', errors='ignore')
                buf += chunk
                while '\n' in buf:
                    line, buf = buf.split('\n', 1)
                    line = line.strip()
                    if not line:
                        continue
                    ts = datetime.now().strftime('%H:%M:%S')
                    if is_alert(line):
                        print(f"{color}{BOLD}[{label}]{RESET} {ts}  {WHITE}{BOLD}{line}{RESET}", flush=True)
                    else:
                        print(f"{color}{BOLD}[{label}]{RESET} {ts}  {line}", flush=True)
            else:
                time.sleep(0.02)
        except serial.SerialException:
            print(f"{RED}[{label}] Disconnected{RESET}", flush=True)
            break
        except Exception as exc:
            print(f"{RED}[{label}] Error: {exc}{RESET}", flush=True)
            time.sleep(0.5)


def send_all(cmd):
    for i, ser in enumerate(ports):
        try:
            ser.write(f"{cmd}\n".encode())
            print(f"  -> '{cmd}' sent to {port_labels[i]}", flush=True)
        except Exception as exc:
            print(f"  -> FAILED {port_labels[i]}: {exc}", flush=True)


def send_one(idx, cmd):
    if 0 <= idx < len(ports):
        try:
            ports[idx].write(f"{cmd}\n".encode())
            print(f"  -> '{cmd}' sent to {port_labels[idx]}", flush=True)
        except Exception as exc:
            print(f"  -> FAILED: {exc}", flush=True)
    else:
        print(f"  -> No device #{idx+1}", flush=True)


def main():
    target_ports = sys.argv[1:] if len(sys.argv) > 1 else find_esp32_ports()

    if not target_ports:
        print("\nNo ESP32 devices found automatically.")
        print("Plug in your pucks/gateway and try again, or specify manually:")
        print("  python monitor.py COM3 COM4 COM5 COM6")
        print("\nAll ports currently visible:")
        for p in serial.tools.list_ports.comports():
            print(f"  {p.device}  -  {p.description}")
        sys.exit(1)

    print(f"\n{'='*62}")
    print("  ARGUS Multi-Device Monitor  (pucks + gateway)")
    print(f"{'='*62}")
    print("  Alert lines (detections, threats, GPS events) print BOLD")
    print(f"{'='*62}")

    for i, port in enumerate(target_ports):
        color = COLORS[i % len(COLORS)]
        # Label: first 3 devices are pucks, rest labelled as GW
        if i < 3:
            label = f"PUCK{i+1}:{port}"
        else:
            label = f"GW:{port}"
        try:
            ser = serial.Serial(port, BAUD, timeout=0.1)
            time.sleep(0.5)
            ports.append(ser)
            port_labels.append(label)
            port_colors.append(color)

            t = threading.Thread(target=reader_thread, args=(ser, label, color), daemon=True)
            t.start()
            print(f"  {color}OK  {label}{RESET}")
        except Exception as exc:
            print(f"  {RED}ERR {port}: {exc}{RESET}")

    if not ports:
        print("\nCould not open any ports. Check connections.")
        sys.exit(1)

    print(f"\nMonitoring {len(ports)} device(s).")
    print("Commands:")
    print("  s          status from ALL pucks")
    print("  t          test ESP-NOW ping from ALL pucks")
    print("  1s/2s/3s   status from specific puck")
    print("  1t/2t/3t   test ping from specific puck")
    print("  q          quit")
    print(f"{'='*62}\n")

    time.sleep(1.0)
    print("Requesting status from all devices...\n")
    send_all('s')

    try:
        while True:
            try:
                cmd = input().strip().lower()
            except EOFError:
                break

            if cmd == 'q':
                break
            elif cmd == 's':
                send_all('s')
            elif cmd == 't':
                send_all('t')
            elif len(cmd) == 2 and cmd[0].isdigit() and cmd[1] in ('s', 't'):
                send_one(int(cmd[0]) - 1, cmd[1])
            elif cmd:
                print("Commands: s, t, 1s, 2s, 3s, 1t, 2t, 3t, q")

    except KeyboardInterrupt:
        pass
    finally:
        for ser in ports:
            try:
                ser.close()
            except Exception:
                pass
        print("\nMonitor closed.")


if __name__ == '__main__':
    main()
