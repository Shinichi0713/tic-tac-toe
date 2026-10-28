#!/usr/bin/env python3
"""
Raspberry Pi Host Example for Stewart Platform Teensy Controller
- Sends target pose (x,y,z,roll,pitch,yaw) as JSON over USB Serial
- Receives leg lengths and alarm status
"""

import serial
import json
import time
import numpy as np


class StewartHost:
    def __init__(self, port='/dev/ttyACM0', baud=921600):
        self.ser = serial.Serial(port, baud, timeout=0.01)
        time.sleep(1.5)  # wait for Teensy boot

    def send_target(self, x, y, z, roll, pitch, yaw):
        cmd = {"x": x, "y": y, "z": z,
               "r": roll, "p": pitch, "y": yaw}
        packet = json.dumps(cmd) + '\n'
        self.ser.write(packet.encode('utf-8'))

    def read_status(self):
        line = self.ser.readline()
        if line:
            try:
                return json.loads(line.decode('utf-8'))
            except Exception as e:
                print("JSON parse error:", e)
        return None


def demo_sine_motion():
    host = StewartHost(port='/dev/ttyACM0', baud=921600)
    print("Host started. Sending sine motion...")

    t0 = time.time()
    try:
        while True:
            t = time.time() - t0

            # Example: 0.5Hz sine on pitch, small Z wobble
            pitch = 0.15 * np.sin(2.0 * np.pi * 0.5 * t)
            z     = 120.0 + 5.0 * np.sin(2.0 * np.pi * 0.3 * t)

            host.send_target(0.0, 0.0, z, 0.0, pitch, 0.0)

            status = host.read_status()
            if status:
                print(f"leg={status.get('leg')}, alarm={status.get('alarm')}")

            time.sleep(0.010)  # 100Hz command rate
    except KeyboardInterrupt:
        print("Stopped.")


if __name__ == '__main__':
    demo_sine_motion()
