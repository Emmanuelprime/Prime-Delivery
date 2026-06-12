#!/usr/bin/env python3
"""
Simple Motor Test - No navigation, just drive motors
"""

import serial
import time
import sys
import glob

class SimpleMotorTest:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        
    def connect(self):
        print(f"Connecting to ESP32 on {self.port}...")
        
        self.serial = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=1.0
        )
        
        # Wait for ESP32 ready
        print("Waiting for ESP32 to boot...")
        start_time = time.time()
        while time.time() - start_time < 15:
            if self.serial.in_waiting:
                line = self.serial.readline().decode('latin-1').strip()
                if line:
                    print(f"  ESP32: {line}")
                if "ESP32_READY" in line:
                    print("✅ Connected!\n")
                    return True
            time.sleep(0.1)
        
        print("❌ Timeout!")
        return False
    
    def send_velocity(self, v, omega):
        """Send velocity command"""
        cmd = f"v {v:.3f} {omega:.3f}\n"
        print(f"Sending: {cmd.strip()}")
        self.serial.write(cmd.encode())
        self.serial.flush()
    
    def stop(self):
        """Stop motors"""
        print("Stopping motors...")
        self.serial.write(b"s\n")
        self.serial.flush()
        time.sleep(0.1)
        self.serial.close()
    
    def read_telemetry(self, duration=1.0):
        """Read and print telemetry for given duration"""
        print("Reading telemetry...")
        start = time.time()
        while time.time() - start < duration:
            if self.serial.in_waiting:
                try:
                    line = self.serial.readline().decode('latin-1').strip()
                    if line.startswith('o '):
                        parts = line.split()
                        if len(parts) >= 9:
                            print(f"  x={float(parts[1]):.3f}, y={float(parts[2]):.3f}, "
                                  f"v={float(parts[4]):.3f}, ω={float(parts[5]):.3f}, "
                                  f"L_vel={float(parts[7]):.3f}, R_vel={float(parts[8]):.3f}")
                except:
                    pass
            time.sleep(0.01)

if __name__ == "__main__":
    print("=" * 50)
    print("SIMPLE MOTOR TEST")
    print("=" * 50)
    
    # Find port
    ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
    if not ports:
        print("No ESP32 found!")
        sys.exit(1)
    
    port = ports[0]
    print(f"Using port: {port}\n")
    
    # Connect
    tester = SimpleMotorTest(port)
    if not tester.connect():
        sys.exit(1)
    
    try:
        # Test 1: Very slow forward
        print("\n" + "=" * 50)
        print("TEST 1: Slow forward (0.10 m/s for 3 seconds)")
        print("=" * 50)
        print("Robot should move FORWARD slowly...")
        tester.send_velocity(0.10, 0.0)
        tester.read_telemetry(3.0)
        
        # Stop
        print("\nStopping...")
        tester.send_velocity(0.0, 0.0)
        time.sleep(1.0)
        
        # Test 2: Medium forward
        # print("\n" + "=" * 50)
        # print("TEST 2: Medium forward (0.20 m/s for 3 seconds)")
        # print("=" * 50)
        # print("Robot should move FORWARD at medium speed...")
        # tester.send_velocity(0.20, 0.0)
        # tester.read_telemetry(3.0)
        
        # # Stop
        # print("\nStopping...")
        # tester.send_velocity(0.0, 0.0)
        # time.sleep(1.0)
        
        # # Test 3: Turn left
        # print("\n" + "=" * 50)
        # print("TEST 3: Turn left (ω = 0.5 rad/s for 2 seconds)")
        # print("=" * 50)
        # print("Robot should TURN LEFT...")
        # tester.send_velocity(0.0, 0.5)
        # tester.read_telemetry(2.0)
        
        # # Stop
        # print("\nStopping...")
        # tester.send_velocity(0.0, 0.0)
        # time.sleep(1.0)
        
        # # Test 4: Turn right
        # print("\n" + "=" * 50)
        # print("TEST 4: Turn right (ω = -0.5 rad/s for 2 seconds)")
        # print("=" * 50)
        # print("Robot should TURN RIGHT...")
        # tester.send_velocity(0.0, -0.5)
        # tester.read_telemetry(2.0)
        
        # # Stop
        # print("\nStopping...")
        # tester.send_velocity(0.0, 0.0)
        
        print("\n" + "=" * 50)
        print("ALL TESTS COMPLETE")
        print("=" * 50)
        
    except KeyboardInterrupt:
        print("\n\nInterrupted!")
    finally:
        tester.stop()
        print("Done.")