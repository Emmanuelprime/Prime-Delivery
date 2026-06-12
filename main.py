#!/usr/bin/env python3
"""
High-level robot controller
Sends velocity commands to ESP32, receives odometry telemetry
"""

import serial
import math
import time
import threading
from dataclasses import dataclass
from typing import Optional
# import numpy as np

@dataclass
class RobotState:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    v: float = 0.0
    omega: float = 0.0
    gyro_z: float = 0.0
    left_vel: float = 0.0
    right_vel: float = 0.0
    timestamp: float = 0.0

class ESP32Interface:
    """Handles serial communication with ESP32"""
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=460800):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.state = RobotState()
        self.lock = threading.Lock()
        self.running = False
        
    def connect(self):
        self.serial = serial.Serial(self.port, self.baudrate, timeout=0.1)
        # Wait for ESP32 ready signal
        while True:
            line = self.serial.readline().decode().strip()
            if line == "ESP32_READY":
                print("Connected to ESP32!")
                break
                
    def start_reading(self):
        """Start telemetry reading thread"""
        self.running = True
        self.read_thread = threading.Thread(target=self._read_loop)
        self.read_thread.daemon = True
        self.read_thread.start()
        
    def _read_loop(self):
        """Read telemetry from ESP32"""
        while self.running:
            if self.serial and self.serial.in_waiting:
                try:
                    line = self.serial.readline().decode().strip()
                    if line.startswith('o '):
                        parts = line.split()
                        with self.lock:
                            self.state.x = float(parts[1])
                            self.state.y = float(parts[2])
                            self.state.theta = float(parts[3])
                            self.state.v = float(parts[4])
                            self.state.omega = float(parts[5])
                            self.state.gyro_z = float(parts[6])
                            self.state.left_vel = float(parts[7])
                            self.state.right_vel = float(parts[8])
                            self.state.timestamp = time.time()
                except Exception as e:
                    print(f"Parse error: {e}")
                    
    def send_velocity(self, v: float, omega: float):
        """Send velocity command to ESP32"""
        cmd = f"v {v:.3f} {omega:.3f}\n"
        if self.serial:
            self.serial.write(cmd.encode())
            
    def emergency_stop(self):
        """Immediate stop"""
        if self.serial:
            self.serial.write(b"s\n")
            
    def tune_pid(self, kp=None, ki=None, kd=None):
        """Remotely tune PID gains"""
        if kp is not None:
            self.serial.write(f"kp {kp}\n".encode())
        if ki is not None:
            self.serial.write(f"ki {ki}\n".encode())
        if kd is not None:
            self.serial.write(f"kd {kd}\n".encode())
            
    def reset_odometry(self):
        self.serial.write(b"reset\n")
        
    def get_state(self) -> RobotState:
        with self.lock:
            return self.state
            
    def stop(self):
        self.running = False
        self.emergency_stop()
        if self.serial:
            self.serial.close()


class NavigationController:
    """Go-to-point controller"""
    
    def __init__(self, kp_heading=1.5, ki_heading=0.1, kd_heading=0.3,
                 max_linear_vel=0.3, max_angular_vel=1.0):
        self.kp_h = kp_heading
        self.ki_h = ki_heading
        self.kd_h = kd_heading
        self.max_v = max_linear_vel
        self.max_omega = max_angular_vel
        
        self.heading_integral = 0
        self.prev_heading_error = 0
        
    def compute(self, current: RobotState, goal_x: float, goal_y: float, dt: float):
        """Returns (v_cmd, omega_cmd)"""
        dx = goal_x - current.x
        dy = goal_y - current.y
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < 0.1:  # Goal reached
            return 0.0, 0.0
        
        # Target heading
        target_heading = math.atan2(dy, dx)
        
        # Heading error
        heading_error = target_heading - current.theta
        heading_error = math.atan2(math.sin(heading_error), 
                                   math.cos(heading_error))  # Normalize
        
        # Heading PID
        self.heading_integral += heading_error * dt
        self.heading_integral = max(-0.5, min(0.5, self.heading_integral))
        
        heading_derivative = (heading_error - self.prev_heading_error) / dt
        self.prev_heading_error = heading_error
        
        omega_cmd = (self.kp_h * heading_error + 
                     self.ki_h * self.heading_integral + 
                     self.kd_h * heading_derivative)
        
        # Limit angular velocity
        omega_cmd = max(-self.max_omega, min(self.max_omega, omega_cmd))
        
        # Velocity profile (trapezoidal)
        v_cmd = min(self.max_v, 0.1 * math.sqrt(distance))
        
        # Reduce speed when heading error is large
        if abs(heading_error) > 0.5:
            v_cmd *= 0.2
        elif abs(heading_error) > 0.3:
            v_cmd *= 0.5
            
        return v_cmd, omega_cmd


class DeliveryRobot:
    """Main robot controller"""
    
    def __init__(self, esp32: ESP32Interface):
        self.esp32 = esp32
        self.navigator = NavigationController()
        self.state = RobotState()
        
    def goto(self, x: float, y: float, timeout=30.0):
        """Blocking go-to-point with timeout"""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            self.state = self.esp32.get_state()
            
            # Check if arrived
            dist = math.sqrt((x - self.state.x)**2 + (y - self.state.y)**2)
            if dist < 0.1:
                print(f"Arrived! Distance: {dist:.3f}m")
                return True
            
            # Compute control
            v, omega = self.navigator.compute(self.state, x, y, 0.02)
            
            # Send to ESP32
            self.esp32.send_velocity(v, omega)
            
            # Print status
            print(f"\rDist: {dist:.2f}m | Heading err: {math.degrees(self.navigator.prev_heading_error):.1f}° | "
                  f"v: {v:.2f} ω: {omega:.2f}", end="")
            
            time.sleep(0.02)  # 50Hz
            
        print("\nTimeout!")
        return False
    
    def deliver(self, waypoints):
        """Execute delivery mission"""
        for i, (x, y) in enumerate(waypoints):
            print(f"\n--- Going to delivery point {i+1}: ({x}, {y}) ---")
            success = self.goto(x, y)
            if not success:
                print("Failed to reach waypoint!")
                break
            print(f"Delivering package {i+1}...")
            time.sleep(2)  # Simulate delivery
            

# ============================================
# MAIN
# ============================================
if __name__ == "__main__":
    # Connect to ESP32
    esp32 = ESP32Interface(port='/dev/ttyUSB0')
    esp32.connect()
    esp32.start_reading()
    
    # Create robot controller
    robot = DeliveryRobot(esp32)
    
    try:
        # Test: Send velocity command for 1 second
        print("Testing forward motion...")
        esp32.send_velocity(0.2, 0.0)  # 0.2 m/s straight
        time.sleep(2)
        esp32.send_velocity(0.0, 0.0)  # Stop
        
        # Execute delivery mission
        waypoints = [
            (2.0, 0.0),   # First delivery
            (2.0, 2.0),   # Second delivery
            (0.0, 0.0),   # Return home
        ]
        
        robot.deliver(waypoints)
        
    except KeyboardInterrupt:
        print("\nInterrupted!")
    finally:
        esp32.stop()