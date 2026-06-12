#!/usr/bin/env python3
"""
Prime-Delivery Robot - High-Level Controller
Sends velocity commands to ESP32, receives odometry telemetry
"""

import serial
import math
import time
import threading
import sys
import glob
from dataclasses import dataclass

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
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.state = RobotState()
        self.lock = threading.Lock()
        self.running = False
        
    def connect(self):
        print(f"Connecting to ESP32 on {self.port} at {self.baudrate} baud...")
        
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1.0,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
        except serial.SerialException as e:
            print(f"❌ Failed to open serial port: {e}")
            print("\nTroubleshooting:")
            print("  1. Is ESP32 plugged in?")
            print(f"  2. Available ports: {glob.glob('/dev/tty*')}")
            print("  3. Permission: sudo chmod 666 /dev/ttyUSB0")
            sys.exit(1)
        
        # Flush any garbage data
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()
        
        # Wait for ESP32 ready signal with timeout
        print("Waiting for ESP32 ready signal...")
        start_time = time.time()
        timeout = 10  # 10 second timeout
        
        while time.time() - start_time < timeout:
            if self.serial.in_waiting:
                try:
                    raw_line = self.serial.readline()
                    
                    # Try different encodings
                    line = None
                    for encoding in ['utf-8', 'ascii', 'latin-1']:
                        try:
                            line = raw_line.decode(encoding).strip()
                            break
                        except (UnicodeDecodeError, UnicodeError):
                            continue
                    
                    if line:
                        # Print everything during connection phase
                        if "ESP32_READY" not in line:
                            print(f"  ESP32: {line}")
                    
                    if line and "ESP32_READY" in line:
                        print("✅ Connected to ESP32 successfully!")
                        return True
                        
                except Exception as e:
                    print(f"  Error reading: {e}")
                    continue
            
            time.sleep(0.1)
        
        print("❌ Timeout waiting for ESP32 ready signal!")
        print("\nCheck:")
        print("  1. Is the correct firmware uploaded?")
        print("  2. Is ESP32 connected to correct USB port?")
        print("  3. Try pressing the RST button on ESP32")
        print("  4. Close Arduino IDE Serial Monitor first!")
        sys.exit(1)
                
    def start_reading(self):
        """Start telemetry reading thread"""
        self.running = True
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()
        
    def _read_loop(self):
        """Read telemetry from ESP32"""
        while self.running:
            if self.serial and self.serial.in_waiting:
                try:
                    raw_line = self.serial.readline()
                    
                    # Try multiple encodings
                    line = None
                    for encoding in ['utf-8', 'ascii', 'latin-1']:
                        try:
                            line = raw_line.decode(encoding).strip()
                            break
                        except (UnicodeDecodeError, UnicodeError):
                            continue
                    
                    if line and line.startswith('o '):
                        parts = line.split()
                        if len(parts) >= 9:  # Make sure we have all fields
                            with self.lock:
                                try:
                                    self.state.x = float(parts[1])
                                    self.state.y = float(parts[2])
                                    self.state.theta = float(parts[3])
                                    self.state.v = float(parts[4])
                                    self.state.omega = float(parts[5])
                                    self.state.gyro_z = float(parts[6])
                                    self.state.left_vel = float(parts[7])
                                    self.state.right_vel = float(parts[8])
                                    self.state.timestamp = time.time()
                                except ValueError:
                                    pass  # Skip malformed numbers
                                    
                except Exception:
                    pass  # Ignore occasional read errors
                    
    def send_velocity(self, v: float, omega: float):
        """Send velocity command to ESP32"""
        cmd = f"v {v:.3f} {omega:.3f}\n"
        if self.serial:
            try:
                self.serial.write(cmd.encode())
                self.serial.flush()  # Ensure it's sent immediately
            except Exception as e:
                print(f"Send error: {e}")
            
    def emergency_stop(self):
        """Immediate stop"""
        if self.serial:
            try:
                self.serial.write(b"s\n")
                self.serial.flush()
            except Exception as e:
                print(f"Emergency stop error: {e}")
            
    def tune_pid(self, kp=None, ki=None, kd=None):
        """Remotely tune PID gains"""
        try:
            if kp is not None:
                self.serial.write(f"kp {kp}\n".encode())
            if ki is not None:
                self.serial.write(f"ki {ki}\n".encode())
            if kd is not None:
                self.serial.write(f"kd {kd}\n".encode())
        except Exception as e:
            print(f"Tune error: {e}")
            
    def reset_odometry(self):
        """Reset odometry to zero"""
        try:
            self.serial.write(b"reset\n")
            print("Odometry reset")
        except Exception as e:
            print(f"Reset error: {e}")
        
    def get_state(self) -> RobotState:
        """Get latest robot state (thread-safe)"""
        with self.lock:
            return self.state
            
    def stop(self):
        """Shutdown communication"""
        self.running = False
        self.emergency_stop()
        time.sleep(0.1)
        if self.serial:
            self.serial.close()
            print("Serial connection closed")


class NavigationController:
    """Go-to-point controller with heading PID"""
    
    def __init__(self, kp_heading=1.5, ki_heading=0.1, kd_heading=0.3,
                 max_linear_vel=0.3, max_angular_vel=1.0):
        self.kp_h = kp_heading
        self.ki_h = ki_heading
        self.kd_h = kd_heading
        self.max_v = max_linear_vel
        self.max_omega = max_angular_vel
        
        self.heading_integral = 0.0
        self.prev_heading_error = 0.0
        self.last_time = time.time()
        
    def compute(self, current: RobotState, goal_x: float, goal_y: float):
        """
        Returns (v_cmd, omega_cmd) in m/s and rad/s
        """
        dt = time.time() - self.last_time
        self.last_time = time.time()
        
        # Prevent division by zero
        if dt <= 0.0:
            dt = 0.02
        
        # Calculate distance and angle to goal
        dx = goal_x - current.x
        dy = goal_y - current.y
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < 0.1:  # Goal reached
            self.heading_integral = 0.0
            return 0.0, 0.0
        
        # Target heading
        target_heading = math.atan2(dy, dx)
        
        # Heading error (normalized to [-PI, PI])
        heading_error = target_heading - current.theta
        heading_error = math.atan2(math.sin(heading_error), 
                                   math.cos(heading_error))
        
        # Heading PID controller
        self.heading_integral += heading_error * dt
        self.heading_integral = max(-0.5, min(0.5, self.heading_integral))
        
        heading_derivative = (heading_error - self.prev_heading_error) / dt
        self.prev_heading_error = heading_error
        
        omega_cmd = (self.kp_h * heading_error + 
                     self.ki_h * self.heading_integral + 
                     self.kd_h * heading_derivative)
        
        # Limit angular velocity
        omega_cmd = max(-self.max_omega, min(self.max_omega, omega_cmd))
        
        # Velocity profile - trapezoidal
        v_cmd = min(self.max_v, 0.3 * math.sqrt(distance))
        
        # Reduce speed when heading error is large
        if abs(heading_error) > 0.5:      # >30 degrees off
            v_cmd *= 0.2
        elif abs(heading_error) > 0.3:    # >17 degrees off
            v_cmd *= 0.5
            
        return v_cmd, omega_cmd


class DeliveryRobot:
    """Main robot controller for delivery missions"""
    
    def __init__(self, esp32: ESP32Interface):
        self.esp32 = esp32
        self.navigator = NavigationController()
        self.state = RobotState()
        
    def goto(self, x: float, y: float, timeout=30.0):
        """Blocking go-to-point with timeout"""
        print(f"Going to ({x:.2f}, {y:.2f})...")
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            # Get latest state
            self.state = self.esp32.get_state()
            
            # Check if arrived
            dist = math.sqrt((x - self.state.x)**2 + (y - self.state.y)**2)
            if dist < 0.1:
                print(f"\n✅ Arrived! Final distance: {dist:.3f}m")
                return True
            
            # Compute control
            v, omega = self.navigator.compute(self.state, x, y)
            
            # Send to ESP32
            self.esp32.send_velocity(v, omega)
            
            # Print status
            heading_err = math.degrees(self.navigator.prev_heading_error)
            print(f"\r  Dist: {dist:.2f}m | Heading err: {heading_err:5.1f}° | "
                  f"v: {v:4.2f} | ω: {omega:5.2f} | "
                  f"X: {self.state.x:5.2f} Y: {self.state.y:5.2f}  ", end="")
            
            time.sleep(0.02)  # 50Hz
            
        print("\n❌ Timeout - failed to reach goal")
        return False
    
    def deliver(self, waypoints):
        """Execute delivery mission"""
        print(f"\n{'='*50}")
        print(f"Starting delivery mission with {len(waypoints)} stops")
        print(f"{'='*50}")
        
        for i, (x, y) in enumerate(waypoints):
            print(f"\n📍 Stop {i+1}/{len(waypoints)}: ({x:.2f}, {y:.2f})")
            
            success = self.goto(x, y)
            if not success:
                print("Mission aborted!")
                return False
            
            # Simulate delivery
            if i < len(waypoints) - 1:  # Don't wait at final stop
                print(f"  📦 Delivering package...")
                time.sleep(2)
                
        print(f"\n{'='*50}")
        print("✅ Mission complete!")
        print(f"{'='*50}")
        return True


# ============================================
# MAIN
# ============================================
if __name__ == "__main__":
    print("Prime-Delivery Robot Controller")
    print("=" * 40)
    
    # Auto-detect USB port
    possible_ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
    
    if not possible_ports:
        print("❌ No USB serial devices found!")
        print("Check if ESP32 is connected:")
        print("  $ ls /dev/tty*")
        sys.exit(1)
    
    # Use first available port
    port = possible_ports[0]
    print(f"Found ESP32 on: {port}")
    
    # Connect to ESP32
    esp32 = ESP32Interface(port=port, baudrate=115200)
    esp32.connect()
    esp32.start_reading()
    
    # Create robot controller
    robot = DeliveryRobot(esp32)
    
    try:
        # Give telemetry time to start flowing
        print("\nWaiting for telemetry...")
        time.sleep(0.5)
        
        # Quick motor test
        print("\n🔧 Motor test - moving forward slowly for 1 second...")
        esp32.send_velocity(0.15, 0.0)  # 0.15 m/s straight
        time.sleep(1.0)
        esp32.send_velocity(0.0, 0.0)
        print("Motor test complete")
        
        time.sleep(0.5)
        
        # Execute delivery mission
        waypoints = [
            (1.0, 0.0),   # First delivery - 1m forward
            (1.0, 1.0),   # Second delivery - 1m left
            (0.0, 0.0),   # Return home
        ]
        
        robot.deliver(waypoints)
        
    except KeyboardInterrupt:
        print("\n\n⏹️  Interrupted by user!")
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\nShutting down...")
        esp32.stop()
        print("Done.")