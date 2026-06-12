#!/usr/bin/env python3
"""
Simple Motor Test - Just drive motors
"""

import serial
import time
import sys
import glob

def connect_esp32(port, baudrate=115200):
    """Connect to ESP32 with robust handling"""
    print(f"Opening {port} at {baudrate} baud...")
    
    ser = serial.Serial(port, baudrate, timeout=1.0)
    
    # Flush everything
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    
    print("Connected! Sending test commands...")
    print("(Press Ctrl+C to stop)\n")
    
    return ser

def send_command(ser, cmd):
    """Send a command string"""
    full_cmd = cmd + "\n"
    ser.write(full_cmd.encode())
    ser.flush()

def read_telemetry(ser, duration=0.5):
    """Read and display telemetry for duration seconds"""
    start = time.time()
    while time.time() - start < duration:
        if ser.in_waiting:
            try:
                line = ser.readline().decode('latin-1').strip()
                if line.startswith('o ') or line.startswith('DBG:') or line.startswith('CMD:') or line.startswith('SET:'):
                    print(f"  {line}")
            except:
                pass
        time.sleep(0.01)

def motor_test_sequence(ser):
    """Run a series of motor tests"""
    
    print("=" * 60)
    print("MOTOR TEST SEQUENCE")
    print("=" * 60)
    
    # Test 1: Very slow forward
    print("\n[TEST 1] Slow forward - 0.10 m/s for 3 seconds")
    print("Robot should move FORWARD slowly")
    send_command(ser, "v 0.10 0.0")
    
    for i in range(6):
        time.sleep(0.5)
        if ser.in_waiting:
            line = ser.readline().decode('latin-1').strip()
            if 'DBG:' in line or 'SET:' in line:
                print(f"  {line}")
        print(f"  ...{i+1}/6", end='\r')
    
    # Stop
    print("\n[STOP] Stopping for 1 second")
    send_command(ser, "v 0.0 0.0")
    time.sleep(1.0)
    
    # Test 2: Medium forward
    print("\n[TEST 2] Medium forward - 0.20 m/s for 3 seconds")
    print("Robot should move FORWARD faster")
    send_command(ser, "v 0.20 0.0")
    
    for i in range(6):
        time.sleep(0.5)
        if ser.in_waiting:
            line = ser.readline().decode('latin-1').strip()
            if 'DBG:' in line or 'SET:' in line:
                print(f"  {line}")
        print(f"  ...{i+1}/6", end='\r')
    
    # Stop
    print("\n[STOP] Stopping for 1 second")
    send_command(ser, "v 0.0 0.0")
    time.sleep(1.0)
    
    # Test 3: Turn left
    print("\n[TEST 3] Turn left - ω=0.5 rad/s for 2 seconds")
    print("Robot should TURN LEFT (counter-clockwise)")
    send_command(ser, "v 0.0 0.5")
    
    for i in range(4):
        time.sleep(0.5)
        if ser.in_waiting:
            line = ser.readline().decode('latin-1').strip()
            if 'DBG:' in line or 'SET:' in line:
                print(f"  {line}")
        print(f"  ...{i+1}/4", end='\r')
    
    # Stop
    print("\n[STOP] Stopping for 1 second")
    send_command(ser, "v 0.0 0.0")
    time.sleep(1.0)
    
    # Test 4: Turn right
    print("\n[TEST 4] Turn right - ω=-0.5 rad/s for 2 seconds")
    print("Robot should TURN RIGHT (clockwise)")
    send_command(ser, "v 0.0 -0.5")
    
    for i in range(4):
        time.sleep(0.5)
        if ser.in_waiting:
            line = ser.readline().decode('latin-1').strip()
            if 'DBG:' in line or 'SET:' in line:
                print(f"  {line}")
        print(f"  ...{i+1}/4", end='\r')
    
    # Stop
    print("\n[STOP] Test complete - stopping")
    send_command(ser, "s")
    
    print("\n" + "=" * 60)
    print("ALL TESTS COMPLETE")
    print("If robot didn't move, check:")
    print("  1. Motor battery connected?")
    print("  2. Motor driver enabled?")
    print("  3. Check debug output for 'PWM(L=X R=X)' - should be non-zero")
    print("=" * 60)

def interactive_mode(ser):
    """Interactive control mode"""
    print("\n" + "=" * 60)
    print("INTERACTIVE MODE")
    print("Commands:")
    print("  f = forward (0.15 m/s)")
    print("  b = backward (-0.15 m/s)")
    print("  l = turn left (0.5 rad/s)")
    print("  r = turn right (-0.5 rad/s)")
    print("  s = stop")
    print("  1-5 = set speed 0.1 to 0.5 m/s forward")
    print("  q = quit")
    print("=" * 60)
    
    while True:
        try:
            cmd = input("\nCommand: ").strip().lower()
            
            if cmd == 'q':
                break
            elif cmd == 'f':
                send_command(ser, "v 0.15 0.0")
                print("Forward 0.15 m/s")
            elif cmd == 'b':
                send_command(ser, "v -0.15 0.0")
                print("Backward 0.15 m/s")
            elif cmd == 'l':
                send_command(ser, "v 0.0 0.5")
                print("Turn left")
            elif cmd == 'r':
                send_command(ser, "v 0.0 -0.5")
                print("Turn right")
            elif cmd == 's':
                send_command(ser, "v 0.0 0.0")
                print("Stop")
            elif cmd in ['1', '2', '3', '4', '5']:
                speed = int(cmd) * 0.1
                send_command(ser, f"v {speed:.1f} 0.0")
                print(f"Forward {speed:.1f} m/s")
            else:
                print("Unknown command!")
                
            # Read any response
            time.sleep(0.1)
            while ser.in_waiting:
                try:
                    line = ser.readline().decode('latin-1').strip()
                    if line and ('DBG:' in line or 'CMD:' in line or 'SET:' in line):
                        print(f"  {line}")
                except:
                    pass
                    
        except KeyboardInterrupt:
            break
        except EOFError:
            break
    
    send_command(ser, "s")
    print("Exiting interactive mode")

if __name__ == "__main__":
    print("=" * 60)
    print("ESP32 MOTOR TEST")
    print("=" * 60)
    
    # Find port
    ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
    if not ports:
        print("❌ No ESP32 found!")
        print("Check: ls /dev/tty*")
        sys.exit(1)
    
    port = ports[0]
    print(f"Found device: {port}\n")
    
    # Connect
    ser = connect_esp32(port, 115200)
    
    try:
        # Wait a moment for ESP32 to be ready
        print("Waiting 3 seconds for ESP32 initialization...")
        time.sleep(3.0)
        
        # Flush startup messages
        ser.reset_input_buffer()
        
        # Send a test command and check response
        print("Sending test command...")
        send_command(ser, "debug")  # Toggle debug to see response
        time.sleep(0.5)
        
        # Read response
        print("ESP32 Response:")
        while ser.in_waiting:
            line = ser.readline().decode('latin-1').strip()
            if line:
                print(f"  {line}")
        
        # Ask user what they want
        print("\nOptions:")
        print("  1. Automatic test sequence")
        print("  2. Interactive control")
        print("  3. Single command test")
        
        choice = input("\nChoice (1/2/3): ").strip()
        
        if choice == '1':
            motor_test_sequence(ser)
        elif choice == '2':
            interactive_mode(ser)
        elif choice == '3':
            print("\nEnter velocity command (e.g., 'v 0.15 0.0'):")
            cmd = input("> ").strip()
            send_command(ser, cmd)
            time.sleep(2.0)
            send_command(ser, "s")
            print("Command sent, then stopped")
        else:
            print("Invalid choice, running test sequence...")
            motor_test_sequence(ser)
        
    except KeyboardInterrupt:
        print("\n\nInterrupted!")
    finally:
        print("\nStopping motors and closing connection...")
        send_command(ser, "s")
        time.sleep(0.2)
        ser.close()
        print("Done.")