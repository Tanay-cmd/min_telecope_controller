import os
import sys
import time
import struct
import threading

# Joystick Constants
DEADZONE = 4000      # Deadzone threshold (range: 0 to 32767)
MAX_RA_RATE = 0.05   # Max Right Ascension change rate (hours/sec)
MAX_DEC_RATE = 0.5   # Max Declination change rate (degrees/sec)
MAX_STEP_RATE = 1000  # Max manual Alt/Az step pulses rate (steps/sec)
LOOP_INTERVAL = 0.1  # Update loop period (seconds)

def start_joystick_thread(controller):
    """Start background threads to read joystick inputs and update the telescope controller."""
    js_path = "/dev/input/js0"
    if not os.path.exists(js_path):
        print("[INFO] Joystick device '/dev/input/js0' not found. Joystick control disabled.")
        return False
        
    def worker():
        try:
            js_dev = open(js_path, "rb")
        except Exception as e:
            print(f"[WARNING] Could not open joystick device {js_path}: {e}")
            return
            
        print(f"[SUCCESS] Joystick found and active at {js_path}")
        print("Default Mappings: Left Stick = RA/Dec nudge | Right Stick = Alt/Az step nudge")
        print("                  Button A = Start Track | Button B = Set Pos (0, 90) | Button X = Emergency Halt | Button Y = Toggle Calibrate")
        
        axis_states = {}
        button_states = {}
        
        # 1. Event Reader Thread
        def event_reader():
            while True:
                try:
                    evbuf = js_dev.read(8)
                    if not evbuf:
                        break
                    time_ms, value, ev_type, number = struct.unpack("IhBB", evbuf)
                    
                    # Strip initialization flag
                    ev_type_actual = ev_type & ~0x80
                    
                    if ev_type_actual == 1:    # Button press/release
                        button_states[number] = value
                        if value == 1:         # On Button Press
                            print(f"[JOYSTICK] Button {number} Pressed")
                            if number == 0:    # Typically A
                                controller.start_tracking()
                            elif number == 1:  # Typically B
                                controller.set_position("0.0", "90.0")
                            elif number == 2:  # Typically X
                                with controller.lock:
                                    controller.send_command("HALT")
                                    print("[JOYSTICK] Sent EMERGENCY HALT to mount.")
                            elif number == 3:  # Typically Y
                                controller.set_calibration_mode(not controller.calibration_mode)
                                    
                    elif ev_type_actual == 2:  # Axis movement
                        axis_states[number] = value
                        
                except Exception:
                    break
            js_dev.close()
            print("[INFO] Joystick reader thread terminated.")
            
        threading.Thread(target=event_reader, daemon=True).start()
        
        # 2. Control Application Loop
        while True:
            # Check if connection was closed
            if not controller.ser and controller.serial_port != "MOCK":
                break
                
            try:
                # Read stick inputs (Left Stick: Axes 0, 1 | Right Stick: Axes 3, 4)
                left_x = axis_states.get(0, 0)
                left_y = axis_states.get(1, 0)
                right_x = axis_states.get(3, 0)
                right_y = axis_states.get(4, 0)
                
                # --- Handle RA/Dec target coordinate adjustments ---
                d_ra = 0.0
                d_dec = 0.0
                
                if abs(left_x) > DEADZONE:
                    d_ra = (left_x / 32768.0) * MAX_RA_RATE * LOOP_INTERVAL
                if abs(left_y) > DEADZONE:
                    # Invert Y so pushing stick up/forward increases Declination
                    d_dec = (left_y / -32768.0) * MAX_DEC_RATE * LOOP_INTERVAL
                    
                if d_ra != 0.0 or d_dec != 0.0:
                    controller.nudge_ra_dec(d_ra, d_dec)
                    
                # --- Handle direct Alt/Az motor step adjustments ---
                d_az = 0
                d_alt = 0
                
                if abs(right_x) > DEADZONE:
                    d_az = int((right_x / 32768.0) * MAX_STEP_RATE * LOOP_INTERVAL)
                if abs(right_y) > DEADZONE:
                    # Invert Y so pushing stick up/forward moves Altitude upward
                    d_alt = int((right_y / -32768.0) * MAX_STEP_RATE * LOOP_INTERVAL)
                    
                if d_az != 0 or d_alt != 0:
                    controller.nudge_steps(d_az, d_alt)
                    
            except Exception as e:
                print(f"[ERROR] Joystick control loop: {e}")
                
            time.sleep(LOOP_INTERVAL)
            
    threading.Thread(target=worker, daemon=True).start()
    return True
