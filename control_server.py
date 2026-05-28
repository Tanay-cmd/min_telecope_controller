#!/usr/bin/env python3
import sys
import math
import time
import argparse
import datetime
import threading
try:
    import serial
except ImportError:
    print("Error: 'pyserial' is not installed. Please run: pip install pyserial")
    sys.exit(1)


DEFAULT_LATITUDE = 24.16  # Degrees North
DEFAULT_LONGITUDE = 72.78 # Degrees East

# Hardware configuration
STEPS_PER_REV = 2000  # DIP switches set on DM542
AZ_GEAR_RATIO = 1.0   # Adjust if motor drives a gear/belt system
ALT_GEAR_RATIO = 1.0  # Adjust if motor drives a gear/belt system

# Calibration Reference Position (physical pointing at startup, step 0, 0)
REF_ALT = 90.0  # Zenith
REF_AZ = 0  # South

def calculate_julian_date(dt: datetime.datetime) -> float:
    """Calculate Julian Date for a given UTC datetime."""
    year = dt.year
    month = dt.month
    day = dt.day + (dt.hour + dt.minute / 60.0 + dt.second / 3600.0 + dt.microsecond / 3600000000.0) / 24.0
    
    if month <= 2:
        year -= 1
        month += 12
        
    A = int(year / 100)
    B = 2 - A + int(A / 4)
    
    jd = int(365.25 * (year + 4716)) + int(30.6001 * (month + 1)) + day + B - 1524.5
    return jd

def calculate_lst(longitude: float, dt: datetime.datetime) -> float:
    """Calculate Local Sidereal Time (LST) in hours for a longitude (East positive)."""
    jd = calculate_julian_date(dt)
    d = jd - 2451545.0
    # Greenwich Mean Sidereal Time (GMST) in hours
    gmst = (18.697374558 + 24.06570982441908 * d) % 24.0
    # Local Sidereal Time (LST) in hours
    lst = (gmst + longitude / 15.0) % 24.0
    return lst

def ra_dec_to_alt_az(ra: float, dec: float, lat: float, lon: float, dt: datetime.datetime):
    """
    Convert Right Ascension (hours) and Declination (degrees) to Altitude and Azimuth (degrees).
    Uses Local Sidereal Time to calculate Hour Angle.
    """
    lst = calculate_lst(lon, dt)
    ha = (lst - ra) % 24.0
    ha_deg = ha * 15.0
    
    ha_rad = math.radians(ha_deg)
    dec_rad = math.radians(dec)
    lat_rad = math.radians(lat)
    
    # Calculate Altitude
    sin_alt = math.sin(dec_rad) * math.sin(lat_rad) + math.cos(dec_rad) * math.cos(lat_rad) * math.cos(ha_rad)
    sin_alt = max(-1.0, min(1.0, sin_alt))  # Clamp to prevent domain errors
    alt_rad = math.asin(sin_alt)
    alt = math.degrees(alt_rad)
    
    # Calculate Azimuth
    y = -math.sin(ha_rad) * math.cos(dec_rad)
    x = math.sin(dec_rad) * math.cos(lat_rad) - math.cos(dec_rad) * math.sin(lat_rad) * math.cos(ha_rad)
    az_rad = math.atan2(y, x)
    az = math.degrees(az_rad) % 360.0
    
    return alt, az, lst

def parse_ra(ra_str: str) -> float:
    """Parse RA string (HH:MM:SS or decimal hours) to float hours."""
    try:
        if ":" in ra_str:
            parts = [float(x) for x in ra_str.split(":")]
            h = parts[0]
            m = parts[1] if len(parts) > 1 else 0.0
            s = parts[2] if len(parts) > 2 else 0.0
            return h + m/60.0 + s/3600.0
        return float(ra_str)
    except ValueError:
        raise ValueError(f"Invalid RA format: {ra_str}. Use HH:MM:SS or decimal hours.")

def parse_dec(dec_str: str) -> float:
    """Parse Dec string (DD:MM:SS or decimal degrees) to float degrees."""
    try:
        if ":" in dec_str:
            parts = dec_str.split(":")
            # Handle negative sign correctly
            sign = -1.0 if parts[0].strip().startswith("-") else 1.0
            d = abs(float(parts[0]))
            m = float(parts[1]) if len(parts) > 1 else 0.0
            s = float(parts[2]) if len(parts) > 2 else 0.0
            return sign * (d + m/60.0 + s/3600.0)
        return float(dec_str)
    except ValueError:
        raise ValueError(f"Invalid Dec format: {dec_str}. Use DD:MM:SS or decimal degrees.")

def deg_to_steps_relative(deg_target: float, deg_ref: float, gear_ratio: float, is_azimuth: bool = False) -> int:
    """Convert a target angle to motor steps relative to the calibration reference point."""
    delta = deg_target - deg_ref
    if is_azimuth:
        # Normalize azimuth delta to shortest path [-180, 180]
        delta = (delta + 180.0) % 360.0 - 180.0
    return round(delta * (STEPS_PER_REV / 360.0) * gear_ratio)

class TelescopeController:
    def __init__(self, port, baud=9600, lat=DEFAULT_LATITUDE, lon=DEFAULT_LONGITUDE):
        self.lat = lat
        self.lon = lon
        self.serial_port = port
        self.baud = baud
        self.ser = None
        self.tracking = False
        self.target_ra = None
        self.target_dec = None
        self.thread = None
        self.lock = threading.Lock()
        self.current_az_steps = 0
        self.current_alt_steps = 0
        self.target_az_steps = 0
        self.target_alt_steps = 0

    def connect(self):
        if self.serial_port == "MOCK":
            print("[INFO] Running in MOCK Serial Mode.")
            return True
        try:
            self.ser = serial.Serial(self.serial_port, self.baud, timeout=1.0)
            time.sleep(2.0)  # Wait for Arduino reboot
            # Clear input buffer
            self.ser.reset_input_buffer()
            print(f"[SUCCESS] Connected to Arduino on {self.serial_port}")
            # Start telemetry reading thread
            threading.Thread(target=self._read_telemetry, daemon=True).start()
            return True
        except Exception as e:
            print(f"[ERROR] Failed to connect to serial port {self.serial_port}: {e}")
            return False

    def send_command(self, cmd: str):
        if self.ser and self.ser.is_open:
            self.ser.write(f"{cmd}\n".encode('ascii'))
        else:
            print(f"[MOCK SEND] {cmd.strip()}")

    def _read_telemetry(self):
        while self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('ascii', errors='ignore').strip()
                    if line:
                        if line.startswith("TELEMETRY"):
                            parts = line.split()
                            if len(parts) >= 3:
                                try:
                                    self.current_az_steps = int(parts[1])
                                    self.current_alt_steps = int(parts[2])
                                except ValueError:
                                    pass
                        else:
                            # Print confirmation ACKs or other messages cleanly, re-printing the prompt
                            print(f"\n[Arduino] {line}\nControl CLI> ", end="", flush=True)
            except Exception as e:
                print(f"\n[ERROR] Reading serial telemetry: {e}\nControl CLI> ", end="", flush=True)
                break
            time.sleep(0.1)

    def set_target(self, ra_str: str, dec_str: str):
        with self.lock:
            try:
                self.target_ra = parse_ra(ra_str)
                self.target_dec = parse_dec(dec_str)
                print(f"[INFO] New Target Set: RA={self.target_ra:.4f}h, Dec={self.target_dec:.4f}°")
                
                # Perform an immediate GOTO slew command
                now = datetime.datetime.now(datetime.timezone.utc)
                alt, az, _ = ra_dec_to_alt_az(self.target_ra, self.target_dec, self.lat, self.lon, now)
                
                az_steps = deg_to_steps_relative(az, REF_AZ, AZ_GEAR_RATIO, is_azimuth=True)
                alt_steps = deg_to_steps_relative(alt, REF_ALT, ALT_GEAR_RATIO, is_azimuth=False)
                
                self.target_az_steps = az_steps
                self.target_alt_steps = alt_steps
                
                self.send_command(f"MOVE {az_steps} {alt_steps}")
                print(f"[GOTO] Slewing immediately to Alt: {alt:.2f}° (Steps: {alt_steps}), Az: {az:.2f}° (Steps: {az_steps})")
                return True
            except ValueError as e:
                print(f"[ERROR] {e}")
                return False

    def slew_to_alt_az(self, az_str: str, alt_str: str):
        """Directly slew to specific Alt-Az coordinates in degrees."""
        with self.lock:
            try:
                az = float(az_str)
                alt = float(alt_str)
                
                # Suspend tracking if active to allow manual slew
                if self.tracking:
                    self.tracking = False
                    print("[INFO] Tracking suspended for manual Alt-Az slew.")
                
                az_steps = deg_to_steps_relative(az, REF_AZ, AZ_GEAR_RATIO, is_azimuth=True)
                alt_steps = deg_to_steps_relative(alt, REF_ALT, ALT_GEAR_RATIO, is_azimuth=False)
                
                self.target_az_steps = az_steps
                self.target_alt_steps = alt_steps
                
                self.send_command(f"MOVE {az_steps} {alt_steps}")
                print(f"[GOTO] Slewing immediately to manual Alt: {alt:.2f}° (Steps: {alt_steps}), Az: {az:.2f}° (Steps: {az_steps})")
                return True
            except ValueError:
                print("[ERROR] Invalid Alt/Az value. Must be decimal degrees.")
                return False

    def set_position(self, az_str: str, alt_str: str):
        """Calibrate the current position of the motors to the given Alt-Az coordinates in degrees without moving."""
        with self.lock:
            try:
                az = float(az_str)
                alt = float(alt_str)
                
                # Compute what steps correspond to these degrees
                az_steps = deg_to_steps_relative(az, REF_AZ, AZ_GEAR_RATIO, is_azimuth=True)
                alt_steps = deg_to_steps_relative(alt, REF_ALT, ALT_GEAR_RATIO, is_azimuth=False)
                
                # Update current steps cache
                self.current_az_steps = az_steps
                self.current_alt_steps = alt_steps
                self.target_az_steps = az_steps
                self.target_alt_steps = alt_steps
                
                # Send calibration command to Arduino
                self.send_command(f"SET {az_steps} {alt_steps}")
                print(f"[CALIBRATE] Calibrated current telescope position to Alt: {alt:.2f}° (Steps: {alt_steps}), Az: {az:.2f}° (Steps: {az_steps})")
                return True
            except ValueError:
                print("[ERROR] Invalid Alt/Az value. Must be decimal degrees.")
                return False

    def send_raw_pulses(self, az_steps_str: str, alt_steps_str: str):
        """Directly command the Arduino with raw step/pulse numbers."""
        with self.lock:
            try:
                az_steps = int(az_steps_str)
                alt_steps = int(alt_steps_str)
                
                # Suspend tracking if active to allow manual step movement
                if self.tracking:
                    self.tracking = False
                    print("[INFO] Tracking suspended for manual pulse command.")
                
                self.target_az_steps = az_steps
                self.target_alt_steps = alt_steps
                
                self.send_command(f"MOVE {az_steps} {alt_steps}")
                print(f"[PULSE] Sending raw steps directly -> Az: {az_steps}, Alt: {alt_steps}")
                return True
            except ValueError:
                print("[ERROR] Step values must be integers.")
                return False

    def start_tracking(self):
        with self.lock:
            if self.target_ra is None or self.target_dec is None:
                print("[ERROR] Set target RA/Dec before tracking.")
                return
            if self.tracking:
                return
            self.tracking = True
            self.thread = threading.Thread(target=self._tracking_loop, daemon=True)
            self.thread.start()
            print("[SUCCESS] Tracking started.")

    def stop_tracking(self):
        with self.lock:
            if not self.tracking:
                return
            self.tracking = False
            self.send_command("HALT")
            print("[INFO] Tracking stopped.")

    def _tracking_loop(self):
        while True:
            with self.lock:
                if not self.tracking:
                    break
                ra = self.target_ra
                dec = self.target_dec
                
            now = datetime.datetime.now(datetime.timezone.utc)
            alt, az, lst = ra_dec_to_alt_az(ra, dec, self.lat, self.lon, now)
            
            az_steps = deg_to_steps_relative(az, REF_AZ, AZ_GEAR_RATIO, is_azimuth=True)
            alt_steps = deg_to_steps_relative(alt, REF_ALT, ALT_GEAR_RATIO, is_azimuth=False)
            
            with self.lock:
                self.target_az_steps = az_steps
                self.target_alt_steps = alt_steps
                
            # Send movement target to Arduino
            self.send_command(f"MOVE {az_steps} {alt_steps}")
            
            # Update every 2 seconds
            time.sleep(2.0)

def main():
    parser = argparse.ArgumentParser(description="Telescope Alt-Az Mount Tracking Control Server")
    parser.add_argument("--port", type=str, default="MOCK", help="Serial port of Arduino Mega (e.g. /dev/ttyACM0). Use 'MOCK' for test run.")
    parser.add_argument("--lat", type=float, default=DEFAULT_LATITUDE, help=f"Observer latitude in degrees (default: {DEFAULT_LATITUDE})")
    parser.add_argument("--lon", type=float, default=DEFAULT_LONGITUDE, help=f"Observer longitude in degrees (default: {DEFAULT_LONGITUDE})")
    
    args = parser.parse_args()
    
    print("====================================================")
    print("   Telescope Mount Alt-Az Tracking Server Started   ")
    print(f"   Location: Lat={args.lat:.4f}°, Lon={args.lon:.4f}°")
    print("====================================================")
    
    controller = TelescopeController(port=args.port, lat=args.lat, lon=args.lon)
    if not controller.connect():
        sys.exit(1)
        
    print("\nCommands available:")
    print("  target <RA> <Dec>   - Set target coordinates and slew immediately")
    print("  goto <Az> <Alt>     - Slew directly to Alt-Az in degrees (e.g., 'goto 180 45')")
    print("  set_pos <Az> <Alt>  - Calibrate current position to Alt-Az degrees (without moving)")
    print("  pulse <Az> <Alt>    - Command raw step pulses directly (e.g., 'pulse 333 0')")
    print("  track               - Start continuous Alt-Az tracking loop")
    print("  status              - Show current targets, calculated positions, and motor telemetry")
    print("  stop                - Stop tracking and halt motors")
    print("  exit                - Close program")
    print("  lst                 - Calculate and print current LST")
    
    try:
        while True:
            cmd_line = input("\nControl CLI> ").strip()
            if not cmd_line:
                continue
            
            parts = cmd_line.split()
            cmd = parts[0].lower()
            
            if cmd == "exit":
                controller.stop_tracking()
                break
            elif cmd == "lst":
                now = datetime.datetime.now(datetime.timezone.utc)
                lst = calculate_lst(args.lon, now)
                lst_h = int(lst)
                lst_m = int((lst - lst_h) * 60)
                lst_s = int(((lst - lst_h) * 60 - lst_m) * 60)
                print(f"Current UTC: {now.strftime('%Y-%m-%d %H:%M:%S')}")
                print(f"Current LST: {lst_h:02d}:{lst_m:02d}:{lst_s:02d} ({lst:.4f} hours)")
            elif cmd == "status":
                now = datetime.datetime.now(datetime.timezone.utc)
                lst = calculate_lst(args.lon, now)
                lst_h = int(lst)
                lst_m = int((lst - lst_h) * 60)
                lst_s = int(((lst - lst_h) * 60 - lst_m) * 60)
                lst_str = f"{lst_h:02d}:{lst_m:02d}:{lst_s:02d}"
                
                print("----------------------------------------------------")
                print(f"System Time:    UTC {now.strftime('%Y-%m-%d %H:%M:%S')} | LST {lst_str}")
                with controller.lock:
                    ra = controller.target_ra
                    dec = controller.target_dec
                    tracking = controller.tracking
                
                if ra is not None and dec is not None:
                    alt, az, _ = ra_dec_to_alt_az(ra, dec, args.lat, args.lon, now)
                    print(f"Target Object:  RA {ra:.4f}h | Dec {dec:.4f}°")
                    print(f"Target Alt/Az:  Alt {alt:.2f}° | Az {az:.2f}°")
                    print(f"Target Steps:   Az {controller.target_az_steps} | Alt {controller.target_alt_steps}")
                else:
                    print("Target Object:  None Set")
                
                print(f"Motor Steps:    Az {controller.current_az_steps} | Alt {controller.current_alt_steps}")
                print(f"Tracking State: {'ACTIVE' if tracking else 'INACTIVE'}")
                print("----------------------------------------------------")
            elif cmd == "target":
                if len(parts) < 3:
                    print("Usage: target <RA> <Dec>   (e.g., target 05:35:17 -05:23:28 or target 5.588 -5.391)")
                    continue
                controller.set_target(parts[1], parts[2])
            elif cmd == "goto":
                if len(parts) < 3:
                    print("Usage: goto <Az> <Alt>   (e.g., goto 180 45)")
                    continue
                controller.slew_to_alt_az(parts[1], parts[2])
            elif cmd == "set_pos":
                if len(parts) < 3:
                    print("Usage: set_pos <Az> <Alt>   (e.g., set_pos 180 90)")
                    continue
                controller.set_position(parts[1], parts[2])
            elif cmd == "pulse":
                if len(parts) < 3:
                    print("Usage: pulse <Az_pulses> <Alt_pulses>   (e.g., pulse 333 0)")
                    continue
                controller.send_raw_pulses(parts[1], parts[2])
            elif cmd == "stop":
                controller.stop_tracking()
            else:
                print(f"Unknown command: '{cmd}'. Available: target, goto, set_pos, pulse, track, status, stop, lst, exit.")
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        controller.stop_tracking()

if __name__ == "__main__":
    main()
