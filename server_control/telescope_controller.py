#!/usr/bin/env python3
import os
import sys

# Ensure parent directory of server_control is in sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import math
import time
import datetime
import threading
from server_control.simbad_resolver import resolve_target_via_simbad

try:
    import serial
except ImportError:
    print("Error: 'pyserial' is not installed. Please run: pip install pyserial")
    sys.exit(1)

try:
    from astropy.coordinates import EarthLocation, SkyCoord, AltAz
    from astropy.time import Time
    import astropy.units as u
except ImportError:
    print("Error: 'astropy' is not installed. Please run: pip install astropy")
    sys.exit(1)


DEFAULT_LATITUDE = 24.0 + 39.0/60.0 + 11.15/3600.0  # 24 39 11.15 North
DEFAULT_LONGITUDE = 72.0 + 46.0/60.0 + 47.43/3600.0 # 72 46 47.43 East
DEFAULT_ELEVATION = 1680.0                           # Mount Abu elevation (meters)

# Hardware configuration
STEPS_PER_REV = 2000  # DIP switches set on DM542
AZ_GEAR_RATIO = -1.0  # Adjust if motor drives a gear/belt system (negative to reverse direction)
ALT_GEAR_RATIO = 1.0  # Adjust if motor drives a gear/belt system

# Calibration Reference Position default (physical pointing at startup, step 0, 0)
DEFAULT_REF_ALT = 90.0  # Zenith
DEFAULT_REF_AZ = 0.0   # North

# Safety travel limits (in absolute degrees)
# When calibration_mode is False, the telescope is restricted to these physical ranges:
AZ_LIMIT_MIN_DEG = -90.0   # -90 degrees relative to North (West)
AZ_LIMIT_MAX_DEG = 90.0    # +90 degrees relative to North (East)
ALT_LIMIT_MIN_DEG = 5.0    # 5 degrees above the horizon
ALT_LIMIT_MAX_DEG = 175.0  # 5 degrees above the horizon on the opposite (flipped) side

def calculate_julian_date(dt: datetime.datetime) -> float:
    """Calculate Julian Date for a given UTC datetime."""
    t = Time(dt, scale='utc')
    return float(t.jd)

def calculate_lst(longitude: float, dt: datetime.datetime) -> float:
    """Calculate Local Sidereal Time (LST) in hours for a longitude (East positive)."""
    t = Time(dt, scale='utc')
    return float(t.sidereal_time('mean', longitude=longitude*u.deg).hour)

def ra_dec_to_alt_az(ra: float, dec: float, lat: float, lon: float, elevation: float, dt: datetime.datetime):
    """
    Convert Right Ascension (hours) and Declination (degrees) to Altitude and Azimuth (degrees).
    """
    loc = EarthLocation(lat=lat*u.deg, lon=lon*u.deg, height=elevation*u.m)
    t = Time(dt, scale='utc')
    coord = SkyCoord(ra=ra*u.hourangle, dec=dec*u.deg, frame='icrs')
    altaz_frame = AltAz(location=loc, obstime=t)
    altaz = coord.transform_to(altaz_frame)
    lst = t.sidereal_time('mean', longitude=lon*u.deg).hour
    return float(altaz.alt.deg), float(altaz.az.deg), float(lst)

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

def map_target_to_physical_limits(az_deg: float, alt_deg: float):
    """
    Map target Azimuth and Altitude coordinates to physical mount limits.
    Since Azimuth travel is limited to [-90, +90] relative to North (0), 
    any target falling outside this range is mapped to the opposite side 
    by tilting Altitude over-the-top (past 90 degrees).
    """
    # 1. Normalize Azimuth relative to North (0) in the range [-180, 180]
    az_rel = (az_deg + 180.0) % 360.0 - 180.0
    
    # 2. Check if we need to flip the mount
    if az_rel > 90.0:
        # Southeastern quadrant: flip Azimuth to Northwest, Altitude past Zenith
        phys_az = az_rel - 180.0
        phys_alt = 180.0 - alt_deg
    elif az_rel < -90.0:
        # Southwestern quadrant: flip Azimuth to Northeast, Altitude past Zenith
        phys_az = az_rel + 180.0
        phys_alt = 180.0 - alt_deg
    else:
        # Northern sky (within limit): use directly
        phys_az = az_rel
        phys_alt = alt_deg
        
    return phys_az, phys_alt

def deg_to_steps_relative(deg_target: float, deg_ref: float, gear_ratio: float, is_azimuth: bool = False) -> int:
    """Convert a target angle to motor steps relative to the calibration reference point."""
    delta = deg_target - deg_ref
    if is_azimuth:
        # Normalize azimuth delta to shortest path [-180, 180]
        delta = (delta + 180.0) % 360.0 - 180.0
    return round(delta * (STEPS_PER_REV / 360.0) * gear_ratio)

def steps_to_deg(steps: float, deg_ref: float, gear_ratio: float, is_azimuth: bool = False) -> float:
    """Convert motor step position to absolute physical degrees."""
    deg = deg_ref + (steps / ((STEPS_PER_REV / 360.0) * gear_ratio))
    if is_azimuth:
        # Normalize to range [-180, 180] relative to North (0)
        deg = (deg + 180.0) % 360.0 - 180.0
    return deg


class TelescopeController:
    def __init__(self, port, baud=9600, lat=DEFAULT_LATITUDE, lon=DEFAULT_LONGITUDE, elevation=DEFAULT_ELEVATION):
        self.lat = lat
        self.lon = lon
        self.elevation = elevation
        self.serial_port = port
        self.baud = baud
        self.ser = None
        self.tracking = False
        self.target_ra = None
        self.target_dec = None
        self.thread = None
        self.lock = threading.RLock()
        self.current_az_steps = 0
        self.current_alt_steps = 0
        self.target_az_steps = 0
        self.target_alt_steps = 0
        self.is_parked = False
        self.calibration_mode = False  # Safety limits enforced initially
        self.ref_az = DEFAULT_REF_AZ
        self.ref_alt = DEFAULT_REF_ALT

    def set_calibration_mode(self, enabled: bool):
        """Thread-safely toggle calibration mode."""
        with self.lock:
            self.calibration_mode = enabled
            state_str = "ENABLED (Safety limits BYPASSED)" if enabled else "DISABLED (Safety limits ENFORCED)"
            print(f"\n[CALIBRATION] Calibration mode is now {state_str}\nControl CLI> ", end="", flush=True)

    def connect(self):
        if self.serial_port == "MOCK":
            print("[INFO] Running in MOCK Serial Mode.")
            try:
                import server_control.joystick_control as joystick_control
                joystick_control.start_joystick_thread(self)
            except ImportError:
                print("[WARNING] joystick_control.py module not found. Joystick control disabled.")
            return True
        try:
            self.ser = serial.Serial(self.serial_port, self.baud, timeout=1.0)
            time.sleep(2.0)  # Wait for Arduino reboot
            # Clear input buffer
            self.ser.reset_input_buffer()
            print(f"[SUCCESS] Connected to Arduino on {self.serial_port}")
            # Start telemetry reading thread
            threading.Thread(target=self._read_telemetry, daemon=True).start()
            # Request stored reference coordinates from Arduino
            time.sleep(0.5)
            self.send_command("GET_REF")
            # Auto-align: slew back to calibrated Home position (0, 0) on boot
            time.sleep(0.1)
            print("[BOOT] Auto-aligning: Slewing telescope back to calibrated Home position (0, 0)...")
            self.send_command("MOVE 0 0")
            try:
                import server_control.joystick_control as joystick_control
                joystick_control.start_joystick_thread(self)
            except ImportError:
                print("[WARNING] joystick_control.py module not found. Joystick control disabled.")
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
                        elif line.startswith("REF"):
                            parts = line.split()
                            if len(parts) >= 3:
                                try:
                                    self.ref_az = float(parts[1])
                                    self.ref_alt = float(parts[2])
                                    print(f"\n[INFO] Loaded reference coordinates from Arduino: Az={self.ref_az}°, Alt={self.ref_alt}°\nControl CLI> ", end="", flush=True)
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
                alt, az, _ = ra_dec_to_alt_az(self.target_ra, self.target_dec, self.lat, self.lon, self.elevation, now)
                
                # Map coordinates to safety limits (flip if needed)
                phys_az, phys_alt = map_target_to_physical_limits(az, alt)
                
                az_steps = deg_to_steps_relative(phys_az, self.ref_az, AZ_GEAR_RATIO, is_azimuth=True)
                alt_steps = deg_to_steps_relative(phys_alt, self.ref_alt, ALT_GEAR_RATIO, is_azimuth=False)
                
                # Enforce safety limits if NOT in calibration mode
                if not self.calibration_mode:
                    if phys_az < AZ_LIMIT_MIN_DEG or phys_az > AZ_LIMIT_MAX_DEG or phys_alt < ALT_LIMIT_MIN_DEG or phys_alt > ALT_LIMIT_MAX_DEG:
                        print(f"[ERROR] Target coordinates (Az: {phys_az:.2f}°, Alt: {phys_alt:.2f}°) exceed physical safety limits (Az: {AZ_LIMIT_MIN_DEG}° to {AZ_LIMIT_MAX_DEG}°, Alt: {ALT_LIMIT_MIN_DEG}° to {ALT_LIMIT_MAX_DEG}°).")
                        return False
                
                self.target_az_steps = az_steps
                self.target_alt_steps = alt_steps
                
                self.send_command(f"MOVE {az_steps} {alt_steps}")
                print(f"[GOTO] Slewing immediately to Alt: {phys_alt:.2f}° (Steps: {alt_steps}), Az: {phys_az:.2f}° (Steps: {az_steps}) (Mapped from Target Az:{az:.2f}° Alt:{alt:.2f}°)")
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
                
                # Map coordinates to safety limits (flip if needed)
                phys_az, phys_alt = map_target_to_physical_limits(az, alt)
                
                az_steps = deg_to_steps_relative(phys_az, self.ref_az, AZ_GEAR_RATIO, is_azimuth=True)
                alt_steps = deg_to_steps_relative(phys_alt, self.ref_alt, ALT_GEAR_RATIO, is_azimuth=False)
                
                # Enforce safety limits if NOT in calibration mode
                if not self.calibration_mode:
                    if phys_az < AZ_LIMIT_MIN_DEG or phys_az > AZ_LIMIT_MAX_DEG or phys_alt < ALT_LIMIT_MIN_DEG or phys_alt > ALT_LIMIT_MAX_DEG:
                        print(f"[ERROR] Target coordinates (Az: {phys_az:.2f}°, Alt: {phys_alt:.2f}°) exceed physical safety limits (Az: {AZ_LIMIT_MIN_DEG}° to {AZ_LIMIT_MAX_DEG}°, Alt: {ALT_LIMIT_MIN_DEG}° to {ALT_LIMIT_MAX_DEG}°).")
                        return False
                
                self.target_az_steps = az_steps
                self.target_alt_steps = alt_steps
                
                self.send_command(f"MOVE {az_steps} {alt_steps}")
                print(f"[GOTO] Slewing immediately to manual Alt: {phys_alt:.2f}° (Steps: {alt_steps}), Az: {phys_az:.2f}° (Steps: {az_steps}) (Mapped from Target Az:{az:.2f}° Alt:{alt:.2f}°)")
                return True
            except ValueError:
                print("[ERROR] Invalid Alt/Az value. Must be decimal degrees.")
                return False

    def set_position(self, az_str: str, alt_str: str):
        """Calibrate the telescope reference coordinates to the given Alt-Az in degrees at the current physical position."""
        with self.lock:
            try:
                az = float(az_str)
                alt = float(alt_str)
                
                # Update reference coordinates
                self.ref_az = az
                self.ref_alt = alt
                
                # Reset current step counts to 0, 0 since this physical position is the new reference
                self.current_az_steps = 0
                self.current_alt_steps = 0
                self.target_az_steps = 0
                self.target_alt_steps = 0
                
                # Send update commands to Arduino
                self.send_command(f"SET_REF {az} {alt}")
                self.send_command("SET 0 0")
                
                print(f"[CALIBRATE] Set new Home reference position to Alt: {alt:.2f}°, Az: {az:.2f}° (Saved to EEPROM)")
                self.set_calibration_mode(False)
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
                
                # Enforce safety limits if NOT in calibration mode
                if not self.calibration_mode:
                    az_deg = steps_to_deg(az_steps, self.ref_az, AZ_GEAR_RATIO, is_azimuth=True)
                    alt_deg = steps_to_deg(alt_steps, self.ref_alt, ALT_GEAR_RATIO, is_azimuth=False)
                    if az_deg < AZ_LIMIT_MIN_DEG or az_deg > AZ_LIMIT_MAX_DEG or alt_deg < ALT_LIMIT_MIN_DEG or alt_deg > ALT_LIMIT_MAX_DEG:
                        print(f"[ERROR] Target steps correspond to Az: {az_deg:.2f}°, Alt: {alt_deg:.2f}° which exceed physical safety limits (Az: {AZ_LIMIT_MIN_DEG}° to {AZ_LIMIT_MAX_DEG}°, Alt: {ALT_LIMIT_MIN_DEG}° to {ALT_LIMIT_MAX_DEG}°).")
                        return False
                
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

    def nudge_steps(self, d_az_str: str, d_alt_str: str):
        """Nudge the current motor target positions by a step delta."""
        with self.lock:
            try:
                d_az = int(d_az_str)
                d_alt = int(d_alt_str)
                
                new_az = self.target_az_steps + d_az
                new_alt = self.target_alt_steps + d_alt
                
                # Enforce safety limits if NOT in calibration mode
                if not self.calibration_mode:
                    new_az_deg = steps_to_deg(new_az, self.ref_az, AZ_GEAR_RATIO, is_azimuth=True)
                    new_alt_deg = steps_to_deg(new_alt, self.ref_alt, ALT_GEAR_RATIO, is_azimuth=False)
                    
                    if new_az_deg < AZ_LIMIT_MIN_DEG or new_az_deg > AZ_LIMIT_MAX_DEG:
                        clamped_az_deg = max(AZ_LIMIT_MIN_DEG, min(AZ_LIMIT_MAX_DEG, new_az_deg))
                        new_az = deg_to_steps_relative(clamped_az_deg, self.ref_az, AZ_GEAR_RATIO, is_azimuth=True)
                        print(f"\n[WARNING] Azimuth safety limit reached ({clamped_az_deg:.1f}°). Clamping movement to {new_az} steps.\nControl CLI> ", end="", flush=True)
                        
                    if new_alt_deg < ALT_LIMIT_MIN_DEG or new_alt_deg > ALT_LIMIT_MAX_DEG:
                        clamped_alt_deg = max(ALT_LIMIT_MIN_DEG, min(ALT_LIMIT_MAX_DEG, new_alt_deg))
                        new_alt = deg_to_steps_relative(clamped_alt_deg, self.ref_alt, ALT_GEAR_RATIO, is_azimuth=False)
                        print(f"\n[WARNING] Altitude safety limit reached ({clamped_alt_deg:.1f}°). Clamping movement to {new_alt} steps.\nControl CLI> ", end="", flush=True)
                
                self.target_az_steps = new_az
                self.target_alt_steps = new_alt
                
                self.send_command(f"MOVE {self.target_az_steps} {self.target_alt_steps}")
                print(f"[NUDGE STEPS] Nudged target by Az:{d_az} Alt:{d_alt} steps. New targets: Az:{self.target_az_steps} Alt:{self.target_alt_steps}")
                return True
            except ValueError:
                print("[ERROR] Nudge step offsets must be integers.")
                return False

    def nudge_ra_dec(self, d_ra_str: str, d_dec_str: str):
        """Nudge the tracking target RA and Dec coordinates by a delta (RA in hours, Dec in degrees)."""
        with self.lock:
            try:
                d_ra = float(d_ra_str)
                d_dec = float(d_dec_str)
                
                if self.target_ra is None or self.target_dec is None:
                    print("[ERROR] No target active to nudge. Set a target first using 'target RA Dec'.")
                    return False
                
                self.target_ra = (self.target_ra + d_ra) % 24.0
                self.target_dec = max(-90.0, min(90.0, self.target_dec + d_dec))
                
                print(f"[NUDGE RA/DEC] Nudged target by RA:{d_ra:.6f}h Dec:{d_dec:.4f}°. New target: RA:{self.target_ra:.4f}h Dec:{self.target_dec:.4f}°")
                return True
            except ValueError:
                print("[ERROR] Nudge RA/Dec offsets must be floats.")
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

    def park_and_shutdown(self):
        """Park the telescope at the calibrated Home position (0 steps, 0 steps - which is REF_AZ, REF_ALT) and wait until slew is complete before exiting."""
        with self.lock:
            if self.is_parked:
                return
            self.is_parked = True
            self.tracking = False
            self.target_ra = None
            self.target_dec = None
            
        print(f"\n[PARK] Returning telescope to Calibrated Home position (Az: {self.ref_az:.2f}°, Alt: {self.ref_alt:.2f}°)...")
        self.send_command("MOVE 0 0")
        
        # Wait until current steps match 0, 0
        timeout = 30.0  # seconds
        start_time = time.time()
        
        if self.serial_port == "MOCK":
            print("[PARK] Mock park completed.")
            return

        while time.time() - start_time < timeout:
            with self.lock:
                current_az = self.current_az_steps
                current_alt = self.current_alt_steps
            
            # Print a progress indicator
            print(f"[PARK] Current steps: Az={current_az}, Alt={current_alt} | Target: 0, 0", end="\r", flush=True)
            
            if current_az == 0 and current_alt == 0:
                print("\n[PARK] Telescope successfully parked at Home!")
                break
            
            time.sleep(0.5)
        else:
            print("\n[WARNING] Park timeout reached. Telescope may not be fully returned to Home.")
        
        # Halt motors to disable power/forces and stop serial connection
        self.send_command("HALT")
        time.sleep(0.5)
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[INFO] Serial connection closed.")

    def _tracking_loop(self):
        while True:
            with self.lock:
                if not self.tracking:
                    break
                ra = self.target_ra
                dec = self.target_dec
                
            now = datetime.datetime.now(datetime.timezone.utc)
            alt, az, lst = ra_dec_to_alt_az(ra, dec, self.lat, self.lon, self.elevation, now)
            
            # Map coordinates to safety limits (flip if needed)
            phys_az, phys_alt = map_target_to_physical_limits(az, alt)
            
            az_steps = deg_to_steps_relative(phys_az, self.ref_az, AZ_GEAR_RATIO, is_azimuth=True)
            alt_steps = deg_to_steps_relative(phys_alt, self.ref_alt, ALT_GEAR_RATIO, is_azimuth=False)
            
            # Enforce safety limits if NOT in calibration mode
            if not self.calibration_mode:
                if phys_az < AZ_LIMIT_MIN_DEG or phys_az > AZ_LIMIT_MAX_DEG or phys_alt < ALT_LIMIT_MIN_DEG or phys_alt > ALT_LIMIT_MAX_DEG:
                    print(f"\n[WARNING] Tracking halted: Target coordinates (Az:{phys_az:.2f}°, Alt:{phys_alt:.2f}°) exceed safety boundaries.\nControl CLI> ", end="", flush=True)
                    with self.lock:
                        self.tracking = False
                        self.send_command("HALT")
                    break
            
            with self.lock:
                self.target_az_steps = az_steps
                self.target_alt_steps = alt_steps
                
            # Send movement target to Arduino
            self.send_command(f"MOVE {az_steps} {alt_steps}")
            
            # Update every 2 seconds
            time.sleep(2.0)
