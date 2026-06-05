#!/usr/bin/env python3
import os
import sys

# Ensure parent directory of server_control is in sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import time
import argparse
import datetime
from server_control.simbad_resolver import resolve_target_via_simbad
from server_control.telescope_controller import (
    TelescopeController,
    calculate_lst,
    ra_dec_to_alt_az,
    steps_to_deg,
    DEFAULT_LATITUDE,
    DEFAULT_LONGITUDE,
    DEFAULT_ELEVATION,
    AZ_GEAR_RATIO,
    ALT_GEAR_RATIO,
)


def main():
    parser = argparse.ArgumentParser(description="Telescope Alt-Az Mount Tracking Control Server")
    parser.add_argument("--port", type=str, default="MOCK", help="Serial port of Arduino Mega (e.g. /dev/ttyACM0). Use 'MOCK' for test run.")
    parser.add_argument("--lat", type=float, default=DEFAULT_LATITUDE, help=f"Observer latitude in degrees (default: {DEFAULT_LATITUDE})")
    parser.add_argument("--lon", type=float, default=DEFAULT_LONGITUDE, help=f"Observer longitude in degrees (default: {DEFAULT_LONGITUDE})")
    parser.add_argument("--elevation", type=float, default=DEFAULT_ELEVATION, help=f"Observer elevation in meters (default: {DEFAULT_ELEVATION})")
    
    args = parser.parse_args()
    
    print("====================================================")
    print("   Telescope Mount Alt-Az Tracking Server Started   ")
    print(f"   Location: Lat={args.lat:.4f}°, Lon={args.lon:.4f}°, Elev={args.elevation:.1f}m")
    print("====================================================")
    
    controller = TelescopeController(port=args.port, lat=args.lat, lon=args.lon, elevation=args.elevation)
    if not controller.connect():
        sys.exit(1)
        
    print("\nCommands available:")
    print("  target <RA> <Dec>           - Set target coordinates and slew immediately")
    print("  target resolve <ObjectName> - Resolve coordinates via Simbad and slew immediately")
    print("  goto <Az> <Alt>             - Slew directly to Alt-Az in degrees (e.g., 'goto 180 45')")
    print("  set_pos <Az> <Alt>  - Calibrate current position to Alt-Az degrees (without moving)")
    print("  pulse <Az> <Alt>    - Command raw step pulses directly (e.g., 'pulse 333 0')")
    print("  nudge_steps <X> <Y> - Nudge current motor step targets relatively")
    print("  nudge_ra_dec <H> <D>- Nudge tracking target RA hours and Dec degrees relatively")
    print("  track               - Start continuous Alt-Az tracking loop")
    print("  status              - Show current targets, calculated positions, and motor telemetry")
    print("  stop                - Stop tracking and halt motors")
    print("  calibrate on/off    - Toggle calibration mode (bypasses safety limits when on)")
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
                controller.park_and_shutdown()
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
                    alt, az, _ = ra_dec_to_alt_az(ra, dec, args.lat, args.lon, args.elevation, now)
                    print(f"Target Object:  RA {ra:.4f}h | Dec {dec:.4f}°")
                    print(f"Target Alt/Az:  Alt {alt:.2f}° | Az {az:.2f}°")
                    print(f"Target Steps:   Az {controller.target_az_steps} | Alt {controller.target_alt_steps}")
                else:
                    print("Target Object:  None Set")
                
                cur_az_deg = steps_to_deg(controller.current_az_steps, controller.ref_az, AZ_GEAR_RATIO, is_azimuth=True)
                cur_alt_deg = steps_to_deg(controller.current_alt_steps, controller.ref_alt, ALT_GEAR_RATIO, is_azimuth=False)
                print(f"Mount Alt/Az:   Alt {cur_alt_deg:.2f}° | Az {cur_az_deg:.2f}°")
                print(f"Motor Steps:    Az {controller.current_az_steps} | Alt {controller.current_alt_steps}")
                print(f"Tracking State: {'ACTIVE' if tracking else 'INACTIVE'}")
                print("----------------------------------------------------")
            elif cmd == "target":
                if len(parts) >= 2 and parts[1].lower() == "resolve":
                    if len(parts) < 3:
                        print("Usage: target resolve <ObjectName>   (e.g., target resolve M42)")
                        continue
                    object_name = " ".join(parts[2:])
                    print(f"Resolving '{object_name}' via Simbad...")
                    res = resolve_target_via_simbad(object_name)
                    if res:
                        ra_d = res.get('ra_d') or res.get('ra')
                        dec_d = res.get('dec_d') or res.get('dec')
                        name = res.get('name') or object_name
                        if ra_d is not None and dec_d is not None:
                            ra_h = ra_d / 15.0
                            print(f"[SIMBAD] Resolved '{name}' to RA: {ra_h:.4f}h ({ra_d:.4f}°), Dec: {dec_d:.4f}°")
                            controller.set_target(str(ra_h), str(dec_d))
                        else:
                            print(f"[ERROR] Resolved object '{name}' is missing coordinates.")
                    else:
                        print(f"[ERROR] Could not resolve '{object_name}' via Simbad.")
                else:
                    if len(parts) < 3:
                        print("Usage: target <RA> <Dec>   (e.g., target 05:35:17 -05:23:28 or target 5.588 -5.391)")
                        print("       To resolve via Simbad: target resolve <ObjectName> (e.g., target resolve M42)")
                        continue
                    controller.set_target(parts[1], parts[2])
            elif cmd == "goto":
                if len(parts) < 2:
                    print("Usage: goto <Az> [Alt]   (e.g., goto 180 or goto 180 45, default Alt is 90)")
                    continue
                alt_val = parts[2] if len(parts) >= 3 else "90.0"
                controller.slew_to_alt_az(parts[1], alt_val)
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
            elif cmd == "nudge_steps":
                if len(parts) < 3:
                    print("Usage: nudge_steps <d_az> <d_alt>   (e.g., nudge_steps 100 -50)")
                    continue
                controller.nudge_steps(parts[1], parts[2])
            elif cmd == "nudge_ra_dec":
                if len(parts) < 3:
                    print("Usage: nudge_ra_dec <d_ra> <d_dec>   (e.g., nudge_ra_dec 0.05 0.5)")
                    continue
                controller.nudge_ra_dec(parts[1], parts[2])
            elif cmd == "calibrate":
                if len(parts) < 2:
                    print("Usage: calibrate <on|off>")
                    continue
                arg = parts[1].lower()
                if arg == "on":
                    controller.set_calibration_mode(True)
                elif arg == "off":
                    controller.set_calibration_mode(False)
                else:
                    print("Invalid argument. Use: calibrate on or calibrate off")
            elif cmd == "stop":
                controller.stop_tracking()
            else:
                print(f"Unknown command: '{cmd}'. Available: target, goto, set_pos, pulse, nudge_steps, nudge_ra_dec, track, status, stop, lst, exit.")
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        controller.park_and_shutdown()

if __name__ == "__main__":
    main()
