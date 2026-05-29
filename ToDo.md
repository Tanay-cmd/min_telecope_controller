# Telescope Alt-Az mount project

This workspace contains a coordinate calculation tracking system for a dual-axis Alt-Azimuth telescope mount.

## File Structure
- `control_server.py`: Python CLI server script that runs on the laptop. It calculates LST, converts target RA/Dec to Alt-Az, converts to steps, and controls the Arduino over Serial.
- `mount_control/mount_control.ino`: Lightweight Arduino Mega sketch that parses serial commands (`MOVE`, `HALT`) and drives the Azimuth/Altitude motors.
- `test_calculations.py`: Validation script that tests the LST and Alt-Az coordinate conversion algorithms.
- `venv/`: Python virtual environment containing required dependencies (`pyserial`).

## Quick Start Instructions

### 1. Run Coordinate Verification Tests
To run the automated calculation validation tests:
```bash
./venv/bin/python3 test_calculations.py
```

### 2. Run the Laptop Control Server
To run the tracking server (defaulting to Mock serial mode for testing):
```bash
./venv/bin/python3 control_server.py --port MOCK --lat 12.9716 --lon 77.5946
```

To run with a connected Arduino Mega (replace `/dev/ttyACM0` with your Arduino's serial port):
```bash
./venv/bin/python3 control_server.py --port /dev/ttyACM0 --lat <your_latitude> --lon <your_longitude>
```

#### Inside the CLI Server:
- `target <RA> <Dec>`: Sets celestial target coordinates (e.g. `target 05:35:17 -05:23:28` or `target 10.25 -12.5`) and slews immediately.
- `goto <Az> <Alt>`: Directly slews the mount to specific Alt-Az angles in degrees (e.g. `goto 180 45`).
- `set_pos <Az> <Alt>`: Calibrates the current position of the mount to the specified Alt-Az degrees without moving.
- `pulse <Az_pulses> <Alt_pulses>`: Commands raw step pulse counts directly to the motors (e.g. `pulse 333 0`).
- `track`: Starts continuous real-time tracking (Alt-Az is recalculated and updated every 2 seconds).
- `status`: Shows current targets, calculated positions, and live motor step telemetry.
- `stop`: Stops tracking and halts both motors immediately.
- `lst`: Prints the current UTC and Local Sidereal Time.
- `exit`: Slews the telescope back to the Park/Home position (Az: 0°, Alt: 90°), halts motors, and closes the connection.

### 3. Flash the Arduino Mega
Open the `mount_control/mount_control.ino` sketch in the Arduino IDE (or use the Arduino CLI) and flash it onto the Mega 2560.
Ensure your DM542 drivers are wired according to the wiring matrix:
- **Azimuth Driver**: PUL+ -> Pin 22, DIR+ -> Pin 23
- **Altitude Driver**: PUL+ -> Pin 24, DIR+ -> Pin 25
- **GND**: Common Cathode connected to Mega GND
- **DIP Switches**: Configured for 2000 steps/revolution.