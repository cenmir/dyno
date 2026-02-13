# Dyno - Arduino Stepper Motor Dynamometer

An Arduino-controlled dynamometer for characterizing stepper motor and BLDC motor performance. Generate torque-speed curves by measuring motor output under controlled braking load.

![License](https://img.shields.io/badge/license-MIT-blue.svg)
![Platform](https://img.shields.io/badge/platform-Arduino%20Mega-orange.svg)
![PlatformIO](https://img.shields.io/badge/build-PlatformIO-brightgreen.svg)

## Overview

This dynamometer uses a dual-motor system with **open-loop speed control** to run the test motor at a constant RPM while progressively applying brake load. Key features:

- **Open-loop stepper control** - Steppers maintain commanded speed until stall (no PID needed)
- **AS5600 RPM measurement** - Contactless magnetic sensor for stall detection
- **Automated torque testing** - `runTest` command with stall detection
- **HX711 load cell** - Precise torque measurement via brake arm force
- **Python/Jupyter interface** - Generate torque curves with interactive notebook
- **Serial command interface** - Easy control via Serial Monitor

**Inspiration:** [Engineer Bo's NEMA17 Dynamometer](https://www.youtube.com/watch?v=MNTuMiNC2TU)

## Hardware Requirements

| Component | Description |
|-----------|-------------|
| Arduino Mega | Microcontroller |
| RAMPS 1.4 | Motor driver shield |
| TMC2226 (x2) | Stepper drivers (8 microsteps default, no jumpers) |
| NEMA 17 stepper (x2) | Test motor + brake actuator |
| AS5600 | Magnetic rotary encoder (I2C) |
| HX711 + Load Cell | 5 kg Wheatstone bridge for torque measurement |
| Bicycle brake | Disc brake + caliper |
| Trapezoidal screw | Converts rotation to linear brake actuation |

### Wiring (RAMPS 1.4)

| Function | RAMPS Connection | Arduino Pin |
|----------|------------------|-------------|
| Test Motor (X-axis) | X stepper driver | STEP=54, DIR=55, EN=38 |
| Brake Motor (Y-axis) | Y stepper driver | STEP=60, DIR=61, EN=56 |
| AS5600 (RPM) | AUX-4 I2C | SDA=20, SCL=21 |
| HX711 (Torque) | SERVOS header | DOUT=D4, SCK=D5 |

## Quick Start

### Option 1: PlatformIO (Recommended)

1. **Install PlatformIO** in VS Code
2. **Clone/download** this repository
3. **Open project folder** in VS Code
4. **Build and upload**: Click the PlatformIO upload button or run:
   ```bash
   pio run -t upload
   ```
5. **Open Serial Monitor**: `pio device monitor`

### Option 2: Arduino IDE

1. **Install Libraries** via Tools -> Manage Libraries:
   - `AccelStepper`
   - `AS5600`
   - `HX711`

2. **Copy library files** from `lib/` to your Arduino libraries folder:
   - `lib/SerialCommander/SerialCommander.h`

3. **Rename** `src/main.cpp` to `src/main.ino` (or use the `.ino.bak` as reference)

4. **Open Serial Monitor** at 115200 baud

### Verify Connection

You should see:
```
AS5600 connected: Yes
HX711 initialized. Ready for calibration ('tare' then 'calibrate').
Dyno Ready - Open-Loop Control v2.0
Type 'help' for available commands
```

## Commands

```
# Automated Testing
runTest 500       # Run automated torque test at 500 RPM
abortTest         # Abort running test

# Test Motor Control
setRPM 100        # Set test motor to 100 RPM (open-loop)
setSpeed 1600     # Set motor speed in steps/s (raw)
status            # Show current RPM, torque, brake position
autoStatus true   # Enable live status updates every 500ms

# Brake Control
brake 500         # Move brake motor by steps (+apply / -release)
brakeApply 500    # Apply brake (500 steps)
brakeRelease 500  # Release brake
brakeHome         # Return brake to zero position

# Load Cell & Torque
readLoad          # Read load (kg), force (N), torque (Nm)
readTorque        # Read torque only (Nm)
debugLoad         # Toggle raw load cell output (every 300ms)
tare              # Zero the load cell
calibrate         # Interactive load cell calibration

# Sensors & Debug
readSensor        # Read AS5600 angle & RPM
readSensor C      # Toggle continuous sensor output (every 300ms)
debug             # Show raw sensor/motor values

# Motor Power
enable            # Enable all motor drivers
disable           # Disable all motor drivers (motors off by default)
stop              # Emergency stop all motors
help              # Show all commands
```

## Test Procedure

### Automated (Recommended)

1. **Enable motors**: `enable`
2. **Calibrate load cell** (first time only): `calibrate`
3. **Prepare**: `brakeHome` then `tare`
4. **Run test**: `runTest 500` (runs automatically until stall)
5. **Record result**: Note the "Maximum torque at X RPM" value
6. **Repeat** at different RPM values: `runTest 600`, `runTest 700`, etc.

### Manual

1. **Enable motors**: `enable`
2. **Calibrate load cell** (first time only): `calibrate`
3. **Prepare**: `brakeHome` then `tare`
4. **Set speed**: `setRPM 600` then `autoStatus true`
5. **Apply load**: `brakeApply 100` (repeat gradually)
6. **Find max torque**: Keep applying brake until motor stalls
7. **Repeat** at different RPM values to build torque-speed curve

## Python Interface

A Jupyter notebook is provided for automated testing and visualization:

```bash
uv pip install pyserial matplotlib ipywidgets jupyter
jupyter notebook tools/dyno_control.ipynb
```

Features:
- Interactive buttons for test control
- Automated multi-RPM curve generation
- Real-time torque-speed curve plotting

## Project Structure

```
dyno/
├── src/
│   └── main.cpp                 # Main application (open-loop control v2.0)
├── lib/
│   ├── SerialCommander/
│   │   └── SerialCommander.h    # Zero-dependency command handler
│   └── SpeedController/
│       └── SpeedController.h    # PID controller (kept for reference, unused)
├── tools/
│   └── dyno_control.ipynb       # Python/Jupyter control panel
├── examples/
│   ├── AS5600_Test/             # Sensor test
│   ├── loadcellTest/            # Load cell calibration
│   └── CmdParserExample/        # Alternative command parsing
├── docs/
│   ├── guide.md                 # Complete user & technical guide
│   ├── PROJECT_DESCRIPTION.md   # System specification
│   ├── AS5600_code_explanation.md
│   ├── I2C_explanation.md
│   └── background.md
├── platformio.ini               # PlatformIO configuration
├── README.md                    # This file
├── CLAUDE.md                    # Developer guide
└── LICENSE                      # MIT License
```

## Why Open-Loop (No PID)?

Stepper motors are fundamentally different from DC or BLDC motors: they execute each commanded step precisely and maintain the commanded speed until the load exceeds their torque capacity, at which point they **stall** abruptly. There is no gradual speed droop for a PID controller to correct.

The AS5600 sensor serves a different but important purpose:
- **Stall detection** - Confirms when the motor can no longer maintain speed
- **RPM verification** - Validates the commanded speed matches reality
- **Data logging** - Records actual RPM during torque tests

## Documentation

**Recommended reading order:**

1. **[background.md](docs/background.md)** - Why this project exists (Engineer Bo's inspiration)
2. **[guide.md](docs/guide.md)** - Complete user manual and technical guide
3. **[PROJECT_DESCRIPTION.md](docs/PROJECT_DESCRIPTION.md)** - System specification and requirements

**Code explanations** (for understanding/modifying the code):
- [AS5600_code_explanation.md](docs/AS5600_code_explanation.md) - Sensor code walkthrough
- [I2C_explanation.md](docs/I2C_explanation.md) - I2C protocol basics

## Troubleshooting

| Problem | Solution |
|---------|----------|
| "AS5600 connected: No" | Check I2C wiring (SDA=20, SCL=21), verify 5V power |
| RPM shows 0 | Verify magnet is 1-3mm from sensor, check magnet polarity |
| Motor doesn't spin | Run `enable` first (motors disabled by default) |
| Torque reads 0 | Run `calibrate`, check HX711 wiring |

## Contributing

Contributions welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Submit a pull request

## License

MIT License - see [LICENSE](LICENSE) for details.

## Acknowledgments

- [Engineer Bo](https://www.youtube.com/watch?v=MNTuMiNC2TU) - Original dynamometer concept
- [RepRap Wiki](https://reprap.org/wiki/RAMPS_1.4) - RAMPS 1.4 documentation
