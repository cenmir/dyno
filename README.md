# Dyno - Arduino Stepper Motor Dynamometer

An Arduino-controlled dynamometer for characterizing stepper motor and BLDC motor performance. Generate torque-speed curves by measuring motor output under controlled braking load.

![License](https://img.shields.io/badge/license-MIT-blue.svg)
![Platform](https://img.shields.io/badge/platform-Arduino%20Mega-orange.svg)
![PlatformIO](https://img.shields.io/badge/build-PlatformIO-brightgreen.svg)

## Overview

This dynamometer uses a dual-motor system with **closed-loop PID speed control** to maintain constant test motor RPM while progressively applying brake load. Key features:

- **PID-controlled constant speed** - Maintains set RPM even under varying load
- **Automated torque testing** - `runTest` command with stall detection
- **AS5600 magnetic sensor** - Contactless RPM measurement (no dust issues)
- **HX711 load cell** - Precise torque measurement via brake arm force
- **Python/Jupyter interface** - Generate torque curves with interactive notebook
- **Serial command interface** - Easy control via Serial Monitor

**Inspiration:** [Engineer Bo's NEMA17 Dynamometer](https://www.youtube.com/watch?v=MNTuMiNC2TU)

## Hardware Requirements

| Component | Description |
|-----------|-------------|
| Arduino Mega | Microcontroller |
| RAMPS 1.4 | Motor driver shield |
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
   - `lib/SpeedController/SpeedController.h`
   - `lib/SerialCommander/SerialCommander.h`

3. **Open and upload** `src/Dyno_ClosedLoop.ino`

4. **Open Serial Monitor** at 115200 baud

### Verify Connection

You should see:
```
AS5600 connected: Yes
HX711 initialized. Ready for calibration ('tare' then 'calibrate').
Dyno Ready - Closed-Loop Control v1.2
Type 'help' for available commands
```

## Commands

```
# Automated Testing
runTest 500       # Run automated torque test at 500 RPM
abortTest         # Abort running test

# Manual Control
setRPM 100        # Set test motor to 100 RPM (PID controlled)
status            # Show current RPM, torque, brake position
autoStatus true   # Enable live status updates every 500ms

brakeApply 500    # Apply brake (500 steps)
brakeRelease 500  # Release brake
brakeHome         # Return brake to zero position

readTorque        # Read current torque (Nm)
tare              # Zero the load cell
calibrate         # Interactive load cell calibration

tune 50 10 1      # Adjust PID parameters (Kp Ki Kd)
stop              # Emergency stop all motors
help              # Show all commands
```

## Test Procedure

### Automated (Recommended)

1. **Calibrate load cell** (first time only): `calibrate`
2. **Prepare**: `brakeHome` then `tare`
3. **Run test**: `runTest 500` (runs automatically until stall)
4. **Record result**: Note the "Maximum torque at X RPM" value
5. **Repeat** at different RPM values: `runTest 600`, `runTest 700`, etc.

### Manual

1. **Calibrate load cell** (first time only): `calibrate`
2. **Prepare**: `brakeHome` then `tare`
3. **Run test**: `setRPM 600` then `autoStatus true`
4. **Apply load**: `brakeApply 100` (repeat gradually)
5. **Find max torque**: Keep applying brake until motor stalls
6. **Repeat** at different RPM values to build torque-speed curve

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
│   └── Dyno_ClosedLoop.ino       # Main application
├── lib/
│   ├── SpeedController/
│   │   └── SpeedController.h     # PID controller class
│   └── SerialCommander/
│       └── SerialCommander.h     # Command handler
├── tools/
│   └── dyno_control.ipynb        # Python/Jupyter control panel
├── examples/
│   ├── AS5600_Test/              # Sensor test
│   ├── loadcellTest/             # Load cell calibration
│   └── CmdParserExample/         # Alternative command parsing
├── docs/
│   ├── guide.md                  # Complete user & technical guide
│   ├── PROJECT_DESCRIPTION.md    # System specification
│   ├── SpeedController_explanation.md  # PID code explanation
│   ├── AS5600_code_explanation.md
│   ├── I2C_explanation.md
│   └── background.md
├── platformio.ini                # PlatformIO configuration
├── README.md                     # This file
├── CLAUDE.md                     # Developer guide
└── LICENSE                       # MIT License
```

## PID Tuning

Default parameters: `Kp=50, Ki=10, Kd=1`

### Quick Tuning Process

1. **P-only**: `tune 10 0 0` - Increase until motor responds quickly
2. **Add I**: `tune 30 5 0` - Eliminate steady-state error
3. **Add D**: `tune 30 5 1` - Dampen oscillations

See [docs/guide.md](docs/guide.md) for detailed tuning instructions.

## Documentation

**Recommended reading order:**

1. **[background.md](docs/background.md)** - Why this project exists (Engineer Bo's inspiration)
2. **[guide.md](docs/guide.md)** - Complete user manual + PID tuning guide
3. **[PROJECT_DESCRIPTION.md](docs/PROJECT_DESCRIPTION.md)** - System specification and requirements

**Code explanations** (for understanding/modifying the code):
- [SpeedController_explanation.md](docs/SpeedController_explanation.md) - PID controller code walkthrough
- [AS5600_code_explanation.md](docs/AS5600_code_explanation.md) - Sensor code walkthrough
- [I2C_explanation.md](docs/I2C_explanation.md) - I2C protocol basics

## Troubleshooting

| Problem | Solution |
|---------|----------|
| "AS5600 connected: No" | Check I2C wiring (SDA=20, SCL=21), verify 5V power |
| RPM shows 0 | Verify magnet is 1-3mm from sensor, check magnet polarity |
| Motor oscillates wildly | Reduce PID gains: `tune 10 0 0`, then tune gradually |
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
