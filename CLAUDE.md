# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an **Arduino-controlled dynamometer (dyno)** for testing stepper motor and BLDC motor performance. The project is part of the course **Konstruktion1 - HT25** at JTH and inspired by [Engineer Bo's YouTube video](https://www.youtube.com/watch?v=MNTuMiNC2TU).

**Primary Goal:** Generate torque-speed curves for motor-driver combinations by measuring motor performance under controlled brake load.

**Language:** English (documentation and code comments)

### System Description

The dynamometer uses a **dual-motor system**:

1. **Test Motor (Motor Under Test)**
   - NEMA 17 stepper or BLDC motor being characterized
   - Runs at constant RPM (open-loop) while brake is progressively applied
   - Stepper motors maintain commanded speed until stall (no PID needed)
   - Performance specs: up to 0.8 Nm torque, max 3000 RPM

2. **Brake Actuator Motor**
   - NEMA 17 stepper motor driving a trapezoidal screw
   - Pulls a wire that actuates bicycle brake pads
   - Provides controlled, progressive braking force

### Measurement System

- **AS5600 Magnetic Sensor:** Measures actual RPM of test motor (contactless, I2C, 12-bit resolution)
  - Used for RPM verification and stall detection (not feedback control)
  - RPM sampled every 100ms using `getAngularSpeed(AS5600_MODE_DEGREES)`
  - See `docs/AS5600_code_explanation.md` for implementation details
- **Load Cell:** 5 kg full Wheatstone bridge with HX711 amplifier
  - Measures force on brake arm, converts to torque (t = F x r)
  - Requires calibration with known weights before testing
- **Test Procedure:** Set constant RPM, increase brake force until motor stalls, record max torque vs RPM

### Project Inspiration

Based on Engineer Bo's NEMA17 dynamometer project. Our improvements:
- AS5600 (contactless magnetic) instead of optical encoder (more reliable)
- **Open-loop speed control** with AS5600 stall detection
- SerialCommander for automated test procedures
- See `docs/background.md` for project inspiration and comparison

See `docs/PROJECT_DESCRIPTION.md` for complete technical specification.

## Hardware Configuration

- **Platform:** Arduino Mega with RAMPS 1.4 shield
- **Stepper Drivers:** TMC2226 (8 microsteps default, no jumpers on RAMPS)
- **Stepper Motors:** Connected via RAMPS, 200 full steps x 8 microsteps = 1600 steps/rev
  - Test Motor (X-axis): STEP=54, DIR=55, ENABLE=38 (DIR inverted)
  - Brake Actuator Motor (Y-axis): STEP=60, DIR=61, ENABLE=56
- **Sensors:**
  - AS5600 magnetic rotary encoder (I2C: SDA=20, SCL=21) - RPM measurement
  - HX711 load cell amplifier (D4, D5) - Torque measurement
- **Mechanical:**
  - Bicycle disc brake + brake caliper
  - Trapezoidal screw for wire actuation
  - Brake arm with measured length for torque calculation
- **Serial:** 115200 baud
- **Motors disabled by default** on startup (send `enable` to power)

## Project Structure (PlatformIO)

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
│   ├── AS5600_Test/             # Magnetic sensor tutorial
│   │   └── AS5600_Test.ino
│   ├── loadcellTest/            # Load cell calibration tool
│   │   └── loadcellTest.ino
│   └── CmdParserExample/        # Alternative using CmdParser library
│       └── CmdParserExample.ino
├── docs/
│   ├── guide.md                 # Complete user & technical guide
│   ├── PROJECT_DESCRIPTION.md   # Detailed specification
│   ├── AS5600_code_explanation.md
│   ├── I2C_explanation.md
│   └── background.md
├── platformio.ini               # PlatformIO configuration
├── README.md                    # Project overview
├── LICENSE                      # MIT License
└── CLAUDE.md                    # This file
```

## Key Features

### Open-Loop Speed Control
The main application uses open-loop stepper control with AS5600 monitoring:
- `setMotorRPM(rpm)` converts RPM to steps/s: `rpm * 1600 / 60`
- AS5600 provides RPM measurement for stall detection
- `updateRPM()` samples every 100ms using `getAngularSpeed()`
- No PID needed: steppers hold speed until stall

### SerialCommander
Custom zero-dependency serial command handler:
- Clean callback system: `cmd.addCommand("name", handlerFunction)`
- Type-safe parameters: `getInt(0)`, `getFloat(1)`, `getBool(2)`, `getString(3)`
- Automatic help system
- Limits: 20 commands, 10 parameters, 64-byte buffer

### Available Commands
```
# Automated Testing
runTest <rpm>           # Run automated torque test (applies brake until stall)
abortTest               # Abort running test

# Test Motor Control
setRPM <value>          # Set motor speed in RPM (open-loop, 0 to stop)
setSpeed <steps/s>      # Set motor speed in raw steps/s
status                  # Show RPM, torque, brake position
autoStatus [true/false] # Toggle live status display

# Brake Control
brake <steps>           # Move brake motor (+apply / -release)
brakeApply [steps]      # Apply brake (default: 500)
brakeRelease [steps]    # Release brake (default: 500)
brakeHome               # Return brake to zero

# Load Cell & Torque
readLoad                # Read load (kg), force (N), torque (Nm)
readTorque              # Read torque only (Nm)
debugLoad               # Toggle raw load cell output (every 300ms)
tare                    # Zero the load cell
calibrate               # Interactive calibration

# Sensors & Debug
readSensor [C]          # Read AS5600 angle & RPM (C=continuous)
debug                   # Show raw sensor/motor values

# Motor Power
enable                  # Enable all motor drivers
disable                 # Disable all motor drivers
stop                    # Emergency stop
help                    # Show all commands
```

### Python/Jupyter Interface
A control notebook is provided in `tools/dyno_control.ipynb`:
- Interactive buttons for running tests
- Automated multi-RPM curve generation
- Real-time plotting of torque-speed curves

## Common Development Tasks

### Building & Uploading (PlatformIO - Recommended)

```bash
# Build
pio run

# Upload
pio run -t upload

# Open Serial Monitor
pio device monitor
```

### Building & Uploading (Arduino IDE)

1. Copy `lib/SerialCommander/SerialCommander.h` to Arduino libraries folder
2. Install required libraries via Library Manager
3. Rename `src/main.cpp` to `.ino` and upload

### Required Libraries (managed by PlatformIO)
- **robtillaart/AS5600** - Magnetic encoder
- **robtillaart/HX711** - Load cell amplifier
- **waspinator/AccelStepper** - Stepper motor control

### Testing
1. Open Serial Monitor at 115200 baud
2. Verify startup: `Dyno Ready - Open-Loop Control v2.0`
3. Check sensor: `AS5600 connected: Yes`
4. Send `enable` to power motors
5. Test basic commands: `setRPM 100`, `status`, `brakeApply`

## Architecture Notes

### Educational Design
- Multiple patterns shown (custom vs library approaches)
- Clarity prioritized over optimization
- Modular independence - each example works standalone
- Comprehensive documentation with code explanations

### Motor Control Functions
```cpp
void setMotorRPM(float rpm);  // Set RPM (converts to steps/s)
void stopMotor();              // Stop test motor
void updateRPM();              // Sample AS5600 (call in loop)
float getRPM();                // Get last measured RPM
```

### Adding Commands
```cpp
void handleMyCommand(SerialCommander* c) {
  int param = c->getInt(0);
  // Your logic here
}

void setup() {
  cmd.addCommand("myCommand", handleMyCommand);
}
```

### When Making Changes
- Maintain simplicity - this is teaching code
- Keep modules independent
- Test with actual hardware when possible
- Update documentation to match changes

## Pin Mappings (RAMPS 1.4)

| Axis | STEP | DIR | ENABLE |
|------|------|-----|--------|
| X (Test Motor) | 54 | 55 | 38 |
| Y (Brake Motor) | 60 | 61 | 56 |
| Z (unused) | 46 | 48 | 62 |
| E0 (unused) | 26 | 28 | 24 |

**I2C:** SDA=20, SCL=21
**HX711:** DOUT=D4, SCK=D5
