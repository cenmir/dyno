# Dyno User Guide (Open-Loop v2.0)


## Introduction

This document is a guide for using and understanding the dynamometer for stepper motors. This guide describes the usage of firmware (software running on a microcontroller).

The guide is divided into two parts:

1. **User Guide:** Focuses on how to use the dynamometer without needing to program.
2. **Technical Guide:** For those who want to understand the code to make changes or further develop the system.

---

# Part 1: User Guide

## Wiring Diagram - Detailed Connections

All electronics are controlled by an Arduino Mega with a RAMPS 1.4 shield.

### RAMPS 1.4 Pin Mapping

All hardware connects to the RAMPS 1.4 board, which in turn controls the Arduino Mega. The image below provides an overview of the board's connections.

![RAMPS 1.4 Wiring Diagram](https://reprap.org/mediawiki/images/c/ca/Arduinomega1-4connectors.png)
*Figure 1: Overview of RAMPS 1.4 connections. Source: RepRap Wiki*

Below is a table specifying exactly which pins on the RAMPS board are used in this project and which Arduino Mega pins they correspond to.

| Function | RAMPS 1.4 Connection | Arduino Mega Pin |
| :--- | :--- | :--- |
| **Test Motor (X-axis)** | Stepper Driver `X` | - |
| Step | `X_STEP_PIN` | D54 |
| Direction | `X_DIR_PIN` | D55 |
| Enable | `X_ENABLE_PIN` | D38 |
| **Brake Motor (Y-axis)** | Stepper Driver `Y` | - |
| Step | `Y_STEP_PIN` | D60 |
| Direction | `Y_DIR_PIN` | D61 |
| Enable | `Y_ENABLE_PIN` | D56 |
| **Torque Sensor (Load Cell)** | `SERVOS` | - |
| Data (DOUT) | Pin `D4` | D4 |
| Clock (SCK) | Pin `D5` | D5 |
| **RPM Sensor (AS5600)** | `AUX-4` | - |
| Data (SDA) | `SDA` | D20 |
| Clock (SCL) | `SCL` | D21 |

**Connection Details:**

* **Power Supply:**
    * The Arduino connects to the computer with a USB cable.
    * The RAMPS board and motor drivers need external power (12V or 24V) via the `5A` or `11A` inputs.

* **Motors to RAMPS 1.4:**
    * **Test Motor:** Connects to the 4-pin connector for the `X`-axis.
    * **Brake Motor:** Connects to the 4-pin connector for the `Y`-axis.

* **Sensors:**
    * **AS5600 (RPM):** Connects to the I2C port (`SDA`, `SCL`, `5V`, `GND`) on `AUX-4`.
    * **HX711 (Torque):** Connects to `D4` and `D5` on the servo pins, plus `5V` and `GND`.

## Software Interaction

Control is done via "Serial Monitor" in Arduino IDE or PlatformIO.

1. **Connection:** Connect the Arduino, select the correct `Port` and open Serial Monitor.
2. **Baud Rate:** Set the speed to **115200**.
3. **Startup Message:** You will see `Dyno Ready - Open-Loop Control v2.0`.

**Important:** Motors are disabled by default on startup. Send `enable` before operating the motors.

## Commands

Here is an overview of the most important commands. Send `help` for a complete list.

| Command | Description | Example |
| --- | --- | --- |
| **enable** | Enables all motor drivers (required after startup). | `enable` |
| **disable** | Disables all motor drivers and stops motors. | `disable` |
| **setRPM `<rpm>`** | Sets the target speed in RPM. The motor runs open-loop at this speed until stall. `0` to stop. | `setRPM 800` |
| **setSpeed `<steps/s>`** | Sets motor speed in raw steps per second. | `setSpeed 1600` |
| **status** | Shows current status: target RPM, measured RPM, brake position and torque. | `status` |
| **autoStatus `<true/false>`** | Toggles automatic status update every half second. | `autoStatus true` |
| **brakeApply `[steps]`** | Applies the brake by moving the brake motor a number of steps (default 500). | `brakeApply 250` |
| **brakeRelease `[steps]`** | Releases the brake (default 500 steps). | `brakeRelease` |
| **brakeHome** | Returns the brake to its zero position. | `brakeHome` |
| **readLoad** | Reads load (kg), force (N), and torque (Nm). | `readLoad` |
| **readTorque** | Reads and prints the current torque in Newton-meters (Nm). | `readTorque` |
| **debugLoad** | Toggles continuous raw load cell output every 300ms. | `debugLoad` |
| **tare** | Zeros (tares) the load cell. **Always run before a test series.** | `tare` |
| **calibrate** | Starts an interactive guide to calibrate the load cell. | `calibrate` |
| **readSensor `[C]`** | Reads AS5600 angle and RPM. `C` toggles continuous output. | `readSensor C` |
| **debug** | Shows raw sensor and motor values for troubleshooting. | `debug` |
| **stop** | Emergency stop. Immediately turns off all motors. | `stop` |

## Load Cell Calibration (Important First Step!)

Before you can get correct measurements, you must calibrate the load cell. **This only needs to be done once**, or if you rebuild the mechanics.

1. Send the command `calibrate`.
2. Follow the instructions:
    a. Remove all weight from the load cell, type `ok` and press Enter.
    b. Place a known weight on the load cell.
    c. Enter the weight in kilograms (e.g., `0.5` for 500g) and press Enter.
3. The program prints the `offset` and `scale` values that you must paste into the code. Ask an instructor for help with this.

## Measurement Process (Open-Loop)

There are two ways to run tests: **automated** (recommended) or **manual**.

### Why Open-Loop?

Stepper motors maintain the exact speed you command until the load becomes too high and they **stall** (skip steps). Unlike DC motors, there is no gradual speed droop. This means:

- No PID controller is needed to maintain speed
- The AS5600 sensor is used for **stall detection** and **RPM verification**
- When the measured RPM drops below the target, the motor has stalled

### Automated Testing (Recommended)

The `runTest` command automatically runs a complete torque test:

1. **Preparation:**
   * `enable` - Enable motor drivers
   * `brakeHome` - Release brake
   * `tare` - Zero the load cell

2. **Run the test:**
   ```
   runTest 500
   ```
   This command:
   - Sets the motor to 500 RPM (open-loop)
   - Incrementally applies brake (50 steps every 500ms)
   - Records torque while motor maintains speed
   - Detects stall (when RPM drops below 90% of target for 1 second)
   - Reports maximum torque achieved

3. **Output format:**
   ```
   === AUTOMATED TEST STARTED ===
   Target RPM: 500
   DATA:498.2,0.0523,50
   DATA:499.1,0.0891,100
   ...
   === STALL DETECTED ===
   RESULT:maxTorque=0.2845,stallTorque=0.2712,targetRPM=500.0,stallRPM=421.3
   --- Test Complete ---
   Maximum torque at 500 RPM: 0.2845 Nm
   ```

4. **Abort if needed:** Send `abortTest` to stop a running test.

### Stall Detection Logic

The system detects a stall when:
- Actual RPM drops below **90%** of target RPM
- This condition persists for **1 second**

The **maximum torque** reported is the highest torque measured while the motor was still maintaining the target speed (before stall).

### Manual Testing (Alternative)

For more control, you can run tests manually:

1. **Preparation:** Make sure the load cell is calibrated and restart the Arduino.
2. **Enable motors:** Send `enable` to power the motor drivers.
3. **Zero brake & load cell:**
    * Send `brakeHome` to ensure the brake is fully released.
    * Send `tare` to zero the torque reading.
4. **Set speed:** Choose an RPM, e.g., `setRPM 600`. The motor starts and runs at that speed.
5. **Enable monitoring:** Send `autoStatus true`. You now see a stream of data with speed and torque.
6. **Apply load gradually:** Start applying the brake in small steps with the command `brakeApply 100`. Wait a few seconds between each command.
7. **Observe:** In the status updates, you see how the torque (`Torque`) increases. The stepper motor will maintain 600 RPM until the load exceeds its torque capacity.
8. **Find max torque:** Continue applying the brake until the motor stalls (RPM drops to 0). The highest torque value you saw just before the motor stalled is the maximum torque for that RPM.
9. **End the test:** Send `stop` and `brakeHome`.
10. **Repeat:** Perform the same process for other RPMs (e.g., 200, 400, 800 RPM) to collect data for a complete torque-speed curve.

## Understanding Torque-Speed Curves

### What is a Torque-Speed Curve?

A torque-speed curve shows **how much torque a motor can deliver at different speeds**. This is critical for selecting motors for applications like 3D printers, CNC machines, or robots.

```
Torque (Nm)
    |
0.5 |####
    |########
0.3 |############
    |################
0.1 |####################
    +------------------------> RPM
      200   400   600   800
```

**Key insight:** At low RPM, motors deliver more torque. As speed increases, torque drops due to inductive effects in the motor windings.

### How Torque is Calculated

The load cell measures force on the brake arm. Torque is calculated as:

$$\tau = F \times r$$

Where:
- $\tau$ = Torque (Nm)
- $F$ = Force measured by load cell (N) = mass (kg) x 9.81 m/s^2
- $r$ = Brake arm length (m)

In the code (`src/main.cpp`):
```cpp
#define ARM_LENGTH_METERS  0.1f   // Measure your brake arm!
#define GRAVITY            9.80665f

float getTorque() {
  return LoadCell.get_units(1) * GRAVITY * ARM_LENGTH_METERS;
}
```

**Important:** You must measure your brake arm length and update `ARM_LENGTH_METERS` for accurate torque readings!

## Recording Data and Creating the Curve

### Method 1: Python/Jupyter Notebook (Recommended)

A complete Jupyter notebook is provided in `tools/dyno_control.ipynb` with:
- Serial connection management
- Interactive buttons for running tests
- Automated multi-RPM curve generation
- Real-time plotting

**Setup:**
```bash
uv pip install pyserial matplotlib ipywidgets jupyter
```

**Usage:**
1. Open the notebook: `jupyter notebook tools/dyno_control.ipynb`
2. Update `SERIAL_PORT` to your Arduino's port
3. Run the cells to connect and test

**Key features:**
- `run_test(500)` - Run a single test at 500 RPM
- `generate_torque_curve([200, 400, 600, 800])` - Generate complete curve
- Interactive control panel with buttons

### Method 2: Manual Recording

1. Run the test as described above
2. Write down the max torque for each RPM in a table:

| RPM | Max Torque (Nm) |
|-----|-----------------|
| 200 | 0.45 |
| 300 | 0.38 |
| 400 | 0.32 |
| 500 | 0.29 |
| 600 | 0.24 |
| 700 | 0.18 |
| 800 | 0.12 |

3. Plot in Excel, Google Sheets, or any plotting tool.

### Method 3: Serial Logging

Use a serial terminal that can log to file:

**PlatformIO:**
```bash
pio device monitor > test_data.txt
```

**Arduino IDE:**
- No built-in logging, but you can copy/paste from Serial Monitor

**PuTTY (Windows):**
- Session -> Logging -> "All session output"
- Set filename, then connect

**Screen (Linux/Mac):**
```bash
screen -L /dev/ttyUSB0 115200
```

The log file will contain all the `autoStatus` output. Extract the torque values when RPM starts dropping.

### Method 4: Custom Python Script

For custom automation, see `tools/dyno_control.ipynb` as a starting point. The key functions:

```python
import serial

ser = serial.Serial('COM3', 115200)

# Run automated test
ser.write(b'runTest 500\n')

# Parse DATA: lines for logging
# Format: DATA:rpm,torque,brake_pos
while True:
    line = ser.readline().decode()
    if line.startswith('DATA:'):
        rpm, torque, pos = line[5:].split(',')
        # Process data...
    if 'STALL DETECTED' in line:
        break
```

### Plotting the Curve

Once you have the data, plot RPM (x-axis) vs Max Torque (y-axis):

**Excel/Google Sheets:**
1. Enter data in two columns (RPM, Torque)
2. Insert -> Chart -> Scatter plot

**Python (matplotlib):**
```python
import matplotlib.pyplot as plt

rpm = [200, 300, 400, 500, 600, 700, 800]
torque = [0.45, 0.38, 0.32, 0.29, 0.24, 0.18, 0.12]

plt.plot(rpm, torque, 'b-o')
plt.xlabel('RPM')
plt.ylabel('Torque (Nm)')
plt.title('Motor Torque-Speed Curve')
plt.grid(True)
plt.savefig('torque_curve.png')
plt.show()
```

---

# Part 2: Technical Guide

## System Architecture

```
+-----------+
|  AS5600   | --> Measures actual RPM (stall detection)
| (Sensor)  |
+-----------+
      |
      |      +-----------+
      +----> | main.cpp  | --> Open-loop speed command
             +-----------+
                   |
             +-----------+
             | Test Motor| --> Runs at commanded RPM until stall
             | (X-axis)  |
             +-----------+
```

### How It Works

1. **Speed Command:** The user sets the desired speed (e.g., `setRPM 100`).
2. **Open-Loop Conversion:** RPM is converted to steps/second: `steps/s = RPM * 1600 / 60`
3. **AccelStepper Executes:** The motor runs at the commanded step rate.
4. **AS5600 Monitors:** The sensor continuously measures actual RPM (every 100ms).
5. **Stall Detection:** If measured RPM drops below threshold, the motor has stalled.

### Why Open-Loop (Not PID)?

Stepper motors are synchronous: they execute every step pulse precisely. Unlike DC motors, they don't gradually slow down under load. Instead, they maintain exact speed until the load exceeds their torque capacity, at which point they stall abruptly (skip steps).

This means:
- A PID controller has nothing to correct (speed is either perfect or the motor has stalled)
- Open-loop control is simpler and equally effective
- The AS5600 sensor is used for monitoring and stall detection, not feedback control

### RPM Measurement

The AS5600 library's `getAngularSpeed()` function measures rotational speed:

```cpp
void updateRPM() {
  if (millis() - lastRpmTime >= 100) {  // Sample every 100ms
    currentRPM = -as5600.getAngularSpeed(AS5600_MODE_DEGREES) / 6.0;
    lastRpmTime = millis();
  }
}
```

- **100ms sampling interval** avoids aliasing from AS5600's angle quantization (12-bit, 4096 steps)
- **Negation** (`-`) corrects for motor direction (DIR pin inverted)
- **Division by 6.0** converts degrees/second to RPM: `360 deg/rev / 60 s/min = 6`

### Motor Speed Conversion

```cpp
void setMotorRPM(float rpm) {
  targetRPM = rpm;
  float stepsPerSec = rpm * TEST_MOTOR_STEPS_PER_REV / 60.0;
  testMotor.setSpeed(stepsPerSec);
}
```

With TMC2226 drivers (no jumpers on RAMPS = 8 microsteps):
- `TEST_MOTOR_STEPS_PER_REV = 200 * 8 = 1600`
- Example: 60 RPM = `60 * 1600 / 60 = 1600 steps/s`

## Advanced: Code Modifications

### Change Steps Per Revolution (Microstepping)

Microstepping is configured at the top of `main.cpp`:
```cpp
#define TEST_MOTOR_STEPS_PER_REV    (200 * 8)  // 200 full steps x 8 microsteps (TMC2226)
#define BRAKE_MOTOR_STEPS_PER_REV   (200 * 8)
```

Adjust the multiplier for your driver's microstepping setting:
- No jumpers (TMC2226 default): 8 microsteps
- MS1 only: 2 microsteps
- MS2 only: 4 microsteps
- MS1 + MS2: 16 microsteps

### Adjust Brake Arm Length

For accurate torque measurement, update the arm length:
```cpp
#define ARM_LENGTH_METERS  0.1f   // IMPORTANT: Measure and update this value!
```

### Adjust Stall Detection Parameters

```cpp
int testBrakeIncrement = 50;          // Steps per brake increment
unsigned long testBrakeInterval = 500; // ms between brake increments
float testStallThreshold = 0.90;      // Stall if RPM < 90% of target
unsigned long testStallDuration = 1000; // ms below threshold to confirm stall
```

## Troubleshooting

### "AS5600 connected: No"
* **Problem:** Sensor not detected.
* **Solution:**
    1. Check I2C wiring (`SDA`=20, `SCL`=21 on RAMPS).
    2. Check 5V power supply to sensor.
    3. Verify magnet is correctly mounted (1-3 mm distance, centered).

### RPM Shows 0 or Unreasonable Values
* **Problem:** Poor magnet alignment or interference.
* **Solution:**
    1. Check magnet distance (1-3 mm is optimal).
    2. Ensure magnet is diametrically polarized.
    3. Avoid metal objects near the sensor.

### Motor Doesn't Spin
* **Problem:** Motor drivers are disabled by default.
* **Solution:** Send `enable` command before using the motors.

---

# References

- [RAMPS 1.4 Documentation](https://reprap.org/wiki/RAMPS_1.4) - RepRap Wiki
- [AS5600 Datasheet](https://ams.com/as5600) - ams AG
- [HX711 Datasheet](https://cdn.sparkfun.com/datasheets/Sensors/ForceFlex/hx711_english.pdf) - Sparkfun
- [AccelStepper Library](https://www.airspayce.com/mikem/arduino/AccelStepper/) - Mike McCauley
