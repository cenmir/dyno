# Dyno User Guide (Closed-Loop Version)


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

Control is done via "Serial Monitor" in Arduino IDE.

1. **Connection:** Connect the Arduino, select the correct `Port` in Arduino IDE and open Serial Monitor.
2. **Baud Rate:** Set the speed to **115200**.
3. **Startup Message:** You will see `Dyno Ready - Closed-Loop Control v1.2`.

## Commands

Here is an overview of the most important commands. Send `help` for a complete list.

| Command | Description | Example |
| --- | --- | --- |
| **setRPM `<rpm>`** | Sets the target speed. The PID controller will actively maintain this speed. `0` turns off. | `setRPM 800` |
| **status** | Shows current status: target speed, actual speed, brake position and torque. | `status` |
| **autoStatus `<true/false>`** | Toggles automatic status update every half second. | `autoStatus true` |
| **brakeApply `[steps]`** | Applies the brake by moving the brake motor a number of steps (default 500). | `brakeApply 250` |
| **brakeRelease `[steps]`** | Releases the brake (default 500 steps). | `brakeRelease` |
| **brakeHome** | Returns the brake to its zero position. | `brakeHome` |
| **readTorque** | Reads and prints the current torque in Newton-meters (Nm). | `readTorque` |
| **tare** | Zeros (tares) the load cell. **Always run before a test series.** | `tare` |
| **calibrate** | Starts an interactive guide to calibrate the load cell. | `calibrate` |
| **tune `<p> <i> <d>`** | Adjusts the PID controller's parameters (for advanced users). | `tune 50 10 1` |
| **stop** | Emergency stop. Immediately turns off all motors. | `stop` |

## Load Cell Calibration (Important First Step!)

Before you can get correct measurements, you must calibrate the load cell. **This only needs to be done once**, or if you rebuild the mechanics.

1. Send the command `calibrate`.
2. Follow the instructions:
    a. Remove all weight from the load cell, type `ok` and press Enter.
    b. Place a known weight on the load cell.
    c. Enter the weight in kilograms (e.g., `0.5` for 500g) and press Enter.
3. The program prints the `offset` and `scale` values that you must paste into the code. Ask an instructor for help with this.

## Measurement Process (Closed-Loop)

There are two ways to run tests: **automated** (recommended) or **manual**.

### Automated Testing (Recommended)

The `runTest` command automatically runs a complete torque test:

1. **Preparation:**
   * `brakeHome` - Release brake
   * `tare` - Zero the load cell

2. **Run the test:**
   ```
   runTest 500
   ```
   This command:
   - Sets the motor to 500 RPM
   - Waits for stabilization
   - Incrementally applies brake (50 steps every 500ms)
   - Records torque while maintaining speed
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
2. **Zero brake & load cell:**
    * Send `brakeHome` to ensure the brake is fully released.
    * Send `tare` to zero the torque reading.
3. **Set speed:** Choose an RPM, e.g., `setRPM 600`. The motor starts and the PID controller ensures it maintains 600 RPM.
4. **Enable monitoring:** Send `autoStatus true`. You now see a stream of data with speed and torque.
5. **Apply load gradually:** Start applying the brake in small steps with the command `brakeApply 100`. Wait a few seconds between each command.
6. **Observe:** In the status updates, you see how the torque (`Torque`) increases. Thanks to the PID controller, the speed (`Current RPM`) will stay very close to your setpoint (e.g., 600 RPM).
7. **Find max torque:** Continue applying the brake until the motor can no longer resist and the speed collapses. The highest torque value you saw just before the motor stalled is the maximum torque for that RPM.
8. **End the test:** Send `stop` or `setRPM 0` and `brakeHome`.
9. **Repeat:** Perform the same process for other RPMs (e.g., 200, 400, 800 RPM) to collect data for a complete torque-speed curve.

## Understanding Torque-Speed Curves

### What is a Torque-Speed Curve?

A torque-speed curve shows **how much torque a motor can deliver at different speeds**. This is critical for selecting motors for applications like 3D printers, CNC machines, or robots.

```
Torque (Nm)
    │
0.5 │■■■■
    │■■■■■■■■
0.3 │■■■■■■■■■■■■
    │■■■■■■■■■■■■■■■■
0.1 │■■■■■■■■■■■■■■■■■■■■
    └────────────────────────► RPM
      200   400   600   800
```

**Key insight:** At low RPM, motors deliver more torque. As speed increases, torque drops due to inductive effects in the motor windings.

### How Torque is Calculated

The load cell measures force on the brake arm. Torque is calculated as:

$$\tau = F \times r$$

Where:
- $\tau$ = Torque (Nm)
- $F$ = Force measured by load cell (N) = mass (kg) × 9.81 m/s²
- $r$ = Brake arm length (m)

In the code ([src/Dyno_ClosedLoop.ino](../src/Dyno_ClosedLoop.ino)):
```cpp
#define ARM_LENGTH_METERS  0.1f   // Measure your brake arm!
#define GRAVITY            9.80665f

float getTorque() {
  return LoadCell.get_units(1) * GRAVITY * ARM_LENGTH_METERS;
}
```

**Important:** You must measure your brake arm length and update `ARM_LENGTH_METERS` for accurate torque readings!

### Why PID Control Matters

Without PID control, when you apply the brake:
- Motor slows down uncontrollably
- You don't know what RPM you're measuring at

With PID control:
- Set exact RPM (e.g., 500 RPM)
- Apply brake → PID fights to maintain 500 RPM
- You know precisely: "At 500 RPM, max torque = X Nm"

## Recording Data and Creating the Curve

### Method 1: Python/Jupyter Notebook (Recommended)

A complete Jupyter notebook is provided in `tools/dyno_control.ipynb` with:
- Serial connection management
- Interactive buttons for running tests
- Automated multi-RPM curve generation
- Real-time plotting

**Setup:**
```bash
pip install pyserial matplotlib ipywidgets jupyter
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
- Session → Logging → "All session output"
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
2. Insert → Chart → Scatter plot

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

# Part 2: Technical Guide - Speed Control

This section is a detailed technical walkthrough of the PID-based system for constant speed control.

## System Architecture

```
┌─────────────┐
│   AS5600    │ ──► Measures actual RPM
│  (Sensor)   │
└─────────────┘
       │
       ├──► PID Controller ──► Adjusts motor speed
       │
┌─────────────┐
│ Test Motor  │ ──► Runs at constant RPM
│  (X-axis)   │
└─────────────┘
```

### How It Works

1. **RPM Setpoint:** The user sets the desired speed (e.g., `setRPM 100`).
2. **AS5600 Measures:** The sensor reads the actual motor speed continuously (approximately every 10 ms).
3. **PID Calculates Error:** The controller compares the setpoint with the actual value (process value).
4. **Speed is Adjusted:** Motor speed is increased or decreased to minimize the error.
5. **Loop Repeats:** This continuous adjustment maintains a constant speed even when the motor is loaded by the brake.

## Understanding PID Control

The core of the dynamometer is a PID controller that ensures the test motor maintains a constant speed regardless of how much the brake loads it. PID stands for **Proportional + Integral + Derivative**.

An error value (`Error`) is continuously calculated as the difference between the desired `Setpoint` and the actual measured speed (`Process Value`). This error is then fed into the controller's three parts (P, I, and D), whose combined result becomes a `Control Signal` to the motor.

```
                    ┌─────────────────────────────────┐
                    │         PID Logic               │
                    │                                 │
  Error = SP - PV ──┼──► P = Kp × Error              │
                    │                                 │
                    │──► I = Ki × ∫Error dt          │──► Control Signal
                    │                                 │      to Motor
                    │──► D = Kd × d(Error)/dt        │
                    │                                 │
                    └─────────────────────────────────┘
```

* **P (Proportional):** Reacts to **current error**.
    * If the RPM is too low, speed is increased proportionally to the error magnitude.
    * Gives a fast response but can cause the system to oscillate around the target value.

* **I (Integral):** Eliminates **steady-state error**.
    * Accumulates the error over time. If the motor constantly sits 5 RPM below target, the I-term will grow and provide the extra "push" needed to reach the setpoint.

* **D (Derivative):** Dampens **oscillations**.
    * Predicts future error based on how fast the error is changing.
    * This "brakes" the system when it approaches the target and prevents overshoot.

### Default Parameters in Code

```cpp
Kp = 50.0   // Proportional gain
Ki = 10.0   // Integral gain
Kd = 1.0    // Derivative gain
```

These are starting values. **You MUST tune them** to fit your specific motor and mechanical setup!

## PID Tuning Guide

Adjust parameters in real-time with the command `tune <p> <i> <d>`. Example: `tune 50 10 1`.

### Step-by-Step Process

#### 1. Start with P-Only Control

Zero the I and D terms and find a good P value.
```
tune 10 0 0         # Kp=10, Ki=0, Kd=0
setRPM 100
autoStatus true
```
**Observe:**
* Does the motor reach the setpoint?
* Does it oscillate (swing over and under the target)?

**Adjust:**
* **Too slow response:** Increase `Kp` (try `20`, `30`, `50`).
* **Oscillates:** Decrease `Kp` (try `5`, `8`).

**Goal:** Find a `Kp` value where the motor responds quickly but only oscillates a little.

---

#### 2. Add I to Eliminate Steady-State Error

Use your `Kp` value and add a small `Ki`.
```
tune 30 5 0         # Use your Kp, add small Ki
```
**Observe:**
* Is the last bit of error corrected so the motor lands exactly at setpoint?
* Is there too much overshoot during load changes?

**Adjust:**
* **Still a steady-state error:** Increase `Ki` (try `10`, `15`).
* **Too much overshoot:** Decrease `Ki` (try `2`, `3`).

**Goal:** The motor should stabilize exactly at the setpoint with no remaining error.

---

#### 3. Add D to Reduce Oscillations

Add a small `Kd` to dampen the system.
```
tune 30 5 1         # Add small Kd
```
**Observe:**
* Do the oscillations decrease?
* Is the response smoother when the brake is applied?

**Adjust:**
* **Still oscillating:** Increase `Kd` (try `2`, `5`).
* **Too sluggish/slow response:** Decrease `Kd` (try `0.5`, `0.1`).

**Goal:** A smooth, fast response with minimal overshoot.

### Tuning Tips

* **If the motor is unstable (severe oscillations):**
    1. Stop immediately: `stop`
    2. Halve all values: `tune 15 2.5 0.5`
    3. Restart tuning from step 1.

* **If the motor doesn't reach setpoint:**
    * `Kp` is too low → Increase it.
    * Check for mechanical obstructions.

* **If the RPM constantly sits off by a certain value:**
    * `Ki` is too low → Increase it.

## Test Procedures

### Test 1: Stability Without Load

```
setRPM 50
autoStatus true
```
Wait 10 seconds. Check that the error between setpoint and actual value is:
- `< 2%`: Good.
- `2-5%`: Acceptable, but can be improved.
- `> 5%`: Poor, requires better PID tuning.

### Test 2: Response Under Load

```
setRPM 100
# Wait until RPM is stable
brakeApply 500 # Apply brake gradually
```
**Observe:**
- RPM should drop slightly and then quickly recover.
- **Recovery time < 2 seconds:** Good.
- **Never recovers:** `Ki` is too low or brake is too strong.

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

### Motor Speed Fluctuates Severely
* **Problem:** PID parameters (`Kp`, `Ki`, `Kd`) are too high.
* **Solution:**
    1. Stop the motor (`stop`).
    2. Reduce all values (`tune 10 0 0`).
    3. Follow the tuning guide from the beginning.

## Advanced: Code Modifications

### Change Steps Per Revolution (Microstepping)
Microstepping is configured at the top of `Dyno_ClosedLoop.ino`:
```cpp
#define TEST_MOTOR_STEPS_PER_REV    3200   // 200 (steps/rev) * 16 (microsteps)
#define BRAKE_MOTOR_STEPS_PER_REV   3200
```
The `SpeedController` class automatically uses this value for correct RPM calculation.

### Adjust Update Frequency
The PID loop updates at approximately 100 Hz (every 10 ms). This is controlled in `SpeedController.h`:
```cpp
if (dt >= 0.01) {  // Update at max 100 Hz
```
A higher frequency gives better control but requires more processing power.

---

# References

- [RAMPS 1.4 Documentation](https://reprap.org/wiki/RAMPS_1.4) - RepRap Wiki
- [AS5600 Datasheet](https://ams.com/as5600) - ams AG
- [HX711 Datasheet](https://cdn.sparkfun.com/datasheets/Sensors/ForceFlex/hx711_english.pdf) - Sparkfun
- [AccelStepper Library](https://www.airspayce.com/mikem/arduino/AccelStepper/) - Mike McCauley
