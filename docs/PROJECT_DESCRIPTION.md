# Dynamometer for Stepper Motor Testing

## Project Overview

This project involves constructing an advanced dynamometer for characterizing the performance of stepper motors and BLDC motors. The system uses **closed-loop PID control** to maintain a constant motor speed even under varying brake load, while simultaneously measuring the applied torque. The project is inspired by Engineer Bo's YouTube video.

**Inspiration:** [Engineer Bo - "Finding NEMA 17"](https://www.youtube.com/watch?v=MNTuMiNC2TU)

**Project Goal:** Generate torque-speed curves for stepper motor-driver combinations through systematic testing under controlled load, with active RPM regulation of the test motor.

---

## System Overview

The dynamometer consists of two main motor systems and a sensor system working together to perform performance measurements:

### 1. Test Motor (Motor Under Test - MUT)
- **Function:** The motor being tested, whose speed is actively **PID-controlled** to maintain a constant value.
- **Type:** NEMA 17 stepper motor or BLDC motor
- **Performance:** Up to 0.8 Nm torque, max 3000 RPM (target)
- **Mounting:** Easy swap of test motors should be possible

### 2. Brake Motor (Brake Actuator)
- **Function:** Pulls the cable that activates the brake progressively and in a controlled manner.
- **Type:** NEMA 17 stepper motor
- **Mechanism:** Trapezoidal screw that converts rotation to linear motion
- **Controlled Load:** Gradually increases brake torque. Controlled via serial commands.

### 3. Sensor System
- **Speed:** AS5600 magnetic rotation sensor for real-time measurement of the test motor's RPM.
- **Torque:** Load cell with HX711 amplifier for real-time measurement of brake torque.

---

## Operating Principle

### Test Procedure (with Closed-Loop Control)

1. **Initialization:**
    - Reset the brake with `brakeHome`.
    - Tare (zero) the load cell with `tare`.
    - Start the test motor and accelerate to the desired constant RPM (e.g., 1000 RPM) with the command `setRPM 1000`. The PID controller activates and ensures the motor maintains this speed.
    - Allow the system to stabilize.

2. **Progressive Brake Application:**
    - Gradually activate the brake motor with commands like `brakeApply 100`, which increases the braking force.
    - The PID controller for the test motor compensates for the increased load by adjusting the test motor's step frequency to maintain the set RPM.

3. **Torque Measurement & Data Collection:**
    - During braking, continuously record:
        - Actual RPM from AS5600 (actively regulated to setpoint).
        - Brake torque (Nm) from the load cell.
    - Data can be logged via `autoStatus true` or manual `status`/`readTorque` commands.

4. **Maximum Torque Identification:**
    - Increase brake force until the PID controller can no longer maintain the set RPM (motor loses steps or stalls). The highest measured torque at this RPM is logged as the motor's maximum torque.
    - End the test with `stop` and release the brake with `brakeRelease` or `brakeHome`.

5. **Torque-Speed Curve:** Repeat the process at different RPM values to map the motor's complete torque-speed curve.

---

## Mechanical Components

### Brake System

**Brake Disc & Caliper**
- Standard bicycle brake components
- One brake caliper per group with torsion spring
- Brake disc mounted on the test motor shaft

**Brake Arm**
- **Function:** Converts brake force to measurable force on the load cell
- **Dimensioning:** Must handle full brake force (safety factor ≥ 3)
- **Material:** Must withstand repeated loading

**Cable System**
- **Function:** Transfers pulling force from trapezoidal screw to brake caliper
- **Dimensioning:** Cable force should be calculated and measured
- **Critical Parameter:** Determines maximum brake torque capacity

**Trapezoidal Screw**
- **Function:** Converts brake motor rotation to linear cable pull
- **Dimensioning:** Screw should be sized so NEMA 17 motor can handle the cable force
- **Advantage:** Self-locking (holds position without power)

### Construction Materials

**Frames & Structure**
- 20×20 mm aluminum profiles from [Alucon](https://alucon.se)
- Long pieces: approx. 400 mm
- Short pieces: approx. 250 mm
- Mounted on wooden board (available on campus)

**Shafts**
- Available dimensions: 5 mm and 8 mm
- Selected based on load analysis

**3D-Printed Parts**
- **Material:** PLA (CV) or engineering materials from JTH (carbon fiber reinforced PETG)
- **Dimensioning:**
  - Safety factor ≥ 3 against yield strength
  - Max displacement ≤ 1 mm
- **Temperature Requirements:** Motors can reach up to 100°C → material selection is critical

---

## Sensor System

### AS5600 Magnetic Rotation Sensor
- **Type:** Contactless magnetic encoder (12-bit absolute position)
- **Function:** Measures the test motor's actual rotational speed, used as feedback in the PID controller.
- **Advantage:** No mechanical contact (better than optical encoder - no dust problems)
- **Installation:**
  - Magnet mounted on the rotation shaft
  - Sensor placed a few mm from the magnet (contactless)
- **Output:** 12-bit angle position (0-4095) → internally converted to RPM.
- **Communication:** I2C.

### Load Cell
- **Type:** 5 kg full Wheatstone bridge load cell
- **Amplifier:** HX711 ADC module for signal processing
- **Function:** Measures force from the brake arm when brake is applied.
- **Installation:** Mounted where the brake arm presses during brake application.
- **Output:** Digitized signal via HX711 → serial communication to Arduino.
- **Calibration:**
  - Requires calibration with known weights (interactive calibration routine is built-in).
  - Calibration factors are saved in the code.
  - Taring (zeroing) before each test.

---

## Electronics & Mechatronics

### Control System
- **Microcontroller:** Arduino Mega with RAMPS 1.4 shield.
- **Test Motor Drive:** Stepper motor driver (e.g., TMC2209, A4988) on X-axis, controlled by PID controller.
- **Brake Motor Drive:** Stepper motor driver for trapezoidal screw on Y-axis.
- **Sensors:**
  - AS5600: I2C communication.
  - HX711: Digital pins.
- **Serial Communication:** 115200 baud for commands and data transfer.

### Commands (via Serial)
Command handling is done via a `SerialCommander` implementation. Below is an overview of available commands (for complete list, type `help` in Serial Monitor):
```
setRPM <value>          # Set PID target RPM for test motor (0 to disable)
brake <steps>           # Move brake motor (positive = apply, negative = release)
brakeApply [steps]      # Apply brake with specified steps (default: 500)
brakeRelease [steps]    # Release brake with specified steps (default: 500)
brakeHome               # Reset brake to zero position
readTorque              # Read current torque from load cell
tare                    # Zero the load cell
calibrate               # Start interactive load cell calibration
status                  # Show current system status (RPM, Torque, brake position)
autoStatus [true/false] # Toggle automatic status display
tune <kp> <ki> <kd>     # Adjust PID parameters
stop                    # Emergency stop: stops all motors
help                    # Show this help list
```

---

## Safety & Usability

### Finger Guard
- **Requirement:** Cover should protect rotating parts to avoid pinch injuries
- **Design:** Designed from user perspective
- **Material:** Transparent plastic to allow test observation

### Electronics Enclosure
- **Function:** Protect electronics, organize cables
- **Design:** Accessible for troubleshooting

### Usability
- **Quick Motor Swap:** Test motor should be easily replaceable
- **BLDC Compatibility:** Different screw profiles → adaptable mounts
- **Ergonomics:** Easy access to all components

---

## Requirements Specification

### Performance Requirements
- ✅ Test NEMA 17 stepper motors and BLDC motors
- ✅ Max torque: 0.8 Nm
- ✅ Max RPM: 3000 RPM
- ✅ Generate torque-speed curves
- ✅ **PID control for constant RPM during testing.**

### Mechanical Dimensioning Requirements
1. **Brake Arm:** Handles brake torque with safety factor ≥ 3
2. **Cable System:** Cable force calculated and verified experimentally
3. **Trapezoidal Screw:** Dimensioned for brake motor capacity
4. **Plastic Parts:**
   - Safety factor ≥ 3 against yield strength
   - Max displacement ≤ 1 mm
5. **Torsion Spring (brake caliper):** Stiffness should be measured and included in calculations
6. **Temperature Resistance:** Materials withstand 100°C (motors under load)

### Structural Requirements
- ✅ 20×20 mm aluminum profiles from Alucon
- ✅ Wooden board as base
- ✅ Shafts: 5 mm or 8 mm depending on load

---

## Comparison: Engineer Bo vs. Our Project

### Engineer Bo's Setup
- **Sensor:** Optical encoder (mechanical, sensitive to dust)
- **Load Cell:** HX711-based
- **Control:** Manual/script-based
- **Tested Motors:** LDO Motors, STEPPERONLINE, Usongshine, JKONGMOTOR
- **Drivers:** TMC2209 (24V), TB6600, TMC5160

*See `background.md` for complete summary of Engineer Bo's results.*

### Our Project - Improvements
- **Sensor:** AS5600 magnetic (contactless, more reliable, no dust problems) for RPM.
- **Load Cell:** 5 kg full Wheatstone bridge with HX711 amplifier for Torque.
- **Control:** Advanced **PID-based Closed-Loop speed control** for the test motor.
- **Automation:** `SerialCommander`-based command-driven test procedure and interactive calibration.
- **Usability:** Easy motor swap, clear commands, real-time data (RPM & Torque).

**Our implementation combines Engineer Bo's proven concept with more modern sensors and a robust PID-controlled algorithm for superior test precision.**

---

## Technical Background

### Torque-Speed Curves for Stepper Motors

Stepper motors have characteristic torque-speed curves that depend on:
- **Motor Design:** Inductance, resistance, number of turns, magnet strength
- **Driver Circuit:** Voltage, current, microstepping resolution, current regulation
- **Combination:** Same motor + different drivers = different curves

**The curve shows:**
- **Low RPM:** Higher holding torque (motor can resist more force)
- **High RPM:** Torque drops (inductive reactance limits current)
- **Max Torque:** Varies with RPM and driver

**Practical Use:**
- Choose the right motor-driver combination for your application
- Avoid running motors beyond their capacity
- Optimize speed vs. torque for 3D printers, CNC, etc.

---

## References and Inspiration

**Engineer Bo's Dynamometer Project:**
- [YouTube - "Finding the Best NEMA17 Stepper Motor"](https://www.youtube.com/watch?v=MNTuMiNC2TU)
- [Hackaday Article - "Putting Some Numbers On Your NEMAs"](https://hackaday.com/2024/07/02/putting-some-numbers-on-your-nemas/)
- [Hackster.io - "Dynamometer for Stepper Motors"](https://www.hackster.io/motor-torque-test-bench-team/dynamometer-for-stepper-motors-84864e)
- Design uses bicycle brake, load cell, and optical encoder
- Tested 5 different NEMA 17 motors with TMC2209 and TB6600 drivers
- **See `background.md` for detailed summary**

**Suppliers:**
- [Alucon - Aluminum Profiles](https://alucon.se)
- [Amazon - Load Cells (HX711-compatible)](https://www.amazon.se/dp/B0895KP2FH)

**Sensor Documentation:**
- AS5600 implementation and tutorial: `AS5600_Test/`
- I2C protocol explanation: `AS5600_Test/I2C_explanation.md`

---

## Project Status

This project is under active development as part of the course **Konstruktion1 - HT25** at JTH.
**Current Status:** Mechatronics implementation with PID-based closed-loop speed control and torque measurement is fully functional.
**Next Steps:** Mechanical construction and dimensioning.
