# SpeedController Code Explanation

This document provides a detailed explanation of the `SpeedController` class (`lib/SpeedController/SpeedController.h`), which implements PID-based constant speed control for stepper motors.

## Overview

The SpeedController maintains a constant motor RPM even when load is applied (e.g., during brake testing). It does this by:
1. Reading actual motor speed from the AS5600 magnetic sensor
2. Comparing it to the target RPM (setpoint)
3. Using a PID algorithm to calculate speed adjustments
4. Continuously correcting the motor speed

## Class Structure

```cpp
class SpeedController {
public:
  // Constructor - sets up the controller
  SpeedController(AccelStepper* motor, AS5600* sensor, int stepsPerRev = 200);

  void begin();                      // Initialize the controller
  void setTargetRPM(float rpm);      // Set desired speed
  void update();                     // Main control loop (call in loop())
  float getCurrentRPM();             // Get measured speed
  float getTargetRPM();              // Get target speed
  void setPID(float p, float i, float d);  // Tune PID parameters

private:
  void calculateRPM();               // Internal RPM calculation
  // ... member variables ...
};
```

## Constructor

```cpp
SpeedController(AccelStepper* motor, AS5600* sensor, int stepsPerRev = 200)
```

**Parameters:**
- `motor`: Pointer to the AccelStepper motor object
- `sensor`: Pointer to the AS5600 sensor object
- `stepsPerRev`: Steps per revolution (default 200 for full-step)

**What it does:**
- Stores references to motor and sensor
- Initializes PID parameters to default values (Kp=50, Ki=10, Kd=1)
- Sets anti-windup limit to prevent integral term from growing too large

```cpp
// Default PID parameters (tune these for your system)
kp = 50.0;    // Proportional gain
ki = 10.0;    // Integral gain
kd = 1.0;     // Derivative gain

// Anti-windup limit for integral term
integralLimit = 1000.0;
```

## begin() - Initialization

```cpp
void begin() {
  lastAngle = sensor->readAngle();  // Record starting angle
  lastTime = millis();               // Record starting time
  revolutions = 0;                   // Reset revolution counter
  enabled = false;                   // Start disabled
}
```

**What it does:**
- Reads the initial sensor angle for RPM calculation reference
- Records the starting time
- Resets the revolution counter (used for angle wrap-around)
- Starts in disabled state (motor won't move until setTargetRPM is called)

## setTargetRPM() - Setting Speed

```cpp
void setTargetRPM(float rpm) {
  targetRPM = rpm;
  if (rpm > 0 && !enabled) {
    enabled = true;
    integral = 0;    // Reset integral when starting
    lastError = 0;
  } else if (rpm == 0) {
    enabled = false;
    motor->setSpeed(0);
  }
}
```

**What it does:**
- If RPM > 0: Enables the controller and resets the integral term (prevents "wind-up" from previous runs)
- If RPM = 0: Disables the controller and stops the motor

**Why reset integral?** If the integral term accumulated error from a previous run, it would cause the motor to immediately overshoot when restarted.

## update() - The Main Control Loop

This is the heart of the controller and should be called every iteration of `loop()`.

```cpp
void update() {
  if (!enabled) {
    motor->runSpeed();  // Still call runSpeed even when disabled
    return;
  }

  calculateRPM();  // Calculate current RPM from sensor

  // PID control
  unsigned long now = millis();
  float dt = (now - lastUpdateTime) / 1000.0;  // Time delta in seconds

  if (dt >= 0.01) {  // Update at ~100 Hz max
    // ... PID calculation ...
  }

  motor->runSpeed();  // Execute motor movement
}
```

### The PID Calculation

```cpp
float error = targetRPM - currentRPM;

// Proportional term: Immediate response to error
float P = kp * error;

// Integral term: Accumulates error over time
integral += error * dt;
if (integral > integralLimit) integral = integralLimit;    // Anti-windup
if (integral < -integralLimit) integral = -integralLimit;
float I = ki * integral;

// Derivative term: Rate of change of error
float D = 0;
if (dt > 0) {
  D = kd * (error - lastError) / dt;
}

// Calculate total output
float output = P + I + D;
```

**Understanding each term:**

| Term | Formula | Purpose |
|------|---------|---------|
| **P** | `Kp * error` | Immediate reaction to current error |
| **I** | `Ki * sum(error * dt)` | Eliminates steady-state error |
| **D** | `Kd * d(error)/dt` | Dampens oscillations |

### Converting PID Output to Motor Speed

```cpp
// Convert target RPM to steps/sec
float baseSpeed = targetRPM * stepsPerRevolution / 60.0;

// Add PID correction
float adjustedSpeed = baseSpeed + output;

// Limit to safe range
if (adjustedSpeed > 10000) adjustedSpeed = 10000;
if (adjustedSpeed < -10000) adjustedSpeed = -10000;

motor->setSpeed(adjustedSpeed);
```

**Why steps/sec?** AccelStepper's `setSpeed()` expects steps per second, not RPM. The formula is:
```
steps/sec = RPM * steps_per_rev / 60
```

## calculateRPM() - Speed Measurement

This internal function calculates the actual RPM from AS5600 readings.

```cpp
void calculateRPM() {
  int currentAngle = sensor->readAngle();  // 0-4095 (12-bit)
  unsigned long currentTime = millis();

  // Detect wrap-around (full revolution)
  int angleDiff = currentAngle - lastAngle;

  if (angleDiff < -2048) {
    revolutions++;   // Wrapped forward (0 -> 4095)
  } else if (angleDiff > 2048) {
    revolutions--;   // Wrapped backward (4095 -> 0)
  }

  // Calculate RPM every 50ms
  float timeDiff = (currentTime - lastTime) / 60000.0;  // ms to minutes

  if (timeDiff > 0.05) {  // 50ms minimum
    float totalAngle = (revolutions * 4096.0 + currentAngle) - lastAngle;
    float degrees = (totalAngle / 4096.0) * 360.0;

    currentRPM = degrees / (360.0 * timeDiff);

    // Reset for next calculation
    lastAngle = currentAngle;
    lastTime = currentTime;
    revolutions = 0;
  }
}
```

### Understanding Wrap-Around

The AS5600 outputs values 0-4095 (12-bit). When the magnet completes a full rotation:
- Going forward: angle jumps from ~4095 to ~0 (angleDiff < -2048)
- Going backward: angle jumps from ~0 to ~4095 (angleDiff > 2048)

The 2048 threshold (half of 4096) distinguishes wrap-around from normal movement.

### RPM Formula

```
RPM = (angle_change_degrees / 360) / time_minutes
    = degrees / (360 * time_minutes)
```

## Usage Example

```cpp
#include <AccelStepper.h>
#include "AS5600.h"
#include "SpeedController.h"

AccelStepper motor(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
AS5600 as5600;
SpeedController controller(&motor, &as5600, 200);

void setup() {
  motor.setMaxSpeed(10000);
  as5600.begin(3);
  controller.begin();
}

void loop() {
  // Read commands, set target...
  controller.setTargetRPM(100);

  // IMPORTANT: Call update() every loop iteration!
  controller.update();
}
```

## Key Points for Students

1. **Always call `update()` in `loop()`** - The controller needs frequent updates to work properly.

2. **Tune PID for your hardware** - Default values (50, 10, 1) are starting points. Use the `tune` command to adjust.

3. **Mind the update frequency** - The controller runs at ~100 Hz. Faster loops give better control.

4. **Anti-windup is critical** - Without the integral limit, the motor could overshoot dramatically after long errors.

5. **Steps per revolution must match** - If your driver uses microstepping (e.g., 1/16), multiply: `200 * 16 = 3200`.

## Common Issues

| Problem | Cause | Solution |
|---------|-------|----------|
| Motor oscillates | PID gains too high | Reduce Kp, Ki, Kd |
| Motor never reaches target | PID gains too low | Increase Kp first |
| Constant offset from target | Ki too low | Increase Ki |
| Overshoot on speed change | Ki too high or Kd too low | Decrease Ki or increase Kd |
| Jerky movement | Update frequency too low | Check loop() runs fast enough |

## See Also

- [guide.md](guide.md) - Complete user and technical guide with PID tuning process
- [SPEED_CONTROL_GUIDE.md](SPEED_CONTROL_GUIDE.md) - Additional tuning details
- [AS5600_code_explanation.md](AS5600_code_explanation.md) - Sensor code explanation
