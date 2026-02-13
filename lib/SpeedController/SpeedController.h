/*
 * SpeedController - PID-based constant speed controller for stepper motors
 *
 * Uses AS5600 magnetic sensor feedback to maintain constant RPM even under
 * varying load conditions (e.g., brake application in dynamometer testing)
 *
 * Uses AS5600 library's getAngularSpeed() for RPM measurement.
 *
 * Usage:
 *   SpeedController controller(&stepper, &as5600);
 *   controller.begin();
 *   controller.setTargetRPM(100);
 *
 *   void loop() {
 *     controller.update();
 *   }
 */

#ifndef SPEED_CONTROLLER_H
#define SPEED_CONTROLLER_H

#include <Arduino.h>
#include <AccelStepper.h>
#include "AS5600.h"

class SpeedController {
public:
  // Constructor
  SpeedController(AccelStepper* motor, AS5600* sensor, int stepsPerRev = 200)
    : motor(motor), sensor(sensor), stepsPerRevolution(stepsPerRev),
      targetRPM(0), currentRPM(0),
      enabled(false), integral(0), lastError(0), lastUpdateTime(0) {

    // Default PID parameters (tune these for your system)
    kp = 50.0;    // Proportional gain
    ki = 10.0;    // Integral gain
    kd = 1.0;     // Derivative gain

    // Anti-windup limit for integral term
    integralLimit = 1000.0;
  }

  // Initialize the controller
  void begin() {
    sensor->resetCumulativePosition();
    enabled = false;
  }

  // Set target RPM
  void setTargetRPM(float rpm) {
    targetRPM = rpm;
    if (rpm > 0 && !enabled) {
      enabled = true;
      integral = 0;  // Reset integral when starting
      lastError = 0;
    } else if (rpm == 0) {
      enabled = false;
      motor->setSpeed(0);
    }
  }

  // Get current RPM
  float getCurrentRPM() {
    return currentRPM;
  }

  // Get target RPM
  float getTargetRPM() {
    return targetRPM;
  }

  // Set PID parameters
  void setPID(float p, float i, float d) {
    kp = p;
    ki = i;
    kd = d;
  }

  // Main update function - call this in loop()
  void update() {
    // Update RPM at fixed interval to avoid aliasing from AS5600 quantization
    unsigned long now = millis();
    if (now - lastRpmTime >= rpmInterval) {
      float degPerSec = sensor->getAngularSpeed(AS5600_MODE_DEGREES);
      currentRPM = -degPerSec / 6.0;  // Negate to match motor direction
      lastRpmTime = now;
    }

    if (!enabled) {
      motor->runSpeed();
      return;
    }

    // PID control
    now = millis();
    float dt = (now - lastUpdateTime) / 1000.0;  // Convert to seconds

    if (dt >= 0.01) {  // Update at ~100 Hz max
      float error = targetRPM - currentRPM;

      // Proportional term
      float P = kp * error;

      // Integral term with anti-windup
      integral += error * dt;
      if (integral > integralLimit) integral = integralLimit;
      if (integral < -integralLimit) integral = -integralLimit;
      float I = ki * integral;

      // Derivative term
      float D = 0;
      if (dt > 0) {
        D = kd * (error - lastError) / dt;
      }

      // Calculate output (steps per second)
      float output = P + I + D;

      // Convert RPM to steps/sec
      float baseSpeed = targetRPM * stepsPerRevolution / 60.0;  // steps/sec
      float adjustedSpeed = baseSpeed + output;

      // Limit speed to reasonable range
      if (adjustedSpeed > 10000) adjustedSpeed = 10000;
      if (adjustedSpeed < -10000) adjustedSpeed = -10000;

      motor->setSpeed(adjustedSpeed);

      lastError = error;
      lastUpdateTime = now;
    }

    motor->runSpeed();
  }

  // Enable/disable controller
  void enable() {
    enabled = true;
    integral = 0;
    lastError = 0;
  }

  void disable() {
    enabled = false;
    motor->setSpeed(0);
  }

  bool isEnabled() {
    return enabled;
  }

  // Get PID terms for debugging
  void getDebugInfo(float &p, float &i, float &d, float &err) {
    float error = targetRPM - currentRPM;
    p = kp * error;
    i = ki * integral;
    d = kd * lastError;
    err = error;
  }

private:
  AccelStepper* motor;
  AS5600* sensor;

  int stepsPerRevolution;  // Motor steps per revolution (accounts for microstepping)

  float targetRPM;
  float currentRPM;

  // PID parameters
  float kp, ki, kd;
  float integral;
  float lastError;
  float integralLimit;

  unsigned long lastUpdateTime;
  unsigned long lastRpmTime = 0;
  unsigned long rpmInterval = 100;  // ms between RPM samples
  bool enabled;
};

#endif
