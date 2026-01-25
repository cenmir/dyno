/*
 * SpeedController - PID-based constant speed controller for stepper motors
 *
 * Uses AS5600 magnetic sensor feedback to maintain constant RPM even under
 * varying load conditions (e.g., brake application in dynamometer testing)
 *
 * Features:
 * - PID control for accurate speed regulation
 * - RPM calculation from AS5600 angle readings
 * - Configurable PID parameters
 * - Automatic speed adjustment
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
      lastAngle(0), lastTime(0), enabled(false),
      integral(0), lastError(0) {

    // Default PID parameters (tune these for your system)
    kp = 50.0;    // Proportional gain
    ki = 10.0;    // Integral gain
    kd = 1.0;     // Derivative gain

    // Anti-windup limit for integral term
    integralLimit = 1000.0;
  }

  // Initialize the controller
  void begin() {
    lastAngle = sensor->readAngle();
    lastTime = millis();
    revolutions = 0;
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
    if (!enabled) {
      motor->runSpeed();
      return;
    }

    // Calculate current RPM
    calculateRPM();

    // PID control
    unsigned long now = millis();
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

  // RPM calculation
  int lastAngle;
  unsigned long lastTime;
  unsigned long lastUpdateTime;
  long revolutions;
  bool enabled;

  // Calculate RPM from AS5600 readings
  void calculateRPM() {
    int currentAngle = sensor->readAngle();
    unsigned long currentTime = millis();

    // Detect full revolution (angle wraps from 4095 to 0)
    int angleDiff = currentAngle - lastAngle;

    // Handle wrap-around
    if (angleDiff < -2048) {
      // Wrapped forward (0 -> 4095)
      revolutions++;
    } else if (angleDiff > 2048) {
      // Wrapped backward (4095 -> 0)
      revolutions--;
    }

    // Calculate time difference in minutes
    float timeDiff = (currentTime - lastTime) / 60000.0;  // ms to minutes

    if (timeDiff > 0.05) {  // Update RPM every 50ms minimum
      // Calculate total angle change in degrees
      float totalAngle = (revolutions * 4096.0 + currentAngle) - lastAngle;
      float degrees = (totalAngle / 4096.0) * 360.0;

      // Calculate RPM
      currentRPM = degrees / (360.0 * timeDiff);

      // Reset for next calculation
      lastAngle = currentAngle;
      lastTime = currentTime;
      revolutions = 0;
    }
  }
};

#endif
