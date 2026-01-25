# Project Background

## Inspiration: Engineer Bo's Dynamometer

This project is inspired by the YouTube video ["Finding the Best NEMA17 Stepper Motor"](https://www.youtube.com/watch?v=MNTuMiNC2TU) by Engineer Bo.

In the video, Engineer Bo builds a dynamometer to answer a simple question: **which NEMA 17 stepper motor is actually the best?** Datasheets list specifications, but real-world performance varies significantly based on the motor-driver combination and operating conditions.

### What Engineer Bo Built

Engineer Bo's dynamometer uses:
- A **bicycle brake** to apply controlled load to the test motor
- An **HX711 load cell** to measure braking force (converted to torque)
- An **optical encoder** to measure motor RPM
- Manual test procedures to generate torque-speed curves

He tested motors from LDO, StepperOnline, Usongshine, and others with different drivers (TMC2209, TB6600, TMC5160) at various voltages.

### Key Finding

There is no single "best" motor. Performance depends on:
- **Speed range** - Some motors excel at low RPM, others at high RPM
- **Driver choice** - The same motor performs differently with different drivers
- **Voltage** - Higher voltage generally improves high-speed performance

This is why you need a dynamometer: to test your specific motor-driver combination for your application.

## Our Project: Dyno

We built upon Engineer Bo's concept with several improvements:

| Feature | Engineer Bo | Our Dyno |
|---------|-------------|----------|
| Speed measurement | Optical encoder | AS5600 magnetic sensor (contactless, dust-resistant) |
| Speed control | Manual | **PID closed-loop** (maintains constant RPM under load) |
| Interface | Scripts | Serial commands (`setRPM`, `brakeApply`, etc.) |
| Automation | Limited | Built-in calibration, status monitoring |

### Purpose

The Dyno generates **torque-speed curves** for stepper motors:

1. Set a constant RPM (e.g., `setRPM 500`)
2. Gradually apply brake load (`brakeApply`)
3. The PID controller maintains speed while torque increases
4. Record the maximum torque before the motor stalls
5. Repeat at different RPM values

The result is a curve showing how much torque the motor can deliver at each speed - essential data for selecting motors for 3D printers, CNC machines, or any motion control application.

## References

- **Video:** [Engineer Bo - Finding the Best NEMA17 Stepper Motor](https://www.youtube.com/watch?v=MNTuMiNC2TU)
- **Article:** [Hackaday - Putting Some Numbers On Your NEMAs](https://hackaday.com/2024/07/02/putting-some-numbers-on-your-nemas/)
- **Project:** [Hackster.io - Dynamometer for Stepper Motors](https://www.hackster.io/motor-torque-test-bench-team/dynamometer-for-stepper-motors-84864e)
