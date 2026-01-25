# AS5600 Code Explanation

**NOTE:** This document describes how to use the AS5600 sensor in a simple test environment (`examples/AS5600_Test/AS5600_Test.ino`). In the complete dynamometer project (`src/Dyno_ClosedLoop.ino`), the AS5600 sensor is integrated to measure the test motor's RPM and is actively used in PID control to maintain a constant speed.

The code is written for an Arduino microcontroller and its purpose is to read the rotational position from an AS5600 magnetic sensor and print the angle in degrees to the computer.

Here is an explanation of the different parts:

## Includes and Initialization

* `#include "AS5600.h"`: Includes a library that simplifies communication with the AS5600 sensor.
* `AS5600 as5600;`: Creates an object of the sensor so you can call functions from the library.

## `setup()`

This function runs once when the Arduino starts.

* `Serial.begin(115200);`: Starts serial communication with the computer so the Arduino can send data (e.g., angle values) that can be read in "Serial Monitor".
* `Wire.begin();`: Initializes the I2C communication protocol, which is the language that the Arduino and AS5600 sensor use to talk to each other.
* `as5600.begin(3);`: Initializes the sensor and tells it that a specific pin (pin 3) is used to determine the rotation direction.
* `Serial.println(as5600.isConnected());`: Checks if the Arduino has contact with the sensor and prints `true` or `false` to the computer.
* `delay(3000);`: Pauses the program for 3 seconds.

## `loop()`

This function runs in an infinite loop after `setup()` is complete.

* `int rawAngle = as5600.readAngle();`: Reads the raw angle data from the sensor. This is a value between 0 and 4095, because the sensor has a 12-bit resolution (2^12 = 4096).
* `float angle = map(rawAngle, 0, 4095, 0, 36000) / 100.0;`: Converts the raw value to degrees.
    * `map(...)`: Scales the value from the range 0-4095 to 0-36000.
    * `/ 100.0`: Divides by 100 to get an angle in degrees with two decimal places of precision (0.00 to 360.00).
* `Serial.println(angle);`: Sends the calculated angle to the computer.
* `delay(100);`: Pauses for 100 milliseconds before the loop starts over. This results in the angle being read and sent approximately 10 times per second.

## Complete Code

```cpp
#include "AS5600.h"
#include <Wire.h>

AS5600 as5600;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  as5600.begin(3);

  Serial.print("AS5600 connected: ");
  Serial.println(as5600.isConnected());

  delay(3000);
}

void loop() {
  int rawAngle = as5600.readAngle();
  float angle = map(rawAngle, 0, 4095, 0, 36000) / 100.0;

  Serial.println(angle);
  delay(100);
}
```

## Hardware Setup

1. Connect the AS5600 sensor to the Arduino:
   - VCC -> 5V
   - GND -> GND
   - SDA -> A4 (or SDA pin on Mega: pin 20)
   - SCL -> A5 (or SCL pin on Mega: pin 21)

2. Mount a diametrically magnetized magnet on the rotating shaft, 1-3 mm from the sensor.

3. Upload the code and open Serial Monitor at 115200 baud.

4. Rotate the magnet and observe the angle values changing from 0.00 to 360.00.
