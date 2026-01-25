# Understanding I2C Communication

## What is I2C?

I2C (Inter-Integrated Circuit, pronounced "I-squared-C") is a serial communication protocol that allows multiple devices to communicate using only **two wires**:

- **SDA (Serial Data)** - Carries the actual data
- **SCL (Serial Clock)** - Synchronizes the communication

This makes wiring simple: all devices share the same two wires, and the Arduino (master) selects which device to talk to by calling its address.

## How It Works

1. **Arduino sends address:** The Arduino broadcasts an address (e.g., `0x36` for AS5600)
2. **Device responds:** Only the device with that address responds
3. **Data transfer:** Arduino and device exchange data, synchronized by the clock
4. **Done:** Arduino ends the transmission

```
Arduino (Master)          AS5600 (Slave)
     |                         |
     |--- "Address 0x36" ----->|  (AS5600 responds)
     |--- "Give me angle" ---->|
     |<---- angle data --------|
     |                         |
```

## Technical Details

| Term | Description |
|------|-------------|
| **SDA** | Serial Data Line - bidirectional data |
| **SCL** | Serial Clock Line - timing signal from master |
| **Master** | Device that controls the bus (Arduino) |
| **Slave** | Device that responds to requests (sensors) |
| **Address** | 7-bit identifier (0x00 to 0x7F) |

## AS5600 Address Limitation

The AS5600 has a **fixed I2C address of `0x36`** that cannot be changed. This means:

- You can only connect **one AS5600** per I2C bus
- If you need multiple AS5600 sensors, you must use:
  - An **I2C multiplexer** (e.g., TCA9548A)
  - Separate I2C buses (some Arduinos have multiple)
  - A different sensor with configurable address

This is a hardware limitation of the AS5600 chip - there are no address pins to change it.

## I2C on Arduino

```cpp
#include <Wire.h>

void setup() {
  Wire.begin();  // Initialize as master
}

void loop() {
  // Talk to AS5600 at address 0x36
  Wire.beginTransmission(0x36);
  Wire.write(0x0C);              // Angle register address
  Wire.endTransmission();

  Wire.requestFrom(0x36, 2);     // Request 2 bytes
  int highByte = Wire.read();
  int lowByte = Wire.read();

  int angle = (highByte << 8) | lowByte;  // 12-bit angle (0-4095)
}
```

## Wiring

```
Arduino Mega          AS5600
-----------          -------
    5V  ------------> VCC
   GND  ------------> GND
   D20 (SDA) -------> SDA
   D21 (SCL) -------> SCL
```

**Note:** Most AS5600 breakout boards have built-in pull-up resistors. If using bare chips, add 4.7k resistors from SDA and SCL to VCC.

## Common Problems

| Problem | Cause | Solution |
|---------|-------|----------|
| Device not found | Wrong wiring or no power | Check connections, verify 5V |
| Address conflict | Two devices with same address | Use I2C multiplexer |
| Corrupted data | Long wires or noise | Shorter cables, add pull-ups |
| Intermittent readings | Poor connections | Check solder joints |

## Scanning for I2C Devices

If you're unsure what address a device uses, run this scanner:

```cpp
#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Wire.begin();

  Serial.println("Scanning I2C bus...");
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      Serial.println(addr, HEX);
    }
  }
  Serial.println("Scan complete.");
}

void loop() {}
```

For AS5600, you should see: `Found device at 0x36`
