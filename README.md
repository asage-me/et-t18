# TTGO T18 ESP32 - VL53L0X Distance Sensor

## Project Overview
This is a PlatformIO project featuring a VL53L0X Time-of-Flight distance sensor that controls LED brightness based on proximity. The closer an object is to the sensor, the brighter the LED becomes.

### VL53L0X Features
- **Distance Range**: 25mm (minimum/touch) to 1000mm (maximum)
- **LED Control**: Inversely proportional brightness (closer = brighter)
- **LED Brightness Range**: 3-125 (50% duty cycle = full brightness)
- **Fast Fade Rate**: ±48 per loop = approximately 0.25 seconds for full fade
- **IR Remote Control**: Toggle sensor/LED on/off with IR remote (GPIO 26)
- **Power Management**: Deep sleep mode with toggle switch on GPIO 27
- **Error Handling**: Returns maximum distance when no valid object detected
- **Library**: VL53L0X_mod (schnoog's modified library with continuous mode)

## Hardware Components

### TTGO T18 ESP32 Board
- **Product Link**: [LILYGO T-Energy TTGO T18](https://github.com/LilyGO/LILYGO-T-Energy)
- **Custom Board**: `ttgo-t18-v3` (defined in `boards/ttgo-t18-v3.json`)
- **Description**: ESP32-based development board with battery management capabilities
- **Key Specifications** (from T18 V3):
  - **Chipset**: ESPRESSIF ESP32-WROVER (240MHz Xtensa dual-core 32-bit LX6 microprocessor)
  - **Flash**: QSPI flash/SRAM, up to 32 MB
  - **SRAM**: 520 kB SRAM (with PSRAM support)
  - **WiFi**: 802.11 b/g/n/e/i (802.11n speed up to 150Mbps), 2.4GHz-2.5GHz
  - **Bluetooth**: Bluetooth v4.2BR/EDR and BLE standard
  - **Power Supply**: USB 5V/1A, 3.7V lithium battery, 1000mA charging current
  - **Working Voltage**: 2.3V-3.6V
  - **Operating Temp**: -40°C ~ +85°C
  - **Interfaces**: SD card, UART, SPI, SDIO, I2C, LED PWM, TV PWM, I2S, IRGPIO, capacitor touch sensor, ADC, DACLNA pre-amplifier
  - **Built-in battery charging circuit**
  - **Multiple GPIO pins available**

### VL53L0X Time-of-Flight Distance Sensor
- **Product Link**: [VL53L0X on Amazon](https://www.amazon.com/dp/B0B1M79WMP)
- **Datasheet**: [ST VL53L0X Datasheet](https://www.st.com/resource/en/datasheet/vl53l0x.pdf)
- **Description**: Laser-ranging module based on Time-of-Flight (ToF) technology
- **Key Features**:
  - I2C communication interface
  - Accurate distance measurement up to 2 meters
  - 940nm VCSEL emitter
  - Operating voltage: 2.6V - 3.5V (module includes voltage regulator for 5V)
  - Controls LED brightness on GPIO 25
  - Project range: 25mm - 1000mm
  - Continuous reading mode for faster response

### IR Receiver Module
- **Description**: Infrared receiver for remote control input
- **Key Features**:
  - Compatible with NEC protocol remotes
  - Connected to GPIO 26
  - Toggle sensor/LED enable/disable with button code 0xFFA25D
  - 2-second debounce prevents accidental toggling
  - Sensor/LED starts in disabled mode by default

## Project Plan

### Functionality

#### VL53L0X Distance Sensor System
1. **Continuous Distance Measurement**: Reads distance via I2C (GPIO 21/22)
2. **LED Brightness Control**: Inversely proportional to distance
   - 25mm or closer = Maximum brightness (125)
   - 1000mm or farther = Minimum brightness (3)
   - Linear mapping between these points
3. **Smooth Transitions**: Fast fade rate (±48 per loop = ~0.25 seconds full fade)
4. **Error Handling**: Returns max distance when sensor reading is invalid

#### IR Remote Control
1. **IR Receiver**: Connected to GPIO 26, monitors for IR signals
2. **Toggle Button**: Button code 0xFFA25D toggles sensor/LED on/off
3. **Default State**: Sensor/LED starts disabled, press IR button to enable
4. **Debounce**: 2-second debounce prevents accidental toggling
5. **Behavior**:
   - Press button = Toggle sensor/LED state (enabled ↔ disabled)
   - When disabled = LED turns off immediately, distance readings stop
   - When enabled = Sensor resumes normal operation

#### Power Management
1. **Toggle Switch**: Connected to GPIO 27 with internal pullup resistor
2. **Deep Sleep Mode**: System enters deep sleep when switch is open (pulled HIGH)
3. **Wake-Up**: System wakes from deep sleep when switch is closed to GND (LOW)
4. **Power Conservation**: Minimal current draw in deep sleep mode
5. **Behavior**:
   - Switch open (pulled HIGH) = Deep sleep, all peripherals off
   - Switch closed to GND (LOW) = Normal operation, sensor active

### Technical Implementation
- **I2C Configuration**: Standard I2C (GPIO 21 SDA, GPIO 22 SCL)
- **PWM Configuration**: 5kHz frequency, 8-bit resolution, PWM channel 0
- **Distance Range**: 25mm (touch/minimum) to 1000mm (far/maximum)
- **LED Brightness**: 3 (minimum) to 125 (max - 50% duty cycle)
- **Fade Rate**: ±48 per loop = very fast response (~0.25 seconds full fade)
- **Update Rate**: 50ms loop delay with smooth brightness transitions
- **Sensor Mode**: Continuous reading mode for faster updates
- **GPIO Pins**:
  - I2C SDA: GPIO 21 (VL53L0X sensor)
  - I2C SCL: GPIO 22 (VL53L0X sensor)
  - LED PWM: GPIO 25 (brightness control)
  - IR Receiver: GPIO 26 (remote control input)
  - Power Switch: GPIO 27 (deep sleep control, internal pullup)

## Wiring Diagram

### VL53L0X to TTGO T18
```
VL53L0X          TTGO T18
--------         --------
VCC       --->   3.3V
GND       --->   GND
SDA       --->   GPIO 21 (SDA)
SCL       --->   GPIO 22 (SCL)
```

### LED Connection
```
TTGO T18         LED
--------         ---
GPIO 25   --->   LED Anode (+ via resistor)
GND       --->   LED Cathode (-)
```

**Note**: Use appropriate current-limiting resistor for your LED (typically 220Ω - 1kΩ for standard LEDs)

### IR Receiver Connection
```
TTGO T18         IR Receiver
--------         -----------
3.3V      --->   VCC
GND       --->   GND
GPIO 26   --->   DATA/OUT
```

**Note**: IR receiver supports multiple button codes for mode selection and on/off toggle.

### Power Switch Connection
```
TTGO T18         Toggle Switch
--------         -------------
GPIO 27   --->   Switch Terminal 1
GND       --->   Switch Terminal 2
```

**Note**: GPIO 27 has internal pullup resistor enabled. Switch open = deep sleep (HIGH), switch closed = normal operation (LOW).

## Finger Schematics

### VL53L0X Sensor Wiring
```
VL53L0X Pin      Wire Color    TTGO T18
-----------      ----------    --------
VCC              Orange        3.3V
GND              White Orange  GND
SDA              Blue          GPIO 21 (SDA)
SCL              Green         GPIO 22 (SCL)
```

### LED Wiring
```
LED Terminal     Wire Color    TTGO T18
------------     ----------    --------
Anode (+)        Brown         GPIO 25 (via resistor)
Cathode (-)      White Brown   GND
```

## IR/On-Off Box Wiring

### VS1838B IR Receiver
```
VS1838B Pin      Wire Color    TTGO T18
-----------      ----------    --------
Signal           Green         GPIO 26
GND              Black         GND
VCC              Red           3.3V
```

### Power Switch
```
Switch Terminal  Wire Color    TTGO T18
---------------  ----------    --------
Signal           White         GPIO 27
GND              Black         GND
```

## Pin Configuration Summary
| Component     | Function | GPIO Pin |
|---------------|----------|----------|
| VL53L0X       | I2C SDA  | GPIO 21  |
| VL53L0X       | I2C SCL  | GPIO 22  |
| LED           | PWM Out  | GPIO 25  |
| IR Receiver   | Remote Control | GPIO 26  |
| Power Switch  | Deep Sleep Control | GPIO 27  |

## Building and Uploading

### Prerequisites
- [PlatformIO](https://platformio.org/) installed in VS Code
- USB cable to connect TTGO T18 to computer
- [USB to UART Bridge VCP Drivers](https://www.silabs.com/software-and-tools/usb-to-uart-bridge-vcp-drivers?tab=downloads) (Silicon Labs CP210x drivers for board communication)

### Commands
```bash
# Build the project
pio run

# Upload to board
pio run --target upload

# Monitor serial output
pio device monitor

# Build, upload, and monitor
pio run --target upload && pio device monitor
```

## Serial Monitor
The project outputs debug information to Serial (USB) at 115200 baud rate showing:
- **VL53L0X**: Distance in mm, LED brightness (3-125)
- **IR Remote**: Toggle enable/disable messages
- **Status updates**: Every 1 second
- **Debug messages**: Significant brightness changes (>10 difference)

Example output:
```
=== TTGO T18 - VL53L0X Distance Sensor ===
Initializing...

IR receiver initialized on GPIO 26
I2C initialized
VL53L0X sensor... SUCCESS!
VL53L0X LED PWM initialized on GPIO 25

Setup complete! VL53L0X system active...

--- Status ---
VL53: DISABLED (IR remote)

*** IR Remote: Sensor/LED ENABLED ***

--- Status ---
VL53: 156 mm | LED: 45
--- Status ---
VL53: 89 mm | LED: 65
[VL53 DEBUG] Distance: 45 mm | Current: 78 | Target: 115
--- Status ---
VL53: 45 mm | LED: 115

*** IR Remote: Sensor/LED DISABLED ***

--- Status ---
VL53: DISABLED (IR remote)
```

## References
- [VL53L0X Datasheet (ST)](https://www.st.com/resource/en/datasheet/vl53l0x.pdf)
- [VL53L0X Amazon Product](https://www.amazon.com/dp/B0B1M79WMP)
- [LILYGO T-Energy GitHub](https://github.com/LilyGO/LILYGO-T-Energy)
- [VL53L0X_mod Library (schnoog)](https://github.com/schnoog/vl53l0x-arduino-mod)
- [IRremoteESP8266 Library](https://github.com/crankyoldgit/IRremoteESP8266)
- [PlatformIO ESP32 Documentation](https://docs.platformio.org/en/latest/platforms/espressif32.html)
