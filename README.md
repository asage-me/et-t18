# TTGO T18 ESP32 - Dual Sensor System

## Project Overview
This is a PlatformIO project featuring two independent sensor systems:
1. **VL53L0X Distance Sensor** - Controls LED #1 brightness based on distance (closer = brighter)
2. **LD2410C Human Presence Sensor** - Controls Servo and LED #2 based on presence within 1 meter, with smooth 5-second transitions and 8-second debounce timers

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
  - Controls LED #1 on GPIO 25

### LD2410C Human Presence Sensor
- **Product Link**: [HLK-LD2410C](https://www.hlktech.net/index.php?id=1095)
- **Description**: 24GHz millimeter-wave radar sensor for human presence detection
- **Key Features**:
  - UART communication interface (256000 baud)
  - Detects static and moving targets
  - Range: 0-6 meters (project uses 1m threshold)
  - Low power consumption
  - Operating voltage: 3.3V - 5V
  - Controls Servo on GPIO 26 and LED #2 on GPIO 13

## Project Plan

### Functionality

#### VL53L0X System (LED #1)
1. **Continuous Distance Measurement**: Reads distance via I2C
2. **LED Brightness Control**: Inversely proportional to distance
   - 50mm or closer = Maximum brightness (255)
   - 2000mm or farther = Minimum brightness (0)
   - Linear mapping between these points
3. **Smooth Transitions**: Gradual brightness changes

#### LD2410C System (Servo + LED #2)
1. **Human Detection**: Continuously monitors for human presence within 1 meter using UART
2. **Detection Event** (human enters 1m range):
   - 8 second debounce timer starts
   - After debounce: Servo moves from 0° to 180° over 5 seconds
   - After debounce: LED #2 fades from off to full brightness over 5 seconds
3. **Departure Event** (human leaves 1m range):
   - 8 second debounce timer starts
   - After debounce: Servo returns from 180° to 0° over 5 seconds
   - After debounce: LED #2 fades from full brightness to off over 5 seconds
4. **Debounce Logic**: 8-second timers prevent false triggers from unstable readings at 1m boundary

### Technical Implementation
- **I2C Configuration**: Standard I2C for VL53L0X (GPIO 21/22)
- **UART Configuration**: 256000 baud rate for LD2410C (GPIO 12/14)
- **PWM Configuration**: 5kHz frequency, 8-bit resolution for both LEDs
- **Servo Control**: ESP32Servo library, 0-180° range
- **State Machine**: 6 states for LD2410C (IDLE, DEBOUNCE_ENTER, TRANSITIONING_ON, ACTIVE, DEBOUNCE_EXIT, TRANSITIONING_OFF)
- **GPIO Pins**:
  - I2C SDA: GPIO 21 (VL53L0X)
  - I2C SCL: GPIO 22 (VL53L0X)
  - LED #1 (VL53L0X): GPIO 25
  - UART RX: GPIO 12 (LD2410C)
  - UART TX: GPIO 14 (LD2410C)
  - LED #2 (LD2410C): GPIO 26
  - Servo Control: GPIO 27

### Development Steps
1. Initialize PlatformIO project for ESP32
2. Configure I2C for VL53L0X communication
3. Configure UART2 for LD2410C communication
4. Implement VL53L0X distance reading and LED control
5. Implement LD2410C data frame parsing
6. Implement state machine with debounce logic for LD2410C
7. Configure PWM outputs for both LEDs
8. Configure servo control
9. Implement smooth 5-second transitions for LD2410C system
10. Test and calibrate both systems

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

### LD2410C to TTGO T18
```
LD2410C          TTGO T18
--------         --------
VCC       --->   3.3V or 5V
GND       --->   GND
TX        --->   GPIO 12 (RX2)
RX        --->   GPIO 14 (TX2)
```

### Servo Connection
```
TTGO T18         Servo
--------         -----
GPIO 27   --->   Signal (PWM)
5V        --->   VCC (Power)
GND       --->   GND
```

### LED Connections
```
TTGO T18         LED #1 (VL53L0X)
--------         ----------------
GPIO 25   --->   LED Anode (+ via resistor)
GND       --->   LED Cathode (-)

TTGO T18         LED #2 (LD2410C)
--------         ----------------
GPIO 26   --->   LED Anode (+ via resistor)
GND       --->   LED Cathode (-)
```

**Note**: Use appropriate current-limiting resistor for your LEDs (typically 220Ω - 1kΩ for standard LEDs)

## Pin Configuration Summary
| Component | Function | GPIO Pin |
|-----------|----------|----------|
| VL53L0X   | I2C SDA  | GPIO 21  |
| VL53L0X   | I2C SCL  | GPIO 22  |
| LED #1    | PWM Out (VL53L0X) | GPIO 25 |
| LD2410C   | UART RX  | GPIO 12  |
| LD2410C   | UART TX  | GPIO 14  |
| LED #2    | PWM Out (LD2410C) | GPIO 26 |
| Servo     | PWM Out  | GPIO 27  |

## Building and Uploading

### Prerequisites
- [PlatformIO](https://platformio.org/) installed in VS Code
- USB cable to connect TTGO T18 to computer

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
- **VL53L0X**: Distance in mm, LED #1 brightness (0-255)
- **LD2410C**: Current state, presence status, distance in mm, LED #2 brightness (0-255), servo angle (0-180°)

## Future Enhancements
- Add adjustable detection range via serial commands
- Implement WiFi connectivity for remote monitoring
- Add multiple detection zones with different behaviors
- Battery level monitoring
- Deep sleep mode for power saving
- Add OLED display for local status display
- Combine sensor data for advanced logic

## License
This project is provided as-is for educational and development purposes.

## References
- [VL53L0X Datasheet (ST)](https://www.st.com/resource/en/datasheet/vl53l0x.pdf)
- [VL53L0X Amazon Product](https://www.amazon.com/dp/B0B1M79WMP)
- [LD2410C Product Page](https://www.hlktech.net/index.php?id=1095)
- [LILYGO T-Energy GitHub](https://github.com/LilyGO/LILYGO-T-Energy)
- [ESP32Servo Library](https://github.com/madhephaestus/ESP32Servo)
- [Adafruit VL53L0X Library](https://github.com/adafruit/Adafruit_VL53L0X)
- [PlatformIO ESP32 Documentation](https://docs.platformio.org/en/latest/platforms/espressif32.html)
