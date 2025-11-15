# Copilot Instructions for TTGO T18 V3 ESP32 Project

## Project Overview
This is a PlatformIO-based ESP32 project for the TTGO T18 V3 board featuring two independent sensor systems:
1. **VL53L0X Time-of-Flight distance sensor** (I2C) - Controls LED #1 brightness based on distance
2. **LD2410C human presence sensor** (UART) - Controls Servo and LED #2 based on presence

### LD2410C Timing Sequence
**Detection (13 seconds total):**
- 2s debounce → 5s transition ON (servo/LED) → 6s active wait

**Departure (13 seconds total):**
- 2s debounce → 5s transition OFF (servo/LED) → 6s idle wait

## Board Configuration
- **Custom Board Definition**: `boards/ttgo-t18-v3.json`
- **Chipset**: ESP32-WROVER (240MHz dual-core, 520KB RAM, 4MB Flash)
- **PSRAM**: Enabled with cache issue fix (`-DBOARD_HAS_PSRAM -mfix-esp32-psram-cache-issue`)
- **PlatformIO Environment**: `[env:ttgo-t18-v3]`

## Hardware Configuration

### VL53L0X System
- **VL53L0X Distance Sensor**:
  - SDA: GPIO 21
  - SCL: GPIO 22
  - Range: 50mm (max brightness) to 2000mm (min brightness)
  - Communication: I2C

- **LED #1 PWM Output**:
  - Pin: GPIO 25
  - Frequency: 5kHz
  - Resolution: 8-bit (0-255)
  - Controlled by: VL53L0X distance readings

### LD2410C System
- **LD2410C Presence Sensor**:
  - RX: GPIO 32
  - TX: GPIO 33
  - Baud: 256000
  - Detection Range: 1 meter threshold
  - Communication: UART2

- **Servo Motor**:
  - Pin: GPIO 27
  - Range: 0° to 180°
  - Transition Time: 5 seconds

- **LED #2 PWM Output**:
  - Pin: GPIO 26
  - Frequency: 5kHz
  - Resolution: 8-bit (0-255)
  - Transition Time: 5 seconds

## Code Architecture

### Key Functions
- `readVL53Distance()` - Reads VL53L0X sensor, returns distance in millimeters
- `updateVL53LED()` - Maps VL53L0X distance to LED #1 brightness with smooth transitions
- `parseLD2410CData()` - Parses UART data frames from LD2410C sensor
- `updateLD2410Outputs()` - Updates servo position and LED #2 brightness based on transition progress
- `updateLD2410StateMachine()` - Manages 8-state state machine with timing logic
- `setup()` - Initializes I2C, UART, servo, and both PWM channels
- `loop()` - Continuous processing for both sensor systems

### Control Logic

#### VL53L0X System
- Distance 50mm or less = LED #1 at maximum brightness (255)
- Distance 2000mm or more = LED #1 at minimum brightness (0)
- Linear mapping between these points using Arduino `map()` function
- Smooth brightness transitions (±5 per loop iteration)

#### LD2410C System
- **Detection Sequence** (13 seconds total):
  - 2-second debounce → 5-second transition ON → 6-second active wait
- **Departure Sequence** (13 seconds total):
  - 2-second debounce → 5-second transition OFF → 6-second idle wait
- **State Machine States**:
  - STATE_IDLE: No presence detected, ready to detect
  - STATE_DEBOUNCE_ENTER: 2-second debounce after detection
  - STATE_TRANSITIONING_ON: 5-second gradual fade in LED and servo
  - STATE_ACTIVE: Fully on, waiting before departure monitoring
  - STATE_ACTIVE_WAIT: 6-second wait complete, monitoring for departure
  - STATE_DEBOUNCE_EXIT: 2-second debounce after departure
  - STATE_TRANSITIONING_OFF: 5-second gradual fade out LED and servo
  - STATE_IDLE_WAIT: 6-second wait before ready to detect again

## Libraries Used
- `Adafruit_VL53L0X` v1.2.4 - VL53L0X ToF sensor driver
- `ESP32Servo` v3.0.5+ - Servo motor control
- `Wire` - I2C communication for VL53L0X
- `HardwareSerial` - UART communication for LD2410C
- `Arduino.h` - Arduino framework for ESP32

### Variable Naming Convention
Follow these standardized naming rules throughout the codebase:
- **Case Style**: Use camelCase for all variables and functions (e.g., `temperatureReading`, `indoorHumidityReading`)
- **Descriptive Names**: Use fully spelled out words instead of abbreviations (e.g., `temperature` not `temp`, `humidity` not `hum`)
- **Device Model Numbers**: Keep original numeric designations when referencing specific device models (e.g., `dht22Indoor`, `ds18b20Temp`, `sht85Indoor`, `tsl2561`, `tca9548A`)
- **Other Numbers**: Spell out numbers in general variable names (e.g., `buttonFirstPin`, `oledFirst`, `screenSecondCurrentPage`)
- **Constants**: Use UPPER_SNAKE_CASE for error codes and configuration constants (e.g., `ERROR_DHT22_TEMPERATURE_FAIL`)
- **Consistency**: Maintain consistent terminology across related variables (e.g., all temperature variables should use `Temperature`, all humidity variables should use `Humidity`)

**✅ Consistency Status**: Variable naming conventions have been validated across all source files (October 2025). One inconsistency was found and corrected: `loopDelayMs` → `loopDelayMilliseconds` to follow the spelled-out numbers rule.

### Code Formatting Rules
**CRITICAL**: All code files must follow these formatting standards for CI/CD compliance:

**No Trailing Whitespace:**
- **Rule**: NEVER leave trailing whitespace at the end of lines in any source file
- **Enforcement**: GitHub Actions CI/CD checks fail if trailing whitespace is detected
- **Check Command**: `find src/ test/ -name "*.cpp" -o -name "*.h" | xargs grep -l '[[:space:]]$'`
- **Fix Command (PowerShell)**: `(Get-Content file.cpp) -replace '\s+$', '' | Set-Content file.cpp`
- **Rationale**: Trailing whitespace causes inconsistent diffs, merge conflicts, and CI/CD failures
- **Scope**: Applies to all `.cpp`, `.h`, `.md`, and configuration files

**Indentation:**
- **Rule**: Use spaces only (no tabs) for indentation
- **Standard**: 4 spaces per indentation level
- **Check Command**: `find src/ -name "*.cpp" -o -name "*.h" | xargs grep -l $'\t'`

**CRITICAL BUILD WORKFLOW**:
- **ALWAYS use PowerShell commands via `run_in_terminal` tool** - NEVER use VS Code tasks for builds
- **Reason**: AI agents cannot reliably wait for task completion and will incorrectly report build success before tasks finish
- **Correct Build Command**:
  ```powershell
  C:\Users\adam\.platformio\penv\Scripts\platformio.exe run
  ```
- **Correct Build with Error Filtering**:
  ```powershell
  C:\Users\adam\.platformio\penv\Scripts\platformio.exe run 2>&1 | Select-String -Pattern "(error|warning|SUCCESS|FAILED)" -Context 0,2
  ```
- **Upload Command**:
  ```powershell
  C:\Users\adam\.platformio\penv\Scripts\platformio.exe run --target upload
  ```
- **Why This Matters**: Task-based builds return immediately, causing premature "build successful" declarations before compilation completes. PowerShell commands block properly and provide accurate completion status.

## Comment Alignment Strategy
**CRITICAL**: Maintain consistent end-of-line comment alignment for professional code appearance and readability:

**Alignment Rules by Context**:
- **Short variable declarations** (like `int pin = 13`): Align comments to **column 40-43**
- **Medium variable declarations** (like sensor readings): Align comments to **column 45-47**
- **Long constant declarations** (like error codes): Align comments to **column 60+**
- **Inline code comments**: Use **2-4 spaces** after code statement
- **Case statement comments**: Use **single space** after colon

**Examples**:
```cpp
// Short variables (column 40-43)
int heaterPin = 13;                        // Heater relay (solid state)
int fanPin = 14;                           // Fan relay (standard)

// Medium variables (column 45-47)
float indoorTemperatureReading = -999;     // Indoor temperature (averaged/validated)
int loopTimeCounter = 599;                 // Initialize at 599 for immediate data

// Long constants (column 60+)
const String ERROR_DHT22_TEMPERATURE_FAIL = "0x01";            // DHT22 temperature read failure
const String ERROR_SHT85_IN_INIT_FAIL = "0x0B";                // SHT85 indoor initialization failure

// Inline code comments (2-4 spaces)
temperature = sht85.getTemp() * 9.0 / 5.0 + 32.0;  // Convert C to F
delay(1500);                               // Show message briefly

// Case statements (single space)
case 0: // MAC Addresses
case 1: // WiFi Info
```

**Quality Standards**:
- **No excessive whitespace**: Avoid spreading short declarations to column 60+
- **Visual balance**: Match alignment within related variable groups
- **Remove trailing spaces**: Clean up any extra spaces after comments
- **Consistent spacing**: Use same alignment pattern within each code section
