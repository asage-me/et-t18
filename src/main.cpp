/**
 * TTGO T18 ESP32 - VL53L0X Distance Sensor with 3-Mode Operation
 *
 * Mode 1 - VL53L0X Distance Sensor (IR Code: 0xFF30CF):
 * - Controls LED brightness based on distance (closer = brighter)
 * - Range: 25mm (max brightness 255) to 300mm (min brightness 1)
 * - Fast fade rate: ±48 per loop (~0.25 seconds full fade)
 *
 * Mode 2 - Full Brightness (IR Code: 0xFF18E7):
 * - LED at maximum brightness (255)
 * - Distance sensor disabled
 *
 * Mode 3 - Breathing Effect (IR Code: 0xFF7A85):
 * - LED slowly fades from off to full brightness and back (4-second cycle)
 * - Distance sensor disabled
 *
 * Power Control:
 * - Toggle switch on GPIO 27 (with internal pullup)
 * - Switch connected to GND = System ON
 * - Switch open (pulled HIGH by internal pullup) = Deep Sleep
 * - System wakes from deep sleep when switch is closed to GND
 */

#include <Arduino.h>
#include <Wire.h>
#include "VL53L0X_mod.h"
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>

// Pin definitions
#define LED_VL53_PIN        25              // PWM output for VL53L0X distance LED
#define IR_RECV_PIN         26              // IR receiver input
#define POWER_SWITCH_PIN    27              // Toggle switch (GND = ON, pulled HIGH = OFF)
#define I2C_SDA             21              // I2C SDA (default ESP32 SDA)
#define I2C_SCL             22              // I2C SCL (default ESP32 SCL)

// IR Remote button codes
#define IR_BUTTON_TOGGLE    0xFFA25D        // On/Off toggle button
#define IR_MODE_1_VL53      0xFF30CF        // Mode 1: VL53L0X distance sensor mode
#define IR_MODE_2_FULL      0xFF18E7        // Mode 2: Full brightness mode
#define IR_MODE_3_BREATHING 0xFF7A85        // Mode 3: Breathing effect mode

// VL53L0X settings
#define VL53_DISTANCE_MIN       20          // Closest distance for max brightness (mm) - sensor physical limit
#define VL53_DISTANCE_MAX       300         // Farthest distance for min brightness (mm)

// PWM settings
#define PWM_FREQ                5000        // 5 kHz
#define PWM_CHANNEL_VL53        0           // PWM channel 0 for VL53L0X LED
#define PWM_RESOLUTION          8           // 8-bit resolution (0-255)

// LED brightness levels
#define LED_BRIGHTNESS_MIN      1           // Minimum brightness (off when far)
#define VL53_LED_MAX            255         // VL53L0X LED max brightness (50% duty cycle = full brightness)

// Create sensor objects
VL53L0X_mod vl53l0x;
IRrecv irrecv(IR_RECV_PIN);

// VL53L0X global variables
uint16_t vl53CurrentDistance = 0;
uint8_t vl53CurrentBrightness = 0;
bool vl53SensorReady = false;

// Operating mode definitions
enum OperatingMode {
    MODE_OFF = 0,           // System off (default)
    MODE_1_VL53 = 1,        // VL53L0X distance sensor mode
    MODE_2_FULL = 2,        // Full brightness mode  
    MODE_3_BREATHING = 3    // Breathing effect mode
};

// System control variables
OperatingMode currentMode = MODE_OFF;       // Default to off mode
OperatingMode lastActiveMode = MODE_1_VL53; // Remember last active mode for toggle
bool systemEnabled = false;                 // System enabled/disabled state
unsigned long lastModeChangeMilliseconds = 0;  // Last time mode was changed
unsigned long lastToggleMilliseconds = 0;   // Last time toggle button was pressed
const unsigned long MODE_DEBOUNCE_MS = 500;    // 500ms debounce for mode changes
const unsigned long TOGGLE_DEBOUNCE_MS = 2000; // 2-second debounce for toggle

// Mode 3 breathing effect variables
unsigned long breathingStartMilliseconds = 0;
const unsigned long BREATHING_CYCLE_MS = 4000;  // 4-second full cycle (0 -> 255 -> 0)

/**
 * Read distance from VL53L0X sensor
 * Returns distance in millimeters, or VL53_DISTANCE_MAX if reading fails
 */
uint16_t readVL53Distance() {
    if (!vl53SensorReady) return VL53_DISTANCE_MAX;

    uint16_t distance = vl53l0x.readRangeSingleMillimeters();
    
    // Check if timeout occurred or invalid reading
    if (vl53l0x.timeoutOccurred() || distance >= 8000) {
        return VL53_DISTANCE_MAX;
    }
    
    return distance;
}

/**
 * Update VL53L0X LED brightness based on distance measurement (Mode 1 only)
 */
void updateVL53LED() {
    uint8_t targetBrightness;

    if (vl53CurrentDistance > 0) {
        uint16_t constrainedDistance = constrain(vl53CurrentDistance, VL53_DISTANCE_MIN, VL53_DISTANCE_MAX);
        targetBrightness = map(constrainedDistance, VL53_DISTANCE_MIN, VL53_DISTANCE_MAX,
                              VL53_LED_MAX, LED_BRIGHTNESS_MIN);
    } else {
        targetBrightness = LED_BRIGHTNESS_MIN;
    }

    // Debug: Print when target changes significantly
    static uint8_t lastTarget = 0;
    if (abs(targetBrightness - lastTarget) > 10) {
        Serial.print("[VL53 DEBUG] Distance: ");
        Serial.print(vl53CurrentDistance);
        Serial.print(" mm | Current: ");
        Serial.print(vl53CurrentBrightness);
        Serial.print(" | Target: ");
        Serial.println(targetBrightness);
        lastTarget = targetBrightness;
    }

    // Smooth transition (±48 per loop = ±960 per second with 50ms delay)
    if (vl53CurrentBrightness < targetBrightness) {
        vl53CurrentBrightness = min(vl53CurrentBrightness + 48, (int)targetBrightness);
    } else if (vl53CurrentBrightness > targetBrightness) {
        if (vl53CurrentBrightness >= 48) {
            vl53CurrentBrightness = max((int)(vl53CurrentBrightness - 48), (int)targetBrightness);
        } else {
            vl53CurrentBrightness = targetBrightness;
        }
    }

    ledcWrite(PWM_CHANNEL_VL53, vl53CurrentBrightness);
}

/**
 * Update LED based on current operating mode
 */
void updateLEDForCurrentMode() {
    // Check power switch first - if OFF, turn LED off immediately regardless of mode
    if (digitalRead(POWER_SWITCH_PIN) == HIGH) {
        vl53CurrentBrightness = 0;
        ledcWrite(PWM_CHANNEL_VL53, vl53CurrentBrightness);
        return;  // Skip normal mode processing
    }
    
    switch (currentMode) {
        case MODE_OFF:
            // System off - LED off
            vl53CurrentBrightness = 0;
            ledcWrite(PWM_CHANNEL_VL53, vl53CurrentBrightness);
            break;
            
        case MODE_1_VL53:
            // VL53L0X distance sensor mode - handled by updateVL53LED()
            if (vl53SensorReady) {
                vl53CurrentDistance = readVL53Distance();
                updateVL53LED();
            }
            break;
            
        case MODE_2_FULL:
            // Full brightness mode
            vl53CurrentBrightness = VL53_LED_MAX;
            ledcWrite(PWM_CHANNEL_VL53, vl53CurrentBrightness);
            break;
            
        case MODE_3_BREATHING:
            // Breathing effect mode (4-second cycle: 0 -> 255 -> 0)
            unsigned long elapsedMilliseconds = millis() - breathingStartMilliseconds;
            unsigned long cyclePosition = elapsedMilliseconds % BREATHING_CYCLE_MS;
            
            if (cyclePosition <= BREATHING_CYCLE_MS / 2) {
                // First half: fade up (0 -> 255)
                vl53CurrentBrightness = map(cyclePosition, 0, BREATHING_CYCLE_MS / 2, 0, VL53_LED_MAX);
            } else {
                // Second half: fade down (255 -> 0)
                vl53CurrentBrightness = map(cyclePosition, BREATHING_CYCLE_MS / 2, BREATHING_CYCLE_MS, VL53_LED_MAX, 0);
            }
            
            ledcWrite(PWM_CHANNEL_VL53, vl53CurrentBrightness);
            break;
    }
}

/**
 * Process IR remote toggle on/off
 */
void processIRToggle() {
    systemEnabled = !systemEnabled;
    
    if (systemEnabled) {
        // Enable system - restore last active mode
        currentMode = lastActiveMode;
        Serial.print("\n*** System ENABLED - Mode ");
        Serial.print(currentMode);
        switch (currentMode) {
            case MODE_1_VL53:
                Serial.println(": VL53L0X Distance Sensor ***");
                break;
            case MODE_2_FULL:
                Serial.println(": Full Brightness ***");
                break;
            case MODE_3_BREATHING:
                Serial.println(": Breathing Effect ***");
                breathingStartMilliseconds = millis();  // Reset breathing cycle
                break;
        }
    } else {
        // Disable system - save current mode and go to OFF
        if (currentMode != MODE_OFF) {
            lastActiveMode = currentMode;
        }
        currentMode = MODE_OFF;
        Serial.println("\n*** System DISABLED ***");
    }
    Serial.println();
}

/**
 * Process IR remote mode selection
 */
void processIRModeChange(uint32_t irCode) {
    OperatingMode newMode = currentMode;
    
    switch (irCode) {
        case IR_MODE_1_VL53:
            newMode = MODE_1_VL53;
            break;
        case IR_MODE_2_FULL:
            newMode = MODE_2_FULL;
            break;
        case IR_MODE_3_BREATHING:
            newMode = MODE_3_BREATHING;
            breathingStartMilliseconds = millis();  // Reset breathing cycle
            break;
        default:
            return;  // Unknown code, ignore
    }
    
    if (newMode != currentMode) {
        currentMode = newMode;
        lastActiveMode = newMode;  // Remember this as the last active mode
        systemEnabled = true;      // Enable system when selecting a mode
        
        Serial.print("\n*** Mode Changed to ");
        switch (currentMode) {
            case MODE_OFF:
                Serial.println("OFF ***");
                break;
            case MODE_1_VL53:
                Serial.println("1: VL53L0X Distance Sensor ***");
                break;
            case MODE_2_FULL:
                Serial.println("2: Full Brightness ***");
                break;
            case MODE_3_BREATHING:
                Serial.println("3: Breathing Effect ***");
                break;
        }
        Serial.println();
    }
}

/**
 * Setup function - runs once at startup
 */
void setup() {
    // Initialize USB serial for debugging
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n\n=== TTGO T18 - VL53L0X Distance Sensor ===");
    Serial.println("Initializing...\n");

    // Initialize IR receiver
    irrecv.enableIRIn();
    Serial.println("IR receiver initialized on GPIO 26");

    // Configure power switch pin with pullup resistor
    pinMode(POWER_SWITCH_PIN, INPUT_PULLUP);
    
    // Check if switch is OFF (HIGH = pulled up by internal resistor) - enter deep sleep immediately
    if (digitalRead(POWER_SWITCH_PIN) == HIGH) {
        Serial.println("Power switch is OFF - entering deep sleep");
        Serial.println("Toggle switch to GND to wake up");
        delay(100);  // Allow serial to flush
        
        // Configure GPIO 27 as wake-up source (wake when LOW/GND)
        esp_sleep_enable_ext0_wakeup(GPIO_NUM_27, 0);  // 0 = wake on LOW
        
        // Enter deep sleep
        esp_deep_sleep_start();
    }
    
    Serial.println("Power switch is ON - starting normal operation\n");

    // Initialize I2C for VL53L0X
    Wire.begin(I2C_SDA, I2C_SCL);
    Serial.println("I2C initialized");

    // Initialize VL53L0X sensor (with timeout)
    Serial.print("VL53L0X sensor... ");
    vl53l0x.setTimeout(500);
    if (vl53l0x.init()) {
        Serial.println("SUCCESS!");
        vl53l0x.startContinuous();
        vl53SensorReady = true;
    } else {
        Serial.println("INIT FAILED (sensor not found or failed to initialize)");
        vl53SensorReady = false;
    }

    // Configure VL53L0X LED PWM
    ledcSetup(PWM_CHANNEL_VL53, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(LED_VL53_PIN, PWM_CHANNEL_VL53);
    ledcWrite(PWM_CHANNEL_VL53, 0);
    Serial.println("VL53L0X LED PWM initialized on GPIO 25");

    // Initialize breathing effect timer
    breathingStartMilliseconds = millis();

    Serial.println("\nSetup complete! 3-Mode system ready (starting in OFF mode)...");
    Serial.println("IR Remote Codes:");
    Serial.println("  0xFFA25D = Toggle On/Off (remembers last mode)");
    Serial.println("  0xFF30CF = Mode 1: VL53L0X Distance Sensor");
    Serial.println("  0xFF18E7 = Mode 2: Full Brightness"); 
    Serial.println("  0xFF7A85 = Mode 3: Breathing Effect");
    Serial.println("  System starts OFF - press toggle or any mode button to activate");
    Serial.println();
}

/**
 * Main loop - runs continuously
 */
void loop() {
    static unsigned long lastPrintTimeMilliseconds = 0;
    unsigned long currentTimeMilliseconds = millis();

    // Check power switch - if turned OFF (pulled HIGH), enter deep sleep
    if (digitalRead(POWER_SWITCH_PIN) == HIGH) {
        Serial.println("\n*** Power switch turned OFF - entering deep sleep ***");
        Serial.println("Toggle switch to GND to wake up");
        delay(100);  // Allow serial to flush
        
        // Turn off LED before sleep
        ledcWrite(PWM_CHANNEL_VL53, 0);
        
        // Configure GPIO 27 as wake-up source (wake when LOW/GND)
        esp_sleep_enable_ext0_wakeup(GPIO_NUM_27, 0);  // 0 = wake on LOW
        
        // Enter deep sleep
        esp_deep_sleep_start();
    }

    // Process IR remote commands
    decode_results results;
    if (irrecv.decode(&results)) {
        // Filter out repeat codes (0xFFFFFFFFFFFFFFFF) and only process valid button press
        if (results.value != 0xFFFFFFFFFFFFFFFF) {
            unsigned long currentTime = millis();
            
            // Check for toggle button (0xFFA25D)
            if (results.value == IR_BUTTON_TOGGLE) {
                if (currentTime - lastToggleMilliseconds >= TOGGLE_DEBOUNCE_MS) {
                    processIRToggle();
                    lastToggleMilliseconds = currentTime;
                }
            } 
            // Check for mode selection buttons
            else if (currentTime - lastModeChangeMilliseconds >= MODE_DEBOUNCE_MS) {
                processIRModeChange(results.value);
                lastModeChangeMilliseconds = currentTime;
            }
        }
        irrecv.resume();  // Receive the next value
    }

    // Update LED based on current operating mode (this also checks power switch)
    updateLEDForCurrentMode();

    // Print status every second
    if (currentTimeMilliseconds - lastPrintTimeMilliseconds >= 1000) {
        Serial.println("--- Status ---");

        // Current mode and LED status
        Serial.print("Mode ");
        Serial.print(currentMode);
        Serial.print(": ");
        
        switch (currentMode) {
            case MODE_OFF:
                Serial.print("OFF");
                break;
            case MODE_1_VL53:
                if (vl53SensorReady) {
                    Serial.print("VL53 ");
                    Serial.print(vl53CurrentDistance);
                    Serial.print(" mm");
                } else {
                    Serial.print("VL53 NOT CONNECTED");
                }
                break;
            case MODE_2_FULL:
                Serial.print("Full Brightness");
                break;
            case MODE_3_BREATHING:
                Serial.print("Breathing Effect");
                break;
        }
        
        Serial.print(" | LED: ");
        Serial.println(vl53CurrentBrightness);

        lastPrintTimeMilliseconds = currentTimeMilliseconds;
    }

    delay(50);
}