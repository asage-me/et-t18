/**
 * TTGO T18 ESP32 - VL53L0X Distance Sensor
 *
 * VL53L0X Distance Sensor:
 * - Controls LED brightness based on distance (closer = brighter)
 * - Range: 25mm (max brightness 125) to 400mm (min brightness 3)
 * - LED reaches full brightness at 50% duty cycle (125/255)
 * - Fast fade rate: ±48 per loop (~0.25 seconds full fade)
 */

#include <Arduino.h>
#include <Wire.h>
#include "VL53L0X_mod.h"

// Pin definitions
#define LED_VL53_PIN    25                  // PWM output for VL53L0X distance LED
#define I2C_SDA         21                  // I2C SDA (default ESP32 SDA)
#define I2C_SCL         22                  // I2C SCL (default ESP32 SCL)

// VL53L0X settings
#define VL53_DISTANCE_MIN       25          // Closest distance for max brightness (mm) - sensor physical limit
#define VL53_DISTANCE_MAX       300         // Farthest distance for min brightness (mm)

// PWM settings
#define PWM_FREQ                5000        // 5 kHz
#define PWM_CHANNEL_VL53        0           // PWM channel 0 for VL53L0X LED
#define PWM_RESOLUTION          8           // 8-bit resolution (0-255)

// LED brightness levels
#define LED_BRIGHTNESS_MIN      3           // Minimum brightness (off when far)
#define VL53_LED_MAX            125         // VL53L0X LED max brightness (50% duty cycle = full brightness)

// Create sensor objects
VL53L0X_mod vl53l0x;

// VL53L0X global variables
uint16_t vl53CurrentDistance = 0;
uint8_t vl53CurrentBrightness = 0;
bool vl53SensorReady = false;

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
 * Update VL53L0X LED brightness based on distance measurement
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
 * Setup function - runs once at startup
 */
void setup() {
    // Initialize USB serial for debugging
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n\n=== TTGO T18 - VL53L0X Distance Sensor ===");
    Serial.println("Initializing...\n");

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

    Serial.println("\nSetup complete! VL53L0X system active...\n");
}

/**
 * Main loop - runs continuously
 */
void loop() {
    static unsigned long lastPrintTimeMilliseconds = 0;
    unsigned long currentTimeMilliseconds = millis();

    // VL53L0X distance sensor processing (only if sensor is ready)
    if (vl53SensorReady) {
        vl53CurrentDistance = readVL53Distance();
        updateVL53LED();
    }

    // Print status every second
    if (currentTimeMilliseconds - lastPrintTimeMilliseconds >= 1000) {
        Serial.println("--- Status ---");

        // VL53L0X status
        if (vl53SensorReady) {
            Serial.print("VL53: ");
            Serial.print(vl53CurrentDistance);
            Serial.print(" mm | LED: ");
            Serial.println(vl53CurrentBrightness);
        } else {
            Serial.println("VL53: NOT CONNECTED");
        }

        lastPrintTimeMilliseconds = currentTimeMilliseconds;
    }

    delay(50);
}