/**
 * TTGO T18 ESP32 - Dual Sensor System
 *
 * VL53L0X Distance Sensor:
 * - Controls LED #1 brightness based on distance (closer = brighter)
 * - Range: 50mm (max) to 2000mm (min brightness)
 *
 * LD2410C Human Presence Sensor:
 * - Controls Servo and LED #2 based on presence within 1 meter
 * - 8 second debounce timers on detection/departure
 * - 5 second smooth transitions for servo and LED
 */

#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <ESP32Servo.h>
#include "Adafruit_VL53L0X.h"

// Pin definitions
#define LED_VL53_PIN    25                  // PWM output for VL53L0X distance LED
#define LED_LD2410_PIN  26                  // PWM output for LD2410C presence LED
#define SERVO_PIN       27                  // Servo control pin
#define I2C_SDA         21                  // I2C SDA (default ESP32 SDA)
#define I2C_SCL         22                  // I2C SCL (default ESP32 SCL)
#define UART_RX_PIN     12                  // UART RX from LD2410C TX
#define UART_TX_PIN     14                  // UART TX to LD2410C RX

// VL53L0X settings
#define VL53_DISTANCE_MIN       50          // Closest distance for max brightness (mm)
#define VL53_DISTANCE_MAX       2000        // Farthest distance for min brightness (mm)

// LD2410C UART settings
#define LD2410C_BAUD            256000      // Default baud rate for LD2410C
#define DETECTION_RANGE_MM      1000        // 1 meter detection range
#define TRANSITION_TIME_MS      5000        // 5 seconds for fade/servo transition
#define DEBOUNCE_TIME_MS        8000        // 8 seconds debounce timer

// PWM settings
#define PWM_FREQ                5000        // 5 kHz
#define PWM_CHANNEL_VL53        0           // PWM channel 0 for VL53L0X LED
#define PWM_CHANNEL_LD2410      1           // PWM channel 1 for LD2410C LED
#define PWM_RESOLUTION          8           // 8-bit resolution (0-255)

// Servo settings
#define SERVO_MIN_ANGLE         0           // Servo minimum position
#define SERVO_MAX_ANGLE         180         // Servo maximum position

// LED brightness levels
#define LED_BRIGHTNESS_MAX      255         // Maximum brightness
#define LED_BRIGHTNESS_MIN      0           // Minimum brightness

// LD2410C data frame structure
#define FRAME_HEADER1   0xF4
#define FRAME_HEADER2   0xF3
#define FRAME_HEADER3   0xF2
#define FRAME_HEADER4   0xF1
#define FRAME_TAIL1     0xF8
#define FRAME_TAIL2     0xF7
#define FRAME_TAIL3     0xF6
#define FRAME_TAIL4     0xF5

// State machine states for LD2410C
enum SystemState {
    STATE_IDLE,                             // No presence detected
    STATE_DEBOUNCE_ENTER,                   // Debouncing after detection
    STATE_TRANSITIONING_ON,                 // Fading in LED and moving servo
    STATE_ACTIVE,                           // Presence detected, fully on
    STATE_DEBOUNCE_EXIT,                    // Debouncing after departure
    STATE_TRANSITIONING_OFF                 // Fading out LED and returning servo
};

// Create sensor objects
Adafruit_VL53L0X vl53l0x = Adafruit_VL53L0X();
HardwareSerial ld2410cSerial(2);
Servo servo;

// VL53L0X global variables
uint16_t vl53CurrentDistance = 0;
uint8_t vl53CurrentBrightness = 0;
bool vl53SensorReady = false;

// LD2410C global variables
SystemState currentState = STATE_IDLE;
bool ld2410PresenceDetected = false;
uint16_t ld2410DistanceMillimeters = 0;
uint8_t ld2410CurrentBrightness = 0;
uint16_t currentServoAngle = SERVO_MIN_ANGLE;
unsigned long transitionStartTimeMilliseconds = 0;
unsigned long debounceStartTimeMilliseconds = 0;

/**
 * Read distance from VL53L0X sensor
 * Returns distance in millimeters, or 0 if reading fails
 */
uint16_t readVL53Distance() {
    if (!vl53SensorReady) return 0;

    VL53L0X_RangingMeasurementData_t measure;
    vl53l0x.rangingTest(&measure, false);

    if (measure.RangeStatus != 4) {
        return measure.RangeMilliMeter;
    }
    return VL53_DISTANCE_MAX;
}

/**
 * Update VL53L0X LED brightness based on distance measurement
 */
void updateVL53LED() {
    uint8_t targetBrightness;

    if (vl53CurrentDistance > 0) {
        uint16_t constrainedDistance = constrain(vl53CurrentDistance, VL53_DISTANCE_MIN, VL53_DISTANCE_MAX);
        targetBrightness = map(constrainedDistance, VL53_DISTANCE_MIN, VL53_DISTANCE_MAX,
                              LED_BRIGHTNESS_MAX, LED_BRIGHTNESS_MIN);
    } else {
        targetBrightness = LED_BRIGHTNESS_MIN;
    }

    // Smooth transition
    if (vl53CurrentBrightness < targetBrightness) {
        vl53CurrentBrightness = min((uint8_t)(vl53CurrentBrightness + 5), targetBrightness);
    } else if (vl53CurrentBrightness > targetBrightness) {
        vl53CurrentBrightness = max((uint8_t)(vl53CurrentBrightness - 5), targetBrightness);
    }

    ledcWrite(PWM_CHANNEL_VL53, vl53CurrentBrightness);
}

/**
 * Parse LD2410C data frame
 */
void parseLD2410CData() {
    static uint8_t buffer[32];
    static uint8_t bufferIndex = 0;
    static bool frameStarted = false;

    while (ld2410cSerial.available()) {
        uint8_t byte = ld2410cSerial.read();

        if (!frameStarted) {
            if (bufferIndex == 0 && byte == FRAME_HEADER1) {
                buffer[bufferIndex++] = byte;
            } else if (bufferIndex == 1 && byte == FRAME_HEADER2) {
                buffer[bufferIndex++] = byte;
            } else if (bufferIndex == 2 && byte == FRAME_HEADER3) {
                buffer[bufferIndex++] = byte;
            } else if (bufferIndex == 3 && byte == FRAME_HEADER4) {
                buffer[bufferIndex++] = byte;
                frameStarted = true;
            } else {
                bufferIndex = 0;
            }
        } else {
            buffer[bufferIndex++] = byte;

            if (bufferIndex >= 12) {
                if (buffer[bufferIndex - 4] == FRAME_TAIL1 &&
                    buffer[bufferIndex - 3] == FRAME_TAIL2 &&
                    buffer[bufferIndex - 2] == FRAME_TAIL3 &&
                    buffer[bufferIndex - 1] == FRAME_TAIL4) {

                    uint8_t targetState = buffer[6];
                    ld2410PresenceDetected = (targetState != 0x00);

                    if (ld2410PresenceDetected) {
                        ld2410DistanceMillimeters = buffer[7] | (buffer[8] << 8);
                    } else {
                        ld2410DistanceMillimeters = 0;
                    }

                    bufferIndex = 0;
                    frameStarted = false;
                    break;
                }

                if (bufferIndex >= sizeof(buffer)) {
                    bufferIndex = 0;
                    frameStarted = false;
                }
            }
        }
    }
}

/**
 * Update LD2410C LED and servo based on transition progress
 */
void updateLD2410Outputs(float progress) {
    progress = constrain(progress, 0.0, 1.0);

    // Update LED brightness
    ld2410CurrentBrightness = (uint8_t)(LED_BRIGHTNESS_MIN + (LED_BRIGHTNESS_MAX - LED_BRIGHTNESS_MIN) * progress);
    ledcWrite(PWM_CHANNEL_LD2410, ld2410CurrentBrightness);

    // Update servo position
    currentServoAngle = SERVO_MIN_ANGLE + (uint16_t)((SERVO_MAX_ANGLE - SERVO_MIN_ANGLE) * progress);
    servo.write(currentServoAngle);
}

/**
 * State machine for LD2410C presence detection
 */
void updateLD2410StateMachine() {
    unsigned long currentTimeMilliseconds = millis();
    bool humanWithinRange = ld2410PresenceDetected &&
                           (ld2410DistanceMillimeters <= DETECTION_RANGE_MM) &&
                           (ld2410DistanceMillimeters > 0);

    switch (currentState) {
        case STATE_IDLE:
            if (humanWithinRange) {
                Serial.println(">>> Human detected! Starting debounce");
                currentState = STATE_DEBOUNCE_ENTER;
                debounceStartTimeMilliseconds = currentTimeMilliseconds;
            }
            break;

        case STATE_DEBOUNCE_ENTER:
            if (currentTimeMilliseconds - debounceStartTimeMilliseconds >= DEBOUNCE_TIME_MS) {
                if (humanWithinRange) {
                    Serial.println(">>> Debounce complete, transitioning ON");
                    currentState = STATE_TRANSITIONING_ON;
                    transitionStartTimeMilliseconds = currentTimeMilliseconds;
                } else {
                    Serial.println(">>> False detection, returning to IDLE");
                    currentState = STATE_IDLE;
                }
            }
            break;

        case STATE_TRANSITIONING_ON:
            {
                unsigned long elapsed = currentTimeMilliseconds - transitionStartTimeMilliseconds;
                float progress = (float)elapsed / TRANSITION_TIME_MS;

                if (progress >= 1.0) {
                    updateLD2410Outputs(1.0);
                    currentState = STATE_ACTIVE;
                    Serial.println(">>> Transition ON complete, now ACTIVE");
                } else {
                    updateLD2410Outputs(progress);
                }
            }
            break;

        case STATE_ACTIVE:
            if (!humanWithinRange) {
                Serial.println(">>> Human left! Starting debounce");
                currentState = STATE_DEBOUNCE_EXIT;
                debounceStartTimeMilliseconds = currentTimeMilliseconds;
            }
            break;

        case STATE_DEBOUNCE_EXIT:
            if (currentTimeMilliseconds - debounceStartTimeMilliseconds >= DEBOUNCE_TIME_MS) {
                if (!humanWithinRange) {
                    Serial.println(">>> Debounce complete, transitioning OFF");
                    currentState = STATE_TRANSITIONING_OFF;
                    transitionStartTimeMilliseconds = currentTimeMilliseconds;
                } else {
                    Serial.println(">>> Human returned, back to ACTIVE");
                    currentState = STATE_ACTIVE;
                }
            }
            break;

        case STATE_TRANSITIONING_OFF:
            {
                unsigned long elapsed = currentTimeMilliseconds - transitionStartTimeMilliseconds;
                float progress = 1.0 - ((float)elapsed / TRANSITION_TIME_MS);

                if (progress <= 0.0) {
                    updateLD2410Outputs(0.0);
                    currentState = STATE_IDLE;
                    Serial.println(">>> Transition OFF complete, now IDLE");
                } else {
                    updateLD2410Outputs(progress);
                }
            }
            break;
    }
}

/**
 * Setup function - runs once at startup
 */
void setup() {
    // Initialize USB serial for debugging
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n\n=== TTGO T18 - Dual Sensor System ===");
    Serial.println("Initializing...\n");

    // Initialize I2C for VL53L0X
    Wire.begin(I2C_SDA, I2C_SCL);
    Serial.println("I2C initialized");

    // Initialize VL53L0X sensor
    Serial.print("VL53L0X sensor... ");
    if (!vl53l0x.begin()) {
        Serial.println("FAILED! Check wiring!");
        vl53SensorReady = false;
    } else {
        Serial.println("SUCCESS!");
        vl53SensorReady = true;
    }

    // Initialize UART for LD2410C
    ld2410cSerial.begin(LD2410C_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    Serial.println("LD2410C UART initialized at 256000 baud");

    // Configure VL53L0X LED PWM
    ledcSetup(PWM_CHANNEL_VL53, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(LED_VL53_PIN, PWM_CHANNEL_VL53);
    ledcWrite(PWM_CHANNEL_VL53, 0);
    Serial.println("VL53L0X LED PWM initialized on GPIO 25");

    // Configure LD2410C LED PWM
    ledcSetup(PWM_CHANNEL_LD2410, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(LED_LD2410_PIN, PWM_CHANNEL_LD2410);
    ledcWrite(PWM_CHANNEL_LD2410, 0);
    Serial.println("LD2410C LED PWM initialized on GPIO 13");

    // Initialize servo
    servo.attach(SERVO_PIN);
    servo.write(SERVO_MIN_ANGLE);
    Serial.println("Servo initialized on GPIO 26");

    Serial.println("\nSetup complete! Both systems active...\n");
}

/**
 * Main loop - runs continuously
 */
void loop() {
    static unsigned long lastPrintTimeMilliseconds = 0;
    unsigned long currentTimeMilliseconds = millis();

    // VL53L0X distance sensor processing
    vl53CurrentDistance = readVL53Distance();
    updateVL53LED();

    // LD2410C presence sensor processing
    parseLD2410CData();
    updateLD2410StateMachine();

    // Print status every second
    if (currentTimeMilliseconds - lastPrintTimeMilliseconds >= 1000) {
        Serial.println("--- Status ---");

        // VL53L0X status
        Serial.print("VL53: ");
        Serial.print(vl53CurrentDistance);
        Serial.print(" mm | LED1: ");
        Serial.println(vl53CurrentBrightness);

        // LD2410C status
        Serial.print("LD2410: ");
        switch (currentState) {
            case STATE_IDLE: Serial.print("IDLE"); break;
            case STATE_DEBOUNCE_ENTER: Serial.print("DEBOUNCE_ENTER"); break;
            case STATE_TRANSITIONING_ON: Serial.print("TRANS_ON"); break;
            case STATE_ACTIVE: Serial.print("ACTIVE"); break;
            case STATE_DEBOUNCE_EXIT: Serial.print("DEBOUNCE_EXIT"); break;
            case STATE_TRANSITIONING_OFF: Serial.print("TRANS_OFF"); break;
        }
        Serial.print(" | Presence: ");
        Serial.print(ld2410PresenceDetected ? "YES" : "NO");
        Serial.print(" | ");
        Serial.print(ld2410DistanceMillimeters);
        Serial.print(" mm | LED2: ");
        Serial.print(ld2410CurrentBrightness);
        Serial.print(" | Servo: ");
        Serial.println(currentServoAngle);

        lastPrintTimeMilliseconds = currentTimeMilliseconds;
    }

    delay(50);
}