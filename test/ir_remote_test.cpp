/**
 * IR Remote Test Sketch for TTGO T18 ESP32
 * 
 * Purpose: Decode IR remote button codes
 * IR Receiver: Connected to GPIO 26
 * 
 * Instructions:
 * 1. Upload this sketch
 * 2. Open serial monitor at 115200 baud
 * 3. Press buttons on your remote
 * 4. Record the codes for buttons you want to use
 */

#include <Arduino.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>

#define IR_RECEIVE_PIN 26  // IR receiver connected to GPIO 26

IRrecv irrecv(IR_RECEIVE_PIN);
decode_results results;

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n\n=== TTGO T18 - IR Remote Decoder ===");
    Serial.println("IR Receiver on GPIO 26");
    Serial.println("Press buttons on your remote...\n");
    
    irrecv.enableIRIn();
    Serial.println("Ready to receive IR signals!");
}

void loop() {
    if (irrecv.decode(&results)) {
        // Only print if it's not a repeat signal
        if (results.value != 0xFFFFFFFF) {
            Serial.println("--- IR Signal Received ---");
            Serial.print("Protocol: ");
            Serial.println(typeToString(results.decode_type));
            Serial.print("Code: 0x");
            Serial.println(results.value, HEX);
            Serial.print("Bits: ");
            Serial.println(results.bits);
            
            // Print raw timing if needed for debugging
            if (results.rawlen > 0) {
                Serial.print("Raw data length: ");
                Serial.println(results.rawlen);
            }
            
            Serial.println("--------------------------\n");
        } else {
            // Repeat signal indicator
            Serial.println("[Button held - repeat signal]");
        }
        
        irrecv.resume();  // Receive the next value
        delay(100);  // Small delay to avoid flooding serial
    }
}
