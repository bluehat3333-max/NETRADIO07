#include <Arduino.h>
#include <RotaryEncoder.h>

// Rotary Encoder Pins
#define ROTARY_CLK    8
#define ROTARY_DT     9
#define ROTARY_SW     7

// Rotary Encoder Object
// Using LatchMode::TWO03 as determined from the user's RotaryEncoder.h
RotaryEncoder encoder(ROTARY_DT, ROTARY_CLK, RotaryEncoder::LatchMode::TWO03);

// Flag for switch press
volatile bool switch_pressed = false;
volatile unsigned long last_switch_press_time = 0;
const unsigned long debounce_delay = 200; // Milliseconds

// --- ISRs ---
void IRAM_ATTR readEncoder() {
    encoder.tick();
}

void IRAM_ATTR handleSwitchPress() {
    unsigned long current_time = millis();
    if (current_time - last_switch_press_time > debounce_delay) {
        switch_pressed = true;
        last_switch_press_time = current_time;
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("Rotary Encoder Test Program");

    // Initialize Rotary Encoder Pins
    pinMode(ROTARY_DT, INPUT_PULLUP);
    pinMode(ROTARY_CLK, INPUT_PULLUP);
    pinMode(ROTARY_SW, INPUT_PULLUP);

    // Attach interrupts
    attachInterrupt(digitalPinToInterrupt(ROTARY_DT), readEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ROTARY_CLK), readEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ROTARY_SW), handleSwitchPress, FALLING);
}

void loop() {
    // Check for encoder rotation
    static long oldPosition = -999;
    long newPosition = encoder.getPosition();
    if (oldPosition != newPosition) {
        Serial.print("Encoder Position: ");
        Serial.print(newPosition);
        Serial.print(", Direction: ");
        Serial.println((int)encoder.getDirection());
        oldPosition = newPosition;
    }

    // Check for switch press
    if (switch_pressed) {
        Serial.println("Switch Pressed!");
        switch_pressed = false; // Reset flag
    }

    delay(10); // Small delay to prevent excessive serial output
}