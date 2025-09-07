#include <Arduino.h>

// Rotary Encoder Switch Pin
#define ROTARY_SW     7

// Flag for switch press
volatile bool switch_pressed = false;
volatile unsigned long last_switch_press_time = 0;
const unsigned long debounce_delay = 200; // Milliseconds

// --- ISR ---
void IRAM_ATTR handleSwitchPress() {
    unsigned long current_time = millis();
    if (current_time - last_switch_press_time > debounce_delay) {
        switch_pressed = true;
        last_switch_press_time = current_time;
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("Rotary Encoder Switch Test Program");

    // Initialize Rotary Encoder Switch Pin
    pinMode(ROTARY_SW, INPUT_PULLUP);

    // Attach interrupt for switch
    attachInterrupt(digitalPinToInterrupt(ROTARY_SW), handleSwitchPress, FALLING);
}

void loop() {
    // Check for switch press
    if (switch_pressed) {
        Serial.println("Switch Pressed!");
        switch_pressed = false; // Reset flag
    }

    delay(10); // Small delay to prevent excessive serial output
}