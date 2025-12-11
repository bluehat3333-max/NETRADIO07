// Button test sketch for debugging
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

#define ROTARY_SW     7
#define I2C_SDA_PIN   4 
#define I2C_SCL_PIN   5 

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET     -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
    Serial.begin(115200);
    Serial.println("Button Test Starting...");
    
    pinMode(ROTARY_SW, INPUT_PULLUP);
    
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
        Serial.println(F("SSD1306 allocation failed"));
        for(;;);
    }
    
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0,0);
    display.println("Button Test");
    display.println("Press button now!");
    display.display();
    
    // Wait for system stabilization
    delay(2000);
    
    Serial.println("Ready for button test");
}

void loop() {
    static bool lastState = HIGH;
    static unsigned long pressStart = 0;
    static bool tracking = false;
    
    bool currentState = digitalRead(ROTARY_SW);
    
    // Button pressed (falling edge)
    if (lastState == HIGH && currentState == LOW) {
        pressStart = millis();
        tracking = true;
        Serial.println("Button PRESSED");
        
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("Button PRESSED!");
        display.println("Hold for 3 seconds");
        display.display();
    }
    
    // Button released (rising edge)
    if (lastState == LOW && currentState == HIGH && tracking) {
        unsigned long pressDuration = millis() - pressStart;
        tracking = false;
        
        Serial.printf("Button RELEASED after %lu ms\n", pressDuration);
        
        display.clearDisplay();
        display.setCursor(0,0);
        if (pressDuration >= 3000) {
            display.println("LONG PRESS!");
            display.println("WiFiManager Mode");
        } else {
            display.println("SHORT PRESS!");
            display.println("SmartConfig Mode");
        }
        display.printf("Duration: %lums", pressDuration);
        display.display();
        
        delay(2000);
        
        // Reset display
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("Button Test");
        display.println("Press button again!");
        display.display();
    }
    
    // Show countdown while pressed
    if (tracking && currentState == LOW) {
        unsigned long elapsed = millis() - pressStart;
        if (elapsed % 500 == 0) {
            int remaining = (3000 - elapsed) / 1000;
            if (remaining >= 0) {
                display.fillRect(100, 24, 28, 8, SSD1306_BLACK);
                display.setCursor(100, 24);
                display.print(remaining + 1);
                display.display();
            }
        }
    }
    
    lastState = currentState;
    delay(50);
}