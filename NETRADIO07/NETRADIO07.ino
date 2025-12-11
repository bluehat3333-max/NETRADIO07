#include "Arduino.h"
#include "WiFi.h"
#include "Audio.h"
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <vector>
#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFiManager.h> // For web-based WiFi configuration

// --- Audio Command Queue ---
typedef struct {
    enum { CMD_CHANGE_STATION } type;
    int station_index;
} AudioCommand;

QueueHandle_t audio_command_queue;

// --- WiFi Credentials ---
// PLEASE EDIT WITH YOUR WIFI DETAILS
const char* ssid = "xg100n-9b6475-1";
const char* password = "11922960";

// --- Pin Definitions ---
// I2S Pins
#define I2S_DOUT      15
#define I2S_BCLK      16
#define I2S_LRC       17
// Rotary Encoder Pins
#define ROTARY_CLK    6
#define ROTARY_DT     18
#define ROTARY_SW     7
// Power Switch Pin
#define POWER_SW_PIN  13 // Connect a momentary push button to GND for sleeping

#define I2C_SDA_PIN   4 // I2C SDA pin
#define I2C_SCL_PIN   5 // I2C SCL pin

#include <RotaryEncoder.h> // Include the RotaryEncoder library
#include <Adafruit_NeoPixel.h>

// --- Built-in LED Settings ---
#define NEOPIXEL_PIN 48
#define NEOPIXEL_COUNT 1
Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// --- Global Objects ---
Audio audio;
RotaryEncoder encoder(ROTARY_DT, ROTARY_CLK, RotaryEncoder::LatchMode::FOUR3);
Preferences preferences;

// OLED Display object
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET     -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- Global State Variables ---
volatile int encoder_mode = 0; // 0 for volume, 1 for station
volatile unsigned long last_switch_press_time = 0;
const unsigned long debounce_delay = 1000; // Milliseconds (Refractory period for switch)

// --- Forward Declarations for ISRs ---
void IRAM_ATTR readEncoder();
void IRAM_ATTR handleSwitchPress();

// --- Radio Stations (Loaded from Gist) ---
std::vector<String> station_names;
std::vector<String> station_urls;
std::vector<String> station_types; // For optional codec type
const char* station_list_url = "https://gist.githubusercontent.com/KimYuki3/343960ab2d026203d0fe0445c7dfb357/raw/";
uint8_t cur_station = 0; // Start at the first station, will be overwritten by saved preference

// --- Function to download and parse station list ---
void updateStationList() {
    Serial.println("Updating station list from Gist...");
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println("Updating stations...");
    display.display();

    HTTPClient http;
    WiFiClientSecure client;
    client.setInsecure();

    if (http.begin(client, station_list_url)) {
        int httpCode = http.GET();
        if (httpCode == HTTP_CODE_OK) {
            String payload = http.getString();
            station_names.clear();
            station_urls.clear();
            station_types.clear();
            int from = 0;
            int to;
            do {
                to = payload.indexOf('\n', from);
                String line = (to == -1) ? payload.substring(from) : payload.substring(from, to);
                line.trim();
                if (line.length() > 0) {
                    int comma1_index = line.indexOf(',');
                    if (comma1_index > 0) {
                        String name = line.substring(0, comma1_index);
                        int comma2_index = line.indexOf(',', comma1_index + 1);
                        String url;
                        String type = ""; // Default to empty type

                        if (comma2_index > 0) { // Type is specified
                            url = line.substring(comma1_index + 1, comma2_index);
                            type = line.substring(comma2_index + 1);
                        } else { // Type is not specified
                            url = line.substring(comma1_index + 1);
                        }

                        name.trim();
                        url.trim();
                        type.trim();

                        if (name.length() > 0 && url.length() > 0) {
                            station_names.push_back(name);
                            station_urls.push_back(url);
                            station_types.push_back(type);
                        }
                    }
                }
                from = to + 1;
            } while (to != -1);
            Serial.printf("Loaded %d stations.\n", station_names.size());
        } else {
            Serial.printf("[HTTP] GET... failed, error: %s (code: %d)\n", http.errorToString(httpCode).c_str(), httpCode);
        }
        http.end();
    } else {
        Serial.printf("[HTTP] Unable to connect\n");
    }
}

// --- Task Definitions ---
void audio_loop_task(void *pvParameters) {
    Serial.println("Audio task started.");
    
    while (1) {
        AudioCommand received_command;
        if (xQueueReceive(audio_command_queue, &received_command, 0) == pdPASS) {
            if (received_command.type == AudioCommand::CMD_CHANGE_STATION) {
                int station_to_play = received_command.station_index;
                if (station_to_play < station_urls.size()) {
                    audio.stopSong();
                    vTaskDelay(150 / portTICK_PERIOD_MS);
                    const char* url = station_urls[station_to_play].c_str();
                    const char* type = station_types[station_to_play].c_str();
                    if (strlen(type) > 0) {
                        audio.connecttohost(url, type);
                    } else {
                        audio.connecttohost(url);
                    }
                }
            }
        }
        audio.loop();
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void encoder_task(void *pvParameters) {
    // Debounce for power button
    static unsigned long last_power_press_time = 0;
    const unsigned long power_debounce_delay = 1000; // 1 second debounce

    while (1) {
        // --- Power Button Logic (Short Press to Sleep) ---
        if (digitalRead(POWER_SW_PIN) == LOW) {
            if (millis() - last_power_press_time > power_debounce_delay) {
                last_power_press_time = millis();
                Serial.println("Power button pressed. Entering deep sleep...");
                
                audio.stopSong();
                display.clearDisplay();
                display.setCursor(0,0);
                display.println("Sleeping...");
                display.display();
                delay(1000);
                display.ssd1306_command(SSD1306_DISPLAYOFF);

                esp_deep_sleep_start();
            }
        }

        // --- Encoder Logic (Original) ---
        int new_position = encoder.getPosition();
        if (new_position != 0) {
            if (encoder_mode == 0) { // Volume Control Mode
                int current_volume = audio.getVolume();
                // PCM5102 clips above ~12, use external amplifier for higher volume
                if (new_position > 0) { current_volume = min(current_volume + 1, 12); } 
                else { current_volume = max(current_volume - 1, 0); }
                audio.setVolume(current_volume);
                Serial.printf("Volume: %d/12\n", current_volume);
                preferences.begin("net-radio", false);
                preferences.putUChar("last_volume", current_volume);
                preferences.end();
            } else { // Station Selection Mode
                if (!station_urls.empty()) {
                    if (new_position > 0) { cur_station = (cur_station + 1) % station_urls.size(); } 
                    else { cur_station = (cur_station == 0) ? station_urls.size() - 1 : cur_station - 1; }
                    
                    AudioCommand command = { .type = AudioCommand::CMD_CHANGE_STATION, .station_index = cur_station };
                    xQueueSend(audio_command_queue, &command, portMAX_DELAY);

                    preferences.begin("net-radio", false);
                    preferences.putUInt("last_station", cur_station);
                    preferences.end();
                }
            }
            encoder.setPosition(0);
        }

        vTaskDelay(50 / portTICK_PERIOD_MS); // Poll for button
    }
}

void display_task(void *pvParameters) {
    static int16_t scroll_x = 0;
    static uint32_t last_update = 0;
    const uint32_t update_interval = 150; // Reduced update frequency

    while (1) {
        uint32_t now = millis();
        if (now - last_update < update_interval) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
            continue;
        }
        last_update = now;
        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE);

        // --- Top Line: Mode, Value, RSSI ---
        display.setTextSize(1);
        display.setCursor(0, 0);
        String top_line_str;
        if (encoder_mode == 0) {
            top_line_str = "Vol: " + String(audio.getVolume()) + "/12";
        } else {
            if (!station_names.empty()) {
                top_line_str = "St: " + String(cur_station + 1) + "/" + String(station_names.size());
            } else {
                top_line_str = "Station: N/A";
            }
        }
        display.print(top_line_str);

        if (WiFi.status() == WL_CONNECTED) {
            String rssi_str = String(WiFi.RSSI()) + "dBm";
            int16_t x1, y1;
            uint16_t w, h;
            display.getTextBounds(rssi_str, 0, 0, &x1, &y1, &w, &h);
            display.setCursor(SCREEN_WIDTH - w, 0);
            display.print(rssi_str);
        }

        // --- Main Area: Station Name ---
        display.setTextSize(2);
        String station_name = "No stations loaded";
        if (!station_names.empty()) {
            station_name = station_names[cur_station];
        }
        
        int16_t x1, y1;
        uint16_t w, h;
        display.getTextBounds(station_name, 0, 0, &x1, &y1, &w, &h);

        if (w > SCREEN_WIDTH) { // If text is wider than screen, scroll it
            display.setCursor(scroll_x, 24);
            display.print(station_name);
            scroll_x -= 2;
            if (scroll_x < -w) { 
                scroll_x = SCREEN_WIDTH;
            }
        } else { // Center the text
            display.setCursor((SCREEN_WIDTH - w) / 2, 24);
            display.print(station_name);
            scroll_x = 0; // Reset scroll for next time
        }

        // --- Bottom Line: IP Address ---
        display.setTextSize(1);
        display.setCursor(0, 56);
        if (WiFi.status() == WL_CONNECTED) {
            display.print(WiFi.localIP());
        } else {
            display.print("Connecting...");
        }

        display.display();
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void WiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info){
    Serial.printf("[WiFi-event] event: %d\n", event);
    switch (event) {
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            Serial.printf("WiFi lost connection. Reason: %d\n", info.wifi_sta_disconnected.reason);
            Serial.println("ESP32 will attempt to reconnect automatically.");
            break;
        default:
            break;
    }
}

void startSmartConfig() {
    Serial.println("Starting SmartConfig...");
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println("SmartConfig Mode");
    display.println("Use ESPTouch v2 App");
    display.display();
    
    WiFi.mode(WIFI_AP_STA);
    
    // Use ESPTouch v2 for better Android compatibility
    WiFi.beginSmartConfig(SC_TYPE_ESPTOUCH_V2);
    
    // Add timeout to prevent infinite waiting
    unsigned long startTime = millis();
    const unsigned long timeout = 120000; // 2 minutes timeout
    
    while (!WiFi.smartConfigDone()) { 
        delay(500); 
        Serial.print("."); 
        
        // Check for timeout
        if (millis() - startTime > timeout) {
            Serial.println("\nSmartConfig timeout!");
            display.clearDisplay();
            display.setCursor(0,0);
            display.println("SmartConfig timeout");
            display.println("Restarting...");
            display.display();
            delay(2000);
            ESP.restart();
            return;
        }
    } 
    
    Serial.println("\nSmartConfig received.");
    Serial.println("Waiting for WiFi connection");
    
    startTime = millis();
    while (WiFi.status() != WL_CONNECTED) { 
        delay(500); 
        Serial.print("."); 
        
        // Check for connection timeout
        if (millis() - startTime > 30000) { // 30 seconds for connection
            Serial.println("\nWiFi connection timeout!");
            display.clearDisplay();
            display.setCursor(0,0);
            display.println("Connection failed");
            display.println("Restarting...");
            display.display();
            delay(2000);
            ESP.restart();
            return;
        }
    }
    
    Serial.println("\nWiFi connected via SmartConfig!");
    Serial.printf("IP: %s\n", WiFi.localIP().toString().c_str());
}

void startWiFiManager() {
    Serial.println("Starting WiFiManager...");
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println("WiFi Config Mode");
    display.println("Connect to:");
    display.println("NETRADIO-Setup");
    display.println("192.168.4.1");
    display.display();
    
    WiFiManager wm;
    
    // Set timeout for config portal
    wm.setConfigPortalTimeout(300); // 5 minutes
    
    // Start config portal
    if (!wm.startConfigPortal("NETRADIO-Setup")) {
        Serial.println("Failed to connect and hit timeout");
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("WiFi config failed");
        display.println("Restarting...");
        display.display();
        delay(3000);
        ESP.restart();
    }
    
    Serial.println("WiFi connected via WiFiManager!");
    Serial.printf("IP: %s\n", WiFi.localIP().toString().c_str());
}

void setup() {
    setCpuFrequencyMhz(240);
    Serial.begin(115200);
    Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
    Serial.printf("PSRAM size: %d bytes\n", ESP.getPsramSize());
    
    audio_command_queue = xQueueCreate(5, sizeof(AudioCommand));
    pinMode(ROTARY_DT, INPUT_PULLUP);
    pinMode(ROTARY_CLK, INPUT_PULLUP);
    pinMode(ROTARY_SW, INPUT_PULLUP);
    pinMode(POWER_SW_PIN, INPUT_PULLUP);

    // Temporarily disable interrupts during setup
    // attachInterrupt(digitalPinToInterrupt(ROTARY_DT), readEncoder, CHANGE);
    // attachInterrupt(digitalPinToInterrupt(ROTARY_CLK), readEncoder, CHANGE);
    // attachInterrupt(digitalPinToInterrupt(ROTARY_SW), handleSwitchPress, FALLING);

    // Wakeup from deep sleep when the ROTARY encoder button is pressed (pin goes LOW)
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_7, 0);

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
        Serial.println(F("SSD1306 allocation failed"));
        for(;;);
    }
    display.display();
    delay(1000);
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    
    pixels.begin();
    pixels.setBrightness(20);

    // Wait for system to stabilize first
    delay(1000);
    
    // Check if button is pressed during startup for config mode
    Serial.println("Checking button state...");
    Serial.printf("Button state: %s\n", digitalRead(ROTARY_SW) == LOW ? "PRESSED" : "RELEASED");
    
    if (digitalRead(ROTARY_SW) == LOW) {
        Serial.println("Button detected pressed - entering config mode selection");
        
        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(0,0);
        display.println("BUTTON PRESSED!");
        display.println("Release = SmartConfig");
        display.println("Hold 3sec = WebConfig");
        display.display();
        
        unsigned long pressStart = millis();
        bool longPress = false;
        
        // Monitor button for 3 seconds
        while ((millis() - pressStart) < 3000) {
            if (digitalRead(ROTARY_SW) == HIGH) {
                // Button released - wait a bit more to confirm
                delay(100);
                if (digitalRead(ROTARY_SW) == HIGH) {
                    Serial.println("Button released early - SmartConfig mode");
                    break;
                }
            }
            
            // Update countdown display
            unsigned long elapsed = millis() - pressStart;
            if (elapsed % 500 == 0) {
                int remaining = (3000 - elapsed) / 1000;
                display.fillRect(120, 16, 8, 8, SSD1306_BLACK);
                display.setCursor(120, 16);
                display.print(remaining + 1);
                display.display();
            }
            delay(50);
        }
        
        // Check final state
        if ((millis() - pressStart) >= 3000 && digitalRead(ROTARY_SW) == LOW) {
            longPress = true;
            Serial.println("Long press detected - WiFiManager mode");
        }
        
        // Wait for button release
        while (digitalRead(ROTARY_SW) == LOW) {
            delay(50);
        }
        Serial.println("Button released");
        delay(500); // Debounce
        
        if (longPress) {
            display.clearDisplay();
            display.setCursor(0,0);
            display.println("WEB CONFIG MODE");
            display.println("Starting...");
            display.display();
            delay(1000);
            startWiFiManager();
        } else {
            display.clearDisplay();
            display.setCursor(0,0);
            display.println("SMARTCONFIG MODE");
            display.println("Starting...");
            display.display();
            delay(1000);
            startSmartConfig();
        }
    } else {
        display.setCursor(0,0);
        display.println("Connecting to WiFi...");
        display.display();

        WiFi.onEvent(WiFiEvent);
        WiFi.mode(WIFI_STA);
        WiFi.setSleep(WIFI_PS_NONE); // Disable WiFi power saving for stable streaming
        WiFi.begin(ssid, password);
        Serial.println("Connecting to WiFi...");

        bool led_on = false;
        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED && attempts < 20) {
            pixels.setPixelColor(0, pixels.Color(led_on ? 50 : 0, 0, 0));
            pixels.show();
            led_on = !led_on;
            delay(500);
            Serial.print(".");
            attempts++;
        }
    }

    if (WiFi.status() != WL_CONNECTED) {
        pixels.setPixelColor(0, pixels.Color(50, 0, 0));
        pixels.show();
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("WiFi Connection Failed!");
        display.display();
        Serial.printf("\nWiFi Connection Failed! Status: %d\n", WiFi.status());
        while(1);
    }

    Serial.println("\nWiFi connected");
    pixels.setPixelColor(0, pixels.Color(0, 50, 0));
    pixels.show();
    
    // Now enable interrupts after WiFi is connected
    attachInterrupt(digitalPinToInterrupt(ROTARY_DT), readEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ROTARY_CLK), readEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ROTARY_SW), handleSwitchPress, FALLING);
    
    updateStationList();

    preferences.begin("net-radio", false);
    cur_station = preferences.getUInt("last_station", 30);
    uint8_t saved_volume = preferences.getUChar("last_volume", 7); // Safe default for PCM5102
    preferences.end();

    if (cur_station >= station_urls.size()) { cur_station = 0; }

    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    audio.setInBufferSize(65536); // 64KB buffer for ESP32-S3
    audio.setConnectionTimeout(5000, 10000);
    // PCM5102 has fixed gain, so we limit digital volume to prevent clipping
    audio.setVolume(saved_volume);
    
    if (!station_urls.empty()) {
        AudioCommand command = { .type = AudioCommand::CMD_CHANGE_STATION, .station_index = cur_station };
        xQueueSend(audio_command_queue, &command, portMAX_DELAY);
    } else {
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("Failed to load");
        display.println("station list!");
        display.display();
        while(1) { delay(1000); }
    }

    xTaskCreatePinnedToCore(display_task, "DisplayTask", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(encoder_task, "EncoderTask", 3072, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(audio_loop_task, "AudioLoopTask", 18432, NULL, 10, NULL, 1);
}

void loop() {
    // Main loop is empty, everything is handled by tasks.
}

void IRAM_ATTR readEncoder() {
    encoder.tick();
}

void IRAM_ATTR handleSwitchPress() {
    unsigned long current_time = millis();
    if (current_time - last_switch_press_time > debounce_delay) {
        encoder_mode = 1 - encoder_mode;
        last_switch_press_time = current_time;
    }
}

// optional audio info functions
void audio_info(const char *info){ Serial.print("info        "); Serial.println(info); }
void audio_id3data(const char *info){ Serial.print("id3data     "); Serial.println(info); }
void audio_eof_mp3(const char *info){ Serial.print("eof_mp3     "); Serial.println(info); }
void audio_showstation(const char *info){ Serial.print("station     "); Serial.println(info); }
void audio_showstreaminfo(const char *info){ Serial.print("streaminfo  "); Serial.println(info); }
void audio_showstreamtitle(const char *info){ Serial.print("streamtitle "); Serial.println(info); }
void audio_bitrate(const char *info){ Serial.print("bitrate     "); Serial.println(info); }
void audio_commercial(const char *info){ Serial.print("commercial  "); Serial.println(info); }
void audio_icyurl(const char *info){ Serial.print("icyurl      "); Serial.println(info); }
void audio_lasthost(const char *info){ Serial.print("lasthost    "); Serial.println(info); }
void audio_eof_speech(const char *info){ Serial.print("eof_speech  "); Serial.println(info); }
