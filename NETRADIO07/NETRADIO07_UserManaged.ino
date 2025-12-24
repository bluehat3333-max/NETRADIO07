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
#include "user_station_manager_ssd1306.h"

// --- Audio Command Queue ---
typedef struct {
    enum { CMD_CHANGE_STATION } type;
    int station_index;
} AudioCommand;

QueueHandle_t audio_command_queue;

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
UserStationManager_SSD1306 stationManager(&display);

// OLED Display object
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET     -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- Global State Variables ---
volatile int encoder_mode = 0; // 0 for volume, 1 for station
volatile unsigned long last_switch_press_time = 0;
const unsigned long debounce_delay = 500; // Milliseconds
const unsigned long long_press_duration = 2000; // 2 seconds for long press
const unsigned long config_press_duration = 5000; // 5 seconds for config mode
volatile bool entering_sleep = false;
volatile bool config_mode_active = false;

uint8_t cur_station = 0;
uint8_t internal_volume = 30; // Volume (0-42 for 0.5 step control)

// --- Forward Declarations for ISRs ---
void IRAM_ATTR readEncoder();

// --- Task Definitions ---
void encoder_task(void *pvParameters) {
    unsigned long button_press_time = 0;
    bool button_currently_pressed = false;
    bool long_press_triggered = false;
    bool config_press_triggered = false;
    
    while (1) {
        bool current_state = (digitalRead(ROTARY_SW) == LOW);
        
        // --- Rotary Switch Press Detection ---
        if (current_state && !button_currently_pressed) {
            // Button just pressed
            button_currently_pressed = true;
            button_press_time = millis();
            long_press_triggered = false;
            config_press_triggered = false;
            Serial.println("DEBUG: Button pressed, starting timer...");
        }
        else if (current_state && button_currently_pressed && !config_press_triggered) {
            // Button still pressed, check duration
            unsigned long press_duration = millis() - button_press_time;
            
            if (press_duration > config_press_duration && !config_mode_active) {
                // Enter config mode (5+ second press)
                config_press_triggered = true;
                Serial.println("DEBUG: Config press detected. Entering station config mode...");
                
                config_mode_active = true;
                stationManager.startConfigMode();
                
                // Set NeoPixel to blue for config mode
                pixels.setPixelColor(0, pixels.Color(0, 0, 255));
                pixels.show();
                
            } else if (press_duration > long_press_duration && !long_press_triggered) {
                // Deep sleep (2-5 second press)
                long_press_triggered = true;
                Serial.printf("DEBUG: Long press detected (%lu ms). Entering deep sleep...\n", press_duration);
                
                // Show SLEEP message
                display.clearDisplay();
                display.setTextSize(2);
                display.setTextColor(SSD1306_WHITE);
                display.setCursor(20, 20);
                display.println("SLEEP");
                display.display();
                
                // Set NeoPixel to red for sleep
                pixels.setPixelColor(0, pixels.Color(255, 0, 0));
                pixels.show();
                
                // Wait for button release
                while(digitalRead(ROTARY_SW) == LOW) {
                    vTaskDelay(50 / portTICK_PERIOD_MS);
                }
                button_currently_pressed = false;
                
                // Save settings
                preferences.begin("netradio07", false);
                preferences.putUChar("volume", internal_volume / 2);
                preferences.putUInt("station", cur_station);
                preferences.end();
                
                // Stop audio
                audio.stopSong();
                vTaskDelay(500 / portTICK_PERIOD_MS);
                
                // Turn off NeoPixel and enter deep sleep
                pixels.clear();
                pixels.show();
                
                // Configure wakeup and enter deep sleep
                esp_sleep_enable_ext0_wakeup(GPIO_NUM_7, 0);
                esp_deep_sleep_start();
            }
        }
        else if (!current_state && button_currently_pressed) {
            // Button released
            unsigned long press_duration = millis() - button_press_time;
            Serial.printf("DEBUG: Button released after %lu ms\n", press_duration);
            
            if (config_mode_active) {
                // Exit config mode
                config_mode_active = false;
                stationManager.stopConfigMode();
                Serial.println("Config mode ended");
                
                // Set NeoPixel back to green
                pixels.setPixelColor(0, pixels.Color(0, 255, 0));
                pixels.show();
                
            } else if (!long_press_triggered && !config_press_triggered) {
                // Handle short press - toggle encoder mode
                if (millis() - last_switch_press_time > debounce_delay) {
                    encoder_mode = (encoder_mode == 0) ? 1 : 0;
                    last_switch_press_time = millis();
                    Serial.printf("DEBUG: Mode switched to %d\n", encoder_mode);
                    
                    // Flash NeoPixel to indicate mode change
                    pixels.setPixelColor(0, pixels.Color(255, 255, 0));
                    pixels.show();
                    vTaskDelay(200 / portTICK_PERIOD_MS);
                    pixels.setPixelColor(0, pixels.Color(0, 255, 0));
                    pixels.show();
                }
            }
            
            button_currently_pressed = false;
        }
        
        // Handle config mode client requests
        if (config_mode_active) {
            stationManager.handleClient();
        }
        
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void audio_loop_task(void *pvParameters) {
    Serial.println("Audio task started.");
    
    while (1) {
        AudioCommand received_command;
        if (xQueueReceive(audio_command_queue, &received_command, 0) == pdPASS) {
            if (received_command.type == AudioCommand::CMD_CHANGE_STATION) {
                int station_to_play = received_command.station_index;
                std::vector<String>& station_urls = stationManager.getStationUrls();
                if (station_to_play < station_urls.size()) {
                    Serial.printf("Switching to station %d: %s\n", station_to_play, station_urls[station_to_play].c_str());
                    audio.stopSong();
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                    audio.connecttohost(station_urls[station_to_play].c_str());
                }
            }
        }
        
        audio.loop();
        
        // Handle encoder rotation
        int new_position = encoder.getPosition();
        if (new_position != 0 && !config_mode_active) {
            if (encoder_mode == 0) { // Volume Control Mode
                if (new_position > 0) {
                    if (internal_volume < 42) internal_volume++;
                } else {
                    if (internal_volume > 0) internal_volume--;
                }
                audio.setVolume(internal_volume / 2);
                
                // Save volume preference
                preferences.begin("netradio07", false);
                preferences.putUChar("volume", internal_volume / 2);
                preferences.end();
                
            } else { // Station Selection Mode
                std::vector<String>& station_urls = stationManager.getStationUrls();
                if (!station_urls.empty()) {
                    if (new_position > 0) { 
                        cur_station = (cur_station + 1) % station_urls.size(); 
                    } else { 
                        cur_station = (cur_station == 0) ? station_urls.size() - 1 : cur_station - 1; 
                    }
                    
                    // Queue station change command
                    AudioCommand cmd;
                    cmd.type = AudioCommand::CMD_CHANGE_STATION;
                    cmd.station_index = cur_station;
                    xQueueSend(audio_command_queue, &cmd, portMAX_DELAY);
                    
                    // Save station preference
                    preferences.begin("netradio07", false);
                    preferences.putUInt("station", cur_station);
                    preferences.end();
                }
            }
            encoder.setPosition(0);
        }
        
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void display_task(void *pvParameters) {
    static int16_t text_x_pos = 0;
    static uint16_t text_width = 0;
    static uint32_t last_scroll_time = 0;
    const uint16_t scroll_speed = 2;
    const uint16_t scroll_delay = 100;

    while (1) {
        // Don't update main display while in config mode
        if (config_mode_active) {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.println("NETRADIO07 - Playing:");
        
        // Display WiFi signal strength
        if (WiFi.status() == WL_CONNECTED) {
            int8_t rssi = WiFi.RSSI();
            display.setCursor(90, 0);
            display.print(rssi);
            display.print("dBm");
            
            // Draw WiFi signal bars
            int bars = 0;
            if (rssi >= -50) bars = 4;
            else if (rssi >= -60) bars = 3;
            else if (rssi >= -70) bars = 2;
            else if (rssi >= -80) bars = 1;
            
            for (int i = 0; i < 4; i++) {
                int bar_h = (i + 1) * 2;
                int x = 120 + i * 2;
                int y = 8 - bar_h;
                if (i < bars) {
                    display.fillRect(x, y, 1, bar_h);
                } else {
                    display.drawPixel(x, y + bar_h - 1);
                }
            }
        }
        
        // Display current mode and value
        display.setCursor(0, 16);
        if (encoder_mode == 0) {
            display.printf("Volume: %.1f", internal_volume / 2.0);
        } else {
            int station_count = stationManager.getStationCount();
            if (station_count > 0) {
                display.printf("Station: %d/%d", cur_station + 1, station_count);
            } else {
                display.print("No Stations");
            }
        }
        
        // Display current station name (with scrolling if too long)
        const char* station_name = "No stations loaded";
        std::vector<String>& station_names = stationManager.getStationNames();
        if (!station_names.empty() && cur_station < station_names.size()) {
            station_name = station_names[cur_station].c_str();
        }
        
        display.setTextSize(1);
        text_width = strlen(station_name) * 6; // Approximate character width
        
        if (text_width > SCREEN_WIDTH) {
            // Scroll long text
            if (millis() - last_scroll_time > scroll_delay) {
                text_x_pos -= scroll_speed;
                if (text_x_pos < -(text_width + 20)) {
                    text_x_pos = SCREEN_WIDTH;
                }
                last_scroll_time = millis();
            }
            display.setCursor(text_x_pos, 32);
        } else {
            // Center short text
            text_x_pos = (SCREEN_WIDTH - text_width) / 2;
            display.setCursor(text_x_pos, 32);
        }
        display.println(station_name);
        
        // Display control instructions
        display.setTextSize(1);
        display.setCursor(0, 48);
        display.print("Short:Mode Long:Sleep");
        display.setCursor(0, 56);
        display.print("5s:Config Mode");
        
        display.display();
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("\n=== NETRADIO07 User Station Manager ===");
    
    // Initialize NeoPixel
    pixels.begin();
    pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // Red during setup
    pixels.show();
    
    // Check wakeup reason
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
        Serial.println("Woken up from deep sleep by button press");
        delay(500);
        while(digitalRead(ROTARY_SW) == LOW) {
            delay(50);
        }
        delay(500);
    }
    
    // Initialize pins
    pinMode(ROTARY_DT, INPUT_PULLUP);
    pinMode(ROTARY_CLK, INPUT_PULLUP);
    pinMode(ROTARY_SW, INPUT_PULLUP);
    pinMode(POWER_SW_PIN, INPUT_PULLUP);
    
    // Setup encoder interrupt
    attachInterrupt(digitalPinToInterrupt(ROTARY_DT), readEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ROTARY_CLK), readEncoder, CHANGE);
    
    // Setup wakeup source for deep sleep
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_7, 0);
    
    // Initialize I2C and OLED display
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("SSD1306 allocation failed");
        for(;;);
    }
    
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("NETRADIO07");
    display.println("Initializing...");
    display.display();
    
    // Initialize SPIFFS for user station management
    if (!stationManager.initSPIFFS()) {
        Serial.println("SPIFFS initialization failed!");
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("SPIFFS Error!");
        display.display();
        while(1) { delay(1000); }
    }
    
    // Setup WiFi using WiFiManager
    WiFiManager wm;
    
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("WiFi Setup...");
    display.display();
    
    // Try to connect to saved WiFi or start config portal
    bool res = wm.autoConnect("NETRADIO07-Setup");
    
    if (!res) {
        Serial.println("Failed to connect to WiFi");
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("WiFi Failed!");
        display.display();
        ESP.restart();
    }
    
    Serial.println("WiFi connected successfully");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    
    // Load user stations
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Loading stations...");
    display.display();
    
    if (!stationManager.loadStationsFromFile()) {
        Serial.println("Failed to load station list!");
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("No stations found!");
        display.println("Hold button 5sec");
        display.println("for config mode");
        display.display();
        
        // Set NeoPixel to orange to indicate no stations
        pixels.setPixelColor(0, pixels.Color(255, 165, 0));
        pixels.show();
        
        // Wait for user to enter config mode
        while(stationManager.getStationCount() == 0 && !config_mode_active) {
            delay(100);
        }
    }
    
    // Load preferences
    preferences.begin("netradio07", false);
    cur_station = preferences.getUInt("station", 0);
    uint8_t saved_volume = preferences.getUChar("volume", 15);
    preferences.end();
    
    internal_volume = saved_volume * 2;
    
    if (cur_station >= stationManager.getStationCount()) {
        cur_station = 0;
    }
    
    Serial.printf("Loaded: station=%d, volume=%d\n", cur_station, saved_volume);
    
    // Initialize audio
    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    audio.setInBufferSize(262144); // 256KB buffer
    audio.setVolume(saved_volume);
    audio.setConnectionTimeout(10000, 5000);
    
    // Start playing first station
    std::vector<String>& station_urls = stationManager.getStationUrls();
    if (!station_urls.empty()) {
        Serial.printf("Starting station: %s\n", station_urls[cur_station].c_str());
        audio.connecttohost(station_urls[cur_station].c_str());
    }
    
    // Create audio command queue
    audio_command_queue = xQueueCreate(10, sizeof(AudioCommand));
    
    // Create tasks
    xTaskCreatePinnedToCore(display_task, "DisplayTask", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(encoder_task, "EncoderTask", 8192, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(audio_loop_task, "AudioLoopTask", 16384, NULL, 10, NULL, 0);
    
    // Set NeoPixel to green to indicate ready
    pixels.setPixelColor(0, pixels.Color(0, 255, 0));
    pixels.show();
    
    Serial.println("=== Setup Complete ===");
    Serial.println("Controls:");
    Serial.println("- Short press: Switch Volume/Station mode");
    Serial.println("- Long press (2s): Deep sleep");
    Serial.println("- Very long press (5s): Config mode");
    Serial.printf("- Web interface: http://%s (when in config mode)\n", WiFi.localIP().toString().c_str());
    Serial.printf("Loaded %d stations\n", stationManager.getStationCount());
}

void loop() {
    // Monitor system health
    static unsigned long last_heap_check = 0;
    if (millis() - last_heap_check > 30000) { // Every 30 seconds
        Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
        last_heap_check = millis();
        
        // Update NeoPixel based on WiFi status
        if (WiFi.status() == WL_CONNECTED) {
            if (!config_mode_active) {
                pixels.setPixelColor(0, pixels.Color(0, 255, 0)); // Green for connected
            }
        } else {
            pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // Red for disconnected
        }
        pixels.show();
        
        // Restart if critically low memory
        if (ESP.getFreeHeap() < 10000) {
            Serial.println("CRITICAL: Low memory, restarting...");
            ESP.restart();
        }
    }
    delay(1000);
}

void IRAM_ATTR readEncoder() {
    encoder.tick();
}

// Optional audio info functions
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