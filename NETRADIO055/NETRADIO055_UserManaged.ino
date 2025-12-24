#include "Arduino.h"
#include "WiFi.h"
#include "Audio.h"
#include "U8g2lib.h" // For OLED display
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <vector>
#include <Wire.h>
#include <Preferences.h>
#include <driver/rtc_io.h>
#include "user_station_manager.h"

// --- Pin Definitions ---
// I2S Pins
#define I2S_DOUT      3
#define I2S_BCLK      1
#define I2S_LRC       2
// OLED Display Pins
#define OLED_SCL 6
#define OLED_SDA 5
// Rotary Encoder Pins
#define ROTARY_CLK    8
#define ROTARY_DT     9
#define ROTARY_SW     7

#include <RotaryEncoder.h> // Include the RotaryEncoder library

// --- Global Objects ---
Audio audio;
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, OLED_SCL, OLED_SDA);
RotaryEncoder encoder(ROTARY_DT, ROTARY_CLK, RotaryEncoder::LatchMode::FOUR3);
Preferences preferences;
UserStationManager stationManager(&u8g2);

// Watchdog timer handle
hw_timer_t *watchdog_timer = NULL;
volatile bool watchdog_triggered = false;

// --- Global State Variables ---
volatile int encoder_mode = 0; // 0 for volume, 1 for station, 2 for config mode
uint8_t internal_volume; // For 0.5 step volume control
volatile unsigned long last_switch_press_time = 0;
const unsigned long debounce_delay = 500; // Milliseconds
const unsigned long long_press_duration = 2000; // 2 seconds for long press
const unsigned long config_press_duration = 5000; // 5 seconds for config mode
volatile bool entering_sleep = false; // Flag to stop display updates
volatile bool config_mode_active = false;

// Station variables - now managed by UserStationManager
uint8_t cur_station = 0;

// --- Forward Declarations for ISRs ---
void IRAM_ATTR readEncoder();

// Watchdog ISR
void IRAM_ATTR watchdog_isr() {
    watchdog_triggered = true;
}

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
                
            } else if (press_duration > long_press_duration && !long_press_triggered) {
                // Deep sleep (2-5 second press)
                long_press_triggered = true;
                Serial.printf("DEBUG: Long press detected (%lu ms). Entering deep sleep...\n", press_duration);
                
                // Set flag to stop display task from updating
                entering_sleep = true;
                vTaskDelay(200 / portTICK_PERIOD_MS);
                
                // Show DEEP SLEEP message
                u8g2.clearBuffer();
                u8g2.setFont(u8g2_font_ncenB18_tr);
                u8g2.drawStr(10, 30, "DEEP");
                u8g2.drawStr(10, 50, "SLEEP");
                u8g2.sendBuffer();
                
                // Wait for button release
                while(digitalRead(ROTARY_SW) == LOW) {
                    vTaskDelay(50 / portTICK_PERIOD_MS);
                }
                button_currently_pressed = false;
                
                Serial.println("Button released, waiting 3 seconds...");
                
                // Save current volume and station before sleep
                preferences.begin("net-radio", false);
                preferences.putUChar("last_volume", internal_volume / 2);
                preferences.putUInt("last_station", cur_station);
                preferences.end();
                
                // Stop audio and I2S
                audio.stopSong();
                Serial.println("Audio and I2S stopped");
                vTaskDelay(500 / portTICK_PERIOD_MS);
                
                // Wait 3 seconds while keeping DEEP SLEEP displayed
                for(int i = 0; i < 6; i++) {
                    vTaskDelay(500 / portTICK_PERIOD_MS);
                    if(digitalRead(ROTARY_SW) == LOW) {
                        Serial.println("Button pressed during countdown, cancelling sleep");
                        entering_sleep = false;
                        return;
                    }
                }
                
                // Configure wakeup and enter deep sleep
                esp_sleep_enable_ext0_wakeup(GPIO_NUM_7, 0);
                Serial.println("Entering deep sleep NOW...");
                Serial.flush();
                delay(1000);
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
            } else if (!long_press_triggered && !config_press_triggered) {
                // Handle short press - toggle encoder mode
                if (millis() - last_switch_press_time > debounce_delay) {
                    encoder_mode = (encoder_mode == 0) ? 1 : 0; // Toggle between volume and station
                    last_switch_press_time = millis();
                    Serial.printf("DEBUG: Mode switched to %d\n", encoder_mode);
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
        // Check for freeze
        if (watchdog_triggered) {
            Serial.println("WATCHDOG: Audio task frozen, restarting ESP32...");
            ESP.restart();
        }
        
        // Reset watchdog periodically
        if (watchdog_timer != NULL) {
            timerWrite(watchdog_timer, 0);
        }
        
        audio.loop();
        int new_position = encoder.getPosition();
        if (new_position != 0 && !config_mode_active) {
            if (encoder_mode == 0) { // Volume Control Mode
                if (new_position > 0) {
                    if (internal_volume < 42) internal_volume++;
                } else {
                    if (internal_volume > 0) internal_volume--;
                }
                audio.setVolume(internal_volume / 2);
                preferences.begin("net-radio", false);
                preferences.putUChar("last_volume", internal_volume / 2);
                preferences.end();
            } else { // Station Selection Mode
                std::vector<String>& station_urls = stationManager.getStationUrls();
                if (!station_urls.empty()) {
                    if (new_position > 0) { 
                        cur_station = (cur_station + 1) % station_urls.size(); 
                    } else { 
                        cur_station = (cur_station == 0) ? station_urls.size() - 1 : cur_station - 1; 
                    }
                    
                    Serial.printf("Changing to station %d: %s\n", cur_station, stationManager.getStationNames()[cur_station].c_str());
                    audio.stopSong();
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                    
                    audio.connecttohost(station_urls[cur_station].c_str());
                    
                    preferences.begin("net-radio", false);
                    preferences.putUInt("last_station", cur_station);
                    preferences.end();
                    Serial.printf("Saved station index: %d\n", cur_station);
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
    const uint16_t scroll_speed = 5;
    const uint16_t scroll_delay = 50;

    while (1) {
        // Stop updating display if entering sleep mode
        if (entering_sleep) {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        
        // Don't update main display while in config mode
        if (config_mode_active) {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_ncenB10_tr);
        u8g2.drawStr(0, 12, "Now Playing:");

        // Display WiFi signal strength
        if (WiFi.status() == WL_CONNECTED) {
            int8_t rssi = WiFi.RSSI();
            u8g2.setFont(u8g2_font_6x10_tf);
            char rssi_str[12];
            sprintf(rssi_str, "%ddBm", rssi);
            int str_width = u8g2.getStrWidth(rssi_str);
            u8g2.drawStr(127 - str_width, 60, rssi_str);
            
            // Draw WiFi signal icon
            int icon_x = 127 - str_width - 20;
            int icon_y = 52;
            
            // Determine signal strength (4 bars)
            int bars = 0;
            if (rssi >= -50) bars = 4;      // Excellent
            else if (rssi >= -60) bars = 3; // Good
            else if (rssi >= -70) bars = 2; // Fair
            else if (rssi >= -80) bars = 1; // Weak
            
            // Draw signal bars
            for (int i = 0; i < 4; i++) {
                int bar_h = (i + 1) * 2;
                if (i < bars) {
                    u8g2.drawBox(icon_x + i * 4, icon_y + 8 - bar_h, 3, bar_h);
                } else {
                    u8g2.drawFrame(icon_x + i * 4, icon_y + 8 - bar_h, 3, bar_h);
                }
            }
        }
        
        char display_str[30];
        if (encoder_mode == 0) {
            sprintf(display_str, "Vol: %.1f", internal_volume / 2.0);
        } else {
            int station_count = stationManager.getStationCount();
            if (station_count > 0) { 
                sprintf(display_str, "Station: %d/%d", cur_station + 1, station_count); 
            } else { 
                sprintf(display_str, "No Stations"); 
            }
        }
        int display_str_width = u8g2.getStrWidth(display_str);
        u8g2.drawStr((128 - display_str_width) / 2, 26, display_str);

        const char* current_station_name = "No stations loaded";
        std::vector<String>& station_names = stationManager.getStationNames();
        if (!station_names.empty() && cur_station < station_names.size()) {
            current_station_name = station_names[cur_station].c_str();
        }
        
        text_width = u8g2.getStrWidth(current_station_name);
        if (text_width > 128) {
            if (millis() - last_scroll_time > scroll_delay) {
                text_x_pos -= scroll_speed;
                if (text_x_pos < -(text_width + 20)) { text_x_pos = 128; }
                last_scroll_time = millis();
            }
            u8g2.drawStr(text_x_pos, 40, current_station_name);
        } else {
            text_x_pos = (128 - text_width) / 2;
            u8g2.drawStr(text_x_pos, 40, current_station_name);
        }
        
        u8g2.sendBuffer();
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void startSmartConfig() {
    Serial.println("Starting SmartConfig...");
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 10, "SmartConfig Mode");
    u8g2.drawStr(0, 25, "Use ESPTouch App");
    u8g2.sendBuffer();
    WiFi.mode(WIFI_AP_STA);
    WiFi.beginSmartConfig();
    while (!WiFi.smartConfigDone()) { delay(500); Serial.print("."); }
    Serial.println("\nSmartConfig received.");
    Serial.println("Waiting for WiFi connection");
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
}

void setup() {
    setCpuFrequencyMhz(240);
    Serial.begin(115200);
    
    // Disable WiFi power saving for better streaming
    WiFi.setSleep(false);
    
    // Check wakeup reason
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    Serial.printf("Wakeup reason: %d\n", wakeup_reason);
    if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
        Serial.println("Woken up by button press");
        delay(500);
        while(digitalRead(ROTARY_SW) == LOW) {
            delay(50);
        }
        delay(500);
    }
    
    pinMode(ROTARY_DT, INPUT_PULLUP);
    pinMode(ROTARY_CLK, INPUT_PULLUP);
    pinMode(ROTARY_SW, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ROTARY_DT), readEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ROTARY_CLK), readEncoder, CHANGE);
    
    // Setup wakeup source
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_7, 0);

    Wire.begin(OLED_SDA, OLED_SCL);
    u8g2.begin();
    
    // Initialize SPIFFS for user station management
    if (!stationManager.initSPIFFS()) {
        Serial.println("SPIFFS initialization failed!");
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.drawStr(0, 10, "SPIFFS Error!");
        u8g2.sendBuffer();
        while(1) { delay(1000); }
    }
    
    delay(200); 
    if (digitalRead(ROTARY_SW) == LOW) {
        startSmartConfig();
    } else {
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.drawStr(0, 10, "Connecting to WiFi...");
        u8g2.sendBuffer();
        WiFi.mode(WIFI_STA);
        WiFi.begin();
        Serial.println("Connecting to WiFi...");
        for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) { delay(500); Serial.print("."); }
    }

    if (WiFi.status() != WL_CONNECTED) {
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.drawStr(0, 10, "WiFi Connection");
        u8g2.drawStr(0, 25, "Failed!");
        u8g2.sendBuffer();
        Serial.println("\nWiFi Connection Failed!");
        while(1); // Halt
    }

    Serial.println("\nWiFi connected");
    
    // Load user stations
    if (!stationManager.loadStationsFromFile()) {
        Serial.println("Failed to load station list!");
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.drawStr(0, 10, "No stations found!");
        u8g2.drawStr(0, 25, "Hold button 5sec");
        u8g2.drawStr(0, 40, "for config mode");
        u8g2.sendBuffer();
        
        // Wait for user to enter config mode
        while(stationManager.getStationCount() == 0 && !config_mode_active) {
            delay(100);
        }
    }

    preferences.begin("net-radio", false);
    cur_station = preferences.getUInt("last_station", 0);
    uint8_t saved_volume = preferences.getUChar("last_volume", 15);
    preferences.end();

    internal_volume = saved_volume * 2;

    if (cur_station >= stationManager.getStationCount()) { 
        cur_station = 0; 
    }
    Serial.printf("Loaded last station: %d, last volume: %d\n", cur_station, saved_volume);

    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    audio.setInBufferSize(262144); // 256KB buffer for smoother streaming
    audio.setVolume(saved_volume);
    audio.setConnectionTimeout(10000, 5000); // Standard timeouts
    
    std::vector<String>& station_urls = stationManager.getStationUrls();
    if (!station_urls.empty()) {
        audio.connecttohost(station_urls[cur_station].c_str());
    }

    // Setup hardware watchdog timer (120 seconds)
    watchdog_timer = timerBegin(1000000); // 1MHz frequency
    timerAttachInterrupt(watchdog_timer, &watchdog_isr);
    timerAlarm(watchdog_timer, 120000000, true, 0); // 120 seconds, auto-reload
    Serial.println("Hardware watchdog enabled (120s timeout)");

    // Create tasks
    xTaskCreatePinnedToCore(display_task, "DisplayTask", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(encoder_task, "EncoderTask", 8192, NULL, 2, NULL, 1); // More stack for web server
    xTaskCreatePinnedToCore(audio_loop_task, "AudioLoopTask", 16384, NULL, 10, NULL, 0);
    
    Serial.println("=== NETRADIO User Station Manager ===");
    Serial.println("Controls:");
    Serial.println("- Short press: Switch between Volume/Station mode");
    Serial.println("- Long press (2s): Enter deep sleep");
    Serial.println("- Very long press (5s): Enter station config mode");
    Serial.printf("- Web interface: http://%s (when in config mode)\n", WiFi.localIP().toString().c_str());
    Serial.printf("Loaded %d stations\n", stationManager.getStationCount());
}

void loop() {
    // Monitor system health
    static unsigned long last_heap_check = 0;
    if (millis() - last_heap_check > 30000) { // Every 30 seconds
        Serial.printf("Free heap: %d bytes, Min free heap: %d bytes\n", 
                      ESP.getFreeHeap(), ESP.getMinFreeHeap());
        last_heap_check = millis();
        
        // If memory is critically low, restart
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