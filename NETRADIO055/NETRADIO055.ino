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

// Watchdog timer handle
hw_timer_t *watchdog_timer = NULL;
volatile bool watchdog_triggered = false;

// --- Global State Variables ---
volatile int encoder_mode = 0; // 0 for volume, 1 for station
uint8_t internal_volume; // For 0.5 step volume control
volatile unsigned long last_switch_press_time = 0;
const unsigned long debounce_delay = 1000; // Milliseconds (Refractory period for switch)
const unsigned long long_press_duration = 2000; // 2 seconds for long press
volatile bool entering_sleep = false; // Flag to stop display updates

// --- Forward Declarations for ISRs ---
void IRAM_ATTR readEncoder();
void IRAM_ATTR handleSwitchPress();

// --- Radio Stations (Loaded from Gist) ---
std::vector<String> station_names;
std::vector<String> station_urls;
const char* station_list_url = "https://gist.githubusercontent.com/KimYuki3/343960ab2d026203d0fe0445c7dfb357/raw/";
uint8_t cur_station = 0; // Start at the first station, will be overwritten by saved preference

// --- Function to download and parse station list ---
void updateStationList() {
    Serial.println("Updating station list from Gist...");
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 10, "Updating stations...");
    u8g2.sendBuffer();

    HTTPClient http;
    WiFiClientSecure client;
    client.setInsecure();

    if (http.begin(client, station_list_url)) {
        int httpCode = http.GET();
        if (httpCode == HTTP_CODE_OK) {
            String payload = http.getString();
            station_names.clear();
            station_urls.clear();
            int from = 0;
            int to;
            do {
                to = payload.indexOf('\n', from);
                String line = (to == -1) ? payload.substring(from) : payload.substring(from, to);
                line.trim();
                if (line.length() > 0) {
                    int comma_index = line.indexOf(',');
                    if (comma_index > 0) {
                        String name = line.substring(0, comma_index);
                        String url = line.substring(comma_index + 1);
                        name.trim();
                        url.trim();
                        if (name.length() > 0 && url.length() > 0) {
                            station_names.push_back(name);
                            station_urls.push_back(url);
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

// Watchdog ISR
void IRAM_ATTR watchdog_isr() {
    watchdog_triggered = true;
}

// --- Task Definitions ---
void encoder_task(void *pvParameters) {
    unsigned long button_press_time = 0;
    bool button_currently_pressed = false;
    bool long_press_triggered = false;
    
    while (1) {
        bool current_state = (digitalRead(ROTARY_SW) == LOW);
        
        // --- Rotary Switch Long Press Detection for Deep Sleep ---
        if (current_state && !button_currently_pressed) {
            // Button just pressed
            button_currently_pressed = true;
            button_press_time = millis();
            long_press_triggered = false;
            Serial.println("DEBUG: Button pressed, starting timer...");
        }
        else if (current_state && button_currently_pressed && !long_press_triggered) {
            // Button still pressed, check duration
            unsigned long press_duration = millis() - button_press_time;
            if (press_duration > long_press_duration) {
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
                
                Serial.println("Button released, waiting 10 seconds...");
                
                // Save current volume and station before sleep
                preferences.begin("net-radio", false);
                preferences.putUChar("last_volume", internal_volume / 2);
                preferences.putUInt("last_station", cur_station);
                preferences.end();
                
                // Stop audio and I2S
                audio.stopSong();
                Serial.println("Audio and I2S stopped");
                vTaskDelay(500 / portTICK_PERIOD_MS);
                
                // Wait 10 seconds while keeping DEEP SLEEP displayed
                for(int i = 0; i < 20; i++) {
                    vTaskDelay(500 / portTICK_PERIOD_MS);
                    if(digitalRead(ROTARY_SW) == LOW) {
                        Serial.println("Button pressed during countdown, cancelling sleep");
                        entering_sleep = false;
                        return;
                    }
                }
                
                Serial.println("10 seconds elapsed, final button check...");
                
                // Final verification: ensure button is HIGH
                if(digitalRead(ROTARY_SW) == LOW) {
                    Serial.println("WARNING: Button still LOW, waiting for release...");
                    while(digitalRead(ROTARY_SW) == LOW) {
                        vTaskDelay(100 / portTICK_PERIOD_MS);
                    }
                    Serial.println("Button released, waiting extra 2 seconds...");
                    vTaskDelay(2000 / portTICK_PERIOD_MS);
                }
                
                // One more final check
                int button_state = digitalRead(ROTARY_SW);
                Serial.printf("Final button state: %d (1=HIGH/released, 0=LOW/pressed)\n", button_state);
                
                if(button_state == LOW) {
                    Serial.println("ERROR: Button STILL pressed, aborting sleep!");
                    entering_sleep = false;
                    return;
                }
                
                // Add extra safety delay to ensure GPIO is stable
                Serial.println("Extra 3 second safety delay...");
                for(int i = 0; i < 30; i++) {
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                    int check_state = digitalRead(ROTARY_SW);
                    if(check_state == LOW) {
                        Serial.printf("WARNING: Button went LOW at check %d/30!\n", i);
                        entering_sleep = false;
                        return;
                    }
                }
                
                // Final final check
                button_state = digitalRead(ROTARY_SW);
                Serial.printf("After safety delay, button state: %d\n", button_state);
                
                if(button_state == LOW) {
                    Serial.println("ERROR: Button pressed during safety delay, aborting!");
                    entering_sleep = false;
                    return;
                }
                
                // Monitor GPIO state for 5 seconds before sleep
                Serial.println("Monitoring GPIO for 5 seconds...");
                bool gpio_stable = true;
                int low_count = 0;
                for(int i = 0; i < 50; i++) {
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                    int gpio_state = digitalRead(ROTARY_SW);
                    if(gpio_state == LOW) {
                        low_count++;
                        Serial.printf("WARNING: GPIO went LOW at iteration %d! (count: %d)\n", i, low_count);
                        if(low_count > 2) {
                            gpio_stable = false;
                            break;
                        }
                    }
                    if(i % 10 == 0) {
                        Serial.printf("GPIO check %d/50: %s\n", i, gpio_state == HIGH ? "HIGH" : "LOW");
                    }
                }
                
                if(!gpio_stable) {
                    Serial.println("GPIO unstable (detected LOW 3+ times), aborting sleep!");
                    entering_sleep = false;
                    return;
                }
                
                Serial.printf("GPIO stable for 5 seconds (low_count=%d), proceeding to sleep.\n", low_count);
                
                // Turn off display
                u8g2.setPowerSave(1);
                vTaskDelay(500 / portTICK_PERIOD_MS);
                
                // Detach all interrupts before sleep
                detachInterrupt(digitalPinToInterrupt(ROTARY_DT));
                detachInterrupt(digitalPinToInterrupt(ROTARY_CLK));
                
                // Disable all GPIO interrupts
                gpio_intr_disable((gpio_num_t)ROTARY_SW);
                gpio_intr_disable((gpio_num_t)ROTARY_DT);
                gpio_intr_disable((gpio_num_t)ROTARY_CLK);
                
                // Disable encoder pins completely during sleep
                gpio_deep_sleep_hold_en();
                gpio_hold_en((gpio_num_t)ROTARY_DT);
                gpio_hold_en((gpio_num_t)ROTARY_CLK);
                
                // Configure wakeup source before any RTC GPIO config
                esp_sleep_enable_ext0_wakeup(GPIO_NUM_7, 0);
                
                Serial.println("Entering deep sleep NOW...");
                Serial.flush();
                
                // Final delay to ensure everything is settled
                delay(1000);
                
                esp_deep_sleep_start();
            }
        }
        else if (!current_state && button_currently_pressed && !long_press_triggered) {
            // Button released before long press
            unsigned long press_duration = millis() - button_press_time;
            Serial.printf("DEBUG: Button released after %lu ms (short press)\n", press_duration);
            
            // Handle short press - toggle encoder mode
            if (millis() - last_switch_press_time > debounce_delay) {
                encoder_mode = 1 - encoder_mode;
                last_switch_press_time = millis();
                Serial.printf("DEBUG: Mode switched to %d\n", encoder_mode);
            }
            button_currently_pressed = false;
        }
        else if (!current_state) {
            button_currently_pressed = false;
        }
        
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void audio_loop_task(void *pvParameters) {
    Serial.println("Audio task started.");
    unsigned long last_audio_activity = millis();
    const unsigned long audio_timeout = 60000; // 60 seconds timeout
    
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
        if (new_position != 0) {
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
                if (!station_urls.empty()) {
                    if (new_position > 0) { cur_station = (cur_station + 1) % station_urls.size(); } 
                    else { cur_station = (cur_station == 0) ? station_urls.size() - 1 : cur_station - 1; }
                    
                    Serial.printf("Changing to station %d: %s\n", cur_station, station_names[cur_station].c_str());
                    audio.stopSong();
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                    
                    audio.connecttohost(station_urls[cur_station].c_str());
                    last_audio_activity = millis();
                    
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
            if (!station_names.empty()) { sprintf(display_str, "Station: %d/%d", cur_station + 1, station_names.size()); }
            else { sprintf(display_str, "No Stations"); }
        }
        int display_str_width = u8g2.getStrWidth(display_str);
        u8g2.drawStr((128 - display_str_width) / 2, 26, display_str);

        const char* current_station_name = "Loading stations...";
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
    
    // Disable GPIO hold on wakeup
    gpio_deep_sleep_hold_dis();
    gpio_hold_dis((gpio_num_t)ROTARY_DT);
    gpio_hold_dis((gpio_num_t)ROTARY_CLK);
    
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
    } else if (wakeup_reason == ESP_SLEEP_WAKEUP_UNDEFINED) {
        Serial.println("Normal boot (not from deep sleep)");
    }
    
    pinMode(ROTARY_DT, INPUT_PULLUP);
    pinMode(ROTARY_CLK, INPUT_PULLUP);
    pinMode(ROTARY_SW, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ROTARY_DT), readEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ROTARY_CLK), readEncoder, CHANGE);
    // Disable switch interrupt - handle in encoder_task instead
    // attachInterrupt(digitalPinToInterrupt(ROTARY_SW), handleSwitchPress, FALLING);
    
    // Setup wakeup source
    Serial.println("Using GPIO 7 (ROTARY_SW) for wakeup");
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_7, 0);

    Wire.begin(OLED_SDA, OLED_SCL);
    u8g2.begin();
    
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
    updateStationList();

    preferences.begin("net-radio", false);
    cur_station = preferences.getUInt("last_station", 0);
    uint8_t saved_volume = preferences.getUChar("last_volume", 15);
    preferences.end();

    internal_volume = saved_volume * 2;

    if (cur_station >= station_urls.size()) { cur_station = 0; }
    Serial.printf("Loaded last station: %d, last volume: %d\n", cur_station, saved_volume);

    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    audio.setInBufferSize(262144); // 256KB buffer for smoother streaming
    audio.setVolume(saved_volume);
    audio.setConnectionTimeout(10000, 5000); // Standard timeouts
    
    if (!station_urls.empty()) {
        audio.connecttohost(station_urls[cur_station].c_str());
    } else {
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.drawStr(0, 10, "Failed to load");
        u8g2.drawStr(0, 25, "station list!");
        u8g2.sendBuffer();
        while(1) { delay(1000); }
    }

    // Setup hardware watchdog timer (120 seconds)
    watchdog_timer = timerBegin(1000000); // 1MHz frequency
    timerAttachInterrupt(watchdog_timer, &watchdog_isr);
    timerAlarm(watchdog_timer, 120000000, true, 0); // 120 seconds, auto-reload
    Serial.println("Hardware watchdog enabled (120s timeout)");

    // Optimized task priorities for smooth audio streaming
    xTaskCreatePinnedToCore(display_task, "DisplayTask", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(encoder_task, "EncoderTask", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(audio_loop_task, "AudioLoopTask", 16384, NULL, 10, NULL, 0); // Stable priority & stack
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
