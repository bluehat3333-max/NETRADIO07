#include "Arduino.h"
#include "WiFi.h"
#include "Audio.h"
#include "U8g2lib.h" // For OLED display
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <vector>

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

// Rotary Encoder Object
RotaryEncoder encoder(ROTARY_DT, ROTARY_CLK, RotaryEncoder::LatchMode::TWO03);

// Mode for encoder (0: Volume, 1: Station)
volatile int encoder_mode = 0; // 0 for volume, 1 for station
volatile unsigned long last_switch_press_time = 0;
const unsigned long debounce_delay = 200; // Milliseconds

// --- Forward Declarations for ISRs ---
void IRAM_ATTR readEncoder();
void IRAM_ATTR handleSwitchPress();

// --- Radio Stations (Loaded from Google Drive) ---
std::vector<String> station_names;
std::vector<String> station_urls;
const char* station_list_url = "https://drive.google.com/uc?export=download&id=1XgDMR2SDgbFhCiFQf87smGTiwFgRmkd5";
uint8_t cur_station = 0; // Start at the first station

// --- Function to download and parse station list ---
void updateStationList() {
    Serial.println("Updating station list from Google Drive...");
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 10, "Updating stations...");
    u8g2.sendBuffer();

    HTTPClient http;
    WiFiClientSecure client;
    
    // WARNING: setInsecure() is used for simplicity to bypass SSL certificate validation.
    // This is not recommended for production environments handling sensitive data.
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
            Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
        }
        http.end();
    } else {
        Serial.printf("[HTTP] Unable to connect\n");
    }
}


// --- Task Definitions ---
void audio_loop_task(void *pvParameters) {
    while (1) {
        audio.loop();

        // Read rotary encoder position
        int new_position = encoder.getPosition();
        if (new_position != 0) {
            if (encoder_mode == 0) { // Volume Control Mode
                int current_volume = audio.getVolume();
                if (new_position > 0) { // Clockwise
                    current_volume = min(current_volume + 1, 21);
                } else { // Counter-clockwise
                    current_volume = max(current_volume - 1, 0);
                }
                audio.setVolume(current_volume);
            } else { // Station Selection Mode
                if (!station_urls.empty()) {
                    if (new_position > 0) { // Clockwise
                        cur_station = (cur_station + 1) % station_urls.size();
                    } else { // Counter-clockwise
                        cur_station = (cur_station == 0) ? station_urls.size() - 1 : cur_station - 1;
                    }
                    audio.connecttohost(station_urls[cur_station].c_str());
                }
            }
            encoder.setPosition(0); // Reset encoder position
        }
    }
}

void display_task(void *pvParameters) {
    static int16_t text_x_pos = 0;
    static uint16_t text_width = 0;
    static uint32_t last_scroll_time = 0;
    const uint16_t scroll_speed = 5; // pixels per scroll update
    const uint16_t scroll_delay = 50; // ms between scroll updates

    static uint8_t animation_phase = 0; // For simple animation

    while (1) {
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_ncenB10_tr);
        u8g2.drawStr(0, 15, "Now Playing:");

        // Display Mode and Value
        u8g2.setFont(u8g2_font_6x10_tf); // Smaller font for mode/value
        char display_str[30];
        if (encoder_mode == 0) { // Volume Control Mode
            sprintf(display_str, "Vol: %d", audio.getVolume());
        } else { // Station Selection Mode
            if (!station_names.empty()) {
                sprintf(display_str, "Station: %d/%d", cur_station + 1, station_names.size());
            } else {
                sprintf(display_str, "No Stations");
            }
        }
        int display_str_width = u8g2.getStrWidth(display_str);
        u8g2.drawStr((128 - display_str_width) / 2, 28, display_str); // Centered, new line

        // --- Simple Visualizer Animation ---
        for (int i = 0; i < 128; i += 8) {
            int bar_height = 10 + (sin((i + animation_phase) * 0.1) * 5);
            u8g2.drawBox(i, 64 - bar_height, 6, bar_height);
        }
        animation_phase = (animation_phase + 1) % 360;

        u8g2.setFont(u8g2_font_6x10_tf);
        
        const char* current_station_name = "Loading stations...";
        if (!station_names.empty() && cur_station < station_names.size()) {
            current_station_name = station_names[cur_station].c_str();
        }
        
        text_width = u8g2.getStrWidth(current_station_name);

        if (text_width > 128) { // If text is wider than screen
            if (millis() - last_scroll_time > scroll_delay) {
                text_x_pos -= scroll_speed;
                if (text_x_pos < -(text_width + 128)) { // Reset position after scrolling off
                    text_x_pos = 128;
                }
                last_scroll_time = millis();
            }
            u8g2.drawStr(text_x_pos, 40, current_station_name);
        } else { // Text fits, no scrolling
            text_x_pos = (128 - text_width) / 2; // Center the text
            u8g2.drawStr(text_x_pos, 40, current_station_name);
        }
        u8g2.sendBuffer();
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void setup() {
    Serial.begin(115200);
    randomSeed(analogRead(0));

    pinMode(ROTARY_DT, INPUT_PULLUP);
    pinMode(ROTARY_CLK, INPUT_PULLUP);
    pinMode(ROTARY_SW, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ROTARY_DT), readEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ROTARY_CLK), readEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ROTARY_SW), handleSwitchPress, FALLING);

    u8g2.begin();
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 10, "Connecting to WiFi...");
    u8g2.sendBuffer();

    WiFi.disconnect(true);
    WiFi.mode(WIFI_STA);
    WiFi.begin();

    Serial.println("Connecting to WiFi...");
    for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) {
        delay(500);
        Serial.print(".");
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nFailed to connect. Starting SmartConfig.");
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.drawStr(0, 10, "Start SmartConfig");
        u8g2.drawStr(0, 25, "Use ESPTouch App");
        u8g2.sendBuffer();
        WiFi.beginSmartConfig();
        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.print(".");
        }
    }

    Serial.println("\nWiFi connected");
    u8g2.clearBuffer();
    u8g2.drawStr(0, 10, "WiFi Connected!");
    u8g2.sendBuffer();
    delay(1000);

    updateStationList();

    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    audio.setInBufferSize(131072);
    audio.setVolume(15);
    
    if (!station_urls.empty()) {
        audio.connecttohost(station_urls[cur_station].c_str());
    } else {
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.drawStr(0, 10, "Failed to load");
        u8g2.drawStr(0, 25, "station list!");
        u8g2.sendBuffer();
        Serial.println("Failed to load station list. Halting.");
        while(1) { delay(1000); }
    }

    xTaskCreate(display_task, "DisplayTask", 4096, NULL, 2, NULL);
    xTaskCreate(audio_loop_task, "AudioLoopTask", 8192, NULL, 5, NULL);
}

void loop() {
    // This loop is intentionally empty.
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