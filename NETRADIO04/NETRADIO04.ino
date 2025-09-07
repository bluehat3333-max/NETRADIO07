#include "Arduino.h"
#include "WiFi.h"
#include "Audio.h"
#include "U8g2lib.h" // For OLED display

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

// --- WiFi Credentials ---
//String ssid = "xg100n-9b6475-1";
//String password = "11922960";
String ssid = "";
String password = "";
// --- Radio Stations ---
const char* station_names[] = {
    "SomaFM - Illinois Street Lounge",//1
    "101 SMOOTH JAZZ",//2
    "Your Classical - Relax",//3
    "SomaFM - Secret Agent",//4
    "SomaFM - Left Coast 70s",//5
    "SomaFM - Boot Liquor",//6
    "CLASSIC FRANCE",//7
    "allclassical streamguys1",//8
    "ais-sa8.cdnstream1.com/3629_128.mp3",//9
"jazzfm91.streamb.live/SB00009",//10
"onair22.xdevel.com/proxy/xautocloud_1kha_423?mp=/stream",//11
"wdcb-ice.streamguys1.com/wdcb128",//12
"live-radio02.mediahubaustralia.com/JAZW/mp3",//13
"wdcb-ice.streamguys1.com/mobile-AAC",//14
"jazzfm91.streamb.live/SB00023",//15
"jazzfm91.streamb.live/SB00024",//16
"jazzfm91.streamb.live/SB00137",//17
"media-ice.musicradio.com/ClassicFM",//18
"wrti-live.streamguys1.com/classical-mp3",//19
"wrti-live.streamguys1.com/jazz-mp3",//20
"wrti-live.streamguys1.com/classical-mp3",//21
"livestreaming-node-4.srg-ssr.ch/srgssr/rsc_de/mp3/128",//22
"icecast.radiofrance.fr/francemusique-midfi.mp3",//23
"wdr-wdr3-live.icecast.wdr.de/wdr/wdr3/live/mp3/128/stream.mp3",//24
"radioclassique.ice.infomaniak.ch/radioclassique-high.mp3",//25
"media-ssl.musicradio.com/ClassicFM",//26
"direct.francemusique.fr/live/francemusique-midfi.mp3",//27
"orf-live.ors-shoutcast.at/oe1-q2a",//28
//JAZZ
"wbgo.streamguys1.com/wbgo128",//29
"wrti-live.streamguys1.com/jazz-mp3",//30
"direct.fipradio.fr/live/fip-webradio6.mp3",//31
"tsfjazz.ice.infomaniak.ch/tsfjazz-high.mp3",//32
"wdcb-ice.streamguys1.com/wdcb128",//33
};
String stations[] = {
    "ice1.somafm.com/illstreet-128-mp3",//1
    "ais-sa2.cdnstream1.com/b22139_128mp3",//2
    "relax.stream.publicradio.org/relax.mp3",//3
    "ice1.somafm.com/secretagent-128-mp3",//4JAZZ
    "ice1.somafm.com/seventies-128-mp3",//5JAZZ
    "ice1.somafm.com/bootliquor-128-mp3",//6JAZZ
    "direct.francemusique.fr/live/francemusique-midfi.mp3",//7
    "allclassical.streamguys1.com/ac96kmp3",//8
"ais-sa8.cdnstream1.com/3629_128.mp3",//9jAZZ
"jazzfm91.streamb.live/SB00009",//10 jAZZ
"onair22.xdevel.com/proxy/xautocloud_1kha_423?mp=/stream",//JAZZ
"wdcb-ice.streamguys1.com/wdcb128",//JAZZ
"live-radio02.mediahubaustralia.com/JAZW/mp3",//JAZZ
"wdcb-ice.streamguys1.com/mobile-AAC",//JAZZ
"jazzfm91.streamb.live/SB00023",//JAZZ
"jazzfm91.streamb.live/SB00024",//JAZZ
"jazzfm91.streamb.live/SB00137",// JAZZ
"media-ice.musicradio.com/ClassicFM",//18
"wrti-live.streamguys1.com/classical-mp3",//20
"wrti-live.streamguys1.com/jazz-mp3",//21 jazz

"wrti-live.streamguys1.com/classical-mp3",//23 
"livestreaming-node-4.srg-ssr.ch/srgssr/rsc_de/mp3/128",//24
"icecast.radiofrance.fr/francemusique-midfi.mp3",//25
"wdr-wdr3-live.icecast.wdr.de/wdr/wdr3/live/mp3/128/stream.mp3",//27
"radioclassique.ice.infomaniak.ch/radioclassique-high.mp3",//28

"media-ssl.musicradio.com/ClassicFM",//30
"direct.francemusique.fr/live/francemusique-midfi.mp3",//31
"orf-live.ors-shoutcast.at/oe1-q2a",//32/ototobi
//JAZZ
"wbgo.streamguys1.com/wbgo128",//34
"wrti-live.streamguys1.com/jazz-mp3",//35

"direct.fipradio.fr/live/fip-webradio6.mp3",//38
"tsfjazz.ice.infomaniak.ch/tsfjazz-high.mp3",//39
"wdcb-ice.streamguys1.com/wdcb128",//40
};
uint8_t cur_station = 5;

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
                    current_volume = min(current_volume + 1, 21); // Max volume 21 (from audio.setVolume(15) context)
                } else { // Counter-clockwise
                    current_volume = max(current_volume - 1, 0); // Min volume 0
                }
                audio.setVolume(current_volume);
            } else { // Station Selection Mode
                if (new_position > 0) { // Clockwise
                    cur_station = (cur_station + 1) % (sizeof(stations) / sizeof(stations[0]));
                } else { // Counter-clockwise
                    cur_station = (cur_station - 1 + (sizeof(stations) / sizeof(stations[0]))) % (sizeof(stations) / sizeof(stations[0]));
                }
                audio.connecttohost(stations[cur_station].c_str());
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
            sprintf(display_str, "Station: %d", cur_station + 1); // Display 1-indexed station number
        }
        int display_str_width = u8g2.getStrWidth(display_str);
        u8g2.drawStr((128 - display_str_width) / 2, 28, display_str); // Centered, new line

        // --- Simple Visualizer Animation ---
        // This visualizer is time-based and does not react to actual audio levels.
        // Directly linking to audio levels with the current Audio library setup is complex.
        // It would require accessing raw audio samples or using a different audio input method.
        for (int i = 0; i < 128; i += 8) {
            int bar_height = 10 + (sin((i + animation_phase) * 0.1) * 5);
            u8g2.drawBox(i, 64 - bar_height, 6, bar_height);
        }
        animation_phase = (animation_phase + 1) % 360; // Increment phase for animation

        u8g2.setFont(u8g2_font_6x10_tf);
        const char* current_station_name = station_names[cur_station];
        text_width = u8g2.getStrWidth(current_station_name);

        // --- Simple Sparkle Effect (Lighter) ---
        static uint32_t last_sparkle_trigger = 0;
        const uint32_t sparkle_interval_min = 200; // Minimum 0.1 seconds
        const uint32_t sparkle_interval_max = 1000; // Maximum 0.5 seconds
        const uint8_t num_sparkles_per_burst = 5; // Number of sparkles to draw per burst

        if (millis() - last_sparkle_trigger > random(sparkle_interval_min, sparkle_interval_max)) {
            int center_x = 64; // Center of the screen
            int center_y = 32; // Center of the screen
            int spread_radius = 20; // How far the sparkles spread from the center

            for (int i = 0; i < num_sparkles_per_burst; i++) {
                // Generate random offsets within the spread_radius
                int offset_x = random(-spread_radius, spread_radius + 1);
                int offset_y = random(-spread_radius, spread_radius + 1);

                int x = center_x + offset_x;
                int y = center_y + offset_y;

                // Ensure pixels are within screen bounds
                if (x >= 0 && x < 128 && y >= 0 && y < 64) {
                    u8g2.drawPixel(x, y);
                }
            }
            last_sparkle_trigger = millis();
        }

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
            text_x_pos = 0; // Reset position if it was scrolling before
            u8g2.drawStr(0, 40, current_station_name);
        }
        u8g2.sendBuffer();
        vTaskDelay(100 / portTICK_PERIOD_MS); // Update display less frequently to reduce CPU load
    }
}

void setup() {
    Serial.begin(115200);
    randomSeed(analogRead(0)); // Seed the random number generator

    // --- Initialize Rotary Encoder ---
    pinMode(ROTARY_DT, INPUT_PULLUP);
    pinMode(ROTARY_CLK, INPUT_PULLUP);
    pinMode(ROTARY_SW, INPUT_PULLUP);

    // Attach interrupts for rotary encoder
    attachInterrupt(digitalPinToInterrupt(ROTARY_DT), readEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ROTARY_CLK), readEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ROTARY_SW), handleSwitchPress, FALLING);

    // --- Initialize Display ---
    u8g2.begin();
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 10, "Connecting to WiFi...");
    u8g2.sendBuffer();

    // --- Connect to WiFi ---
    WiFi.disconnect(true);
    WiFi.mode(WIFI_STA);

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 10, "Connecting to WiFi...");
    u8g2.sendBuffer();
    Serial.println("Connecting to WiFi...");

    WiFi.begin(); // Try to connect with saved credentials

    // Wait for connection for 10 seconds
    for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) {
        delay(500);
        Serial.print(".");
    }

    // If not connected, start SmartConfig
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
        Serial.println("\nSmartConfig successful. WiFi Connected.");
    }

    Serial.println("\nWiFi connected");
    u8g2.clearBuffer();
    u8g2.drawStr(0, 10, "WiFi Connected!");
    u8g2.sendBuffer();
    delay(1000);

    // --- Initialize Audio ---
    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    audio.setInBufferSize(131072); // Increase audio buffer for smoother playback
    audio.setVolume(15);
    audio.connecttohost(stations[cur_station].c_str());

    // --- Start Tasks ---
    xTaskCreate(display_task, "DisplayTask", 2048, NULL, 2, NULL);
    xTaskCreate(audio_loop_task, "AudioLoopTask", 8192, NULL, 5, NULL);
}

void loop() {
    // This loop is intentionally empty. audio.loop() is handled in audio_loop_task.
}

// --- Rotary Encoder ISRs ---
void IRAM_ATTR readEncoder() {
    encoder.tick();
}

void IRAM_ATTR handleSwitchPress() {
    unsigned long current_time = millis();
    if (current_time - last_switch_press_time > debounce_delay) {
        encoder_mode = 1 - encoder_mode; // Toggle mode
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
