# include "Arduino.h"
# include "WiFi.h"
// Librarry zip install from: https://github.com/schreibfaul1/ESP32-audioI2S
# include "Audio.h"

// Digital I/O used
# define I2S_DOUT      3  // DIN connection
# define I2S_BCLK      1  // Bit clock
# define I2S_LRC       2  // Left Right Clock
 
Audio audio;
String ssid = "xg100n-9b6475-1";
String password = "11922960";

//String ssid =     "ynAir_G";
//String password = "tsubasahikaru";
String stations[] ={
        "ice1.somafm.com/illstreet-128-mp3",      // SomaFM / Illinois Street Lounge
        "ais-sa2.cdnstream1.com/b22139_128mp3",   // 101 SMOOTH JAZZ
        "relax.stream.publicradio.org/relax.mp3", // Your Classical - Relax
        "16963.live.streamtheworld.com/SAM03AAC226_SC",    // #1980s Zoom
        "ice1.somafm.com/secretagent-128-mp3",    // SomaFM / Secret Agent
        "ice1.somafm.com/seventies-128-mp3",      // SomaFM / Left Coast 70s
        "ice1.somafm.com/bootliquor-128-mp3",     // SomaFM / Boot Liquor
        "musicbird.leanstream.co/JCB032-MP3",     // 84.3 FM Edogawa (FMえどがわ, JOZZ3AS-FM, Edogawa City,...
        "musicbird.leanstream.co/JCB093-MP3",     // Dreams FM (ドリームスエフエム, JOZZ0AI-FM, 76.5 MHz, Kurume...
        "musicbird.leanstream.co/JCB104-MP3",     // Kyoto Living FM (京都リビング FM/きょうと りびんぐ FM)
        "musicbird.leanstream.co/JCB015-MP3",     // FM Blue Shonan (FM・ブルー湘南 , JOZZ3AD-FM, 78.5 MHz, Y...
};

uint8_t cur_station  = 5;         // current station No.

void setup() {
    Serial.begin(115200);
    WiFi.disconnect();
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), password.c_str());
    while (WiFi.status() != WL_CONNECTED) delay(1500);
    Serial.println("WiFi start");
    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    audio.setVolume(15); // 0...21
    
//    audio.connecttospeech("Wenn die Hunde schlafen, kann der Wolf gut Schafe stehlen.", "de");
//    audio.connecttospeech("When the dogs sleep, the wolf is good at stealing sheep.", "en");

    audio.connecttohost(stations[cur_station].c_str());

}
 
void loop()
{
    audio.loop();
}
 
// optional
void audio_info(const char *info){
    Serial.print("info        "); Serial.println(info);
}
void audio_id3data(const char *info){  //id3 metadata
    Serial.print("id3data     ");Serial.println(info);
}
void audio_eof_mp3(const char *info){  //end of file
    Serial.print("eof_mp3     ");Serial.println(info);
}
void audio_showstation(const char *info){
    Serial.print("station     ");Serial.println(info);
}
void audio_showstreaminfo(const char *info){
    Serial.print("streaminfo  ");Serial.println(info);
}
void audio_showstreamtitle(const char *info){
    Serial.print("streamtitle ");Serial.println(info);
}
void audio_bitrate(const char *info){
    Serial.print("bitrate     ");Serial.println(info);
}
void audio_commercial(const char *info){  //duration in sec
    Serial.print("commercial  ");Serial.println(info);
}
void audio_icyurl(const char *info){  //homepage
    Serial.print("icyurl      ");Serial.println(info);
}
void audio_lasthost(const char *info){  //stream URL played
    Serial.print("lasthost    ");Serial.println(info);
}
void audio_eof_speech(const char *info){
    Serial.print("eof_speech  ");Serial.println(info);
}
