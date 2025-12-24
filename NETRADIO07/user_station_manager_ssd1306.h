#ifndef USER_STATION_MANAGER_SSD1306_H
#define USER_STATION_MANAGER_SSD1306_H

#include <vector>
#include <WiFi.h>
#include <WebServer.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <Adafruit_SSD1306.h>

class UserStationManager_SSD1306 {
private:
    WebServer* server;
    Adafruit_SSD1306* display;
    std::vector<String> station_names;
    std::vector<String> station_urls;
    bool web_server_running = false;
    bool station_list_changed = false;
    
    String generateWebInterface();
    void handleRoot();
    void handleAddStation();
    void handleDeleteStation();
    void handleGetStations();
    void handleSaveStations();
    void handleNotFound();
    void saveStationsToFile();
    
public:
    UserStationManager_SSD1306(Adafruit_SSD1306* disp);
    ~UserStationManager_SSD1306();
    
    bool initSPIFFS();
    bool loadStationsFromFile();
    void startConfigMode();
    void stopConfigMode();
    bool isConfigModeActive() { return web_server_running; }
    void handleClient();
    
    // Station management
    bool addStation(const String& name, const String& url);
    bool deleteStation(int index);
    void clearStations();
    
    // Getters
    std::vector<String>& getStationNames() { return station_names; }
    std::vector<String>& getStationUrls() { return station_urls; }
    int getStationCount() { return station_names.size(); }
    
    // Import/Export
    bool importStationsFromUrl(const String& url);
    String exportStationsToJson();
};

#endif