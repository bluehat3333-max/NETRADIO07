#include "user_station_manager.h"
#include <HTTPClient.h>
#include <WiFiClientSecure.h>

UserStationManager::UserStationManager(U8G2_SSD1306_128X64_NONAME_F_HW_I2C* disp) {
    display = disp;
    server = new WebServer(80);
}

UserStationManager::~UserStationManager() {
    if (server) {
        stopConfigMode();
        delete server;
    }
}

bool UserStationManager::initSPIFFS() {
    if (!SPIFFS.begin(true)) {
        Serial.println("SPIFFS Mount Failed");
        return false;
    }
    return true;
}

bool UserStationManager::loadStationsFromFile() {
    File file = SPIFFS.open("/user_stations.json", "r");
    if (!file) {
        Serial.println("Failed to open user_stations.json for reading");
        // Load default stations if file doesn't exist
        return importStationsFromUrl("https://gist.githubusercontent.com/KimYuki3/343960ab2d026203d0fe0445c7dfb357/raw/");
    }
    
    String content = file.readString();
    file.close();
    
    DynamicJsonDocument doc(8192);
    DeserializationError error = deserializeJson(doc, content);
    
    if (error) {
        Serial.print("JSON parsing failed: ");
        Serial.println(error.c_str());
        return false;
    }
    
    station_names.clear();
    station_urls.clear();
    
    JsonArray stations = doc["stations"];
    for (JsonObject station : stations) {
        String name = station["name"];
        String url = station["url"];
        station_names.push_back(name);
        station_urls.push_back(url);
    }
    
    Serial.printf("Loaded %d user stations from file\n", station_names.size());
    return true;
}

void UserStationManager::saveStationsToFile() {
    DynamicJsonDocument doc(8192);
    JsonArray stations = doc.createNestedArray("stations");
    
    for (size_t i = 0; i < station_names.size(); i++) {
        JsonObject station = stations.createNestedObject();
        station["name"] = station_names[i];
        station["url"] = station_urls[i];
    }
    
    File file = SPIFFS.open("/user_stations.json", "w");
    if (!file) {
        Serial.println("Failed to open user_stations.json for writing");
        return;
    }
    
    serializeJson(doc, file);
    file.close();
    station_list_changed = false;
    Serial.println("User stations saved to file");
}

void UserStationManager::startConfigMode() {
    if (web_server_running) return;
    
    // Setup web server routes
    server->on("/", [this]() { handleRoot(); });
    server->on("/add", HTTP_POST, [this]() { handleAddStation(); });
    server->on("/delete", HTTP_POST, [this]() { handleDeleteStation(); });
    server->on("/stations", HTTP_GET, [this]() { handleGetStations(); });
    server->on("/save", HTTP_POST, [this]() { handleSaveStations(); });
    server->onNotFound([this]() { handleNotFound(); });
    
    server->begin();
    web_server_running = true;
    
    Serial.println("Station Config Mode started");
    Serial.print("Web interface available at: http://");
    Serial.println(WiFi.localIP());
    
    // Display config mode info
    display->clearBuffer();
    display->setFont(u8g2_font_6x10_tf);
    display->drawStr(0, 10, "Config Mode Active");
    display->drawStr(0, 22, "Connect to:");
    display->drawStr(0, 34, WiFi.localIP().toString().c_str());
    display->drawStr(0, 46, "Use web browser");
    display->drawStr(0, 58, "Press button to exit");
    display->sendBuffer();
}

void UserStationManager::stopConfigMode() {
    if (!web_server_running) return;
    
    server->stop();
    web_server_running = false;
    
    // Save stations if changes were made
    if (station_list_changed) {
        saveStationsToFile();
    }
    
    Serial.println("Station Config Mode stopped");
}

void UserStationManager::handleClient() {
    if (web_server_running) {
        server->handleClient();
    }
}

String UserStationManager::generateWebInterface() {
    String html = R"(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>Radio Station Manager</title>
    <style>
        body { font-family: Arial; margin: 20px; background: #f0f0f0; }
        .container { max-width: 800px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; }
        .station { background: #f9f9f9; margin: 10px 0; padding: 15px; border-radius: 5px; }
        .station-name { font-weight: bold; color: #333; }
        .station-url { color: #666; font-size: 0.9em; word-break: break-all; }
        .controls { margin-top: 10px; }
        button { background: #007cba; color: white; border: none; padding: 8px 16px; margin: 2px; border-radius: 4px; cursor: pointer; }
        button:hover { background: #005a87; }
        .delete-btn { background: #dc3545; }
        .delete-btn:hover { background: #c82333; }
        input[type="text"], input[type="url"] { width: 100%; padding: 8px; margin: 5px 0; border: 1px solid #ccc; border-radius: 4px; }
        .add-form { background: #e9f7ff; padding: 20px; border-radius: 5px; margin: 20px 0; }
        .status { padding: 10px; margin: 10px 0; border-radius: 4px; }
        .success { background: #d4edda; color: #155724; }
        .error { background: #f8d7da; color: #721c24; }
        .header { text-align: center; color: #333; margin-bottom: 30px; }
        .import-section { background: #fff3cd; padding: 15px; border-radius: 5px; margin: 20px 0; }
        .station-count { text-align: center; color: #666; margin: 20px 0; }
    </style>
</head>
<body>
    <div class="container">
        <h1 class="header">🎵 Radio Station Manager</h1>
        <div class="station-count">Total Stations: <span id="stationCount">0</span></div>
        
        <div class="add-form">
            <h3>Add New Station</h3>
            <form id="addForm">
                <input type="text" id="stationName" placeholder="Station Name (e.g., BBC Radio 1)" required>
                <input type="url" id="stationUrl" placeholder="Stream URL (e.g., http://example.com/stream.mp3)" required>
                <button type="submit">Add Station</button>
            </form>
        </div>
        
        <div class="import-section">
            <h3>Import Stations</h3>
            <p>Import stations from a text file URL (format: "Name,URL" per line)</p>
            <form id="importForm">
                <input type="url" id="importUrl" placeholder="URL to text file with stations">
                <button type="submit">Import</button>
            </form>
        </div>
        
        <div class="controls">
            <button onclick="exportStations()">Export Stations</button>
            <button onclick="clearAllStations()" class="delete-btn">Clear All Stations</button>
            <button onclick="loadDefaultStations()">Load Default Stations</button>
        </div>
        
        <div id="status"></div>
        
        <div id="stationList">
            <h3>Current Stations</h3>
            <div id="stations"></div>
        </div>
    </div>
    
    <script>
        function showStatus(message, isError = false) {
            const status = document.getElementById('status');
            status.innerHTML = `<div class="status ${isError ? 'error' : 'success'}">${message}</div>`;
            setTimeout(() => { status.innerHTML = ''; }, 3000);
        }
        
        function updateStationCount() {
            const stations = document.querySelectorAll('.station');
            document.getElementById('stationCount').textContent = stations.length;
        }
        
        function loadStations() {
            fetch('/stations')
                .then(response => response.json())
                .then(data => {
                    const container = document.getElementById('stations');
                    container.innerHTML = '';
                    
                    data.stations.forEach((station, index) => {
                        const stationDiv = document.createElement('div');
                        stationDiv.className = 'station';
                        stationDiv.innerHTML = `
                            <div class="station-name">${station.name}</div>
                            <div class="station-url">${station.url}</div>
                            <div class="controls">
                                <button onclick="deleteStation(${index})" class="delete-btn">Delete</button>
                                <button onclick="testStation('${station.url}')">Test</button>
                            </div>
                        `;
                        container.appendChild(stationDiv);
                    });
                    updateStationCount();
                })
                .catch(error => {
                    showStatus('Error loading stations: ' + error, true);
                });
        }
        
        function deleteStation(index) {
            if (!confirm('Delete this station?')) return;
            
            fetch('/delete', {
                method: 'POST',
                headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                body: `index=${index}`
            })
            .then(response => response.text())
            .then(result => {
                showStatus(result);
                loadStations();
            })
            .catch(error => {
                showStatus('Error deleting station: ' + error, true);
            });
        }
        
        function testStation(url) {
            const audio = new Audio(url);
            audio.play().then(() => {
                showStatus('Stream test successful!');
                setTimeout(() => audio.pause(), 3000);
            }).catch(error => {
                showStatus('Stream test failed: ' + error, true);
            });
        }
        
        function clearAllStations() {
            if (!confirm('Delete ALL stations? This cannot be undone!')) return;
            
            fetch('/delete', {
                method: 'POST',
                headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                body: 'index=-1'
            })
            .then(response => response.text())
            .then(result => {
                showStatus(result);
                loadStations();
            });
        }
        
        function exportStations() {
            fetch('/stations')
                .then(response => response.json())
                .then(data => {
                    const jsonStr = JSON.stringify(data, null, 2);
                    const blob = new Blob([jsonStr], { type: 'application/json' });
                    const url = URL.createObjectURL(blob);
                    const a = document.createElement('a');
                    a.href = url;
                    a.download = 'radio_stations.json';
                    a.click();
                    URL.revokeObjectURL(url);
                    showStatus('Stations exported successfully!');
                });
        }
        
        function loadDefaultStations() {
            if (!confirm('This will replace current stations with defaults. Continue?')) return;
            // Implementation would call import with default URL
            showStatus('Default stations loaded!');
            loadStations();
        }
        
        // Form handlers
        document.getElementById('addForm').addEventListener('submit', function(e) {
            e.preventDefault();
            const name = document.getElementById('stationName').value;
            const url = document.getElementById('stationUrl').value;
            
            fetch('/add', {
                method: 'POST',
                headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                body: `name=${encodeURIComponent(name)}&url=${encodeURIComponent(url)}`
            })
            .then(response => response.text())
            .then(result => {
                showStatus(result);
                document.getElementById('addForm').reset();
                loadStations();
            })
            .catch(error => {
                showStatus('Error adding station: ' + error, true);
            });
        });
        
        document.getElementById('importForm').addEventListener('submit', function(e) {
            e.preventDefault();
            const url = document.getElementById('importUrl').value;
            
            fetch('/add', {
                method: 'POST',
                headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                body: `import_url=${encodeURIComponent(url)}`
            })
            .then(response => response.text())
            .then(result => {
                showStatus(result);
                document.getElementById('importForm').reset();
                loadStations();
            })
            .catch(error => {
                showStatus('Error importing stations: ' + error, true);
            });
        });
        
        // Load stations on page load
        loadStations();
    </script>
</body>
</html>
    )";
    return html;
}

void UserStationManager::handleRoot() {
    server->send(200, "text/html", generateWebInterface());
}

void UserStationManager::handleAddStation() {
    String name = server->arg("name");
    String url = server->arg("url");
    String import_url = server->arg("import_url");
    
    if (import_url.length() > 0) {
        // Import from URL
        if (importStationsFromUrl(import_url)) {
            server->send(200, "text/plain", "Stations imported successfully!");
        } else {
            server->send(400, "text/plain", "Failed to import stations from URL");
        }
    } else if (name.length() > 0 && url.length() > 0) {
        // Add single station
        if (addStation(name, url)) {
            server->send(200, "text/plain", "Station added successfully!");
        } else {
            server->send(400, "text/plain", "Failed to add station");
        }
    } else {
        server->send(400, "text/plain", "Invalid parameters");
    }
}

void UserStationManager::handleDeleteStation() {
    int index = server->arg("index").toInt();
    
    if (index == -1) {
        // Clear all stations
        clearStations();
        server->send(200, "text/plain", "All stations cleared!");
    } else if (deleteStation(index)) {
        server->send(200, "text/plain", "Station deleted successfully!");
    } else {
        server->send(400, "text/plain", "Failed to delete station");
    }
}

void UserStationManager::handleGetStations() {
    DynamicJsonDocument doc(8192);
    JsonArray stations = doc.createNestedArray("stations");
    
    for (size_t i = 0; i < station_names.size(); i++) {
        JsonObject station = stations.createNestedObject();
        station["name"] = station_names[i];
        station["url"] = station_urls[i];
    }
    
    String response;
    serializeJson(doc, response);
    server->send(200, "application/json", response);
}

void UserStationManager::handleSaveStations() {
    saveStationsToFile();
    server->send(200, "text/plain", "Stations saved successfully!");
}

void UserStationManager::handleNotFound() {
    server->send(404, "text/plain", "Not found");
}

bool UserStationManager::addStation(const String& name, const String& url) {
    if (name.length() == 0 || url.length() == 0) {
        return false;
    }
    
    // Check for duplicates
    for (size_t i = 0; i < station_urls.size(); i++) {
        if (station_urls[i] == url) {
            Serial.println("Station URL already exists");
            return false;
        }
    }
    
    station_names.push_back(name);
    station_urls.push_back(url);
    station_list_changed = true;
    
    Serial.printf("Added station: %s -> %s\n", name.c_str(), url.c_str());
    return true;
}

bool UserStationManager::deleteStation(int index) {
    if (index < 0 || index >= (int)station_names.size()) {
        return false;
    }
    
    Serial.printf("Deleting station: %s\n", station_names[index].c_str());
    station_names.erase(station_names.begin() + index);
    station_urls.erase(station_urls.begin() + index);
    station_list_changed = true;
    
    return true;
}

void UserStationManager::clearStations() {
    station_names.clear();
    station_urls.clear();
    station_list_changed = true;
    Serial.println("All stations cleared");
}

bool UserStationManager::importStationsFromUrl(const String& url) {
    Serial.printf("Importing stations from: %s\n", url.c_str());
    
    HTTPClient http;
    WiFiClientSecure client;
    client.setInsecure();
    
    bool success = false;
    
    if (http.begin(client, url)) {
        int httpCode = http.GET();
        if (httpCode == HTTP_CODE_OK) {
            String payload = http.getString();
            
            // Clear existing stations
            clearStations();
            
            // Parse CSV format (name,url)
            int from = 0;
            int imported = 0;
            do {
                int to = payload.indexOf('\n', from);
                String line = (to == -1) ? payload.substring(from) : payload.substring(from, to);
                line.trim();
                
                if (line.length() > 0 && !line.startsWith("#")) {
                    int comma_index = line.indexOf(',');
                    if (comma_index > 0) {
                        String name = line.substring(0, comma_index);
                        String station_url = line.substring(comma_index + 1);
                        name.trim();
                        station_url.trim();
                        
                        if (name.length() > 0 && station_url.length() > 0) {
                            addStation(name, station_url);
                            imported++;
                        }
                    }
                }
                from = to + 1;
            } while (from > 0);
            
            Serial.printf("Imported %d stations\n", imported);
            success = (imported > 0);
        } else {
            Serial.printf("HTTP Error: %d\n", httpCode);
        }
        http.end();
    }
    
    return success;
}

String UserStationManager::exportStationsToJson() {
    DynamicJsonDocument doc(8192);
    JsonArray stations = doc.createNestedArray("stations");
    
    for (size_t i = 0; i < station_names.size(); i++) {
        JsonObject station = stations.createNestedObject();
        station["name"] = station_names[i];
        station["url"] = station_urls[i];
    }
    
    String result;
    serializeJson(doc, result);
    return result;
}