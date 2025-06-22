/*
 * ESP32 ESP-NOW Receiver and Response Handler
 * 
 * This code receives data from multiple ESP32C3 devices and can send responses
 * to devices that request two-way communication.
 */

#include <HardwareSerial.h>
#include <esp_now.h>
#include <WiFi.h>
#include <time.h>
#include <map>
#include <esp_wifi.h>

#define LEDpin 0
// ==================== DATA STRUCTURES ====================
typedef struct {
  uint16_t device_id;
  float sensor_data[7];  // Array for up to 7 sensor values
  bool request_response; // True if device wants a response
  uint32_t timestamp;    // For debugging/tracking
} sensor_message_t;

typedef struct {
  uint16_t target_device_id;
  float response_data[7]; // Array for response data
  uint32_t timestamp;
} response_message_t;

typedef struct {
  uint8_t mac[6];
  float last_sensor_data[7];
  unsigned long last_seen;
  bool two_way_enabled;
} device_info_t;

// ==================== GLOBAL VARIABLES ====================
std::map<uint16_t, device_info_t> known_devices;
unsigned long last_status_print = 0;
const unsigned long STATUS_INTERVAL = 60000; // Print status every minute
float fridgetemp = 2.0;
float outtemp = 20.0;
float fridgevbat = 3.7;
float fridgehum, fridgepres;
float fridgeRSSI = -65;
float new_fridgetemp = 4.1;
String uartBuffer = "";
unsigned long localTimeUnix = 0;
float pm25in = 0.0;
float pm25out = 0.0;
float bridgehum = 0.0;
float bridgepres = 0.0;
float iaq = 0.0;
float windspeed = 0.0;
float brtemp = 0.0;
float brhum = 0.0;
float bridgeco2 = 0.0;
float bridgeIrms = 0.0;
float watts = 0.0;
float kw = 0.0;
float mintemp = 0.0;
float windgust = 0.0;
float lightread = 0.0;
// NTP settings for time synchronization
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 0;     // Adjust for your timezone
const int daylightOffset_sec = 0; // Adjust for daylight saving
float epapervBat = 0.0;
float epaperTemp = 0.0;
float epaperHum = 0.0;
float epaperPres = 0.0;
float epaperabshum = 0.0;
float epaperRSSI = 0.0; // RSSI from e-paper display
float incomingrssi;
RTC_NOINIT_ATTR  bool firstrun = true;
// ==================== RESPONSE DATA FUNCTIONS ====================
void generateResponseData(uint16_t device_id, float* response_data) {
  // Generate response data based on device ID
  // Customize this function based on what each device needs
  
  // Clear the array first
  for(int i = 0; i < 7; i++) {
    response_data[i] = 0.0;
  }
  
  // Get current time as Unix timestamp
  time_t now;
  time(&now);
  
  switch(device_id) {
    case 1001: // Temperature/Humidity/Pressure device
      response_data[0] = (unsigned long)now;       // Current time
      response_data[1] = fridgetemp;             
      response_data[2] = mintemp;              
      response_data[3] = windspeed;
      response_data[4] = windgust;
      break;
      
    case 1002: // Temperature only device
      //response_data[0] = (float)now;        // Current time
      //response_data[1] = 30.0;              // Temperature threshold
      break;
      
    case 1003: // Salt/Moisture device
      response_data[0] = (float)now;        // Current time
      response_data[4] = 800.0;             // Salt level threshold
      response_data[5] = 50.0;              // Moisture threshold
      break;
      
    default: // Generic response
      response_data[0] = (float)now;        // Current time
      response_data[1] = 1.0;               // Generic config value
      break;
  }
}

void processSensorData(uint16_t device_id, float* sensor_data) {
  // Process received sensor data
  Serial.printf("\n=== Data from Device %d ===\n", device_id);
  
  // Print non-zero values with appropriate labels based on device ID
  switch(device_id) {
    case 1001: // Temperature/Humidity/Pressure/Battery device
      if(sensor_data[0] != 0.0) Serial.printf("Temperature: %.2f°C\n", sensor_data[0]); {epaperTemp = sensor_data[0]; epaperRSSI = incomingrssi;}
      if(sensor_data[1] != 0.0) Serial.printf("Humidity: %.2f%%\n", sensor_data[1]); {epaperHum = sensor_data[1];}
      if(sensor_data[2] != 0.0) Serial.printf("Pressure: %.2f hPa\n", sensor_data[2]); {epaperPres = sensor_data[2];}
      if(sensor_data[3] != 0.0) Serial.printf("Battery: %.2fV\n", sensor_data[3]); {epapervBat = sensor_data[3];}
      if(sensor_data[4] != 0.0) Serial.printf("Abshum: %.2f m/s\n", sensor_data[4]); {epaperabshum = sensor_data[4];}
      sendEpaperUpdate();
      break;
      
    case 1002: // Temperature/Battery only
      if(sensor_data[0] != 0.0) {Serial.printf("Temperature: %.2f°C\n", sensor_data[0]); fridgetemp = sensor_data[0]; fridgeRSSI = incomingrssi;}
      if(sensor_data[1] != 0.0) {Serial.printf("Humidity: %.2f%%\n", sensor_data[1]); fridgehum = sensor_data[1];}
      if(sensor_data[2] != 0.0) {Serial.printf("Pressure: %.2f hPa\n", sensor_data[2]); fridgepres = sensor_data[2];}
      if(sensor_data[3] != 0.0) {Serial.printf("Battery: %.2fV\n", sensor_data[3]); fridgevbat = sensor_data[3];}
      sendFridgeUpdate();
      break;
      
    case 1003: // Salt/Moisture device
      if(sensor_data[3] != 0.0) Serial.printf("Battery: %.2fV\n", sensor_data[3]);
      if(sensor_data[4] != 0.0) Serial.printf("Salt Level: %.2f ppm\n", sensor_data[4]);
      if(sensor_data[5] != 0.0) Serial.printf("Moisture: %.2f%%\n", sensor_data[5]);
      break;
      
    default: // Generic device
      for(int i = 0; i < 7; i++) {
        if(sensor_data[i] != 0.0) {
          Serial.printf("Sensor[%d]: %.2f\n", i, sensor_data[i]);
        }
      }
      break;
  }
  
  // Here you could add data logging, database storage, etc.
  // logDataToSD(device_id, sensor_data);
  // sendDataToCloud(device_id, sensor_data);
}

// ==================== ESP-NOW CALLBACKS ====================
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  digitalWrite(LEDpin, HIGH); // Turn on LED to indicate data received
  const uint8_t *mac = info->src_addr;
  incomingrssi = info->rx_ctrl->rssi;
  Serial.print("Receiving incoming data with RSSI of: ");
  Serial.print(incomingrssi);
  Serial.println("dB.");
  if(len == sizeof(sensor_message_t)) {
    sensor_message_t message;
    memcpy(&message, incomingData, sizeof(message));
    
    // Update device info
    device_info_t& device = known_devices[message.device_id];
    memcpy(device.mac, mac, 6);
    memcpy(device.last_sensor_data, message.sensor_data, sizeof(message.sensor_data));
    device.last_seen = millis();
    device.two_way_enabled = message.request_response;
    
    // Process the sensor data
    processSensorData(message.device_id, message.sensor_data);
    
    // Send response if requested
    if(message.request_response) {
      sendResponseToDevice(mac, message.device_id);
    }
  }
  digitalWrite(LEDpin, LOW); // Turn off LED after processing
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.printf("Response send status: %s\n", 
                status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failed");
}

// ==================== RESPONSE FUNCTIONS ====================
void sendResponseToDevice(const uint8_t* mac, uint16_t device_id) {
  response_message_t response;
  response.target_device_id = device_id;
  response.timestamp = millis();
  
  // Generate response data for this device
  generateResponseData(device_id, response.response_data);
  
  // Add peer if not already added
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, mac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA; // <-- Add this line
  
  // Try to add peer (it's OK if it already exists)
  esp_now_add_peer(&peerInfo);
  
  // Send response
  esp_err_t result = esp_now_send(mac, (uint8_t *) &response, sizeof(response));
  
  if(result == ESP_OK) {
    Serial.printf("Sending response to device %d\n", device_id);
    
    // Print response data being sent
    for(int i = 0; i < 7; i++) {
      if(response.response_data[i] != 0.0) {
        Serial.printf("  Response[%d]: %.2f\n", i, response.response_data[i]);
      }
    }
  } else {
    Serial.printf("Error sending response to device %d\n", device_id);
  }
}

// ==================== STATUS AND MONITORING ====================
void printDeviceStatus() {
  Serial.println("\n========== DEVICE STATUS ==========");
  Serial.printf("Known devices: %d\n", known_devices.size());
  
  unsigned long current_time = millis();
  
  for(auto& pair : known_devices) {
    uint16_t device_id = pair.first;
    device_info_t& device = pair.second;
    
    Serial.printf("\nDevice %d:\n", device_id);
    Serial.printf("  MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", 
                  device.mac[0], device.mac[1], device.mac[2], 
                  device.mac[3], device.mac[4], device.mac[5]);
    Serial.printf("  Two-way: %s\n", device.two_way_enabled ? "Yes" : "No");
    Serial.printf("  Last seen: %lu ms ago\n", current_time - device.last_seen);
    
    // Show last sensor data
    Serial.print("  Last data: ");
    bool has_data = false;
    for(int i = 0; i < 7; i++) {
      if(device.last_sensor_data[i] != 0.0) {
        if(has_data) Serial.print(", ");
        Serial.printf("[%d]:%.2f", i, device.last_sensor_data[i]);
        has_data = true;
      }
    }
    if(!has_data) Serial.print("No data");
    Serial.println();
  }
  Serial.println("===================================\n");
}

void receiveUARTData() {
  while (Serial2.available()) {
    digitalWrite(LEDpin, HIGH); // Turn on LED to indicate UART data received
    char c = Serial2.read();
    if (c == '\n') {
      processReceivedData(uartBuffer);
      uartBuffer = "";
    } else {
      uartBuffer += c;
    }
    digitalWrite(LEDpin, LOW); // Turn off LED after processing
  }
}

void processReceivedData(String data) {
  if (data.startsWith("PERIODIC:")) {
    // Parse periodic data from ESP32C3
    String payload = data.substring(9); // Remove "PERIODIC:"
    
    // Split the comma-separated values
    int values[17];
    int valueCount = 0;
    int lastIndex = 0;
    
    for (int i = 0; i <= payload.length(); i++) {
      if (i == payload.length() || payload.charAt(i) == ',') {
        if (valueCount < 17) {
          String valueStr = payload.substring(lastIndex, i);
          if (valueCount == 16) { // Last value is unix timestamp (unsigned long)
            if (localTimeUnix == valueStr.toInt()) {Serial.println("Error: received duplicate timestamp!");}
            localTimeUnix = valueStr.toInt();
            struct timeval now = {
              .tv_sec = localTimeUnix,
              .tv_usec = 0
            };
            settimeofday(&now, nullptr);
          } else {
            // Store float values
            switch(valueCount) {
              case 0: pm25in = valueStr.toFloat(); break;
              case 1: pm25out = valueStr.toFloat(); break;
              case 2: bridgehum = valueStr.toFloat(); break;
              case 3: bridgepres = valueStr.toFloat(); break;
              case 4: iaq = valueStr.toFloat(); break;
              case 5: windspeed = valueStr.toFloat(); break;
              case 6: brtemp = valueStr.toFloat(); break;
              case 7: brhum = valueStr.toFloat(); break;
              case 8: bridgeco2 = valueStr.toFloat(); break;
              case 9: bridgeIrms = valueStr.toFloat(); break;
              case 10: watts = valueStr.toFloat(); break;
              case 11: kw = valueStr.toFloat(); break;
              case 12: mintemp = valueStr.toFloat(); break;
              case 13: windgust = valueStr.toFloat(); break;
              case 14: break;
              case 15: lightread = valueStr.toFloat(); break;
            }
          }
          valueCount++;
        }
        lastIndex = i + 1;
      }
    }
    
    Serial.println("Received periodic data from ESP32C3");
    Serial.println("PM2.5 In: " + String(pm25in, 2));
    Serial.println("Temperature: " + String(brtemp, 2));
    Serial.println("Out temp: " + String(mintemp, 2));
    Serial.println("Unix Time: " + String(localTimeUnix));
  }
}

// Call this function when you receive ESP-NOW data from fridge monitor
void sendFridgeUpdate() {
  String fridgePacket = "FRIDGE:";
  fridgePacket += String(fridgevbat, 2) + ",";
  fridgePacket += String(fridgeRSSI) + ",";
  fridgePacket += String(fridgetemp, 2);
  fridgePacket += "\n";
  
  Serial2.print(fridgePacket);
  Serial.println("Sent fridge update to ESP32C3");
}

void sendEpaperUpdate() {
  String epaperPacket = "EPAPER:";
  epaperPacket += String(epapervBat, 2) + ",";
  epaperPacket += String(epaperRSSI) + ",";
  epaperPacket += String(epaperTemp, 2) + ",";
  epaperPacket += String(epaperHum, 2) + ",";
  epaperPacket += String(epaperPres, 2) + ",";
  epaperPacket += String(epaperabshum, 2);
  epaperPacket += "\n";
  Serial2.print(epaperPacket);
  Serial.println("Sent epaper update to ESP32C3");
}


// ==================== MAIN FUNCTIONS ====================
void setup() {
  if (firstrun) {
    firstrun = false;
    delay(2000);
    ESP.restart();
  }
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 15, 2);
  delay(1000);
  pinMode(LEDpin, OUTPUT);
  digitalWrite(LEDpin, LOW); // Turn off LED initially
  delay(100);
  digitalWrite(LEDpin, HIGH); // Turn off LED initially
  delay(100);
  digitalWrite(LEDpin, LOW); // Turn off LED initially
  delay(100);
  digitalWrite(LEDpin, HIGH); // Turn off LED initially
  delay(100);
  digitalWrite(LEDpin, LOW); // Turn off LED initially
  Serial.println("ESP32 ESP-NOW Receiver Starting...");
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  //WiFi.setTxPower(WIFI_POWER_8_5dBm); 
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  // Print MAC address
  uint8_t mac[6];
  WiFi.macAddress(mac);
  Serial.printf("Receiver MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n", 
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  
  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Register callbacks
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);
  //WiFi.setTxPower(WIFI_POWER_8_5dBm); 
  // Initialize time (optional, for timestamp responses)
  //configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  
  Serial.println("ESP-NOW Receiver initialized successfully");
  Serial.println("Waiting for sensor data...\n");
}

void loop() {
  unsigned long current_time = millis();
  
  // Print status periodically
  if(current_time - last_status_print >= STATUS_INTERVAL) {
    printDeviceStatus();
    last_status_print = current_time;
  }
  
  // Check for offline devices (optional)
  checkOfflineDevices();
  receiveUARTData();
  delay(10);
}

// ==================== UTILITY FUNCTIONS ====================
void checkOfflineDevices() {
  const unsigned long OFFLINE_THRESHOLD = 300000; // 5 minutes
  unsigned long current_time = millis();
  
  for(auto& pair : known_devices) {
    uint16_t device_id = pair.first;
    device_info_t& device = pair.second;
    
    if(current_time - device.last_seen > OFFLINE_THRESHOLD) {
      // Device hasn't been seen for a while
      // You could log this, send alerts, etc.
      // Serial.printf("Warning: Device %d appears offline\n", device_id);
    }
  }
}

// ==================== OPTIONAL EXTENSIONS ====================
/*
 * You can extend this code with:
 * 
 * 1. Data logging to SD card:
 * void logDataToSD(uint16_t device_id, float* data) {
 *   // Implementation for SD card logging
 * }
 * 
 * 2. WiFi connectivity for cloud uploads:
 * void sendDataToCloud(uint16_t device_id, float* data) {
 *   // Implementation for cloud connectivity
 * }
 * 
 * 3. Web server for monitoring:
 * void setupWebServer() {
 *   // Serve device status via web interface
 * }
 * 
 * 4. MQTT publishing:
 * void publishToMQTT(uint16_t device_id, float* data) {
 *   // Publish sensor data to MQTT broker
 * }
 */