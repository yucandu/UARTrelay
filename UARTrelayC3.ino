#include <Arduino.h>
#include <ArduinoOTA.h>
#include <BlynkSimpleEsp32.h>
#include <WiFi.h>
#include "time.h"
#include "esp_sntp.h"
#include <HardwareSerial.h>


const char* ssid = "mikesnet";
const char* password = "springchicken";
RTC_NOINIT_ATTR  bool firstrun = true;


int hours, mins, secs;
char auth[] = "8_-CN2rm4ki9P3i_NkPhxIbCiKd5RXhK";  //hubert
char fridgeauth[] = "VnFlJdW3V0uZQaqslqPJi6WPA9LaG1Pk"; //fridgetemp
struct tm timeinfo;
bool isSetNtp = false;

// Data variables to send every 5 seconds
float pm25in = 12.5;
float pm25out = 8.3;
float bridgehum = 45.2;
float bridgepres = 1013.25;
float iaq = 25.0;
float windspeed = 5.2;
float brtemp = 22.5;
float brhum = 55.8;
float bridgeco2 = 400.0;
float bridgeIrms = 2.5;
float watts = 250.0;
float kw = 0.25;
float mintemp = 18.5;
float windgust = 8.1;
float fridgetemp = 4.2;
float lightread = 350.0;
unsigned long localTimeUnix = 1640995200;
float epaperTemp, epaperHum, epaperPres, epaperabshum, epapervBat, epaperRSSI, epaperDrain;
float neotemp = 0.0; // Temperature from NeoPixel strip
float jojutemp = 0.0; // Temperature from Joju sensor
float bridgetemp = 999.0; // Temperature from Bridge sensor
unsigned long reconnectTime;
WidgetBridge bridge2(V60);

BLYNK_CONNECTED() {
  bridge2.setAuthToken (fridgeauth);
}



WidgetTerminal terminal(V70);

void printLocalTime() {
  time_t rawtime;
  struct tm* timeinfo;
  time(&rawtime);
  timeinfo = localtime(&rawtime);
  terminal.print(asctime(timeinfo));
}


BLYNK_WRITE(V70) {
  if (String("help") == param.asStr()) {
    terminal.println("==List of available commands:==");
    terminal.println("wifi");
    terminal.println("==End of list.==");
  }
  if (String("wifi") == param.asStr()) {
    terminal.print("Connected to: ");
    terminal.println(ssid);
    terminal.print("IP address:");
    terminal.println(WiFi.localIP());
    terminal.print("Signal strength: ");
    terminal.println(WiFi.RSSI());
    printLocalTime();
  }
    terminal.flush();
}

// Received data from ESP32
float received_fridgevbat = 0.0;
int received_fridgeRSSI = 0;
float received_fridgetemp = 0.0;

// Timing variables
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 5000; // 5 seconds

// UART buffer
String uartBuffer = "";


void cbSyncTime(struct timeval *tv) { // callback function to show when NTP was synchronized
  Serial.println("NTP time synched");
  Serial.println("getlocaltime");
  getLocalTime(&timeinfo);

  time_t rawtime;
  struct tm* timeinfo;
  time(&rawtime);
  timeinfo = localtime(&rawtime);

  Serial.println(asctime(timeinfo));
  time_t now = time(nullptr); // local-adjusted time
  localTimeUnix = static_cast<uint32_t>(now); // 32-bit to send via ESP-NOW
  isSetNtp = true;
}


void initSNTP() {  
  sntp_set_sync_interval(10 * 60 * 1000UL);  // 1 hour
  sntp_set_time_sync_notification_cb(cbSyncTime);
  esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
  esp_sntp_setservername(0, "192.168.50.197");
  esp_sntp_init();
  wait4SNTP();
  setTimezone();
}

void wait4SNTP() {
  Serial.print("Waiting for time...");
  while (sntp_get_sync_status() != SNTP_SYNC_STATUS_COMPLETED) {
    delay(1000);
    Serial.print(".");
  }
}

void setTimezone() {  
  setenv("TZ","EST5EDT,M3.2.0,M11.1.0",1);
  tzset();
}

#define every(interval) \
    static uint32_t __every__##interval = millis(); \
    if (millis() - __every__##interval >= interval && (__every__##interval = millis()))

float findLowestNonZero(float a, float b, float c) {
  // Initialize minimum to a very large value
  float minimum = 999;

  // Check each variable and update the minimum value
  if (a != 0.0 && a < minimum) {
    minimum = a;
  }
  if (b != 0.0 && b < minimum) {
    minimum = b;
  }
  if (c != 0.0 && c < minimum) {
    minimum = c;
  }

  return minimum;
}

BLYNK_WRITE(V41) {
  neotemp = param.asFloat();
}

BLYNK_WRITE(V42) {
  jojutemp = param.asFloat();
}

BLYNK_WRITE(V62) {
  bridgetemp = param.asFloat();
}

BLYNK_WRITE(V63) {
  bridgehum = param.asFloat();
}

BLYNK_WRITE(V66) {
  pm25out = param.asFloat();
}

BLYNK_WRITE(V71) {
  pm25in = param.asFloat();
}

BLYNK_WRITE(V72) {
  brtemp = param.asFloat();
}

BLYNK_WRITE(V74) {
  brhum = param.asFloat();
}

BLYNK_WRITE(V75) {
  iaq = param.asFloat();
}

BLYNK_WRITE(V77) {
  bridgeco2 = param.asFloat();
}

BLYNK_WRITE(V78) {
  float ws = param.asFloat();
  if (!isnan(ws)) {
    windspeed = ws;
  }
}

BLYNK_WRITE(V79) {
  windgust = param.asFloat();
}

BLYNK_WRITE(V80) {
  bridgepres = param.asFloat();
}

BLYNK_WRITE(V81) {
  bridgeIrms = param.asFloat();
  watts = bridgeIrms;
  kw = watts / 1000.0;
}

BLYNK_WRITE(V82) {
  fridgetemp = param.asFloat();
}

BLYNK_WRITE(V83) {
  lightread = param.asFloat();
}






void sendPeriodicData() {
  time_t now = time(nullptr); // local-adjusted time
  localTimeUnix = static_cast<uint32_t>(now); // 32-bit to send via ESP-NOW
  String dataPacket = "PERIODIC:";
  dataPacket += String(pm25in, 2) + ",";
  dataPacket += String(pm25out, 2) + ",";
  dataPacket += String(bridgehum, 2) + ",";
  dataPacket += String(bridgepres, 2) + ",";
  dataPacket += String(iaq, 2) + ",";
  dataPacket += String(windspeed, 2) + ",";
  dataPacket += String(brtemp, 2) + ",";
  dataPacket += String(brhum, 2) + ",";
  dataPacket += String(bridgeco2, 2) + ",";
  dataPacket += String(bridgeIrms, 2) + ",";
  dataPacket += String(watts, 2) + ",";
  dataPacket += String(kw, 2) + ",";
  dataPacket += String(mintemp, 2) + ",";
  dataPacket += String(windgust, 2) + ",";
  dataPacket += String(fridgetemp, 2) + ",";
  dataPacket += String(lightread, 2) + ",";
  dataPacket += String(localTimeUnix);
  dataPacket += "\n";
  
  Serial1.print(dataPacket);
  //Serial.println("Sent periodic data");
}

void receiveUARTData() {
  if (Serial1.available()) {
    analogWrite(8, 20);
    while (Serial1.available()) {
      char c = Serial1.read();
      Serial.print("Received: 0x");
      Serial.println((uint8_t)c, HEX);
      if (c == '\n') {
        processReceivedData(uartBuffer);
        uartBuffer = "";
      } else {
        uartBuffer += c;
      }
   }
  analogWrite(8, 0);
  }
}

void processReceivedData(String data) {
  if (data.startsWith("FRIDGE:")) {
    // Parse fridge data: FRIDGE:fridgevbat,fridgeRSSI,fridgetemp
    String payload = data.substring(7); // Remove "FRIDGE:"
    
    int firstComma = payload.indexOf(',');
    int secondComma = payload.indexOf(',', firstComma + 1);
    
    if (firstComma != -1 && secondComma != -1) {
      received_fridgevbat = payload.substring(0, firstComma).toFloat();
      received_fridgeRSSI = payload.substring(firstComma + 1, secondComma).toInt();
      received_fridgetemp = payload.substring(secondComma + 1).toFloat();
      
      // Update the fridgetemp variable with new reading
      fridgetemp = received_fridgetemp;
      
      terminal.println("Received fridge update:");
      terminal.println("  Temperature: " + String(received_fridgetemp, 2) + "°C");
      terminal.println("  Battery: " + String(received_fridgevbat, 2) + "V");
      terminal.println("  RSSI: " + String(received_fridgeRSSI) + "dBm");
      printLocalTime();
      terminal.flush();
      bridge2.virtualWrite(V1, fridgetemp);
      bridge2.virtualWrite(V6, received_fridgevbat);
      bridge2.virtualWrite(V4, received_fridgeRSSI);
    }
  }
  else if (data.startsWith("EPAPER:")) {
    String payload = data.substring(7); // Remove "FRIDGE:"
    
    int firstComma = payload.indexOf(',');
    int secondComma = payload.indexOf(',', firstComma + 1);
    int thirdComma = payload.indexOf(',', secondComma + 1);
    int fourthComma = payload.indexOf(',', thirdComma + 1);
    int fifthComma = payload.indexOf(',', fourthComma + 1);
    int sixthComma = payload.indexOf(',', fifthComma + 1);
    int seventhComma = payload.indexOf(',', sixthComma + 1);
    
    if (firstComma != -1 && secondComma != -1) {
      epapervBat = payload.substring(0, firstComma).toFloat();
      epaperRSSI = payload.substring(firstComma + 1, secondComma).toInt();
      epaperTemp = payload.substring(secondComma + 1, thirdComma).toFloat();
      epaperHum = payload.substring(thirdComma + 1, fourthComma).toFloat();
      epaperPres = payload.substring(fourthComma + 1, fifthComma).toFloat();
      epaperabshum = payload.substring(fifthComma + 1, sixthComma).toFloat();
      epaperDrain = payload.substring(sixthComma + 1).toFloat();
      terminal.println("Received epaper update:");
      terminal.println("  Battery: " + String(epapervBat, 2) + "V");
      terminal.println("  RSSI: " + String(epaperRSSI) + "dBm");
      terminal.println("  Temperature: " + String(epaperTemp, 2) + "°C");
      terminal.println("  Humidity: " + String(epaperHum, 2) + "%");
      terminal.println("  Pressure: " + String(epaperPres, 2) + " hPa");
      terminal.println("  Absolute Humidity: " + String(epaperabshum, 2) + " m/s");
      terminal.println("  Drain: " + String(epaperDrain, 2) + " mV/D");
      printLocalTime();
      terminal.flush();
      Blynk.virtualWrite(V111, epaperTemp);
      Blynk.virtualWrite(V112, epaperHum);
      Blynk.virtualWrite(V113, epaperPres);
      Blynk.virtualWrite(V114, epaperabshum);
      Blynk.virtualWrite(V115, epapervBat);
      Blynk.virtualWrite(V116, epaperDrain);
      Blynk.virtualWrite(V117, epaperRSSI);
    }
  }
  //digitalWrite(8, LOW);
}

void setup() {
  delay(1000);
  Serial.begin(115200);
  delay(500);
  //sntp_set_time_sync_notification_cb(cbSyncTime);
  Serial.println("ESP32C3 UART Started");
  Serial1.begin(9600, SERIAL_8N1, 21, 20);
  pinMode(8, OUTPUT);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  WiFi.setTxPower(WIFI_POWER_8_5dBm); //low power for better connectivity
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(200);
    digitalWrite(8, HIGH);
    delay(200);
    digitalWrite(8, LOW);
  }

    digitalWrite(8, HIGH);
    delay(500);
    digitalWrite(8, LOW);
    delay(500);
    digitalWrite(8, HIGH);
    delay(500);
    digitalWrite(8, LOW);


  Blynk.config(auth, IPAddress(192, 168, 50, 197), 8080);
  Blynk.connect();
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("Init time");
  initSNTP();
  //initTime("EST5EDT,M3.2.0,M11.1.0");
  //Serial.println("Set env");
  //setenv("TZ","EST5EDT,M3.2.0,M11.1.0",1);  //  Now adjust the TZ.  Clock settings are adjusted to show the new local time
  //Serial.println("tzset");
  //tzset();
  Serial.println("getlocaltime");
  getLocalTime(&timeinfo);

  time_t rawtime;
  struct tm* timeinfo;
  time(&rawtime);
  timeinfo = localtime(&rawtime);

  Serial.println(asctime(timeinfo));
  time_t now = time(nullptr); // local-adjusted time
  localTimeUnix = static_cast<uint32_t>(now); // 32-bit to send via ESP-NOW
  Serial.println(localTimeUnix);
  Serial.println("Connecting to Blynk...");
  while ((!Blynk.connected()) && (millis() < 20000)){
    Serial.print(".");
       delay(500);}
  if (Blynk.connected()) {
    Serial.println("Connected to Blynk!");
    Blynk.syncVirtual(V41);
    Blynk.syncVirtual(V42);
    Blynk.syncVirtual(V62);
    Blynk.syncVirtual(V63);
    Blynk.syncVirtual(V66);
    Blynk.syncVirtual(V71);
    Blynk.syncVirtual(V72);
    Blynk.syncVirtual(V74);
    Blynk.syncVirtual(V75);
    Blynk.syncVirtual(V77);
    Blynk.syncVirtual(V78);
    Blynk.syncVirtual(V79);
    Blynk.syncVirtual(V80);
    Blynk.syncVirtual(V81);
    Blynk.syncVirtual(V82);
    Blynk.syncVirtual(V83);
    Blynk.syncVirtual(V120); //flash button
    //mintemp = findLowestNonZero(bridgetemp, neotemp, jojutemp);
    //Blynk.virtualWrite(V118, mintemp);
  } else {
    Serial.println("Failed to connect to Blynk within timeout.");
  }
  ArduinoOTA.setHostname("ESP32C3-UART");
  ArduinoOTA.begin();
  Serial.println("Arduino OTA ready");
  terminal.println("***ESP-NOW UART RELAY v1.1***");
  terminal.print("Connected to ");
  terminal.println(ssid);
  terminal.print("IP address: ");
  terminal.println(WiFi.localIP());
  printLocalTime();
  terminal.print("Compiled on: ");
  terminal.print(__DATE__);
  terminal.print(" at ");
  terminal.println(__TIME__);
  terminal.flush();
    digitalWrite(8, HIGH);
    delay(100);
    digitalWrite(8, LOW);
    delay(100);
    digitalWrite(8, HIGH);
    delay(100);
    digitalWrite(8, LOW);
    delay(100);
    digitalWrite(8, HIGH);
    delay(100);
    digitalWrite(8, LOW);

}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {Blynk.run();   ArduinoOTA.handle();}  //don't do Blynk unless wifi
    else { //if no wifi, try to reconnect
      digitalWrite(8, HIGH);
      if (millis() - reconnectTime > 30000) {
            WiFi.disconnect();
            WiFi.reconnect();
            while (WiFi.status() != WL_CONNECTED) {
              Serial.print(".");
              delay(200);
              digitalWrite(8, HIGH);
              delay(200);
              digitalWrite(8, LOW);
            }
            digitalWrite(8, LOW);
            reconnectTime = millis();
      }

    }

  if (millis() - lastSendTime >= sendInterval) {
    sendPeriodicData();
    lastSendTime = millis();
  }
  
  // Check for incoming UART data
  receiveUARTData();


  
  // Update sensor readings and time here
  // updateSensorReadings();
  // localTimeUnix = getCurrentUnixTime();
  
  delay(10);
  every(30000) {
    mintemp = findLowestNonZero(bridgetemp, neotemp, jojutemp);
    Blynk.virtualWrite(V82, fridgetemp);
    if (mintemp < 70) {Blynk.virtualWrite(V118, mintemp);}

    //Blynk.virtualWrite(V111, epaperTemp);
    //Blynk.virtualWrite(V112, epaperHum);
    //Blynk.virtualWrite(V113, epaperPres);
    //Blynk.virtualWrite(V114, epaperabshum);
    //Blynk.virtualWrite(V115, epapervBat);
  }
}
