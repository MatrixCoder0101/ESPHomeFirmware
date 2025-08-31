#include <EEPROM.h>
#include "RMaker.h"
#include "WiFi.h"
#include "WiFiProv.h"
#include <AceButton.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <HTTPClient.h>
#include <Update.h>
#include <ArduinoJson.h>  // Add this line at the top

// SSD1306 128×64 via I2C
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
using namespace ace_button;

// ======== CONFIGURABLE FLAGS ==========
#define ENABLE_EEPROM true   // Enable EEPROM memory restore (true = Active)
#define USE_LATCHED_SWITCH true  // true = latched switch, false = push button
//#define EEPROM_SIZE 10
#define EEPROM_SIZE 20  // Was 10; now enough space for version string
#define CURRENT_FIRMWARE_VERSION "v1.0.0"
#define GITHUB_USER "MatrixCoder0101"
#define GITHUB_REPO "ESPHomeFirmware"
#define EEPROM_VERSION_ADDR 8  // EEPROM addresses 0‑3 used by relays; 8‑17 for version

// ======== DEVICE SETTINGS =============
const char *service_name = "PROV_12345";
const char *pop = "1234567";

unsigned long lastOTACheckTime = 0;
const unsigned long otaCheckInterval = 5 * 60 * 1000; // 5 minutes in ms
bool initialCheckDone = false;
unsigned long ledPreviousMillis = 0;
int ledState = LOW;
int ledStage = 0;
TaskHandle_t wifiLedTaskHandle = NULL;

char deviceName_1[] = "Light";
char deviceName_2[] = "Light2";
char deviceName_3[] = "Fan";
char deviceName_4[] = "TV";

// GPIO Setup
static uint8_t RelayPin1 = 23;
static uint8_t RelayPin2 = 19;
static uint8_t RelayPin3 = 18;
static uint8_t RelayPin4 = 5;

static uint8_t SwitchPin1 = 13;
static uint8_t SwitchPin2 = 12;
static uint8_t SwitchPin3 = 14;
static uint8_t SwitchPin4 = 27;

static uint8_t wifiLed = 2;
static uint8_t gpio_reset = 0;

bool toggleState_1 = LOW;
bool toggleState_2 = LOW;
bool toggleState_3 = LOW;
bool toggleState_4 = LOW;

ButtonConfig config1;
AceButton button1(&config1);
ButtonConfig config2;
AceButton button2(&config2);
ButtonConfig config3;
AceButton button3(&config3);
ButtonConfig config4;
AceButton button4(&config4);

static Switch my_switch1(deviceName_1, &RelayPin1);
static Switch my_switch2(deviceName_2, &RelayPin2);
static Switch my_switch3(deviceName_3, &RelayPin3);
static Switch my_switch4(deviceName_4, &RelayPin4);

/*
void writeEEPROM(int addr, bool state) {
  if (ENABLE_EEPROM) {
    EEPROM.write(addr, state);
    EEPROM.commit();
    Serial.printf("EEPROM saved: addr %d = %d\n", addr, state);
  }
}
*/
void writeEEPROM(int addr, bool state) {
  if (ENABLE_EEPROM && EEPROM.read(addr) != state) {
    EEPROM.write(addr, state);
    EEPROM.commit();
    Serial.printf("EEPROM saved: addr %d = %d\n", addr, state);
  }
}

bool readEEPROM(int addr) {
  if (ENABLE_EEPROM) {
    return EEPROM.read(addr);
  }
  return false;
}

void displayMessage(const char* line1, const char* line2 = "", const char* line3 = "") {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  
  if (line1 && strlen(line1)) u8g2.drawStr(0, 15, line1);
  if (line2 && strlen(line2)) u8g2.drawStr(0, 30, line2);
  if (line3 && strlen(line3)) u8g2.drawStr(0, 45, line3);
  
  u8g2.sendBuffer();
}

void setRelay(uint8_t pin, int addr, bool state) {
  digitalWrite(pin, state);   // (sahi for active-high relay)
  if (ENABLE_EEPROM) writeEEPROM(addr, state);
}

void buttonHandler(AceButton* button, uint8_t eventType, uint8_t, uint8_t relayPin, int eepromAddr, Switch &sw, bool &state) {
  bool newState = false;

  if (USE_LATCHED_SWITCH) {
    newState = (eventType == AceButton::kEventPressed);
  } else {
    if (eventType != AceButton::kEventReleased) return;
    newState = !(digitalRead(relayPin) == LOW);
  }

  setRelay(relayPin, eepromAddr, newState);
  state = newState;
  sw.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, state);
  Serial.printf("Relay on pin %d toggled manually to %d\n", relayPin, state);
}

String readStoredVersion() {
  String version = "";
  for (int i = EEPROM_VERSION_ADDR; i < EEPROM_VERSION_ADDR + 10; i++) {
    char c = EEPROM.read(i);
    if (c == '\0' || c == 0xFF) break;
    version += c;
  }
  return version;
}

void storeFirmwareVersion(const char* version) {
  for (int i = 0; i < 10; i++) {
    char c = version[i];
    EEPROM.write(EEPROM_VERSION_ADDR + i, c ? c : '\0');
  }
  EEPROM.commit();
  Serial.printf("Firmware version stored: %s\n", version);
}

void checkForOTAUpdate() {
  if (WiFi.status() != WL_CONNECTED) {
  displayMessage("No WiFi", "", "Can't update");
  return;
}

  displayMessage("Checking Update...", "", "");
  Serial.println("[OTA] Checking for updates...");

  HTTPClient http;
  String latestVersion, binURL;

  String apiURL = "https://api.github.com/repos/" GITHUB_USER "/" GITHUB_REPO "/releases/latest";
  http.begin(apiURL);
  http.addHeader("User-Agent", "ESP32-OTA-Updater");

  int httpCode = http.GET();
  if (httpCode != 200) {
    displayMessage("GitHub API Error", (String("Code: ") + httpCode).c_str(), "");
    Serial.printf("[OTA] GitHub API failed, HTTP code: %d\n", httpCode);
    http.end();
    return;
  }

  String payload = http.getString();
  http.end();

  const size_t capacity = 32 * 1024;
  DynamicJsonDocument doc(capacity);
  DeserializationError error = deserializeJson(doc, payload);
  if (error) {
    displayMessage("JSON Parse Error", error.c_str(), "");
    return;
  }
  if (!doc.containsKey("tag_name")) {
    displayMessage("Invalid JSON", "Missing tag_name", "");
    return;
  }

  latestVersion = doc["tag_name"].as<String>();
  displayMessage("Latest Version:", latestVersion.c_str(), "");
  Serial.printf("[OTA] Latest firmware version: %s\n", latestVersion.c_str());

  String stored = readStoredVersion();
  if (latestVersion == CURRENT_FIRMWARE_VERSION || latestVersion == stored) {
    char line2[30];
    snprintf(line2, sizeof(line2), "Version: %s", latestVersion.c_str());
    displayMessage("Up to Date", line2, "");
    Serial.printf("[OTA] Already on latest version: %s\n", latestVersion.c_str());
    return;
  }

  String fileName = "";
  JsonArray assets = doc["assets"];
  for (JsonObject asset : assets) {
    String name = asset["name"].as<String>();
    if (name == "ESPHomeFirmware.ino.bin") {
      binURL = asset["browser_download_url"].as<String>();
      fileName = name;
      break;
    }
  }

  if (binURL == "") {
    displayMessage("No .bin found", "", "");
    return;
  }

  char line2[30];
  snprintf(line2, sizeof(line2), "Version: %s", latestVersion.c_str());
  char line3[30];
  snprintf(line3, sizeof(line3), "File: %s", fileName.c_str());

  displayMessage("Downloading", line2, line3);
  Serial.printf("[OTA] New version available: %s\n", latestVersion.c_str());
  Serial.printf("[OTA] Binary file: %s\n", fileName.c_str());


  // Start OTA
  http.begin(binURL);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  int code2 = http.GET();
  if (code2 != HTTP_CODE_OK) {
    displayMessage("Download Failed", (String("Code: ") + code2).c_str(), "");
    http.end();
    return;
  }

  int contentLength = http.getSize();
  WiFiClient *stream = http.getStreamPtr();

  if (startOTAUpdate(stream, contentLength, latestVersion)) {
    displayMessage("Update OK", "Rebooting...", "");
    //storeFirmwareVersion(latestVersion.c_str());
    delay(2000);
    ESP.restart();
  } else {
    displayMessage("OTA Failed", "", "");
  }

  http.end();
}

bool startOTAUpdate(WiFiClient* client, int contentLength, const String &latestVersion) {
  Serial.println("Initializing update...");

  if (!Update.begin(contentLength)) {
    Serial.printf("Update begin failed: %s\n", Update.errorString());
    return false;
  }

  size_t written = 0;
  int progress = 0;
  int lastProgress = 0;
  const unsigned long timeoutDuration = 300 * 1000;  // 2 minutes
  unsigned long lastDataTime = millis();

 while (written < contentLength) {
  if (client->available()) {
    uint8_t buffer[128];
    size_t len = client->read(buffer, sizeof(buffer));
    if (len > 0) {
      Update.write(buffer, len);
      written += len;

      progress = (written * 100) / contentLength;
      if (progress != lastProgress) {
        Serial.printf("Writing Progress: %d%%\n", progress);
        
        // -------- OLED progress display --------
        char line2[30];
        snprintf(line2, sizeof(line2), "Progress: %d%%", progress);
        displayMessage("Updating...", line2, "");
        // ---------------------------------------

        lastProgress = progress;
      }

      lastDataTime = millis();  // reset timeout
    }
  }

  if (millis() - lastDataTime > timeoutDuration) {
    Serial.println("Timeout during OTA update.");
    displayMessage("Update Timeout", "", "");
    Update.abort();
    return false;
  }

  yield();  // prevent watchdog reset
}

  if (written != contentLength) {
    int percent = (written * 100) / contentLength;
    Serial.printf("Mismatch: expected %d bytes, wrote %d bytes (%d%%)\n", contentLength, written, percent);
    displayMessage("OTA Failed", "Incomplete write", "");
    Update.abort();
    return false;
}

 if (!Update.end()) {
    Serial.printf("Update failed: %s\n", Update.errorString());
    displayMessage("Update Failed", Update.errorString(), "");
    return false;
}

  if (Update.isFinished()) {
    Serial.println("OTA update successful.");
    displayMessage("Device Ready", "", "");
    storeFirmwareVersion(latestVersion.c_str());   // ✅ ab sahi scope me
    return true;
  }
}

void stopWiFiLedTask() {
  if (wifiLedTaskHandle != NULL) {
    vTaskDelete(wifiLedTaskHandle);
    wifiLedTaskHandle = NULL;
  }
}

void setup() {
  pinMode(wifiLed, OUTPUT);
  digitalWrite(wifiLed, LOW);
  Serial.begin(115200);
  u8g2.begin();
  displayMessage("Booting...", "", "");
  Serial.println("[SYSTEM] Booting...");
  displayMessage("Connecting WiFi...", "", "");
  Serial.println("[WIFI] Connecting...");

  if (ENABLE_EEPROM) EEPROM.begin(EEPROM_SIZE);

  toggleState_1 = ENABLE_EEPROM ? readEEPROM(0) : LOW;
  toggleState_2 = ENABLE_EEPROM ? readEEPROM(1) : LOW;
  toggleState_3 = ENABLE_EEPROM ? readEEPROM(2) : LOW;
  toggleState_4 = ENABLE_EEPROM ? readEEPROM(3) : LOW;

  pinMode(RelayPin1, OUTPUT); pinMode(RelayPin2, OUTPUT);
  pinMode(RelayPin3, OUTPUT); pinMode(RelayPin4, OUTPUT);
  pinMode(wifiLed, OUTPUT);

  pinMode(SwitchPin1, INPUT_PULLUP); pinMode(SwitchPin2, INPUT_PULLUP);
  pinMode(SwitchPin3, INPUT_PULLUP); pinMode(SwitchPin4, INPUT_PULLUP);
  pinMode(gpio_reset, INPUT);

  setRelay(RelayPin1, 0, toggleState_1);
  setRelay(RelayPin2, 1, toggleState_2);
  setRelay(RelayPin3, 2, toggleState_3);
  setRelay(RelayPin4, 3, toggleState_4);
  digitalWrite(wifiLed, LOW);

  config1.setEventHandler([](AceButton* b, uint8_t e, uint8_t s) {
    buttonHandler(b, e, s, RelayPin1, 0, my_switch1, toggleState_1);
  });
  config2.setEventHandler([](AceButton* b, uint8_t e, uint8_t s) {
    buttonHandler(b, e, s, RelayPin2, 1, my_switch2, toggleState_2);
  });
  config3.setEventHandler([](AceButton* b, uint8_t e, uint8_t s) {
    buttonHandler(b, e, s, RelayPin3, 2, my_switch3, toggleState_3);
  });
  config4.setEventHandler([](AceButton* b, uint8_t e, uint8_t s) {
    buttonHandler(b, e, s, RelayPin4, 3, my_switch4, toggleState_4);
  });

  button1.init(SwitchPin1);
  button2.init(SwitchPin2);
  button3.init(SwitchPin3);
  button4.init(SwitchPin4);

  Node my_node = RMaker.initNode("ESPHome");
  my_switch1.addCb(write_callback);
  my_switch2.addCb(write_callback);
  my_switch3.addCb(write_callback);
  my_switch4.addCb(write_callback);

  my_node.addDevice(my_switch1);
  my_node.addDevice(my_switch2);
  my_node.addDevice(my_switch3);
  my_node.addDevice(my_switch4);

  RMaker.enableOTA(OTA_USING_PARAMS);
  RMaker.enableTZService();
  RMaker.enableSchedule();

  RMaker.start();
  WiFi.onEvent(sysProvEvent);
  WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE, WIFI_PROV_SCHEME_HANDLER_FREE_BTDM, WIFI_PROV_SECURITY_1, pop, service_name);
  delay(2000);

  my_switch1.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, toggleState_1);
  my_switch2.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, toggleState_2);
  my_switch3.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, toggleState_3);
  my_switch4.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, toggleState_4);

  Serial.println("Setup completed with EEPROM and mode flags.");
  Serial.printf("[DEBUG] Provisioned: %d, WiFi Status: %d\n", WiFi.status());
}

void loop() {
  if (digitalRead(gpio_reset) == LOW) {
    delay(100);
    int startTime = millis();
    while (digitalRead(gpio_reset) == LOW) delay(50);
    int duration = millis() - startTime;
    if (duration > 10000) {
      Serial.println("Factory reset triggered.");
      RMakerFactoryReset(2);
    } else if (duration > 3000) {
      Serial.println("WiFi reset triggered.");
      RMakerWiFiReset(2);
    }
  }

  button1.check();
  button2.check();
  button3.check();
  button4.check();
  unsigned long now = millis();

// Pehla OTA check boot ke thodi der baad hona chahiye
if (!initialCheckDone && millis() > 10000) {
  checkForOTAUpdate();
  lastOTACheckTime = now;
  initialCheckDone = true;
}

// Har 5 minute baad OTA check
if (now - lastOTACheckTime >= otaCheckInterval) {
  Serial.println("[OTA] 5-minute check triggered.");
  checkForOTAUpdate();
  lastOTACheckTime = now;
}

}

void write_callback(Device *device, Param *param, const param_val_t val, void *priv_data, write_ctx_t *ctx) {
  const char *device_name = device->getDeviceName();
  const char *param_name = param->getParamName();

  if(strcmp(param_name, "Power") == 0) {
    bool newState = val.val.b;
    if (strcmp(device_name, deviceName_1) == 0) {
      setRelay(RelayPin1, 0, newState); toggleState_1 = newState; my_switch1.updateAndReportParam(param_name, newState);
    } else if (strcmp(device_name, deviceName_2) == 0) {
      setRelay(RelayPin2, 1, newState); toggleState_2 = newState; my_switch2.updateAndReportParam(param_name, newState);
    } else if (strcmp(device_name, deviceName_3) == 0) {
      setRelay(RelayPin3, 2, newState); toggleState_3 = newState; my_switch3.updateAndReportParam(param_name, newState);
    } else if (strcmp(device_name, deviceName_4) == 0) {
      setRelay(RelayPin4, 3, newState); toggleState_4 = newState; my_switch4.updateAndReportParam(param_name, newState);
    }
    Serial.printf("Write callback for %s: new state = %d\n", device_name, newState);
  }
}

void sysProvEvent(arduino_event_t *sys_event) {
  switch (sys_event->event_id) {
    case ARDUINO_EVENT_PROV_START:
      Serial.printf("Provisioning started: %s\n", service_name);
      printQR(service_name, pop, "ble");
       displayMessage("Provisioning...", "", "Use App to Setup");
       stopWiFiLedTask();
      // Fast double blink → ON-ON-OFF-OFF (150ms interval)
      xTaskCreate([](void *){
        int stage = 0;
        for (;;) {
          if (stage == 0 || stage == 2) digitalWrite(wifiLed, HIGH);
          else digitalWrite(wifiLed, LOW);
          stage = (stage + 1) % 4;
          vTaskDelay(150 / portTICK_PERIOD_MS);
        }
      }, "WiFiLED_Prov", 1024, NULL, 1, &wifiLedTaskHandle);
      break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println("Connected to Wi-Fi");
       displayMessage("WiFi Connected", "", "");
       stopWiFiLedTask();
      // Connected: double blink every 2.5 sec
      xTaskCreate([](void *){
        const int pattern[] = {100, 100, 200, 100, 100, 1500};
        int index = 0;
        bool ledState = LOW;
        for (;;) {
          ledState = !ledState;
          digitalWrite(wifiLed, ledState);
          vTaskDelay(pattern[index] / portTICK_PERIOD_MS);
          index = (index + 1) % 6;
        }
      }, "WiFiLED_Connected", 1024, NULL, 1, &wifiLedTaskHandle);
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("Disconnected from Wi-Fi");
       displayMessage("Disconnected", "", "Retrying...");
       stopWiFiLedTask();
      // Disconnected: 500ms blink
      xTaskCreate([](void *){
        bool ledState = LOW;
        for (;;) {
          ledState = !ledState;
          digitalWrite(wifiLed, ledState);
          vTaskDelay(500 / portTICK_PERIOD_MS);
        }
      }, "WiFiLED_Disconnected", 1024, NULL, 1, &wifiLedTaskHandle);
      break;
    case ARDUINO_EVENT_PROV_END:
      Serial.println("Provisioning ended");
      displayMessage("Provisioning Done", "", "");
      break;
    default:
      break;
  }
}