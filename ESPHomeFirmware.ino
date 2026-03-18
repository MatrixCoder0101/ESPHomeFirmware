/*
 * ╔══════════════════════════════════════════════════════════╗
 * ║         ESPHome Smart Dashboard v2.0                    ║
 * ║         ILI9341 320x240 TFT  |  ESP32  |  RainMaker    ║
 * ╚══════════════════════════════════════════════════════════╝
 */

// ─────────────────────────────────────────────────────────────
//  LIBRARIES
// ─────────────────────────────────────────────────────────────
#include <EEPROM.h>
#include "RMaker.h"
#include "WiFi.h"
#include "WiFiProv.h"
#include <AceButton.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <qrcode_espi.h>  // yoprogramo/QRcode_eSPI
#include <HTTPClient.h>
#include <Update.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <time.h>

using namespace ace_button;

// ─────────────────────────────────────────────────────────────
//  DISPLAY
// ─────────────────────────────────────────────────────────────
TFT_eSPI tft = TFT_eSPI();
// Custom subclass — init() override karo taaki QR specific area mein fit ho
class QRcode_Custom : public QRcode_eSPI {
public:
  QRcode_Custom(TFT_eSPI* d) : QRcode_eSPI(d) {}

  // x, y = top-left of QR area, w, h = available pixels
  void initArea(int x, int y, int w, int h) {
    // Parent init() se inherited members set honge
    // Phir hum override karte hain
    init();  // sets multiply, offsetsX, offsetsY based on full screen

    // Override: recalculate for our specific area
    int minDim = (w < h) ? w : h;
    multiply = minDim / WD;          // WD = module count (21 for version 1)
    offsetsX = x + (w - WD * multiply) / 2;
    offsetsY = y + (h - WD * multiply) / 2;
  }
};

QRcode_Custom qrcode(&tft);

// ─────────────────────────────────────────────────────────────
//  COLORS (RGB565)
// ─────────────────────────────────────────────────────────────
#define C_BG         0x0841
#define C_PANEL      0x1082
#define C_ACCENT     0x07FF   // Cyan
#define C_GREEN      0x07E0
#define C_RED        0xF800
#define C_ORANGE     0xFD20
#define C_YELLOW     0xFFE0
#define C_WHITE      0xFFFF
#define C_GRAY       0x7BEF
#define C_DARKGRAY   0x39E7
#define C_TOPBAR     0x0C4A
#define C_CARD_ON    0x0B4D
#define C_CARD_OFF   0x18C3
#define C_MAGENTA    0xF81F

// ─────────────────────────────────────────────────────────────
//  CONFIG
// ─────────────────────────────────────────────────────────────
#define ENABLE_EEPROM            true
#define USE_LATCHED_SWITCH       true
#define EEPROM_SIZE              20
#define CURRENT_FIRMWARE_VERSION "v1.0.0"
#define GITHUB_USER              "MatrixCoder0101"
#define GITHUB_REPO              "ESPHomeFirmware"
#define EEPROM_VERSION_ADDR      8
#define DHTPIN                   35
#define DHTTYPE                  DHT11

// ─────────────────────────────────────────────────────────────
//  GPIO
// ─────────────────────────────────────────────────────────────
static uint8_t RelayPin1  = 32;
static uint8_t RelayPin2  = 33;
static uint8_t RelayPin3  = 25;
static uint8_t RelayPin4  = 26;

static uint8_t SwitchPin1 = 13;
static uint8_t SwitchPin2 = 12;
static uint8_t SwitchPin3 = 14;
static uint8_t SwitchPin4 = 27;

static uint8_t wifiLed    = 16;  // GPIO 2 = TFT DC, isliye 16
static uint8_t gpio_reset = 0;   // BOOT button

// ─────────────────────────────────────────────────────────────
//  DEVICE NAMES — original se same
// ─────────────────────────────────────────────────────────────
const char *service_name = "PROV_12345";
const char *pop          = "1234567";

char deviceName_1[] = "Light";
char deviceName_2[] = "Light 2";
char deviceName_3[] = "Fan";
char deviceName_4[] = "TV";

// ─────────────────────────────────────────────────────────────
//  STATE
// ─────────────────────────────────────────────────────────────
bool toggleState_1 = LOW, toggleState_2 = LOW;
bool toggleState_3 = LOW, toggleState_4 = LOW;

float currentTemp     = 0;
float currentHumidity = 0;
bool  sensorError     = false;
bool  wifiConnected   = false;
String wifiIP         = "---";
String currentTime    = "--:--";
String currentDate    = "-- --- ----";
String currentDay     = "---";

unsigned long lastSensorRead    = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastOTACheckTime  = 0;
bool initialCheckDone           = false;
TaskHandle_t wifiLedTaskHandle  = NULL;

const unsigned long sensorInterval   = 5000;
const unsigned long displayInterval  = 1000;
const unsigned long otaCheckInterval = 5UL * 60 * 1000;

DHT dht(DHTPIN, DHTTYPE);

// RainMaker devices — original se same
static TemperatureSensor tempSensor("Temperature");
static TemperatureSensor humiditySensor("Humidity");
static Switch my_switch1(deviceName_1, &RelayPin1);
static Switch my_switch2(deviceName_2, &RelayPin2);
static Switch my_switch3(deviceName_3, &RelayPin3);
static Switch my_switch4(deviceName_4, &RelayPin4);

// AceButton
ButtonConfig config1, config2, config3, config4;
AceButton button1(&config1), button2(&config2);
AceButton button3(&config3), button4(&config4);

// OTA
String otaStatusMsg = "Checking...";

// ─────────────────────────────────────────────────────────────
//  SCREEN STATE
// ─────────────────────────────────────────────────────────────
// dashboardActive = true  → dashboard visible
// qrScreenActive  = true  → QR code screen visible (provisional mode)
// both false              → terminal visible
bool dashboardActive = false;
bool qrScreenActive  = false;

// ─────────────────────────────────────────────────────────────
//  QR CODE SCREEN
//  RainMaker BLE provisioning QR — full screen display
//  User RainMaker app se scan kare → PROV_CRED_RECV fire hoga
//  Tab wapas terminal pe aayenge
// ─────────────────────────────────────────────────────────────

// RainMaker BLE QR content format (same as printQR internally)
// {"ver":"v1","name":"PROV_12345","pop":"1234567","transport":"ble"}
void buildQRContent(char* buf, int bufLen) {
  // Exact same format as RainMaker's printQR() function
  // This is what ESP RainMaker app scans to provision
  snprintf(buf, bufLen,
    "{\"ver\":\"v1\",\"name\":\"%s\",\"pop\":\"%s\",\"transport\":\"ble\"}",
    service_name, pop);
  Serial.printf("[QR] Content: %s\n", buf);
}

// Draw QR code screen
// QRcode_eSPI library use kar raha hai (yoprogramo)
// Library internally: multiply = min(w,h)/WD → 240/21 = 11 → QR=231px (full screen)
// Fix: qrcode.create() ke baad header+footer upar draw karo
// QR ke top/bottom thoda cut hoga lekin middle scannable rahega (ECC handles it)
void drawQRScreen() {
  qrScreenActive = true;
  tft.fillScreen(TFT_WHITE);

  // QR area: x=0, y=22, w=320, h=183 (between header and footer)
  // initArea sets multiply + offsets so QR fits perfectly
  qrcode.initArea(0, 22, 320, 183);

  // Generate and draw QR — now positioned in our area
  char qrContent[128];
  buildQRContent(qrContent, sizeof(qrContent));
  qrcode.create(String(qrContent));

  // Header
  tft.fillRect(0, 0, 320, 22, C_TOPBAR);
  tft.drawFastHLine(0, 22, 320, C_ACCENT);
  tft.setTextColor(C_ACCENT, C_TOPBAR); tft.setTextSize(1);
  tft.setCursor(8, 7); tft.print("ESPHome");
  tft.setTextColor(C_WHITE, C_TOPBAR);
  tft.print(" v2.0  |  WiFi Setup via RainMaker");

  // Footer
  tft.fillRect(0, 205, 320, 35, 0x0C18);
  tft.drawFastHLine(0, 205, 320, C_MAGENTA);
  tft.setTextColor(C_GRAY, 0x0C18); tft.setTextSize(1);
  tft.setCursor(6, 209); tft.print("Device:");
  tft.setTextColor(C_GREEN, 0x0C18); tft.print(" "); tft.print(service_name);
  tft.setTextColor(C_GRAY, 0x0C18);
  tft.setCursor(6, 221); tft.print("POP:");
  tft.setTextColor(C_YELLOW, 0x0C18); tft.print(" "); tft.print(pop);
  tft.setTextColor(C_MAGENTA, 0x0C18);
  tft.setCursor(180, 209); tft.print("Waiting for scan...");
  tft.setTextColor(C_DARKGRAY, 0x0C18);
  tft.setCursor(180, 221); tft.print("ESP RainMaker app");
}

// ── Animate "Waiting for scan..." on QR screen ─────────────
unsigned long lastQRAnimMs = 0;
int qrAnimStep = 0;
void qrScreenTick() {
  if (!qrScreenActive) return;
  if (millis() - lastQRAnimMs < 600) return;
  lastQRAnimMs = millis();

  const char* msgs[] = {
    "Waiting for scan.  ",
    "Waiting for scan.. ",
    "Waiting for scan..."
  };
  tft.fillRect(198, 206, 122, 12, 0x0C18);
  tft.setTextColor(C_MAGENTA, 0x0C18); tft.setTextSize(1);
  tft.setCursor(200, 208); tft.print(msgs[qrAnimStep % 3]);
  qrAnimStep++;
}

// ── Transition: QR screen → back to terminal ───────────────
void qrToTerminal() {
  qrScreenActive = false;
  // Redraw terminal
  tft.fillScreen(TFT_BLACK);
  termDrawHeader();
  termRedraw();
  termDrawProvBox();
}

// ─────────────────────────────────────────────────────────────
//  TERMINAL SYSTEM
// ─────────────────────────────────────────────────────────────
#define TERM_LINE_H   13
#define TERM_TOP_Y    19
#define TERM_AREA_H   188   // 19..207
#define TERM_BADGE_W  50
#define TERM_MAX      32

struct TermLine { char badge[8]; char msg[60]; uint16_t bc; uint16_t mc; };
TermLine  tLines[TERM_MAX];
int       tCount     = 0;
int       tScrollTop = 0;

// Visible line count
int termVisLines() { return TERM_AREA_H / TERM_LINE_H; }

void termDrawHeader() {
  tft.fillRect(0, 0, 320, 18, C_TOPBAR);
  tft.drawFastHLine(0, 18, 320, C_ACCENT);
  tft.setTextColor(C_ACCENT, C_TOPBAR); tft.setTextSize(1);
  tft.setCursor(6, 5); tft.print("ESPHome");
  tft.setTextColor(C_WHITE, C_TOPBAR);
  tft.print(" v2.0  |  Smart Dashboard");
  tft.setTextColor(C_DARKGRAY, C_TOPBAR);
  tft.setCursor(262, 5); tft.print(CURRENT_FIRMWARE_VERSION);
}

void termDrawFooter() {
  tft.fillRect(0, 208, 320, 32, 0x0821);
  tft.drawFastHLine(0, 208, 320, C_DARKGRAY);
  tft.setTextColor(C_DARKGRAY, 0x0821); tft.setTextSize(1);
  tft.setCursor(6, 213);
  tft.print("ESP32 | RAM:");
  tft.print(ESP.getFreeHeap() / 1024); tft.print("kB");
  unsigned long s = millis() / 1000;
  char ub[20]; snprintf(ub, sizeof(ub), "Up:%02lu:%02lu:%02lu", s/3600, (s%3600)/60, s%60);
  tft.setCursor(190, 213); tft.print(ub);
}

void termDrawProvBox() {
  tft.fillRect(0, 222, 320, 18, 0x0C10);
  tft.drawFastHLine(0, 222, 320, C_MAGENTA);
  tft.setTextColor(C_MAGENTA, 0x0C10); tft.setTextSize(1);
  tft.setCursor(6, 226); tft.print("BLE:");
  tft.setTextColor(C_WHITE, 0x0C10);
  tft.print(" App=ESP RainMaker  Device=");
  tft.setTextColor(C_GREEN, 0x0C10); tft.print(service_name);
  tft.setTextColor(C_YELLOW, 0x0C10); tft.print("  POP=");
  tft.setTextColor(C_GREEN, 0x0C10); tft.print(pop);
}

void termRedraw() {
  int vis = termVisLines();
  for (int i = 0; i < vis; i++) {
    int idx = tScrollTop + i;
    int y   = TERM_TOP_Y + i * TERM_LINE_H;
    tft.fillRect(0, y, 320, TERM_LINE_H, TFT_BLACK);
    if (idx < tCount) {
      tft.setTextColor(tLines[idx].bc, TFT_BLACK);
      tft.setTextSize(1); tft.setCursor(3, y + 2);
      tft.print(tLines[idx].badge);
      tft.setTextColor(tLines[idx].mc, TFT_BLACK);
      tft.setCursor(TERM_BADGE_W + 2, y + 2);
      tft.print(tLines[idx].msg);
    }
  }
}

// Add line and auto-scroll — NO delay, called from event handler safely
void termAdd(const char* badge, uint16_t bc, const char* msg, uint16_t mc) {
  if (dashboardActive) return;  // dashboard pe ho toh kuch mat karo

  // Store line always
  if (tCount < TERM_MAX) {
    strncpy(tLines[tCount].badge, badge, 7);
    strncpy(tLines[tCount].msg,   msg,   59);
    tLines[tCount].bc = bc; tLines[tCount].mc = mc;
    tCount++;
  } else {
    for (int i = 0; i < TERM_MAX - 1; i++) tLines[i] = tLines[i+1];
    strncpy(tLines[TERM_MAX-1].badge, badge, 7);
    strncpy(tLines[TERM_MAX-1].msg,   msg,   59);
    tLines[TERM_MAX-1].bc = bc; tLines[TERM_MAX-1].mc = mc;
  }
  int vis = termVisLines();
  tScrollTop = (tCount > vis) ? tCount - vis : 0;

  // QR screen active hai toh draw mat karo — QR wipe ho jayega
  if (!qrScreenActive) termRedraw();
}

// Update LAST line — disconnect/retry pe reuse karo, spam mat karo
void termUpdateLast(const char* badge, uint16_t bc, const char* msg, uint16_t mc) {
  if (dashboardActive || tCount == 0) return;
  int idx = tCount - 1;
  strncpy(tLines[idx].badge, badge, 7);
  strncpy(tLines[idx].msg,   msg,   59);
  tLines[idx].bc = bc; tLines[idx].mc = mc;
  // QR screen active hai toh draw mat karo
  if (!qrScreenActive) termRedraw();
}

// Convenience wrappers
void tOK(const char* m)   { termAdd("[ OK ]", C_GREEN,   m, C_WHITE);   }
void tWait(const char* m) { termAdd("[ .. ]", C_YELLOW,  m, C_YELLOW);  }
void tFail(const char* m) { termAdd("[FAIL]", C_RED,     m, C_RED);     }
void tInfo(const char* m) { termAdd("[INFO]", C_GRAY,    m, C_GRAY);    }
void tWifi(const char* m) { termAdd("[WIFI]", C_ACCENT,  m, C_ACCENT);  }
void tBLE(const char* m)  { termAdd("[BLE ]", C_MAGENTA, m, C_MAGENTA); }
void tOTA(const char* m)  { termAdd("[OTA ]", C_ORANGE,  m, C_ORANGE);  }
void tSys(const char* m)  { termAdd("[SYS ]", 0x867F,    m, C_WHITE);   }

// ─────────────────────────────────────────────────────────────
//  TERMINAL WIFI ANIMATION — loop() calls this
// ─────────────────────────────────────────────────────────────
unsigned long lastDotMs  = 0;
int           dotState   = 0;
bool          doingDots  = false;   // true = animate last line

void termStartDots() {
  doingDots  = true;
  dotState   = 0;
  lastDotMs  = 0;
}

void termStopDots() {
  doingDots = false;
}

void termTickDots() {
  if (!doingDots || dashboardActive) return;
  if (millis() - lastDotMs < 400) return;
  lastDotMs = millis();
  const char* msgs[] = {
    "Connecting to WiFi .  ",
    "Connecting to WiFi .. ",
    "Connecting to WiFi ..."
  };
  if (tCount > 0) {
    strncpy(tLines[tCount-1].msg, msgs[dotState % 3], 59);
    tLines[tCount-1].bc = C_YELLOW;
    tLines[tCount-1].mc = C_YELLOW;
    dotState++;
    // Redraw only last visible line
    int vis     = termVisLines();
    int lastVis = tCount - 1 - tScrollTop;
    if (lastVis >= 0 && lastVis < vis) {
      int y = TERM_TOP_Y + lastVis * TERM_LINE_H;
      tft.fillRect(0, y, 320, TERM_LINE_H, TFT_BLACK);
      tft.setTextColor(C_YELLOW, TFT_BLACK); tft.setTextSize(1);
      tft.setCursor(3, y+2); tft.print("[WIFI]");
      tft.setCursor(TERM_BADGE_W+2, y+2); tft.print(tLines[tCount-1].msg);
    }
  }
  termDrawFooter();
}

// ─────────────────────────────────────────────────────────────
//  LAUNCH ANIMATION: terminal → dashboard
// ─────────────────────────────────────────────────────────────
void termLaunch() {
  termStopDots();
  termAdd("[>>>>]", C_ACCENT, "All systems ready! Launching Dashboard...", C_ACCENT);

  // Progress bar
  tft.fillRect(0, 208, 320, 32, TFT_BLACK);
  tft.drawFastHLine(0, 208, 320, C_ACCENT);
  tft.setTextColor(C_ACCENT, TFT_BLACK); tft.setTextSize(1);
  tft.setCursor(10, 211); tft.print("Loading Dashboard...");

  int bx=10, by=220, bw=300, bh=10;
  tft.fillRect(bx, by, bw, bh, C_DARKGRAY);
  tft.drawRect(bx-1, by-1, bw+2, bh+2, C_ACCENT);
  for (int p = 0; p <= 100; p += 4) {
    int f = bw * p / 100;
    for (int i = 0; i < f; i++) {
      tft.drawFastVLine(bx+i, by, bh, i < f/2 ? C_ACCENT : C_GREEN);
    }
    char pb[6]; snprintf(pb, sizeof(pb), "%d%%", p);
    tft.setTextColor(C_BG, C_BG);
    tft.setCursor(bx+bw/2-8, by+1); tft.setTextColor(C_BG);
    tft.print(pb);
    delay(10);
  }
  delay(300);
  tft.fillScreen(C_GREEN); delay(50);
  tft.fillScreen(TFT_BLACK); delay(50);
}

// ─────────────────────────────────────────────────────────────
//  EEPROM
// ─────────────────────────────────────────────────────────────
void writeEEPROM(int addr, bool state) {
  if (ENABLE_EEPROM && EEPROM.read(addr) != state) {
    EEPROM.write(addr, state); EEPROM.commit();
  }
}
bool readEEPROM(int addr) { return ENABLE_EEPROM ? EEPROM.read(addr) : false; }

String readStoredVersion() {
  String v = "";
  for (int i = EEPROM_VERSION_ADDR; i < EEPROM_VERSION_ADDR+10; i++) {
    char c = EEPROM.read(i);
    if (c == '\0' || c == 0xFF) break;
    v += c;
  }
  return v;
}
void storeFirmwareVersion(const char* v) {
  for (int i = 0; i < 10; i++) {
    EEPROM.write(EEPROM_VERSION_ADDR+i, v[i] ? v[i] : '\0');
  }
  EEPROM.commit();
}

// ─────────────────────────────────────────────────────────────
//  RELAY
// ─────────────────────────────────────────────────────────────
void setRelay(uint8_t pin, int addr, bool state) {
  digitalWrite(pin, state);
  if (ENABLE_EEPROM) writeEEPROM(addr, state);
}

// ─────────────────────────────────────────────────────────────
//  WIFI LED TASK
// ─────────────────────────────────────────────────────────────
void stopWiFiLedTask() {
  if (wifiLedTaskHandle != NULL) {
    vTaskDelete(wifiLedTaskHandle);
    wifiLedTaskHandle = NULL;
  }
}

// ─────────────────────────────────────────────────────────────
//  TIME
// ─────────────────────────────────────────────────────────────
void syncTime() { configTime(19800, 0, "pool.ntp.org", "time.nist.gov"); }

void updateTime() {
  struct tm ti;
  if (!getLocalTime(&ti, 100)) return;
  char tb[8], db[16], dy[12];
  strftime(tb, sizeof(tb), "%H:%M",    &ti);
  strftime(db, sizeof(db), "%d %b %Y", &ti);
  strftime(dy, sizeof(dy), "%A",       &ti);
  currentTime = tb; currentDate = db; currentDay = dy;
}

// ─────────────────────────────────────────────────────────────
//  OTA
// ─────────────────────────────────────────────────────────────
String humanReadableSize(size_t b) {
  char buf[20];
  if (b < 1024)        sprintf(buf, "%u B",   (unsigned)b);
  else if (b < 1048576) sprintf(buf, "%.1f kB", b/1024.0);
  else                  sprintf(buf, "%.2f MB", b/1048576.0);
  return String(buf);
}

String getLatestBinUrl(const char* apiUrl, String& latestVersion) {
  HTTPClient http; http.begin(apiUrl);
  int code = http.GET();
  if (code != HTTP_CODE_OK) { http.end(); return ""; }
  StaticJsonDocument<256> filter;
  filter["tag_name"] = true;
  filter["assets"][0]["name"] = true;
  filter["assets"][0]["browser_download_url"] = true;
  DynamicJsonDocument doc(4096);
  DeserializationError err = deserializeJson(doc, http.getStream(), DeserializationOption::Filter(filter));
  http.end();
  if (err) return "";
  latestVersion = doc["tag_name"].as<String>();
  String url = "";
  for (JsonObject a : doc["assets"].as<JsonArray>()) {
    if (String((const char*)a["name"]).endsWith(".ino.bin")) {
      url = String((const char*)a["browser_download_url"]); break;
    }
  }
  return url;
}

bool startOTAUpdate(WiFiClient* client, int contentLength, const String& latestVer) {
  if (!Update.begin(contentLength)) return false;
  size_t written = 0; int lastPct = -1;
  unsigned long lastData = millis();
  while (written < (size_t)contentLength) {
    if (client->available()) {
      uint8_t buf[256];
      size_t len = client->read(buf, sizeof(buf));
      if (len > 0) {
        Update.write(buf, len); written += len;
        int pct = written * 100 / contentLength;
        if (pct != lastPct) {
          lastPct = pct;
          // Show OTA overlay on dashboard or terminal
          if (dashboardActive) {
            // OTA overlay — cyan/accent color, correct title, no duplicate v
            tft.fillRoundRect(20, 75, 280, 90, 6, 0x0820);
            tft.drawRoundRect(20, 75, 280, 90, 6, C_ACCENT);

            // Title
            tft.setTextColor(C_ACCENT, 0x0820); tft.setTextSize(1);
            tft.setCursor(30, 87);
            tft.print("Software Update (OTA)");

            // Version — latestVer already has "v" prefix (e.g. v1.2.0)
            tft.setTextColor(C_GREEN, 0x0820);
            tft.setCursor(30, 100);
            tft.print("New version: "); tft.print(latestVer);

            // Size progress
            char ps[40];
            snprintf(ps, sizeof(ps), "%s  /  %s",
              humanReadableSize(written).c_str(),
              humanReadableSize(contentLength).c_str());
            tft.setTextColor(C_GRAY, 0x0820);
            tft.setCursor(30, 113); tft.print(ps);

            // Progress bar
            int bx=30, by=126, bw=260, bh=14;
            tft.fillRect(bx, by, bw, bh, C_DARKGRAY);
            tft.drawRect(bx-1, by-1, bw+2, bh+2, C_ACCENT);
            int f = bw*pct/100;
            for (int i=0;i<f;i++) {
              uint16_t c = (i < f/2) ? C_ACCENT : C_GREEN;
              tft.drawFastVLine(bx+i, by, bh, c);
            }
            char pb[8]; snprintf(pb, sizeof(pb), "%d%%", pct);
            tft.setTextColor(C_BG, C_BG);
            tft.setCursor(bx+bw/2-8, by+3);
            tft.setTextColor(0x0820); tft.print(pb);
          } else {
            char om[48]; snprintf(om, sizeof(om), "Downloading %d%%  %s",
              pct, humanReadableSize(written).c_str());
            if (lastPct <= 1) tOTA(om);
            else termUpdateLast("[OTA ]", C_ORANGE, om, C_ORANGE);
          }
        }
        lastData = millis();
      }
    }
    if (millis() - lastData > 30000) { Update.abort(); return false; }
    yield();
  }
  if (!Update.end() || !Update.isFinished()) return false;
  storeFirmwareVersion(latestVer.c_str());
  return true;
}

void checkForOTAUpdate() {
  if (WiFi.status() != WL_CONNECTED) {
    otaStatusMsg = "No WiFi";
    if (dashboardActive) { /* refresh bottom bar */ }
    return;
  }
  otaStatusMsg = "Checking...";
  String latestVer;
  String apiURL = "https://api.github.com/repos/" GITHUB_USER "/" GITHUB_REPO "/releases/latest";
  String binURL = getLatestBinUrl(apiURL.c_str(), latestVer);
  if (binURL == "") { otaStatusMsg = "No release"; return; }
  String stored = readStoredVersion();
  if (latestVer == CURRENT_FIRMWARE_VERSION || latestVer == stored) {
    otaStatusMsg = "Up to date " + latestVer;
    return;
  }
  otaStatusMsg = "Updating " + latestVer;
  tOTA(("New version: " + latestVer + " — downloading...").c_str());
  HTTPClient http; http.begin(binURL);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  if (http.GET() != HTTP_CODE_OK) { otaStatusMsg = "DL failed"; http.end(); return; }
  int cl = http.getSize();
  WiFiClient* stream = http.getStreamPtr();
  if (startOTAUpdate(stream, cl, latestVer)) {
    otaStatusMsg = "Done! Rebooting...";
    delay(1500); ESP.restart();
  } else {
    otaStatusMsg = "OTA failed";
  }
  http.end();
}

// ─────────────────────────────────────────────────────────────
//  DASHBOARD DRAW FUNCTIONS
// ─────────────────────────────────────────────────────────────
void drawWiFiIcon(int x, int y, bool on) {
  uint16_t c = on ? C_GREEN : C_RED;
  if (on) {
    tft.fillRect(x, y+8, 3, 4, c); tft.fillRect(x+4, y+5, 3, 7, c);
    tft.fillRect(x+8, y+2, 3, 10, c);
  } else {
    tft.drawLine(x, y+2, x+10, y+11, c); tft.drawLine(x+10, y+2, x, y+11, c);
  }
}

void drawBulbIcon(int x, int y, bool on) {
  uint16_t c = on ? C_YELLOW : C_DARKGRAY;
  tft.drawCircle(x+7, y+6, 6, c);
  tft.fillCircle(x+7, y+6, 4, on ? C_YELLOW : C_PANEL);
  tft.drawFastHLine(x+4, y+13, 6, c); tft.drawFastHLine(x+5, y+15, 4, c);
}
void drawFanIcon(int x, int y, bool on) {
  uint16_t c = on ? C_ACCENT : C_DARKGRAY;
  tft.drawCircle(x+7, y+7, 7, c);
  tft.drawLine(x+7, y, x+7, y+14, c); tft.drawLine(x, y+7, x+14, y+7, c);
  tft.drawLine(x+2, y+2, x+12, y+12, c); tft.drawLine(x+12, y+2, x+2, y+12, c);
  tft.fillCircle(x+7, y+7, 2, c);
}
void drawTVIcon(int x, int y, bool on) {
  uint16_t c = on ? C_GREEN : C_DARKGRAY;
  tft.drawRect(x, y+2, 16, 11, c);
  if (on) tft.fillRect(x+1, y+3, 14, 9, 0x0318);
  tft.drawLine(x+4, y+14, x+5, y+13, c); tft.drawLine(x+12, y+14, x+11, y+13, c);
  tft.drawFastHLine(x+4, y+14, 9, c);
}
void drawTempIcon(int x, int y) {
  tft.drawCircle(x+2, y, 3, C_ORANGE); tft.drawFastVLine(x+2, y+3, 10, C_ORANGE);
  tft.fillCircle(x+2, y+13, 4, C_ORANGE);
}
void drawHumidIcon(int x, int y) {
  tft.fillTriangle(x+5, y, x, y+8, x+10, y+8, C_ACCENT);
  tft.fillCircle(x+5, y+10, 5, C_ACCENT);
}

// Top bar
void drawTopBar() {
  tft.fillRect(0, 0, 320, 22, C_TOPBAR);
  tft.drawFastHLine(0, 22, 320, C_ACCENT);
  drawWiFiIcon(6, 5, wifiConnected);
  tft.setTextColor(wifiConnected ? C_GREEN : C_RED, C_TOPBAR);
  tft.setTextSize(1); tft.setCursor(22, 7);
  tft.print(wifiConnected ? wifiIP : "No WiFi");
  tft.setTextColor(C_WHITE, C_TOPBAR); tft.setCursor(220, 7);
  tft.print(currentTime);
  tft.setTextColor(C_DARKGRAY, C_TOPBAR); tft.setCursor(270, 7);
  tft.print(CURRENT_FIRMWARE_VERSION);
}
void drawDateBar() {
  tft.fillRect(0, 23, 320, 14, C_BG);
  tft.drawFastHLine(0, 36, 320, C_DARKGRAY);
  tft.setTextColor(C_GRAY, C_BG); tft.setTextSize(1); tft.setCursor(8, 27);
  tft.print(currentDay); tft.print(",  "); tft.print(currentDate);
}

// Switch cards (2x2 grid)
struct CardPos { int x, y, w, h; };
CardPos cards[4] = {{4,38,150,55},{162,38,154,55},{4,97,150,55},{162,97,154,55}};

void drawCard(int idx, const char* name, bool state) {
  CardPos& c = cards[idx];
  uint16_t bg  = state ? C_CARD_ON  : C_CARD_OFF;
  uint16_t brd = state ? C_ACCENT   : C_DARKGRAY;
  uint16_t tc  = state ? C_WHITE    : C_GRAY;
  tft.fillRoundRect(c.x, c.y, c.w, c.h, 6, bg);
  tft.drawRoundRect(c.x, c.y, c.w, c.h, 6, brd);
  int ix = c.x+8, iy = c.y+8;
  switch(idx) {
    case 0: drawBulbIcon(ix, iy, state); break;
    case 1: drawBulbIcon(ix, iy, state); break;
    case 2: drawFanIcon(ix, iy, state);  break;
    case 3: drawTVIcon(ix, iy, state);   break;
  }
  tft.setTextColor(tc, bg); tft.setTextSize(1);
  tft.setCursor(c.x+30, c.y+10); tft.print(name);
  tft.fillRoundRect(c.x+c.w-36, c.y+8, 30, 14, 4, state ? C_GREEN : C_DARKGRAY);
  tft.setTextColor(TFT_BLACK, state ? C_GREEN : C_DARKGRAY);
  tft.setCursor(c.x+c.w-36+(state?6:5), c.y+11);
  tft.print(state ? "ON" : "OFF");
  const uint8_t pins[] = {RelayPin1,RelayPin2,RelayPin3,RelayPin4};
  tft.setTextColor(C_DARKGRAY, bg); tft.setCursor(c.x+8, c.y+c.h-13);
  tft.print("GPIO "); tft.print(pins[idx]);
}
void drawAllCards() {
  drawCard(0, deviceName_1, toggleState_1);
  drawCard(1, deviceName_2, toggleState_2);
  drawCard(2, deviceName_3, toggleState_3);
  drawCard(3, deviceName_4, toggleState_4);
}
void refreshCard(int i) {
  const char* n[] = {deviceName_1,deviceName_2,deviceName_3,deviceName_4};
  bool s[] = {toggleState_1,toggleState_2,toggleState_3,toggleState_4};
  drawCard(i, n[i], s[i]);
}

// Sensor bar
void drawSensorBar() {
  tft.fillRect(0, 156, 320, 30, C_PANEL);
  tft.drawFastHLine(0, 156, 320, C_DARKGRAY);
  tft.drawFastHLine(0, 186, 320, C_DARKGRAY);
  drawTempIcon(8, 161);
  tft.setTextSize(2); tft.setCursor(26, 162);
  if (sensorError) {
    tft.setTextColor(C_RED, C_PANEL); tft.setTextSize(1); tft.print("ERR");
  } else {
    tft.setTextColor(C_ORANGE, C_PANEL);
    char tb[10]; snprintf(tb, sizeof(tb), "%.1f", currentTemp);
    tft.print(tb); tft.setTextSize(1); tft.setTextColor(C_ORANGE, C_PANEL); tft.print(" C");
  }
  tft.drawFastVLine(160, 160, 22, C_DARKGRAY);
  drawHumidIcon(170, 160);
  tft.setTextSize(2); tft.setCursor(186, 162);
  if (sensorError) {
    tft.setTextColor(C_RED, C_PANEL); tft.setTextSize(1); tft.print("ERR");
  } else {
    tft.setTextColor(C_ACCENT, C_PANEL);
    char hb[10]; snprintf(hb, sizeof(hb), "%.1f", currentHumidity);
    tft.print(hb); tft.setTextSize(1); tft.setTextColor(C_ACCENT, C_PANEL); tft.print(" %");
  }
}

// Bottom bar
void drawBottomBar() {
  tft.fillRect(0, 187, 320, 53, C_BG);
  tft.drawFastHLine(0, 187, 320, C_ACCENT);
  tft.setTextColor(C_GRAY, C_BG); tft.setTextSize(1);
  tft.setCursor(8, 195); tft.print("OTA: ");
  tft.setTextColor(C_GREEN, C_BG); tft.print(otaStatusMsg);
  tft.setTextColor(C_DARKGRAY, C_BG); tft.setCursor(8, 210);
  tft.print("FW: " CURRENT_FIRMWARE_VERSION);
  tft.setCursor(120, 210); tft.print("RAM: ");
  tft.print(ESP.getFreeHeap()/1024); tft.print(" kB");
  int on = (int)toggleState_1+(int)toggleState_2+(int)toggleState_3+(int)toggleState_4;
  tft.setCursor(8, 224); tft.setTextColor(C_GRAY, C_BG);
  tft.print("Active: ");
  tft.setTextColor(on > 0 ? C_GREEN : C_GRAY, C_BG); tft.print(on);
  tft.setTextColor(C_GRAY, C_BG); tft.print("/4");
  unsigned long s = millis()/1000;
  char ub[24]; snprintf(ub, sizeof(ub), "Up:%02lud %02lu:%02lu:%02lu",
    s/86400, (s%86400)/3600, (s%3600)/60, s%60);
  tft.setTextColor(C_DARKGRAY, C_BG); tft.setCursor(170, 224); tft.print(ub);
}

void drawDashboard() {
  tft.fillScreen(C_BG);
  drawTopBar(); drawDateBar(); drawAllCards();
  drawSensorBar(); drawBottomBar();
}

// Partial refresh helpers
void refreshTopBar()    { if (dashboardActive) drawTopBar(); }
void refreshSensorBar() { if (dashboardActive) drawSensorBar(); }
void refreshBottomBar() { if (dashboardActive) drawBottomBar(); }

// ─────────────────────────────────────────────────────────────
//  BUTTON HANDLER — original se same logic
// ─────────────────────────────────────────────────────────────
void buttonHandler(AceButton*, uint8_t eventType, uint8_t,
                   uint8_t relayPin, int eepromAddr,
                   Switch& sw, bool& state, int cardIdx) {
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
  Serial.printf("[BTN] Relay GPIO%d = %d\n", relayPin, state);
  if (dashboardActive) { refreshCard(cardIdx); refreshBottomBar(); }
}

// ─────────────────────────────────────────────────────────────
//  WRITE CALLBACK — original se same
// ─────────────────────────────────────────────────────────────
void write_callback(Device* device, Param* param,
                    const param_val_t val, void*, write_ctx_t*) {
  const char* dn = device->getDeviceName();
  const char* pn = param->getParamName();
  if (strcmp(pn, "Power") == 0) {
    bool ns = val.val.b;
    int  ci = -1;
    if      (strcmp(dn, deviceName_1)==0){setRelay(RelayPin1,0,ns);toggleState_1=ns;my_switch1.updateAndReportParam(pn,ns);ci=0;}
    else if (strcmp(dn, deviceName_2)==0){setRelay(RelayPin2,1,ns);toggleState_2=ns;my_switch2.updateAndReportParam(pn,ns);ci=1;}
    else if (strcmp(dn, deviceName_3)==0){setRelay(RelayPin3,2,ns);toggleState_3=ns;my_switch3.updateAndReportParam(pn,ns);ci=2;}
    else if (strcmp(dn, deviceName_4)==0){setRelay(RelayPin4,3,ns);toggleState_4=ns;my_switch4.updateAndReportParam(pn,ns);ci=3;}
    Serial.printf("[RM] %s = %d\n", dn, ns);
    if (dashboardActive && ci >= 0) { refreshCard(ci); refreshBottomBar(); }
  }
}

// ─────────────────────────────────────────────────────────────
//  SYSPROVEVENT — original ka exact flow, sirf display add kiya
//
//  KEY INSIGHT: WiFi.onEvent fires on RainMaker's internal thread
//  isliye yahan se sirf flags set karo aur simple display calls
//  karo — koi complex logic nahi, koi dashboardActive check nahi
//  terminal functions thread-safe hain (sirf tft write karte hain)
// ─────────────────────────────────────────────────────────────
void sysProvEvent(arduino_event_t* sys_event) {
  switch (sys_event->event_id) {

    case ARDUINO_EVENT_PROV_START:
      Serial.printf("[PROV] Started: %s\n", service_name);
      printQR(service_name, pop, "ble");  // Serial monitor ke liye bhi
      // QR screen show karo — terminal se switch
      drawQRScreen();
      stopWiFiLedTask();
      xTaskCreate([](void*){
        int s=0; for(;;){
          digitalWrite(wifiLed,(s==0||s==2)?HIGH:LOW);
          s=(s+1)%4; vTaskDelay(150/portTICK_PERIOD_MS);
        }
      }, "led_p", 1024, NULL, 1, &wifiLedTaskHandle);
      break;

    case ARDUINO_EVENT_PROV_CRED_RECV:
      Serial.println("[PROV] Credentials received");
      // QR screen se terminal pe wapas switch karo
      if (qrScreenActive) qrToTerminal();
      // Ab terminal pe status show karo
      tBLE("WiFi credentials received from RainMaker app!");
      tWifi("Connecting to WiFi ...");
      termStartDots();
      break;

    case ARDUINO_EVENT_PROV_CRED_FAIL:
      Serial.println("[PROV] Credentials failed");
      termStopDots();
      // Terminal pe error show karo, phir QR screen wapas
      if (!qrScreenActive) {
        tFail("WiFi credentials invalid — showing QR again");
        delay(1500);
        drawQRScreen();
      }
      break;

    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println("[WIFI] Connected");
      termStopDots();
      wifiConnected = true;
      wifiIP = WiFi.localIP().toString();
      syncTime();
      {
        char cm[48];
        snprintf(cm, sizeof(cm), "Connected!  IP: %s  RSSI: %ddBm",
                 wifiIP.c_str(), WiFi.RSSI());
        tWifi(cm);
        snprintf(cm, sizeof(cm), "SSID: %s", WiFi.SSID().c_str());
        tInfo(cm);
        tOK("NTP time sync started (IST UTC+5:30)");
      }
      stopWiFiLedTask();
      xTaskCreate([](void*){
        const int p[]={100,100,200,100,100,1500}; int idx=0; bool ls=LOW;
        for(;;){ ls=!ls; digitalWrite(wifiLed,ls);
          vTaskDelay(p[idx]/portTICK_PERIOD_MS); idx=(idx+1)%6; }
      }, "led_c", 1024, NULL, 1, &wifiLedTaskHandle);

      // Launch dashboard ONLY once
      if (!dashboardActive) {
        termLaunch();
        dashboardActive = true;
        drawDashboard();
      } else {
        refreshTopBar();
      }
      break;

    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("[WIFI] Disconnected");
      wifiConnected = false;
      wifiIP = "---";
      stopWiFiLedTask();
      xTaskCreate([](void*){
        bool ls=LOW; for(;;){ ls=!ls; digitalWrite(wifiLed,ls);
          vTaskDelay(500/portTICK_PERIOD_MS); }
      }, "led_d", 1024, NULL, 1, &wifiLedTaskHandle);

      if (dashboardActive) {
        // Dashboard pe hain — sirf top bar update
        refreshTopBar();
      } else if (qrScreenActive) {
        // QR screen dikha raha hai — kuch mat karo, QR wipe mat karo
        // RainMaker background mein retry karta rahega
        Serial.println("[WIFI] Disconnected during QR screen — ignoring");
      } else {
        // Terminal pe hain — looping rokne ke liye last line check karo
        termStopDots();
        // Agar last line already disconnect thi toh update karo, naya line mat add karo
        bool lastIsDisconn = (tCount > 0 &&
          strstr(tLines[tCount-1].msg, "disconnected") != NULL);
        if (lastIsDisconn) {
          termUpdateLast("[WIFI]", C_RED, "WiFi disconnected — retrying...", C_RED);
        } else {
          tFail("WiFi disconnected — RainMaker retrying...");
          tWifi("Connecting to WiFi ...");
          termStartDots();
        }
      }
      break;

    case ARDUINO_EVENT_PROV_END:
      Serial.println("[PROV] Ended");
      // Sirf log karo — QR screen band mat karo
      // PROV_CRED_RECV ya WIFI_STA_CONNECTED screen switch karega
      if (!qrScreenActive) {
        tBLE("Provisioning session ended");
      }
      break;

    default: break;
  }
}

// ─────────────────────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────────────────────
void setup() {
  // GPIO
  pinMode(wifiLed, OUTPUT); digitalWrite(wifiLed, LOW);
  Serial.begin(115200);

  // Display
  tft.init(); tft.setRotation(1); tft.fillScreen(TFT_BLACK);
  // qrcode.init() called via initArea() in drawQRScreen

  // Terminal init
  termDrawHeader();
  termDrawFooter();

  // EEPROM
  if (ENABLE_EEPROM) EEPROM.begin(EEPROM_SIZE);
  toggleState_1 = ENABLE_EEPROM ? readEEPROM(0) : LOW;
  toggleState_2 = ENABLE_EEPROM ? readEEPROM(1) : LOW;
  toggleState_3 = ENABLE_EEPROM ? readEEPROM(2) : LOW;
  toggleState_4 = ENABLE_EEPROM ? readEEPROM(3) : LOW;

  tSys("ESP32 booting...  " CURRENT_FIRMWARE_VERSION);
  tOK("TFT ILI9341 320x240 initialized");
  {
    char em[48];
    snprintf(em, sizeof(em), "EEPROM OK  Relays: %d %d %d %d",
             toggleState_1, toggleState_2, toggleState_3, toggleState_4);
    tOK(em);
  }

  // Relay GPIO
  pinMode(RelayPin1, OUTPUT); pinMode(RelayPin2, OUTPUT);
  pinMode(RelayPin3, OUTPUT); pinMode(RelayPin4, OUTPUT);
  setRelay(RelayPin1, 0, toggleState_1);
  setRelay(RelayPin2, 1, toggleState_2);
  setRelay(RelayPin3, 2, toggleState_3);
  setRelay(RelayPin4, 3, toggleState_4);
  {
    char gm[48];
    snprintf(gm, sizeof(gm), "Relay GPIO: %d %d %d %d  configured",
             RelayPin1, RelayPin2, RelayPin3, RelayPin4);
    tOK(gm);
  }

  // Switch GPIO
  pinMode(SwitchPin1, INPUT_PULLUP); pinMode(SwitchPin2, INPUT_PULLUP);
  pinMode(SwitchPin3, INPUT_PULLUP); pinMode(SwitchPin4, INPUT_PULLUP);
  pinMode(gpio_reset, INPUT);
  {
    char sm[48];
    snprintf(sm, sizeof(sm), "Switch GPIO: %d %d %d %d  INPUT_PULLUP",
             SwitchPin1, SwitchPin2, SwitchPin3, SwitchPin4);
    tOK(sm);
  }
  tInfo("Reset: Hold BOOT 3s=WiFiReset  10s=FactoryReset");

  // DHT
  dht.begin();
  {
    float h = dht.readHumidity(), t = dht.readTemperature();
    if (!isnan(t) && !isnan(h)) {
      char dm[40]; snprintf(dm, sizeof(dm), "DHT11 OK  %.1fC  %.1f%%", t, h);
      tOK(dm);
      currentTemp = t; currentHumidity = h;
    } else {
      tInfo("DHT11: no data yet — will read in loop");
    }
  }

  // AceButton
  config1.setEventHandler([](AceButton* b, uint8_t e, uint8_t s){
    buttonHandler(b,e,s,RelayPin1,0,my_switch1,toggleState_1,0);});
  config2.setEventHandler([](AceButton* b, uint8_t e, uint8_t s){
    buttonHandler(b,e,s,RelayPin2,1,my_switch2,toggleState_2,1);});
  config3.setEventHandler([](AceButton* b, uint8_t e, uint8_t s){
    buttonHandler(b,e,s,RelayPin3,2,my_switch3,toggleState_3,2);});
  config4.setEventHandler([](AceButton* b, uint8_t e, uint8_t s){
    buttonHandler(b,e,s,RelayPin4,3,my_switch4,toggleState_4,3);});
  button1.init(SwitchPin1); button2.init(SwitchPin2);
  button3.init(SwitchPin3); button4.init(SwitchPin4);
  tOK("AceButton handlers registered x4");

  // RainMaker — EXACT same as original
  Node my_node = RMaker.initNode("ESPHome");
  my_switch1.addCb(write_callback); my_switch2.addCb(write_callback);
  my_switch3.addCb(write_callback); my_switch4.addCb(write_callback);
  my_node.addDevice(tempSensor);    my_node.addDevice(humiditySensor);
  my_node.addDevice(my_switch1);    my_node.addDevice(my_switch2);
  my_node.addDevice(my_switch3);    my_node.addDevice(my_switch4);
  RMaker.enableOTA(OTA_USING_PARAMS);
  RMaker.enableTZService();
  RMaker.enableSchedule();
  tOK("RainMaker node: ESPHome  devices=6");

  RMaker.start();
  WiFi.onEvent(sysProvEvent);

  // beginProvision — EXACT same as original
  // Already provisioned = auto-connect (WIFI_STA_CONNECTED fires)
  // Not provisioned     = BLE starts (PROV_START fires)
  tWait("Starting WiFi / BLE provisioning...");
  WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE,
                          WIFI_PROV_SCHEME_HANDLER_FREE_BTDM,
                          WIFI_PROV_SECURITY_1, pop, service_name);

  // Original mein delay(2000) tha — same rakho
  delay(2000);

  my_switch1.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, toggleState_1);
  my_switch2.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, toggleState_2);
  my_switch3.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, toggleState_3);
  my_switch4.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, toggleState_4);

  // NOTE: Do NOT add "Connecting to WiFi" here manually.
  // After beginProvision():
  //   Already provisioned → WIFI_STA_CONNECTED fires → dashboard launches
  //   Factory reset / fresh → PROV_START fires → BLE instructions shown
  //   Credentials sent → PROV_CRED_RECV fires → connecting line added
  // Sab events se handle hoga, yahan kuch extra add karne ki zaroorat nahi.
  if (!dashboardActive) {
    tInfo("Starting provisioning — QR screen will appear...");
  }

  Serial.println("[SETUP] Complete");
}

// ─────────────────────────────────────────────────────────────
//  LOOP — original ka exact structure, sirf display adds kiye
// ─────────────────────────────────────────────────────────────
void loop() {
  // ── Reset button — live countdown feedback ─────────────
  if (digitalRead(gpio_reset) == LOW) {
    delay(100);
    unsigned long startTime = millis();
    int lastShownSec = -1;

    while (digitalRead(gpio_reset) == LOW) {
      int heldSec = (millis() - startTime) / 1000;

      if (heldSec != lastShownSec) {
        lastShownSec = heldSec;

        // Build status message
        char line1[40], line2[40];
        uint16_t barColor;

        if (heldSec < 3) {
          snprintf(line1, sizeof(line1), "Reset held: %ds", heldSec);
          snprintf(line2, sizeof(line2), "Hold 3s=WiFi Reset  10s=Factory");
          barColor = C_YELLOW;
        } else if (heldSec < 10) {
          snprintf(line1, sizeof(line1), "Release NOW for WiFi Reset (%ds/10s)", heldSec);
          snprintf(line2, sizeof(line2), "Keep holding for Factory Reset...");
          barColor = C_ORANGE;
        } else {
          snprintf(line1, sizeof(line1), "FACTORY RESET — release to confirm");
          snprintf(line2, sizeof(line2), "All data will be erased!");
          barColor = C_RED;
        }

        // Draw overlay box (works on both terminal and dashboard)
        tft.fillRoundRect(10, 85, 300, 70, 6, 0x1800);
        tft.drawRoundRect(10, 85, 300, 70, 6, barColor);

        tft.setTextColor(barColor, 0x1800); tft.setTextSize(1);
        tft.setCursor(20, 95); tft.print(line1);
        tft.setTextColor(C_GRAY, 0x1800);
        tft.setCursor(20, 108); tft.print(line2);

        // Progress bar (0-3s yellow, 3-10s orange, 10s red)
        int maxSec  = 10;
        int barW    = 270;
        int filled  = min((int)(heldSec * barW / maxSec), barW);
        tft.fillRect(20, 122, barW, 10, C_DARKGRAY);
        tft.drawRect(19, 121, barW+2, 12, barColor);
        for (int i = 0; i < filled; i++)
          tft.drawFastVLine(20+i, 122, 10, barColor);

        // Seconds label
        char secBuf[8]; snprintf(secBuf, sizeof(secBuf), "%ds", heldSec);
        tft.setTextColor(C_WHITE, 0x1800);
        tft.setCursor(20, 137); tft.print(secBuf);
        tft.setCursor(260, 137); tft.print("/ 10s");
      }
      delay(50);
    }

    int duration = millis() - startTime;

    // Clear overlay
    if (dashboardActive) {
      // Redraw the portion we covered
      drawTopBar(); drawDateBar(); drawAllCards();
    } else {
      termRedraw();
      termDrawProvBox();
    }

    if (duration > 10000) {
      Serial.println("[RESET] Factory reset triggered");
      tFail("FACTORY RESET — clearing all data...");
      delay(500);
      RMakerFactoryReset(2);
    } else if (duration > 3000) {
      Serial.println("[RESET] WiFi reset triggered");
      termAdd("[RST ]", C_ORANGE, "WiFi Reset — credentials cleared, re-provisioning...", C_ORANGE);
      delay(500);
      RMakerWiFiReset(2);
    }
    // < 3s = ignore, no action
  }

  // ── Screen animations (only before dashboard) ────────────
  if (!dashboardActive) {
    if (qrScreenActive) {
      qrScreenTick();        // QR "Waiting..." animation
    } else {
      termTickDots();        // Terminal WiFi dots animation
    }
  }

  // ── DHT sensor — original se same ───────────────────────
  if (millis() - lastSensorRead > sensorInterval) {
    float h = dht.readHumidity(), t = dht.readTemperature();
    if (!isnan(h) && !isnan(t)) {
      currentTemp = t; currentHumidity = h; sensorError = false;
      Serial.printf("[DHT] %.2fC  %.2f%%\n", t, h);
      tempSensor.updateAndReportParam("Temperature", t);
      humiditySensor.updateAndReportParam("Temperature", h);
    } else {
      sensorError = true;
      Serial.println("[DHT] Read failed");
    }
    if (dashboardActive) refreshSensorBar();
    lastSensorRead = millis();
  }

  // ── Display refresh (dashboard only) ────────────────────
  if (dashboardActive && millis() - lastDisplayUpdate > displayInterval) {
    if (wifiConnected) updateTime();
    refreshTopBar();
    drawDateBar();
    refreshBottomBar();
    lastDisplayUpdate = millis();
  }

  // ── OTA — original se same, dashboard ke baad ───────────
  if (!initialCheckDone && millis() > 15000 && dashboardActive) {
    checkForOTAUpdate();
    lastOTACheckTime = millis();
    initialCheckDone = true;
  }
  if (initialCheckDone && millis() - lastOTACheckTime >= otaCheckInterval) {
    checkForOTAUpdate();
    lastOTACheckTime = millis();
  }

  // ── Buttons — original se same ──────────────────────────
  button1.check(); button2.check();
  button3.check(); button4.check();
}
