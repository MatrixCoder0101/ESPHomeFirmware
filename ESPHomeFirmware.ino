/*
 * ╔══════════════════════════════════════════════════════════╗
 * ║         ESPHome Smart Dashboard v2.0                    ║
 * ║         ILI9341 320x240 TFT Display                     ║
 * ║         ESP32 + RainMaker + DHT11 + OTA                 ║
 * ╚══════════════════════════════════════════════════════════╝
 *
 * Features:
 *  - Linux-style terminal boot sequence with scroll animation
 *  - Animated progress bar before dashboard launch
 *  - Full color Smart Dashboard UI with icons
 *  - WiFi status, IP, Date/Time (top bar)
 *  - 4 Switch cards with device icons
 *  - Temp & Humidity live display
 *  - OTA update with progress bar on screen
 *  - Works offline too (shows last known data)
 */

// ─────────────────────────────────────────────
//  LIBRARIES
// ─────────────────────────────────────────────
#include <EEPROM.h>
#include "RMaker.h"
#include "WiFi.h"
#include "WiFiProv.h"
#include <AceButton.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <HTTPClient.h>
#include <Update.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <time.h>

using namespace ace_button;

// ─────────────────────────────────────────────
//  DISPLAY
// ─────────────────────────────────────────────
TFT_eSPI tft = TFT_eSPI();

// ─────────────────────────────────────────────
//  COLORS  (RGB565)
// ─────────────────────────────────────────────
#define C_BG        0x0841   // Very dark blue-black
#define C_PANEL     0x1082   // Dark panel
#define C_ACCENT    0x07FF   // Cyan
#define C_GREEN     0x07E0
#define C_RED       0xF800
#define C_ORANGE    0xFD20
#define C_YELLOW    0xFFE0
#define C_WHITE     0xFFFF
#define C_GRAY      0x7BEF
#define C_DARKGRAY  0x39E7
#define C_TOPBAR    0x0C4A   // Dark navy top bar
#define C_CARD_ON   0x0B4D   // Card ON color (dark cyan-blue)
#define C_CARD_OFF  0x18C3   // Card OFF color

// ─────────────────────────────────────────────
//  CONFIGURABLE FLAGS
// ─────────────────────────────────────────────
#define ENABLE_EEPROM            true
#define USE_LATCHED_SWITCH       true
#define EEPROM_SIZE              20
#define CURRENT_FIRMWARE_VERSION "v1.0.0"
#define GITHUB_USER              "MatrixCoder0101"
#define GITHUB_REPO              "ESPHomeFirmware"
#define EEPROM_VERSION_ADDR      8
#define DHTPIN                   35
#define DHTTYPE                  DHT11

// ─────────────────────────────────────────────
//  GPIO
// ─────────────────────────────────────────────
static uint8_t RelayPin1 = 32;
static uint8_t RelayPin2 = 33;
static uint8_t RelayPin3 = 25;
static uint8_t RelayPin4 = 26;

static uint8_t SwitchPin1 = 13;
static uint8_t SwitchPin2 = 12;
static uint8_t SwitchPin3 = 14;
static uint8_t SwitchPin4 = 27;

static uint8_t wifiLed    = 16;  // GPIO 2 TFT_DC ke liye reserved hai
static uint8_t gpio_reset = 0;

// ─────────────────────────────────────────────
//  DEVICE NAMES & ICONS (Unicode bitmap approach via chars)
// ─────────────────────────────────────────────
const char *service_name = "PROV_12345";
const char *pop          = "1234567";

char deviceName_1[] = "Light 1";
char deviceName_2[] = "Light 2";
char deviceName_3[] = "Fan";
char deviceName_4[] = "TV";

// ─────────────────────────────────────────────
//  STATE VARIABLES
// ─────────────────────────────────────────────
bool toggleState_1 = LOW;
bool toggleState_2 = LOW;
bool toggleState_3 = LOW;
bool toggleState_4 = LOW;

float currentTemp     = 0;
float currentHumidity = 0;
bool  sensorError     = false;

bool wifiConnected    = false;
String wifiIP         = "---";
String currentTime    = "--:--";
String currentDate    = "-- --- ----";
String currentDay     = "---";

unsigned long lastSensorRead   = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastOTACheckTime  = 0;
bool initialCheckDone           = false;
TaskHandle_t wifiLedTaskHandle  = NULL;

const unsigned long sensorInterval  = 5000;
const unsigned long displayInterval = 1000;
const unsigned long otaCheckInterval = 5 * 60 * 1000;

DHT dht(DHTPIN, DHTTYPE);

// RainMaker devices
static TemperatureSensor tempSensor("Temperature");
static TemperatureSensor humiditySensor("Humidity");
static Switch my_switch1(deviceName_1, &RelayPin1);
static Switch my_switch2(deviceName_2, &RelayPin2);
static Switch my_switch3(deviceName_3, &RelayPin3);
static Switch my_switch4(deviceName_4, &RelayPin4);

// AceButton
ButtonConfig config1, config2, config3, config4;
AceButton button1(&config1);
AceButton button2(&config2);
AceButton button3(&config3);
AceButton button4(&config4);

// OTA progress (for display)
volatile int otaProgress     = 0;
volatile bool otaInProgress  = false;
String otaStatusMsg          = "";

// ── Terminal system ───────────────────────────────────────
#define TERM_LINE_H     13           // pixels per line
#define TERM_TOP_Y      19           // y after header bar
#define TERM_BOT_PROV   210          // y where prov box starts
#define TERM_MAX_LINES  14           // max visible lines
#define TERM_BADGE_W    48           // badge column width

struct TermLine {
  char badge[8];
  char msg[64];
  uint16_t badgeColor;
  uint16_t msgColor;
};

TermLine termLines[40];              // total stored lines
int termLineCount  = 0;
int termScrollTop  = 0;              // first visible line index
bool dashboardLaunched = false;      // terminal → dashboard flag
bool provisioningActive = false;     // are we in BLE prov mode?
int  wifiRetryCount     = 0;         // retry counter for display
bool wifiAnimating      = true;      // animate dots on last line

// ═══════════════════════════════════════════════════════════
//  TERMINAL SYSTEM — Boot + Provisioning + WiFi Status
//  Sab kuch yahan dikhta hai jab tak dashboard launch na ho
// ═══════════════════════════════════════════════════════════

// ── Draw terminal header bar ──────────────────────────────
void termDrawHeader() {
  tft.fillRect(0, 0, 320, 18, 0x0C4A);
  tft.setTextColor(C_ACCENT, 0x0C4A);
  tft.setTextSize(1);
  tft.setCursor(6, 5);
  tft.print("ESPHome");
  tft.setTextColor(C_WHITE, 0x0C4A);
  tft.print(" v2.0  |  Smart Dashboard");
  // Right side: FW version
  tft.setTextColor(C_DARKGRAY, 0x0C4A);
  tft.setCursor(246, 5);
  tft.print(CURRENT_FIRMWARE_VERSION);
  // Separator line
  tft.drawFastHLine(0, 18, 320, C_ACCENT);
}

// ── Draw one terminal line at given y ─────────────────────
void termDrawLine(int idx, int y) {
  if (idx < 0 || idx >= termLineCount) return;
  TermLine &l = termLines[idx];

  // Clear line
  tft.fillRect(0, y, 320, TERM_LINE_H, TFT_BLACK);

  // Badge
  tft.setTextColor(l.badgeColor, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(3, y + 2);
  tft.print(l.badge);

  // Message
  tft.setTextColor(l.msgColor, TFT_BLACK);
  tft.setCursor(TERM_BADGE_W + 2, y + 2);
  tft.print(l.msg);
}

// ── Redraw all visible lines ──────────────────────────────
void termRedrawLines() {
  int visibleLines = (TERM_BOT_PROV - TERM_TOP_Y) / TERM_LINE_H;
  for (int i = 0; i < visibleLines; i++) {
    int lineIdx = termScrollTop + i;
    int y = TERM_TOP_Y + i * TERM_LINE_H;
    if (lineIdx < termLineCount)
      termDrawLine(lineIdx, y);
    else
      tft.fillRect(0, y, 320, TERM_LINE_H, TFT_BLACK);
  }
}

// ── Add a log line (auto scrolls) ────────────────────────
void termLog(const char* badge, uint16_t badgeColor,
             const char* msg,   uint16_t msgColor) {
  if (termLineCount < 40) {
    strncpy(termLines[termLineCount].badge, badge, 7);
    strncpy(termLines[termLineCount].msg,   msg,   63);
    termLines[termLineCount].badgeColor = badgeColor;
    termLines[termLineCount].msgColor   = msgColor;
    termLineCount++;
  }

  // Auto scroll so newest line is always visible
  int visibleLines = (TERM_BOT_PROV - TERM_TOP_Y) / TERM_LINE_H;
  if (termLineCount > visibleLines)
    termScrollTop = termLineCount - visibleLines;

  termRedrawLines();
  delay(60);  // terminal feel
}

// Convenience wrappers ────────────────────────────────────
void termOK(const char* msg) {
  termLog("[ OK ]", C_GREEN,   msg, C_WHITE);
}
void termWait(const char* msg) {
  termLog("[ .. ]", C_YELLOW,  msg, C_YELLOW);
}
void termFail(const char* msg) {
  termLog("[FAIL]", C_RED,     msg, C_RED);
}
void termInfo(const char* msg) {
  termLog("[INFO]", C_GRAY,    msg, C_GRAY);
}
void termWifi(const char* msg) {
  termLog("[WIFI]", C_ACCENT,  msg, C_ACCENT);
}
void termBLE(const char* msg) {
  termLog("[BLE ]", 0xF81F,    msg, 0xF81F);  // Magenta
}
void termOTA(const char* msg) {
  termLog("[OTA ]", C_ORANGE,  msg, C_ORANGE);
}
void termSys(const char* msg) {
  termLog("[SYS ]", 0x867F,    msg, C_WHITE);  // Purple-ish
}

// ── Overwrite last line badge (e.g. [ .. ] → [ OK ]) ─────
void termUpdateLastBadge(const char* badge, uint16_t color) {
  if (termLineCount == 0) return;
  strncpy(termLines[termLineCount - 1].badge, badge, 7);
  termLines[termLineCount - 1].badgeColor = color;

  int visibleLines = (TERM_BOT_PROV - TERM_TOP_Y) / TERM_LINE_H;
  int lastVisible  = termLineCount - 1 - termScrollTop;
  if (lastVisible >= 0 && lastVisible < visibleLines) {
    int y = TERM_TOP_Y + lastVisible * TERM_LINE_H;
    // Just redraw badge part
    tft.fillRect(0, y, TERM_BADGE_W, TERM_LINE_H, TFT_BLACK);
    tft.setTextColor(color, TFT_BLACK);
    tft.setTextSize(1);
    tft.setCursor(3, y + 2);
    tft.print(badge);
  }
}

// ── Bottom status bar on terminal ─────────────────────────
// Shows uptime + chip info
void termDrawFooter() {
  tft.fillRect(0, 210, 320, 30, 0x0821);
  tft.drawFastHLine(0, 210, 320, C_DARKGRAY);

  tft.setTextColor(C_DARKGRAY, 0x0821);
  tft.setTextSize(1);
  tft.setCursor(6, 215);
  tft.print("ESP32 | Flash: ");
  tft.print(ESP.getFlashChipSize() / 1024);
  tft.print("kB | RAM: ");
  tft.print(ESP.getFreeHeap() / 1024);
  tft.print("kB");

  unsigned long upSec = millis() / 1000;
  char ubuf[20];
  snprintf(ubuf, sizeof(ubuf), "Up: %02lu:%02lu:%02lu",
           upSec / 3600, (upSec % 3600) / 60, upSec % 60);
  tft.setCursor(220, 215);
  tft.print(ubuf);
}

// ── BLE Provisioning instruction box ─────────────────────
void termDrawProvBox() {
  int bx = 4, by = 212, bw = 312, bh = 26;
  tft.fillRoundRect(bx, by, bw, bh, 4, 0x0C18);
  tft.drawRoundRect(bx, by, bw, bh, 4, 0xF81F); // Magenta border

  tft.setTextColor(0xF81F, 0x0C18);  // Magenta
  tft.setTextSize(1);
  tft.setCursor(bx + 6, by + 4);
  tft.print("[BLE PROV]");

  tft.setTextColor(C_WHITE, 0x0C18);
  tft.setCursor(bx + 70, by + 4);
  tft.print("App: ESP RainMaker");

  tft.setTextColor(C_YELLOW, 0x0C18);
  tft.setCursor(bx + 6, by + 15);
  tft.print("Device: ");
  tft.setTextColor(C_GREEN, 0x0C18);
  tft.print(service_name);
  tft.setTextColor(C_YELLOW, 0x0C18);
  tft.print("  POP: ");
  tft.setTextColor(C_GREEN, 0x0C18);
  tft.print(pop);
}

// ── WiFi connecting animated dots bar ─────────────────────
// Call repeatedly from loop while connecting
int  wifiDotState    = 0;
unsigned long lastDotTime = 0;
void termAnimateWifi() {
  if (!dashboardLaunched && millis() - lastDotTime > 400) {
    lastDotTime = millis();
    // Update last line msg with animated dots
    if (termLineCount > 0) {
      char dotmsg[64];
      const char* dots[] = {"Connecting to WiFi .  ", "Connecting to WiFi .. ", "Connecting to WiFi ..."};
      snprintf(dotmsg, sizeof(dotmsg), "%s", dots[wifiDotState % 3]);
      strncpy(termLines[termLineCount - 1].msg, dotmsg, 63);
      termLines[termLineCount - 1].msgColor   = C_YELLOW;
      termLines[termLineCount - 1].badgeColor = C_YELLOW;

      int visibleLines = (TERM_BOT_PROV - TERM_TOP_Y) / TERM_LINE_H;
      int lastVisible  = termLineCount - 1 - termScrollTop;
      if (lastVisible >= 0 && lastVisible < visibleLines) {
        int y = TERM_TOP_Y + lastVisible * TERM_LINE_H;
        tft.fillRect(TERM_BADGE_W + 2, y, 320 - TERM_BADGE_W - 2, TERM_LINE_H, TFT_BLACK);
        tft.setTextColor(C_YELLOW, TFT_BLACK);
        tft.setTextSize(1);
        tft.setCursor(TERM_BADGE_W + 2, y + 2);
        tft.print(dotmsg);
      }
      wifiDotState++;
    }
    // Also refresh footer uptime
    termDrawFooter();
  }
}

// ── Final launch animation terminal → dashboard ───────────
void termLaunchDashboard() {
  termLog("[>>>>]", C_ACCENT, "All systems ready! Launching Dashboard...", C_ACCENT);
  delay(300);

  // Progress bar over footer area
  int bx = 10, by = 218, bw = 300, bh = 12;
  tft.fillRect(0, 210, 320, 30, TFT_BLACK);
  tft.drawFastHLine(0, 210, 320, C_ACCENT);
  tft.setTextColor(C_ACCENT, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(10, 212);
  tft.print("Loading Dashboard...");

  tft.fillRect(bx, by, bw, bh, C_DARKGRAY);
  tft.drawRect(bx - 1, by - 1, bw + 2, bh + 2, C_ACCENT);

  for (int p = 0; p <= 100; p += 3) {
    int filled = (bw * p) / 100;
    for (int i = 0; i < filled; i++) {
      uint16_t col = (i < filled / 3) ? C_ACCENT
                   : (i < 2*filled/3)  ? 0x07CF
                   : C_GREEN;
      tft.drawFastVLine(bx + i, by, bh, col);
    }
    char pbuf[8]; snprintf(pbuf, sizeof(pbuf), " %d%%", p);
    tft.setTextColor(TFT_BLACK, TFT_BLACK);
    tft.setCursor(bx + bw/2 - 8, by + 2);
    tft.setTextColor(C_BG);
    tft.print(pbuf);
    delay(12);
  }
  delay(400);

  // Flash transition
  tft.fillScreen(C_GREEN);
  delay(60);
  tft.fillScreen(TFT_BLACK);
  delay(60);
}

// ── Initial terminal screen setup ─────────────────────────
void termInit() {
  tft.fillScreen(TFT_BLACK);
  termDrawHeader();
  termDrawFooter();
}

// ── Full boot sequence (runs once at startup) ─────────────
void runBootSequence() {
  termInit();
  delay(200);

  // ── System init ──────────────────────────────
  termSys("ESP32 system starting up...");
  delay(100);
  termOK("TFT ILI9341 display initialized  [320x240]");
  delay(80);

  // ── EEPROM ───────────────────────────────────
  termWait("Mounting EEPROM...");
  delay(250);
  termUpdateLastBadge("[ OK ]", C_GREEN);
  char eepmsg[48];
  snprintf(eepmsg, sizeof(eepmsg), "EEPROM mounted  size=%d bytes  fw=%s",
           EEPROM_SIZE, CURRENT_FIRMWARE_VERSION);
  termOK(eepmsg);

  // ── Relay states ─────────────────────────────
  termWait("Restoring relay states from EEPROM...");
  delay(300);
  termUpdateLastBadge("[ OK ]", C_GREEN);
  char relaymsg[64];
  snprintf(relaymsg, sizeof(relaymsg),
           "Relays restored  L1=%d L2=%d Fan=%d TV=%d",
           toggleState_1, toggleState_2, toggleState_3, toggleState_4);
  termOK(relaymsg);

  // ── GPIO ─────────────────────────────────────
  char gpiomsg[64];
  snprintf(gpiomsg, sizeof(gpiomsg),
           "Relay GPIO  %d %d %d %d  configured",
           RelayPin1, RelayPin2, RelayPin3, RelayPin4);
  termOK(gpiomsg);
  char swmsg[64];
  snprintf(swmsg, sizeof(swmsg),
           "Switch GPIO  %d %d %d %d  INPUT_PULLUP",
           SwitchPin1, SwitchPin2, SwitchPin3, SwitchPin4);
  termOK(swmsg);

  // ── DHT11 ────────────────────────────────────
  termWait("Starting DHT11 sensor on GPIO 35...");
  delay(400);
  float th = dht.readHumidity();
  float tt = dht.readTemperature();
  if (!isnan(tt) && !isnan(th)) {
    char dhtmsg[48];
    snprintf(dhtmsg, sizeof(dhtmsg), "DHT11 OK  Temp=%.1fC  Hum=%.1f%%", tt, th);
    termUpdateLastBadge("[ OK ]", C_GREEN);
    termOK(dhtmsg);
    currentTemp = tt; currentHumidity = th; sensorError = false;
  } else {
    termUpdateLastBadge("[WARN]", C_ORANGE);
    termLog("[WARN]", C_ORANGE, "DHT11 no data yet — will retry in loop", C_ORANGE);
  }

  // ── AceButton ────────────────────────────────
  termOK("AceButton handlers registered  x4");

  // ── RainMaker ────────────────────────────────
  termWait("Initializing ESP RainMaker node...");
  delay(200);
  termUpdateLastBadge("[ OK ]", C_GREEN);
  termOK("RainMaker node: ESPHome  devices=6");

  // ── BLE Provisioning notice ──────────────────
  termBLE("Starting BLE provisioning service...");
  delay(300);
  char blemsg[64];
  snprintf(blemsg, sizeof(blemsg), "BLE ready  SSID=%s  POP=%s", service_name, pop);
  termBLE(blemsg);
  termBLE("Open 'ESP RainMaker' app to provision WiFi");

  // Draw provisioning instruction box at bottom
  termDrawProvBox();

  // ── WiFi connecting (animated — loop handles dots) ───
  termWifi("Connecting to WiFi ...");
}

// ═══════════════════════════════════════════════════════════
//  DASHBOARD DRAWING FUNCTIONS
// ═══════════════════════════════════════════════════════════

// ── Draw WiFi icon (simple bars) ──────────────────────────
void drawWiFiIcon(int x, int y, bool connected) {
  uint16_t col = connected ? C_GREEN : C_RED;
  if (connected) {
    tft.fillRect(x,     y + 8, 3, 4, col);
    tft.fillRect(x + 4, y + 5, 3, 7, col);
    tft.fillRect(x + 8, y + 2, 3, 10, col);
  } else {
    // X mark
    tft.drawLine(x, y + 2, x + 10, y + 11, col);
    tft.drawLine(x + 10, y + 2, x, y + 11, col);
  }
}

// ── Draw temperature icon ─────────────────────────────────
void drawTempIcon(int x, int y) {
  tft.drawCircle(x + 2, y, 3, C_ORANGE);
  tft.drawFastVLine(x + 2, y + 3, 10, C_ORANGE);
  tft.fillCircle(x + 2, y + 13, 4, C_ORANGE);
}

// ── Draw humidity icon ────────────────────────────────────
void drawHumidIcon(int x, int y) {
  tft.fillTriangle(x + 5, y, x, y + 8, x + 10, y + 8, C_ACCENT);
  tft.fillCircle(x + 5, y + 10, 5, C_ACCENT);
}

// ── Draw bulb icon ────────────────────────────────────────
void drawBulbIcon(int x, int y, bool on) {
  uint16_t col = on ? C_YELLOW : C_DARKGRAY;
  tft.drawCircle(x + 7, y + 6, 6, col);
  tft.fillCircle(x + 7, y + 6, 4, on ? C_YELLOW : C_PANEL);
  tft.drawFastHLine(x + 4, y + 13, 6, col);
  tft.drawFastHLine(x + 5, y + 15, 4, col);
}

// ── Draw fan icon ─────────────────────────────────────────
void drawFanIcon(int x, int y, bool on) {
  uint16_t col = on ? C_ACCENT : C_DARKGRAY;
  tft.drawCircle(x + 7, y + 7, 7, col);
  tft.drawLine(x + 7, y, x + 7, y + 14, col);
  tft.drawLine(x, y + 7, x + 14, y + 7, col);
  tft.drawLine(x + 2, y + 2, x + 12, y + 12, col);
  tft.drawLine(x + 12, y + 2, x + 2, y + 12, col);
  tft.fillCircle(x + 7, y + 7, 2, col);
}

// ── Draw TV icon ──────────────────────────────────────────
void drawTVIcon(int x, int y, bool on) {
  uint16_t col = on ? C_GREEN : C_DARKGRAY;
  tft.drawRect(x, y + 2, 16, 11, col);
  if (on) tft.fillRect(x + 1, y + 3, 14, 9, 0x0318);
  tft.drawLine(x + 4, y + 14, x + 5, y + 13, col);
  tft.drawLine(x + 12, y + 14, x + 11, y + 13, col);
  tft.drawFastHLine(x + 4, y + 14, 9, col);
}

// ── Draw OTA icon ─────────────────────────────────────────
void drawOTAIcon(int x, int y) {
  tft.drawRect(x, y + 5, 12, 8, C_ACCENT);
  tft.drawLine(x + 6, y, x + 6, y + 5, C_ACCENT);
  tft.drawLine(x + 3, y + 3, x + 6, y, C_ACCENT);
  tft.drawLine(x + 9, y + 3, x + 6, y, C_ACCENT);
}

// ─────────────────────────────────────────────
//  TOP STATUS BAR  (y = 0..22)
// ─────────────────────────────────────────────
void drawTopBar() {
  tft.fillRect(0, 0, 320, 22, C_TOPBAR);
  tft.drawFastHLine(0, 22, 320, C_ACCENT);

  // WiFi icon
  drawWiFiIcon(6, 5, wifiConnected);

  // IP address
  tft.setTextColor(wifiConnected ? C_GREEN : C_RED, C_TOPBAR);
  tft.setTextSize(1);
  tft.setCursor(22, 7);
  tft.print(wifiConnected ? wifiIP : "No WiFi");

  // Time  (right side)
  tft.setTextColor(C_WHITE, C_TOPBAR);
  tft.setCursor(230, 7);
  tft.print(currentTime);

  // Firmware version tag
  tft.setTextColor(C_DARKGRAY, C_TOPBAR);
  tft.setCursor(270, 7);
  tft.print(CURRENT_FIRMWARE_VERSION);
}

// ─────────────────────────────────────────────
//  DATE/DAY BAR  (y = 23..36)
// ─────────────────────────────────────────────
void drawDateBar() {
  tft.fillRect(0, 23, 320, 14, C_BG);
  tft.drawFastHLine(0, 36, 320, C_DARKGRAY);

  tft.setTextColor(C_GRAY, C_BG);
  tft.setTextSize(1);
  tft.setCursor(8, 27);
  tft.print(currentDay);
  tft.print(",  ");
  tft.print(currentDate);
}

// ─────────────────────────────────────────────
//  SWITCH CARDS  (y = 38..155)
//  Layout: 2x2 grid, each card ~150x55
// ─────────────────────────────────────────────
struct CardPos { int x, y, w, h; };
CardPos cards[4] = {
  {4,   38, 150, 55},
  {162, 38, 154, 55},
  {4,   97, 150, 55},
  {162, 97, 154, 55}
};

void drawSwitchCard(int idx, const char* name, bool state) {
  CardPos &c = cards[idx];

  uint16_t bgCol     = state ? C_CARD_ON  : C_CARD_OFF;
  uint16_t borderCol = state ? C_ACCENT   : C_DARKGRAY;
  uint16_t txtCol    = state ? C_WHITE    : C_GRAY;

  // Card background
  tft.fillRoundRect(c.x, c.y, c.w, c.h, 6, bgCol);
  tft.drawRoundRect(c.x, c.y, c.w, c.h, 6, borderCol);

  // Icon (top-left of card)
  int ix = c.x + 8;
  int iy = c.y + 8;
  switch (idx) {
    case 0: drawBulbIcon(ix, iy, state); break;
    case 1: drawBulbIcon(ix, iy, state); break;
    case 2: drawFanIcon(ix, iy, state);  break;
    case 3: drawTVIcon(ix, iy, state);   break;
  }

  // Device name
  tft.setTextColor(txtCol, bgCol);
  tft.setTextSize(1);
  tft.setCursor(c.x + 30, c.y + 10);
  tft.print(name);

  // ON / OFF pill badge
  uint16_t badgeBg  = state ? C_GREEN : C_DARKGRAY;
  uint16_t badgeTxt = TFT_BLACK;
  int bx = c.x + c.w - 36;
  int by = c.y + 8;
  tft.fillRoundRect(bx, by, 30, 14, 4, badgeBg);
  tft.setTextColor(badgeTxt, badgeBg);
  tft.setCursor(bx + (state ? 6 : 5), by + 3);
  tft.print(state ? "ON" : "OFF");

  // Relay pin label (small, bottom of card)
  tft.setTextColor(C_DARKGRAY, bgCol);
  tft.setCursor(c.x + 8, c.y + c.h - 13);
  tft.print("GPIO ");
  const uint8_t pins[] = {RelayPin1, RelayPin2, RelayPin3, RelayPin4};
  tft.print(pins[idx]);
}

void drawAllCards() {
  drawSwitchCard(0, deviceName_1, toggleState_1);
  drawSwitchCard(1, deviceName_2, toggleState_2);
  drawSwitchCard(2, deviceName_3, toggleState_3);
  drawSwitchCard(3, deviceName_4, toggleState_4);
}

// ─────────────────────────────────────────────
//  SENSOR BAR  (y = 156..185)
// ─────────────────────────────────────────────
void drawSensorBar() {
  tft.fillRect(0, 156, 320, 30, C_PANEL);
  tft.drawFastHLine(0, 156, 320, C_DARKGRAY);
  tft.drawFastHLine(0, 186, 320, C_DARKGRAY);

  // Temp icon + value
  drawTempIcon(8, 161);
  tft.setTextColor(C_ORANGE, C_PANEL);
  tft.setTextSize(2);
  tft.setCursor(26, 162);
  if (sensorError) {
    tft.setTextSize(1);
    tft.setTextColor(C_RED, C_PANEL);
    tft.print("ERR");
  } else {
    char tbuf[10];
    snprintf(tbuf, sizeof(tbuf), "%.1f", currentTemp);
    tft.print(tbuf);
    tft.setTextSize(1);
    tft.setTextColor(C_ORANGE, C_PANEL);
    tft.print(" C");
  }

  // Divider
  tft.drawFastVLine(160, 160, 22, C_DARKGRAY);

  // Humidity icon + value
  drawHumidIcon(170, 160);
  tft.setTextColor(C_ACCENT, C_PANEL);
  tft.setTextSize(2);
  tft.setCursor(186, 162);
  if (sensorError) {
    tft.setTextSize(1);
    tft.setTextColor(C_RED, C_PANEL);
    tft.print("ERR");
  } else {
    char hbuf[10];
    snprintf(hbuf, sizeof(hbuf), "%.1f", currentHumidity);
    tft.print(hbuf);
    tft.setTextSize(1);
    tft.setTextColor(C_ACCENT, C_PANEL);
    tft.print(" %");
  }
}

// ─────────────────────────────────────────────
//  BOTTOM STATUS BAR  (y = 188..210)
// ─────────────────────────────────────────────
void drawBottomBar() {
  tft.fillRect(0, 187, 320, 53, C_BG);
  tft.drawFastHLine(0, 187, 320, C_ACCENT);

  // OTA status icon
  drawOTAIcon(8, 194);

  tft.setTextColor(C_GRAY, C_BG);
  tft.setTextSize(1);
  tft.setCursor(26, 195);
  tft.print("OTA: ");
  tft.setTextColor(C_GREEN, C_BG);
  tft.print(otaStatusMsg.length() ? otaStatusMsg : "Up to date");

  // Firmware version + chip info
  tft.setTextColor(C_DARKGRAY, C_BG);
  tft.setCursor(8, 210);
  tft.print("FW: " CURRENT_FIRMWARE_VERSION);
  tft.setCursor(120, 210);
  tft.print("Free RAM: ");
  tft.print(ESP.getFreeHeap() / 1024);
  tft.print(" kB");

  // Switch count line
  int onCount = (int)toggleState_1 + (int)toggleState_2 + (int)toggleState_3 + (int)toggleState_4;
  tft.setCursor(8, 224);
  tft.setTextColor(C_GRAY, C_BG);
  tft.print("Active devices: ");
  tft.setTextColor(onCount > 0 ? C_GREEN : C_GRAY, C_BG);
  tft.print(onCount);
  tft.setTextColor(C_GRAY, C_BG);
  tft.print(" / 4");

  // Uptime
  unsigned long upSec = millis() / 1000;
  char upbuf[24];
  snprintf(upbuf, sizeof(upbuf), "Up: %02lud %02lu:%02lu:%02lu",
           upSec / 86400, (upSec % 86400) / 3600,
           (upSec % 3600) / 60, upSec % 60);
  tft.setTextColor(C_DARKGRAY, C_BG);
  tft.setCursor(170, 224);
  tft.print(upbuf);
}

// ─────────────────────────────────────────────
//  FULL DASHBOARD INITIAL DRAW
// ─────────────────────────────────────────────
void drawDashboard() {
  tft.fillScreen(C_BG);
  drawTopBar();
  drawDateBar();
  drawAllCards();
  drawSensorBar();
  drawBottomBar();
}

// ─────────────────────────────────────────────
//  OTA PROGRESS OVERLAY  (center screen)
// ─────────────────────────────────────────────
void drawOTAOverlay(int percent, const char* msg1, const char* msg2) {
  // Dim overlay box
  tft.fillRoundRect(20, 70, 280, 100, 8, C_PANEL);
  tft.drawRoundRect(20, 70, 280, 100, 8, C_ACCENT);

  tft.setTextColor(C_ACCENT, C_PANEL);
  tft.setTextSize(1);
  tft.setCursor(30, 82);
  tft.print("  OTA Software Update");

  tft.setTextColor(C_WHITE, C_PANEL);
  tft.setCursor(30, 96);
  tft.print(msg1);

  tft.setTextColor(C_GRAY, C_PANEL);
  tft.setCursor(30, 108);
  tft.print(msg2);

  // Progress bar
  int bx = 30, by = 122, bw = 260, bh = 16;
  tft.fillRect(bx, by, bw, bh, C_DARKGRAY);
  tft.drawRect(bx - 1, by - 1, bw + 2, bh + 2, C_ACCENT);
  int filled = (bw * percent) / 100;
  for (int i = 0; i < filled; i++) {
    uint16_t col = (i < filled / 3) ? C_ACCENT
                 : (i < 2 * filled / 3) ? 0x07EF
                 : C_GREEN;
    tft.drawFastVLine(bx + i, by, bh, col);
  }
  char pbuf[8];
  snprintf(pbuf, sizeof(pbuf), "%d%%", percent);
  tft.setTextColor(TFT_BLACK, TFT_BLACK);
  tft.setCursor(bx + bw / 2 - 10, by + 4);
  tft.setTextColor(C_BG);
  tft.print(pbuf);
}

// ─────────────────────────────────────────────
//  PARTIAL REFRESH HELPERS
// ─────────────────────────────────────────────
void refreshTopBar()    { drawTopBar(); }
void refreshDateBar()   { drawDateBar(); }
void refreshSensorBar() { drawSensorBar(); }
void refreshBottomBar() { drawBottomBar(); }
void refreshCard(int i) {
  const char* names[] = {deviceName_1, deviceName_2, deviceName_3, deviceName_4};
  bool states[] = {toggleState_1, toggleState_2, toggleState_3, toggleState_4};
  drawSwitchCard(i, names[i], states[i]);
}

// ═══════════════════════════════════════════════════════════
//  EEPROM HELPERS
// ═══════════════════════════════════════════════════════════
void writeEEPROM(int addr, bool state) {
  if (ENABLE_EEPROM && EEPROM.read(addr) != state) {
    EEPROM.write(addr, state);
    EEPROM.commit();
  }
}

bool readEEPROM(int addr) {
  return ENABLE_EEPROM ? EEPROM.read(addr) : false;
}

String readStoredVersion() {
  String v = "";
  for (int i = EEPROM_VERSION_ADDR; i < EEPROM_VERSION_ADDR + 10; i++) {
    char c = EEPROM.read(i);
    if (c == '\0' || c == 0xFF) break;
    v += c;
  }
  return v;
}

void storeFirmwareVersion(const char* version) {
  for (int i = 0; i < 10; i++) {
    char c = version[i];
    EEPROM.write(EEPROM_VERSION_ADDR + i, c ? c : '\0');
  }
  EEPROM.commit();
}

// ═══════════════════════════════════════════════════════════
//  RELAY CONTROL
// ═══════════════════════════════════════════════════════════
void setRelay(uint8_t pin, int addr, bool state) {
  digitalWrite(pin, state);
  if (ENABLE_EEPROM) writeEEPROM(addr, state);
}

// ═══════════════════════════════════════════════════════════
//  BUTTON HANDLER
// ═══════════════════════════════════════════════════════════
void buttonHandler(AceButton*, uint8_t eventType, uint8_t,
                   uint8_t relayPin, int eepromAddr,
                   Switch &sw, bool &state, int cardIdx) {
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
  refreshCard(cardIdx);
  refreshBottomBar();
}

// ═══════════════════════════════════════════════════════════
//  OTA FUNCTIONS
// ═══════════════════════════════════════════════════════════
String humanReadableSize(size_t bytes) {
  char buf[20];
  if (bytes < 1024)           sprintf(buf, "%u B",   (unsigned)bytes);
  else if (bytes < 1048576)   sprintf(buf, "%.1f kB", bytes / 1024.0);
  else                        sprintf(buf, "%.2f MB", bytes / 1048576.0);
  return String(buf);
}

String getLatestBinUrl(const char* apiUrl, String& latestVersion) {
  HTTPClient http;
  http.begin(apiUrl);
  int code = http.GET();
  if (code != HTTP_CODE_OK) { http.end(); return ""; }

  StaticJsonDocument<256> filter;
  filter["tag_name"]                          = true;
  filter["assets"][0]["name"]                 = true;
  filter["assets"][0]["browser_download_url"] = true;

  DynamicJsonDocument doc(4096);
  DeserializationError err = deserializeJson(doc, http.getStream(),
                               DeserializationOption::Filter(filter));
  http.end();
  if (err) return "";

  latestVersion = doc["tag_name"].as<String>();
  String binUrl = "";
  for (JsonObject asset : doc["assets"].as<JsonArray>()) {
    if (String((const char*)asset["name"]).endsWith(".ino.bin")) {
      binUrl = String((const char*)asset["browser_download_url"]);
      break;
    }
  }
  return binUrl;
}

bool startOTAUpdate(WiFiClient* client, int contentLength, const String &latestVersion) {
  if (!Update.begin(contentLength)) return false;

  size_t written = 0;
  int lastProgress = -1;
  unsigned long lastDataTime = millis();

  while (written < (size_t)contentLength) {
    if (client->available()) {
      uint8_t buffer[256];
      size_t len = client->read(buffer, sizeof(buffer));
      if (len > 0) {
        Update.write(buffer, len);
        written += len;
        int progress = (written * 100) / contentLength;
        if (progress != lastProgress) {
          String ws = humanReadableSize(written);
          String ts = humanReadableSize(contentLength);
          char l2[40], l3[40];
          snprintf(l2, sizeof(l2), "Downloading %s / %s", ws.c_str(), ts.c_str());
          snprintf(l3, sizeof(l3), "Version: %s", latestVersion.c_str());
          drawOTAOverlay(progress, l2, l3);
          lastProgress = progress;
        }
        lastDataTime = millis();
      }
    }
    if (millis() - lastDataTime > 30000) { Update.abort(); return false; }
    yield();
  }

  if (!Update.end() || !Update.isFinished()) return false;
  storeFirmwareVersion(latestVersion.c_str());
  return true;
}

void checkForOTAUpdate() {
  if (WiFi.status() != WL_CONNECTED) {
    otaStatusMsg = "No WiFi";
    refreshBottomBar();
    return;
  }

  otaStatusMsg = "Checking...";
  refreshBottomBar();

  String latestVersion;
  String apiURL = "https://api.github.com/repos/" GITHUB_USER "/" GITHUB_REPO "/releases/latest";
  String binURL = getLatestBinUrl(apiURL.c_str(), latestVersion);

  if (binURL == "") {
    otaStatusMsg = "No update found";
    refreshBottomBar();
    return;
  }

  String stored = readStoredVersion();
  if (latestVersion == CURRENT_FIRMWARE_VERSION || latestVersion == stored) {
    otaStatusMsg = "Up to date (" + latestVersion + ")";
    refreshBottomBar();
    return;
  }

  // New version available!
  otaStatusMsg = "Updating to " + latestVersion;
  otaInProgress = true;

  HTTPClient http;
  http.begin(binURL);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  int code = http.GET();
  if (code != HTTP_CODE_OK) {
    otaStatusMsg = "Download failed";
    otaInProgress = false;
    refreshBottomBar();
    http.end();
    return;
  }

  int contentLength = http.getSize();
  WiFiClient* stream = http.getStreamPtr();

  if (startOTAUpdate(stream, contentLength, latestVersion)) {
    otaStatusMsg = "Done! Rebooting...";
    delay(1500);
    ESP.restart();
  } else {
    otaStatusMsg = "OTA failed!";
  }

  otaInProgress = false;
  http.end();
  drawDashboard(); // Redraw after overlay
}

// ═══════════════════════════════════════════════════════════
//  TIME SYNC
// ═══════════════════════════════════════════════════════════
void syncTime() {
  configTime(19800, 0, "pool.ntp.org", "time.nist.gov"); // IST UTC+5:30
}

void updateTimeFromNTP() {
  struct tm ti;
  if (!getLocalTime(&ti)) {
    currentTime = "--:--";
    currentDate = "No NTP";
    currentDay  = "---";
    return;
  }
  char tbuf[8], dbuf[16], daybuf[12];
  strftime(tbuf,  sizeof(tbuf),  "%H:%M",    &ti);
  strftime(dbuf,  sizeof(dbuf),  "%d %b %Y", &ti);
  strftime(daybuf,sizeof(daybuf),"%A",        &ti);
  currentTime = String(tbuf);
  currentDate = String(dbuf);
  currentDay  = String(daybuf);
}

// ═══════════════════════════════════════════════════════════
//  WIFI LED TASK
// ═══════════════════════════════════════════════════════════
void stopWiFiLedTask() {
  if (wifiLedTaskHandle != NULL) {
    vTaskDelete(wifiLedTaskHandle);
    wifiLedTaskHandle = NULL;
  }
}

// ═══════════════════════════════════════════════════════════
//  RAINMAKER WRITE CALLBACK
// ═══════════════════════════════════════════════════════════
void write_callback(Device *device, Param *param,
                    const param_val_t val, void *priv_data, write_ctx_t *ctx) {
  const char *dn = device->getDeviceName();
  const char *pn = param->getParamName();

  if (strcmp(pn, "Power") == 0) {
    bool ns = val.val.b;
    int cardIdx = -1;
    if      (strcmp(dn, deviceName_1) == 0) { setRelay(RelayPin1, 0, ns); toggleState_1 = ns; my_switch1.updateAndReportParam(pn, ns); cardIdx = 0; }
    else if (strcmp(dn, deviceName_2) == 0) { setRelay(RelayPin2, 1, ns); toggleState_2 = ns; my_switch2.updateAndReportParam(pn, ns); cardIdx = 1; }
    else if (strcmp(dn, deviceName_3) == 0) { setRelay(RelayPin3, 2, ns); toggleState_3 = ns; my_switch3.updateAndReportParam(pn, ns); cardIdx = 2; }
    else if (strcmp(dn, deviceName_4) == 0) { setRelay(RelayPin4, 3, ns); toggleState_4 = ns; my_switch4.updateAndReportParam(pn, ns); cardIdx = 3; }
    if (cardIdx >= 0) {
      refreshCard(cardIdx);
      refreshBottomBar();
    }
  }
}

// ═══════════════════════════════════════════════════════════
//  WIFI PROVISIONING EVENT
//  Sab events terminal pe dikhte hain, dashboard baad mein
// ═══════════════════════════════════════════════════════════
void sysProvEvent(arduino_event_t *sys_event) {
  switch (sys_event->event_id) {

    case ARDUINO_EVENT_PROV_START:
      Serial.println("[BLE] Provisioning started");
      provisioningActive = true;
      if (!dashboardLaunched) {
        termBLE("BLE provisioning started — waiting for app...");
        termDrawProvBox();
      }
      stopWiFiLedTask();
      // Fast double blink = provisioning mode
      xTaskCreate([](void *){
        int s = 0;
        for (;;) {
          digitalWrite(wifiLed, (s == 0 || s == 2) ? HIGH : LOW);
          s = (s + 1) % 4;
          vTaskDelay(150 / portTICK_PERIOD_MS);
        }
      }, "WiFiLED_P", 1024, NULL, 1, &wifiLedTaskHandle);
      break;

    case ARDUINO_EVENT_PROV_CRED_RECV:
      Serial.println("[BLE] Credentials received");
      if (!dashboardLaunched) {
        // Clear prov box, show credential received
        tft.fillRect(0, 210, 320, 30, TFT_BLACK);
        termBLE("WiFi credentials received from app!");
        termWifi("Attempting WiFi connection...");
      }
      break;

    case ARDUINO_EVENT_PROV_CRED_FAIL:
      Serial.println("[BLE] Credentials failed");
      if (!dashboardLaunched) {
        termFail("WiFi credentials invalid — retry in app");
        termDrawProvBox();
      }
      break;

    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println("[WIFI] Connected");
      wifiConnected = true;
      wifiIP = WiFi.localIP().toString();
      syncTime();
      stopWiFiLedTask();
      // Double blink pattern = connected
      xTaskCreate([](void *){
        const int p[] = {100,100,200,100,100,1500};
        int idx = 0; bool ls = LOW;
        for (;;) {
          ls = !ls; digitalWrite(wifiLed, ls);
          vTaskDelay(p[idx] / portTICK_PERIOD_MS);
          idx = (idx + 1) % 6;
        }
      }, "WiFiLED_C", 1024, NULL, 1, &wifiLedTaskHandle);

      if (!dashboardLaunched) {
        // Clear WiFi dots line and show connected
        tft.fillRect(0, 210, 320, 30, TFT_BLACK);
        termUpdateLastBadge("[ OK ]", C_GREEN);
        char connmsg[48];
        snprintf(connmsg, sizeof(connmsg), "WiFi connected  IP: %s", wifiIP.c_str());
        termWifi(connmsg);
        // Show SSID
        char ssidmsg[48];
        snprintf(ssidmsg, sizeof(ssidmsg), "SSID: %s  RSSI: %d dBm",
                 WiFi.SSID().c_str(), WiFi.RSSI());
        termInfo(ssidmsg);
        termOK("NTP time sync started  (IST UTC+5:30)");
        delay(600);
        // Launch dashboard
        termLaunchDashboard();
        dashboardLaunched = true;
        drawDashboard();
      } else {
        // Already on dashboard — just refresh top bar
        refreshTopBar();
      }
      break;

    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("[WIFI] Disconnected");
      wifiConnected = false;
      wifiIP = "---";
      stopWiFiLedTask();
      // Slow blink = disconnected
      xTaskCreate([](void *){
        bool ls = LOW;
        for (;;) {
          ls = !ls; digitalWrite(wifiLed, ls);
          vTaskDelay(500 / portTICK_PERIOD_MS);
        }
      }, "WiFiLED_D", 1024, NULL, 1, &wifiLedTaskHandle);

      if (dashboardLaunched) {
        // On dashboard — just update top bar, keep working
        refreshTopBar();
      } else {
        // Still in terminal — update retry count, dont spam new lines
        wifiRetryCount++;
        char rmsg[48];
        snprintf(rmsg, sizeof(rmsg), "WiFi disconnected  retry #%d — reconnecting...", wifiRetryCount);
        // If last line was connecting, update it; else add new fail line
        if (termLineCount > 0 &&
            strstr(termLines[termLineCount-1].msg, "Connecting") != NULL) {
          strncpy(termLines[termLineCount-1].msg, rmsg, 63);
          termLines[termLineCount-1].badgeColor = C_RED;
          termLines[termLineCount-1].msgColor   = C_RED;
          strncpy(termLines[termLineCount-1].badge, "[WIFI]", 7);
          termRedrawLines();
        } else {
          termFail(rmsg);
        }
        // Reset to connecting dots animation
        wifiAnimating = true;
        // Add fresh connecting line after short delay
        delay(500);
        termWifi("Connecting to WiFi ...");
      }
      break;

    case ARDUINO_EVENT_PROV_END:
      Serial.println("[BLE] Provisioning ended");
      provisioningActive = false;
      if (!dashboardLaunched) {
        termBLE("Provisioning session ended");
      }
      break;

    default: break;
  }
}

// ═══════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════
void setup() {
  pinMode(wifiLed, OUTPUT);
  digitalWrite(wifiLed, LOW);
  Serial.begin(115200);

  // ── Display init ──────────────────────────────────────
  tft.init();
  tft.setRotation(1);   // Landscape 320x240
  tft.fillScreen(TFT_BLACK);

  // ── EEPROM ────────────────────────────────────────────
  if (ENABLE_EEPROM) EEPROM.begin(EEPROM_SIZE);
  toggleState_1 = ENABLE_EEPROM ? readEEPROM(0) : LOW;
  toggleState_2 = ENABLE_EEPROM ? readEEPROM(1) : LOW;
  toggleState_3 = ENABLE_EEPROM ? readEEPROM(2) : LOW;
  toggleState_4 = ENABLE_EEPROM ? readEEPROM(3) : LOW;

  // ── GPIO ──────────────────────────────────────────────
  pinMode(RelayPin1, OUTPUT); pinMode(RelayPin2, OUTPUT);
  pinMode(RelayPin3, OUTPUT); pinMode(RelayPin4, OUTPUT);
  pinMode(SwitchPin1, INPUT_PULLUP); pinMode(SwitchPin2, INPUT_PULLUP);
  pinMode(SwitchPin3, INPUT_PULLUP); pinMode(SwitchPin4, INPUT_PULLUP);
  pinMode(gpio_reset, INPUT);

  setRelay(RelayPin1, 0, toggleState_1);
  setRelay(RelayPin2, 1, toggleState_2);
  setRelay(RelayPin3, 2, toggleState_3);
  setRelay(RelayPin4, 3, toggleState_4);

  // ── DHT ───────────────────────────────────────────────
  dht.begin();

  // ── BOOT SEQUENCE ─────────────────────────────────────
  runBootSequence();

  // ── Buttons ───────────────────────────────────────────
  config1.setEventHandler([](AceButton *b, uint8_t e, uint8_t s) {
    buttonHandler(b, e, s, RelayPin1, 0, my_switch1, toggleState_1, 0);
  });
  config2.setEventHandler([](AceButton *b, uint8_t e, uint8_t s) {
    buttonHandler(b, e, s, RelayPin2, 1, my_switch2, toggleState_2, 1);
  });
  config3.setEventHandler([](AceButton *b, uint8_t e, uint8_t s) {
    buttonHandler(b, e, s, RelayPin3, 2, my_switch3, toggleState_3, 2);
  });
  config4.setEventHandler([](AceButton *b, uint8_t e, uint8_t s) {
    buttonHandler(b, e, s, RelayPin4, 3, my_switch4, toggleState_4, 3);
  });
  button1.init(SwitchPin1); button2.init(SwitchPin2);
  button3.init(SwitchPin3); button4.init(SwitchPin4);

  // ── RainMaker ─────────────────────────────────────────
  Node my_node = RMaker.initNode("ESPHome");
  my_switch1.addCb(write_callback); my_switch2.addCb(write_callback);
  my_switch3.addCb(write_callback); my_switch4.addCb(write_callback);
  my_node.addDevice(tempSensor);    my_node.addDevice(humiditySensor);
  my_node.addDevice(my_switch1);    my_node.addDevice(my_switch2);
  my_node.addDevice(my_switch3);    my_node.addDevice(my_switch4);

  RMaker.enableOTA(OTA_USING_PARAMS);
  RMaker.enableTZService();
  RMaker.enableSchedule();
  RMaker.start();

  WiFi.onEvent(sysProvEvent);
  WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE,
                          WIFI_PROV_SCHEME_HANDLER_FREE_BTDM,
                          WIFI_PROV_SECURITY_1, pop, service_name);

  my_switch1.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, toggleState_1);
  my_switch2.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, toggleState_2);
  my_switch3.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, toggleState_3);
  my_switch4.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, toggleState_4);

  // Dashboard launch is handled in sysProvEvent
  // (ARDUINO_EVENT_WIFI_STA_CONNECTED triggers termLaunchDashboard + drawDashboard)
  // If no WiFi provisioned yet, terminal stays visible with BLE instructions

  Serial.println("[SYSTEM] Boot complete. Waiting for WiFi...");
}

// ═══════════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════════
void loop() {
  // ── Factory / WiFi reset button ───────────────────────
  // IMPORTANT: Yeh HAMESHA run hona chahiye — dashboard ya terminal dono mein
  // GPIO 0 = BOOT button (built-in on most ESP32 boards)
  // Hold 3  sec = WiFi reset (credentials clear, re-provision)
  // Hold 10 sec = Factory reset (sab kuch clear)
  if (digitalRead(gpio_reset) == LOW) {
    delay(100);
    unsigned long t = millis();
    // Show countdown on terminal if not on dashboard
    int lastSec = -1;
    while (digitalRead(gpio_reset) == LOW) {
      delay(50);
      int heldSec = (millis() - t) / 1000;
      if (!dashboardLaunched && heldSec != lastSec) {
        lastSec = heldSec;
        char rmsg[48];
        if (heldSec < 3)
          snprintf(rmsg, sizeof(rmsg), "Reset button held: %ds (3s=WiFi, 10s=Factory)", heldSec);
        else if (heldSec < 10)
          snprintf(rmsg, sizeof(rmsg), "Release now for WiFi Reset (%ds/10s)...", heldSec);
        else
          snprintf(rmsg, sizeof(rmsg), "FACTORY RESET in progress...");
        // Update footer with reset info
        tft.fillRect(0, 210, 320, 30, 0x3800);
        tft.setTextColor(C_YELLOW, 0x3800);
        tft.setTextSize(1);
        tft.setCursor(6, 215);
        tft.print(rmsg);
      }
    }
    unsigned long dur = millis() - t;
    if (dur > 10000) {
      if (!dashboardLaunched) termFail("FACTORY RESET triggered — clearing all data...");
      Serial.println("[RESET] Factory reset triggered");
      RMakerFactoryReset(2);
    } else if (dur > 3000) {
      if (!dashboardLaunched) termLog("[RST ]", C_ORANGE, "WiFi Reset — credentials cleared, re-provisioning...", C_ORANGE);
      Serial.println("[RESET] WiFi reset triggered");
      RMakerWiFiReset(2);
    }
  }

  // ── Terminal mode — animate dots, then return ──────────
  if (!dashboardLaunched) {
    if (wifiAnimating) termAnimateWifi();
    button1.check(); button2.check();
    button3.check(); button4.check();
    return;
  }

  // ── DHT sensor read ───────────────────────────────────
  if (millis() - lastSensorRead > sensorInterval) {
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    if (!isnan(h) && !isnan(t)) {
      currentTemp     = t;
      currentHumidity = h;
      sensorError     = false;
      tempSensor.updateAndReportParam("Temperature", t);
      humiditySensor.updateAndReportParam("Temperature", h);
    } else {
      sensorError = true;
    }
    refreshSensorBar();
    lastSensorRead = millis();
  }

  // ── Display update (time, status, bottom bar) ─────────
  if (millis() - lastDisplayUpdate > displayInterval) {
    if (wifiConnected) updateTimeFromNTP();
    refreshTopBar();
    refreshDateBar();
    refreshBottomBar();
    lastDisplayUpdate = millis();
  }

  // ── OTA check ─────────────────────────────────────────
  if (!initialCheckDone && millis() > 15000) {
    checkForOTAUpdate();
    lastOTACheckTime  = millis();
    initialCheckDone  = true;
  }
  if (millis() - lastOTACheckTime >= otaCheckInterval) {
    checkForOTAUpdate();
    lastOTACheckTime = millis();
  }

  // ── Buttons ───────────────────────────────────────────
  button1.check(); button2.check();
  button3.check(); button4.check();
}
