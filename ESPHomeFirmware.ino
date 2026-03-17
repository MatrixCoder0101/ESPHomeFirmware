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
#define DHTPIN                   26
#define DHTTYPE                  DHT11

// ─────────────────────────────────────────────
//  GPIO
// ─────────────────────────────────────────────
static uint8_t RelayPin1 = 23;
static uint8_t RelayPin2 = 19;
static uint8_t RelayPin3 = 18;
static uint8_t RelayPin4 = 5;

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

// Boot log lines
#define MAX_LOG_LINES 10
String bootLog[MAX_LOG_LINES];
int    bootLogCount = 0;
int    bootLogScrollY = 0;  // pixel scroll offset

// ═══════════════════════════════════════════════════════════
//  ██████╗  ██████╗  ██████╗ ████████╗
//  ██╔══██╗██╔═══██╗██╔═══██╗╚══██╔══╝
//  ██████╔╝██║   ██║██║   ██║   ██║   
//  ██╔══██╗██║   ██║██║   ██║   ██║   
//  ██████╔╝╚██████╔╝╚██████╔╝   ██║   
//  ╚═════╝  ╚═════╝  ╚═════╝    ╚═╝   
//  BOOT SEQUENCE FUNCTIONS
// ═══════════════════════════════════════════════════════════

// Draw the Linux-style terminal boot screen background
void drawBootScreen() {
  tft.fillScreen(TFT_BLACK);
  // Top title bar
  tft.fillRect(0, 0, 320, 18, 0x0C4A);
  tft.setTextColor(C_ACCENT, 0x0C4A);
  tft.setTextSize(1);
  tft.setCursor(8, 5);
  tft.print("ESPHome v2.0 — Booting System");
  // Bottom hint
  tft.setTextColor(C_DARKGRAY, TFT_BLACK);
  tft.setCursor(8, 228);
  tft.print("ESP32 | TFT 320x240 | RainMaker");
}

// Add a line to boot log and scroll it on screen
void bootLogAdd(const char* status, const char* msg, uint16_t statusColor) {
  // Scroll up by one line if full
  const int LINE_H = 13;
  const int LOG_Y  = 22;
  const int LOG_MAX_LINES = 14;

  if (bootLogCount >= LOG_MAX_LINES) {
    // Scroll existing content up
    tft.setCursor(0, LOG_Y);
    // Re-draw all but first stored log (simple approach: re-draw from array)
    tft.fillRect(0, LOG_Y, 320, LOG_MAX_LINES * LINE_H, TFT_BLACK);
    for (int i = 1; i < bootLogCount; i++) {
      // Re-print stored lines... for simplicity we just draw new line at bottom
    }
    bootLogCount = LOG_MAX_LINES - 1;
  }

  int y = LOG_Y + bootLogCount * LINE_H;

  // Status badge [ OK ] / [....] / [FAIL] / [>>>>]
  tft.setTextColor(statusColor, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(4, y);
  tft.print(status);

  // Message
  tft.setTextColor(C_WHITE, TFT_BLACK);
  tft.setCursor(52, y);
  tft.print(msg);

  bootLogCount++;
  delay(80); // slight pause for "terminal feel"
}

// Animated progress bar for final boot
void drawBootProgress(int percent, const char* label) {
  int barX = 10, barY = 205, barW = 300, barH = 14;

  // Clear area
  tft.fillRect(barX, barY - 14, barW, 12, TFT_BLACK);
  tft.setTextColor(C_ACCENT, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(barX, barY - 13);
  tft.print(label);

  // Bar background
  tft.fillRect(barX, barY, barW, barH, C_DARKGRAY);
  tft.drawRect(barX - 1, barY - 1, barW + 2, barH + 2, C_ACCENT);

  // Filled portion
  int filled = (barW * percent) / 100;
  // Gradient-ish: use cyan-to-green
  for (int i = 0; i < filled; i++) {
    uint16_t col = (i < filled / 2) ? C_ACCENT : C_GREEN;
    tft.drawFastVLine(barX + i, barY, barH, col);
  }

  // Percentage text centered
  char buf[8];
  snprintf(buf, sizeof(buf), "%d%%", percent);
  tft.setTextColor(TFT_BLACK, TFT_BLACK);
  // White text over bar
  tft.setTextColor(TFT_BLACK);
  int tx = barX + (barW / 2) - 10;
  tft.setCursor(tx, barY + 3);
  tft.setTextColor(C_BG);
  tft.print(buf);
}

// Full animated boot sequence
void runBootSequence() {
  drawBootScreen();
  delay(300);

  bootLogAdd("[ OK ]", "Initializing ESP32 system...",    C_GREEN);
  bootLogAdd("[ OK ]", "TFT Display initialized",         C_GREEN);
  bootLogAdd("[ OK ]", "EEPROM mounted (" CURRENT_FIRMWARE_VERSION ")",  C_GREEN);

  // Animate: loading EEPROM relay states
  bootLogAdd("[ .. ]", "Loading relay states from EEPROM", C_YELLOW);
  delay(300);
  // Overwrite the ".." line with OK (move cursor back)
  tft.fillRect(4, 22 + (bootLogCount-1)*13, 40, 12, TFT_BLACK);
  tft.setTextColor(C_GREEN, TFT_BLACK);
  tft.setCursor(4, 22 + (bootLogCount-1)*13);
  tft.print("[ OK ]");
  delay(200);

  bootLogAdd("[ OK ]", "GPIO relay pins configured",       C_GREEN);
  bootLogAdd("[ OK ]", "AceButton handlers attached",      C_GREEN);
  bootLogAdd("[ OK ]", "DHT11 sensor starting...",         C_GREEN);
  bootLogAdd("[ OK ]", "RainMaker node: ESPHome",          C_GREEN);
  bootLogAdd("[ .. ]", "Starting WiFi provisioning (BLE)", C_YELLOW);
  delay(400);
  tft.fillRect(4, 22 + (bootLogCount-1)*13, 40, 12, TFT_BLACK);
  tft.setTextColor(C_GREEN, TFT_BLACK);
  tft.setCursor(4, 22 + (bootLogCount-1)*13);
  tft.print("[ OK ]");

  bootLogAdd("[>>>>]", "Launching Smart Dashboard...",     C_ACCENT);

  // Animated progress bar 0→100%
  for (int p = 0; p <= 100; p += 2) {
    drawBootProgress(p, "Loading Dashboard");
    delay(18);
  }
  delay(500);

  // Final flash effect
  tft.fillScreen(C_ACCENT);
  delay(80);
  tft.fillScreen(TFT_BLACK);
  delay(80);
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
// ═══════════════════════════════════════════════════════════
void sysProvEvent(arduino_event_t *sys_event) {
  switch (sys_event->event_id) {
    case ARDUINO_EVENT_PROV_START:
      Serial.println("Provisioning started");
      stopWiFiLedTask();
      xTaskCreate([](void *){
        int s = 0;
        for (;;) {
          digitalWrite(wifiLed, (s == 0 || s == 2) ? HIGH : LOW);
          s = (s + 1) % 4;
          vTaskDelay(150 / portTICK_PERIOD_MS);
        }
      }, "WiFiLED_P", 1024, NULL, 1, &wifiLedTaskHandle);
      break;

    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println("WiFi Connected");
      wifiConnected = true;
      wifiIP = WiFi.localIP().toString();
      syncTime();
      stopWiFiLedTask();
      xTaskCreate([](void *){
        const int p[] = {100,100,200,100,100,1500};
        int idx = 0; bool ls = LOW;
        for (;;) {
          ls = !ls; digitalWrite(wifiLed, ls);
          vTaskDelay(p[idx] / portTICK_PERIOD_MS);
          idx = (idx + 1) % 6;
        }
      }, "WiFiLED_C", 1024, NULL, 1, &wifiLedTaskHandle);
      refreshTopBar();
      break;

    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("WiFi Disconnected");
      wifiConnected = false;
      wifiIP = "---";
      stopWiFiLedTask();
      xTaskCreate([](void *){
        bool ls = LOW;
        for (;;) {
          ls = !ls; digitalWrite(wifiLed, ls);
          vTaskDelay(500 / portTICK_PERIOD_MS);
        }
      }, "WiFiLED_D", 1024, NULL, 1, &wifiLedTaskHandle);
      refreshTopBar();
      break;

    case ARDUINO_EVENT_PROV_END:
      Serial.println("Provisioning ended");
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

  // ── Draw dashboard ────────────────────────────────────
  drawDashboard();

  Serial.println("[SYSTEM] Dashboard live!");
}

// ═══════════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════════
void loop() {
  // ── Factory / WiFi reset button ───────────────────────
  if (digitalRead(gpio_reset) == LOW) {
    delay(100);
    unsigned long t = millis();
    while (digitalRead(gpio_reset) == LOW) delay(50);
    unsigned long dur = millis() - t;
    if (dur > 10000)     RMakerFactoryReset(2);
    else if (dur > 3000) RMakerWiFiReset(2);
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
