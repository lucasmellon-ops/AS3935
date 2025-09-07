// ==== ESP32-C3 + AS3935 + 0.42" OLED + Web Portal ====
// Board: ESP32C3 Dev Module (ABRobot ESP32-C3 OLED board)
// OLED: I2C on SDA=GPIO5, SCL=GPIO6, addr 0x3C (effective 72x40 px; we draw at (30,12))
// WiFi portal with monitor + settings + REST API
//
// Requires: U8g2 by olikraus

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <SPI.h>
#include <U8g2lib.h>

// ---------- Pins (ABRobot ESP32-C3) ----------
static const int PIN_OLED_SDA = 5;
static const int PIN_OLED_SCL = 6;

static const int PIN_SPI_SCLK = 2;
static const int PIN_SPI_MOSI = 3;
static const int PIN_SPI_MISO = 4;

static const int PIN_CS       = 7;   // AS3935 CS
static const int PIN_IRQ      = 1;   // AS3935 IRQ (open-drain; keep pullup)
static const int PIN_LED      = 8;   // Onboard blue LED
static const int PIN_BTN      = 9;   // BOOT button (pulled up; LOW when pressed)

// ---------- OLED ----------
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE, /* clock=*/PIN_OLED_SCL, /* data=*/PIN_OLED_SDA);
const int OLED_OFF_X = 30;  // left offset for 72x40 window
const int OLED_OFF_Y = 12;  // top offset
const int OLED_W = 72;
const int OLED_H = 40;

// ---------- WiFi / Web ----------
WebServer server(80);
Preferences prefs;
String wifi_ssid, wifi_pass;
bool wifi_connected = false;
unsigned long wifi_start_ms = 0;
const char *AP_SSID = "AS3935-Setup";
const char *AP_PASS = "12345678";
const char *MDNS_NAME = "as3935";

// ---------- AS3935 Registers / Masks ----------
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

volatile bool lightningIRQ = false;
void IRAM_ATTR isrIRQ() { lightningIRQ = true; }
volatile bool irqOnRising = true;   // toggle if needed

#define REG_AFE_GAIN       0x00 // [5:1] AFE_GB
#define REG_NOISE_WDTH     0x01 // [6:4] NF_LEV, [3:0] WDTH
#define REG_SREJ_MNL       0x02 // [5:4] MIN_NUM_LIGH, [3:0] SREJ
#define REG_INT            0x03 // [7:6] LCO_FDIV, [5] MASK_DIST, [3:0] INT
#define REG_ENERGY_H       0x04 // [4:0] MSBs of energy
#define REG_ENERGY_M       0x05
#define REG_ENERGY_L       0x06
#define REG_DISTANCE       0x07 // [5:0] km (0x3F => unknown)
#define REG_TUN_CAP_LCO    0x08 // [7] DISP_LCO, [3:0] TUN_CAP
#define REG_CALIB          0x3D // 0x96 => CALIB_RCO

#define INT_NOISE          0x01
#define INT_DISTURBER      0x04
#define INT_LIGHTNING      0x08

// ---------- User Filters ----------
uint32_t minEnergyToReport = 200000;
bool countZeroEnergy = false;

// ---------- Debounce / lockout ----------
unsigned long lastIrqMicros = 0;
const unsigned long irqMinSpacingUs = 10000;
const unsigned long reportLockoutMs = 150;
unsigned long lastReportMs = 0;

// ---------- Stats ----------
struct {
  uint32_t strikes = 0;
  int minKm = 999;
  int maxKm = -1;
  float avgKm = 0.0f;
  uint32_t lastEnergy = 0;
  unsigned long lastMs = 0;
} Stats;

struct Event {
  uint32_t tms;
  uint32_t energy;
  int8_t   distKm; // -1 = unknown
};

const uint8_t RING_N = 32;
Event ringBuf[RING_N];
uint8_t ringHead = 0, ringCount = 0;

uint32_t cntLightning = 0, cntDisturber = 0, cntNoise = 0;

const uint32_t EH[8] = { 50000UL, 100000UL, 200000UL, 400000UL, 800000UL, 1600000UL, 3200000UL, 0xFFFFFFFF };
uint32_t eHist[8] = {0};

const uint8_t RATE_WINDOWS = 5;
uint16_t rateBuckets[RATE_WINDOWS] = {0};
uint8_t  rateIdx = 0;
unsigned long rateBucketStartMs = 0;
const unsigned long RATE_BUCKET_MS = 60000UL;

uint32_t lastStrikeMs = 0;
uint32_t longestQuietMs = 0;

// ---------- SPI helpers ----------
uint8_t as3935ReadReg(uint8_t reg) {
  uint8_t cmd = 0x40 | (reg & 0x3F); // read
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(cmd);
  uint8_t val = SPI.transfer(0x00);
  digitalWrite(PIN_CS, HIGH);
  return val;
}

void as3935WriteReg(uint8_t reg, uint8_t value) {
  uint8_t cmd = 0x00 | (reg & 0x3F); // write
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(cmd);
  SPI.transfer(value);
  digitalWrite(PIN_CS, HIGH);
}

void as3935UpdateBits(uint8_t reg, uint8_t mask, uint8_t shift, uint8_t val) {
  uint8_t cur = as3935ReadReg(reg);
  cur &= ~mask;
  cur |= ((val << shift) & mask);
  as3935WriteReg(reg, cur);
}

// ---------- AS3935 convenience ----------
uint32_t as3935ReadEnergy() {
  // Read in L -> M -> H order (latches properly on many clones)
  uint8_t l = as3935ReadReg(REG_ENERGY_L);
  uint8_t m = as3935ReadReg(REG_ENERGY_M);
  uint8_t h = as3935ReadReg(REG_ENERGY_H) & 0x1F;
  return ((uint32_t)h << 16) | ((uint32_t)m << 8) | l;
}
int as3935ReadDistanceKm() {
  uint8_t d = as3935ReadReg(REG_DISTANCE) & 0x3F;
  return (d == 0x3F) ? -1 : (int)d;
}
void as3935CalibrateOsc() { as3935WriteReg(REG_CALIB, 0x96); delay(2); }
void as3935SetIndoor()  { as3935UpdateBits(REG_AFE_GAIN, 0b00111110, 1, 0b10010); } // 0x12
void as3935SetOutdoor() { as3935UpdateBits(REG_AFE_GAIN, 0b00111110, 1, 0b01110); } // 0x0E
void as3935SetNoiseFloor(uint8_t lvl) { if (lvl>7) lvl=7; as3935UpdateBits(REG_NOISE_WDTH, 0b01110000, 4, lvl); }
void as3935SetWDTH(uint8_t wd)       { if (wd>15) wd=15; as3935UpdateBits(REG_NOISE_WDTH, 0b00001111, 0, wd); }
void as3935SetSREJ(uint8_t s)        { if (s>15) s=15; as3935UpdateBits(REG_SREJ_MNL,  0b00001111, 0, s); }
void as3935SetMinStrikes(uint8_t m)  { if (m>3)  m=3;  as3935UpdateBits(REG_SREJ_MNL,  0b00110000, 4, m); }
void as3935EnableDisturberReject(bool en) {
  uint8_t r = as3935ReadReg(REG_INT);
  if (en) r |=  (1 << 5);
  else    r &= ~(1 << 5);
  as3935WriteReg(REG_INT, r);
}
void as3935SetLCO(bool enable, uint8_t fdivSel/*0:÷16,1:÷32,2:÷64*/) {
  uint8_t r8 = as3935ReadReg(REG_TUN_CAP_LCO);
  if (enable) r8 |=  (1 << 7);
  else        r8 &= ~(1 << 7);
  as3935WriteReg(REG_TUN_CAP_LCO, r8);
  if (fdivSel > 2) fdivSel = 2;
  as3935UpdateBits(REG_INT, 0b11000000, 6, fdivSel);
}
void as3935SetTuningCap(uint8_t steps/*0..15*/) { if (steps>15) steps=15; as3935UpdateBits(REG_TUN_CAP_LCO, 0b00001111, 0, steps); }

// ---------- Stats helpers ----------
void resetStats() {
  Stats = {};
  Stats.minKm = 999;
  ringHead = ringCount = 0;
  memset(eHist, 0, sizeof(eHist));
  memset(rateBuckets, 0, sizeof(rateBuckets));
  lastStrikeMs = longestQuietMs = 0;
}
void pushEvent(uint32_t tms, uint32_t e, int d) {
  ringBuf[ringHead] = {tms, e, (int8_t)(d>127 ? -1 : d)};
  ringHead = (ringHead + 1) % RING_N;
  if (ringCount < RING_N) ringCount++;
  for (uint8_t i=0;i<8;i++) { if (e <= EH[i]) { eHist[i]++; break; } }
  unsigned long now = millis();
  if (now - rateBucketStartMs >= RATE_BUCKET_MS) {
    uint8_t steps = (now - rateBucketStartMs) / RATE_BUCKET_MS;
    while (steps--) { rateIdx = (rateIdx + 1) % RATE_WINDOWS; rateBuckets[rateIdx] = 0; }
    rateBucketStartMs = now - ((now - rateBucketStartMs) % RATE_BUCKET_MS);
  }
  rateBuckets[rateIdx]++;
  if (lastStrikeMs != 0) {
    uint32_t gap = tms - lastStrikeMs;
    if (gap > longestQuietMs) longestQuietMs = gap;
  }
  lastStrikeMs = tms;
}
float strikeRatePerMin() {
  uint32_t sum = 0; for (uint8_t i=0;i<RATE_WINDOWS;i++) sum += rateBuckets[i];
  return sum / (float)RATE_WINDOWS;
}

// ---------- IRQ helpers ----------
void attachIrq(bool rising) {
  detachInterrupt(digitalPinToInterrupt(PIN_IRQ));
  attachInterrupt(digitalPinToInterrupt(PIN_IRQ), isrIRQ, rising ? RISING : FALLING);
}
void waitForIrqRelease() {
  unsigned long t0 = micros();
  while ((micros() - t0) < 100000UL) {
    if (digitalRead(PIN_IRQ) == LOW) break;
  }
}

// ---------- Profiles ----------
void applyStormProfile() {
  as3935SetOutdoor();
  as3935EnableDisturberReject(true);
  as3935SetNoiseFloor(3);
  as3935SetWDTH(2);
  as3935SetSREJ(2);
  as3935SetMinStrikes(0);
  minEnergyToReport = 200000;
  countZeroEnergy = false;
}
void applyDemoProfile() {
  as3935SetIndoor();
  as3935EnableDisturberReject(false);
  as3935SetNoiseFloor(2);
  as3935SetWDTH(2);
  as3935SetSREJ(2);
  as3935SetMinStrikes(0);
  minEnergyToReport = 100000;
  countZeroEnergy = false;
}

// ---------- OLED helpers ----------
enum ScreenMode { SCREEN_INFO, SCREEN_MONITOR };
ScreenMode screenMode = SCREEN_INFO;
unsigned long lastOledMs = 0;

void oledHeader(const char* title){
  u8g2.setDrawColor(1);
  u8g2.drawFrame(OLED_OFF_X-1, OLED_OFF_Y-10, OLED_W+2, OLED_H+12);
  u8g2.setFont(u8g2_font_5x8_mf);
  u8g2.setCursor(OLED_OFF_X+2, OLED_OFF_Y-2);
  u8g2.print(title);
}

void drawInfoScreen(const IPAddress &ip) {
  u8g2.clearBuffer();
  oledHeader("AS3935 ESP32-C3");
  u8g2.setFont(u8g2_font_5x8_mf);
  int y = OLED_OFF_Y + 10;
  u8g2.setCursor(OLED_OFF_X+2, y);      u8g2.print("IP: ");
  u8g2.print(ip.toString());
  y += 9;
  u8g2.setCursor(OLED_OFF_X+2, y);      u8g2.print("Port: 80");
  y += 9;
  u8g2.setCursor(OLED_OFF_X+2, y);      u8g2.print("URL: as3935.local");
  y += 9;
  u8g2.setCursor(OLED_OFF_X+2, y);      u8g2.print("BTN-> Monitor");
  u8g2.sendBuffer();
}

void drawMonitorScreen() {
  u8g2.clearBuffer();
  oledHeader("Monitor");
  u8g2.setFont(u8g2_font_5x8_mf);
  int y = OLED_OFF_Y + 10;
  u8g2.setCursor(OLED_OFF_X+2, y);   u8g2.print("Strikes: "); u8g2.print(Stats.strikes);
  y += 9;
  u8g2.setCursor(OLED_OFF_X+2, y);   u8g2.print("LastE: ");   u8g2.print(Stats.lastEnergy);
  y += 9;
  u8g2.setCursor(OLED_OFF_X+2, y);   u8g2.print("Dist(km): ");u8g2.print(Stats.avgKm,1);
  y += 9;
  u8g2.setCursor(OLED_OFF_X+2, y);   u8g2.print("Open /monitor");
  u8g2.sendBuffer();
}

// ---------- Button (BOOT) ----------
bool btnPrev = true;
void handleButton() {
  bool pressed = (digitalRead(PIN_BTN) == LOW);
  if (pressed != btnPrev) {
    btnPrev = pressed;
    if (pressed) {
      screenMode = (screenMode == SCREEN_INFO) ? SCREEN_MONITOR : SCREEN_INFO;
      digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    }
  }
}

// ---------- Web UI HTML ----------
const char* PAGE_HOME = R"HTML(
<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>AS3935 Home</title><style>
body{font-family:system-ui,Arial;margin:20px;max-width:720px}
a,button,input,select{font-size:16px}
.card{border:1px solid #ccc;border-radius:8px;padding:16px;margin:12px 0}
</style></head><body>
<h2>AS3935 Lightning Sensor</h2>
<div class="card">
  <p><b>Monitor:</b> <a href="/monitor">/monitor</a></p>
  <p><b>Settings:</b> <a href="/settings">/settings</a></p>
  <p><b>API:</b> <a href="/api/stats">/api/stats</a>, <a href="/api/settings">/api/settings</a></p>
</div>
</body></html>)HTML";

const char* PAGE_MONITOR = R"HTML(
<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>AS3935 Monitor</title><style>
body{font-family:system-ui,Arial;margin:20px;max-width:720px}
pre{background:#111;color:#0f0;padding:10px;border-radius:8px;overflow:auto;height:280px}
</style></head><body>
<h2>AS3935 Monitor</h2>
<pre id="log">connecting…</pre>
<script>
async function poll(){
  try{
    const r = await fetch('/api/stats'); const j = await r.json();
    const dist = (j.avgKm===null?'-':j.avgKm.toFixed(1));
    const lines = [
      'Strikes : '+j.strikes,
      'LastE   : '+j.lastEnergy,
      'Min/Avg/Max km : '+j.minKm+' / '+dist+' / '+j.maxKm,
      'Rate/min: '+j.rate.toFixed(2),
      'Lightning/Disturber/Noise: '+j.cntL+'/'+j.cntD+'/'+j.cntN,
      'Last (ms): '+j.lastMs,
      '',
      'Settings:',
      '  mode='+j.mode+'  noise='+j.noise+'  wdth='+j.wdth+'  srej='+j.srej,
      '  minStrikesIdx='+j.minStrikes+'  maskDist='+j.maskDist,
      '  minEnergy='+j.minEnergy+'  zeroE='+(j.zeroE?1:0)
    ];
    document.getElementById('log').textContent = lines.join('\\n');
  }catch(e){ /* ignore */ }
  setTimeout(poll, 1500);
}
poll();
</script></body></html>)HTML";

const char* PAGE_SETTINGS = R"HTML(
<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>AS3935 Settings</title><style>
body{font-family:system-ui,Arial;margin:20px;max-width:720px}
label{display:block;margin-top:8px}
input,select,button{font-size:16px}
.card{border:1px solid #ccc;border-radius:8px;padding:16px;margin:12px 0}
</style></head><body>
<h2>AS3935 Settings</h2>
<form method="POST" action="/api/settings" class="card">
<label>Mode:
<select name="mode"><option>indoor</option><option>outdoor</option></select></label>
<label>Noise (0..7): <input name="noise" type="number" min="0" max="7"></label>
<label>WDTH (0..15): <input name="wdth" type="number" min="0" max="15"></label>
<label>SREJ (0..15): <input name="srej" type="number" min="0" max="15"></label>
<label>Min strikes idx (0:1,1:5,2:9,3:16): <input name="minStrikes" type="number" min="0" max="3"></label>
<label>Mask disturber (0/1): <input name="mask" type="number" min="0" max="1"></label>
<label>Min energy: <input name="minEnergy" type="number" min="0"></label>
<label>Count zero-energy (0/1): <input name="zeroE" type="number" min="0" max="1"></label>
<button type="submit">Apply</button>
</form>
<div class="card"><form method="POST" action="/api/calibrate"><button>Calibrate RCO</button></form></div>
<div class="card"><a href="/">Home</a> | <a href="/monitor">Monitor</a></div>
</body></html>)HTML";

const char* PAGE_WIFI = R"HTML(
<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>WiFi Setup</title><style>
body{font-family:system-ui,Arial;margin:20px;max-width:720px}
input,button{font-size:16px}
.card{border:1px solid #ccc;border-radius:8px;padding:16px;margin:12px 0}
</style></head><body>
<h2>AS3935 WiFi Setup</h2>
<form class="card" method="POST" action="/wifi">
  <p>Connect to your Wi-Fi:</p>
  <p>SSID <input name="ssid" required></p>
  <p>PASS <input name="pass" type="password" required></p>
  <button type="submit">Save & Reboot</button>
</form>
</body></html>)HTML";

// ---------- Web Handlers ----------
String htmlEscape(String s){ s.replace("&","&amp;"); s.replace("<","&lt;"); s.replace(">","&gt;"); return s; }

void handleRoot(){
  if (!wifi_connected) { server.send(200,"text/html", PAGE_WIFI); return; }
  server.send(200,"text/html", PAGE_HOME);
}
void handleMonitor(){ server.send(200,"text/html", PAGE_MONITOR); }
void handleSettingsPage(){ server.send(200,"text/html", PAGE_SETTINGS); }

void jsonStats(String &out){
  // Read current settings snapshot from chip
  uint8_t r0 = as3935ReadReg(REG_AFE_GAIN);
  uint8_t r1 = as3935ReadReg(REG_NOISE_WDTH);
  uint8_t r2 = as3935ReadReg(REG_SREJ_MNL);
  uint8_t r3 = as3935ReadReg(REG_INT);

  int modeIndoor = ((r0 >> 1) & 0x1F) == 0x12;
  int noise = (r1 >> 4) & 0x07;
  int wdth  = r1 & 0x0F;
  int srej  = r2 & 0x0F;
  int minStr = (r2 >> 4) & 0x03;
  int mask = (r3 >> 5) & 1;

  out.reserve(512);
  out = "{";
  out += "\"strikes\":" + String(Stats.strikes);
  out += ",\"minKm\":" + String(Stats.minKm==999?-1:Stats.minKm);
  out += ",\"maxKm\":" + String(Stats.maxKm);
  out += ",\"avgKm\":" + String(Stats.strikes ? String(Stats.avgKm,2) : "null");
  out += ",\"lastEnergy\":" + String(Stats.lastEnergy);
  out += ",\"lastMs\":" + String(Stats.lastMs);
  out += ",\"cntL\":" + String(cntLightning);
  out += ",\"cntD\":" + String(cntDisturber);
  out += ",\"cntN\":" + String(cntNoise);
  out += ",\"rate\":" + String(strikeRatePerMin(),2);
  out += ",\"mode\":\""; out += (modeIndoor?"indoor":"outdoor"); out += "\"";
  out += ",\"noise\":" + String(noise);
  out += ",\"wdth\":"  + String(wdth);
  out += ",\"srej\":"  + String(srej);
  out += ",\"minStrikes\":" + String(minStr);
  out += ",\"maskDist\":" + String(mask);
  out += ",\"minEnergy\":" + String(minEnergyToReport);
  out += ",\"zeroE\":" + String((int)countZeroEnergy);
  out += "}";
}

void handleApiStats(){
  String js; jsonStats(js);
  server.send(200,"application/json", js);
}

int clampi(int v,int lo,int hi){ if(v<lo) return lo; if(v>hi) return hi; return v; }

void applySettings(const String &mode,int noise,int wdth,int srej,int minStr,int mask,uint32_t minE,int zero){
  if (mode=="indoor") as3935SetIndoor(); else if (mode=="outdoor") as3935SetOutdoor();
  as3935SetNoiseFloor(clampi(noise,0,7));
  as3935SetWDTH(clampi(wdth,0,15));
  as3935SetSREJ(clampi(srej,0,15));
  as3935SetMinStrikes(clampi(minStr,0,3));
  as3935EnableDisturberReject(mask!=0);
  minEnergyToReport = minE;
  countZeroEnergy = (zero!=0);
}

void handleApiSettingsGet(){
  String js; jsonStats(js); server.send(200,"application/json", js);
}

void handleApiSettingsPost(){
  String mode = server.hasArg("mode")? server.arg("mode") : "indoor";
  int noise = server.hasArg("noise")? server.arg("noise").toInt(): 3;
  int wdth  = server.hasArg("wdth") ? server.arg("wdth").toInt() : 2;
  int srej  = server.hasArg("srej") ? server.arg("srej").toInt() : 2;
  int minS  = server.hasArg("minStrikes")? server.arg("minStrikes").toInt() : 0;
  int mask  = server.hasArg("mask") ? server.arg("mask").toInt() : 1;
  uint32_t minE = server.hasArg("minEnergy")? (uint32_t)server.arg("minEnergy").toInt() : minEnergyToReport;
  int zero  = server.hasArg("zeroE")? server.arg("zeroE").toInt() : (int)countZeroEnergy;

  applySettings(mode,noise,wdth,srej,minS,mask,minE,zero);

  String js; jsonStats(js);
  server.send(200,"application/json", js);
}

void handleApiCalibrate(){
  as3935CalibrateOsc();
  server.send(200,"text/plain","OK");
}

void handleWifiPost(){
  if (!server.hasArg("ssid") || !server.hasArg("pass")) { server.send(400,"text/plain","ssid/pass required"); return; }
  String s = server.arg("ssid"), p = server.arg("pass");
  prefs.begin("as3935", false);
  prefs.putString("ssid", s);
  prefs.putString("pass", p);
  prefs.end();
  server.send(200,"text/html","<meta http-equiv='refresh' content='2;url=/'/><p>Saved. Rebooting…</p>");
  delay(500);
  ESP.restart();
}

// ---------- WiFi ----------
void startAP(){
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  wifi_connected = false;
}
bool tryConnectSaved(){
  prefs.begin("as3935", true);
  wifi_ssid = prefs.getString("ssid", "");
  wifi_pass = prefs.getString("pass", "");
  prefs.end();
  if (wifi_ssid.isEmpty()) return false;

  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_ssid.c_str(), wifi_pass.c_str());
  wifi_start_ms = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - wifi_start_ms) < 15000) {
    delay(100);
  }
  wifi_connected = (WiFi.status() == WL_CONNECTED);
  return wifi_connected;
}
void startMDNS(){
  if (MDNS.begin(MDNS_NAME)) {
    MDNS.addService("http", "tcp", 80);
  }
}

// ---------- Web server routes ----------
void setupWeb(){
  server.on("/", HTTP_GET, handleRoot);
  server.on("/monitor", HTTP_GET, handleMonitor);
  server.on("/settings", HTTP_GET, handleSettingsPage);
  server.on("/api/stats", HTTP_GET, handleApiStats);
  server.on("/api/settings", HTTP_GET, handleApiSettingsGet);
  server.on("/api/settings", HTTP_POST, handleApiSettingsPost);
  server.on("/api/calibrate", HTTP_POST, handleApiCalibrate);
  server.on("/wifi", HTTP_POST, handleWifiPost);
  server.begin();
}

// ---------- Setup ----------
void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  pinMode(PIN_BTN, INPUT_PULLUP);

  // OLED
  u8g2.begin();
  u8g2.setFont(u8g2_font_5x8_mf);
  u8g2.clearBuffer();
  u8g2.drawStr(0,12,"AS3935 ESP32-C3 boot...");
  u8g2.sendBuffer();

  // SPI + AS3935
  pinMode(PIN_CS, OUTPUT); digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_IRQ, INPUT_PULLUP);
  SPI.begin(PIN_SPI_SCLK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_CS);
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
  attachIrq(true);

  // Sensor init
  as3935CalibrateOsc();
  applyStormProfile();          // start sensible defaults
  (void)as3935ReadReg(REG_INT); // clear stale INT
  rateBucketStartMs = millis();

  // WiFi + Web
  bool ok = tryConnectSaved();
  if (!ok) startAP();
  setupWeb();
  if (wifi_connected) startMDNS();

  // First OLED paint
  delay(50);
}

// ---------- Loop ----------
void loop() {
  server.handleClient();
  handleButton();

  // OLED refresh ~5Hz
  if (millis() - lastOledMs > 200) {
    lastOledMs = millis();
    if (screenMode == SCREEN_INFO) {
      IPAddress ip = wifi_connected ? WiFi.localIP() : WiFi.softAPIP();
      drawInfoScreen(ip);
    } else {
      drawMonitorScreen();
    }
  }

  // Lightning processing
  if (lightningIRQ) {
    unsigned long nowUs = micros();
    if (nowUs - lastIrqMicros < irqMinSpacingUs) { lightningIRQ = false; return; }
    lastIrqMicros = nowUs;
    lightningIRQ = false;

    delay(2); // allow INT/energy/distance to settle

    uint8_t reg3 = as3935ReadReg(REG_INT);
    uint8_t intsrc = reg3 & 0x0F;

    if (intsrc & INT_LIGHTNING) {
      unsigned long nowMs = millis();
      if (nowMs - lastReportMs < reportLockoutMs) { waitForIrqRelease(); return; }

      uint32_t energy = as3935ReadEnergy();
      int distKm = as3935ReadDistanceKm();

      if ((energy == 0 && !countZeroEnergy) || (minEnergyToReport && energy < minEnergyToReport)) {
        waitForIrqRelease(); return;
      }

      // Update stats
      Stats.strikes++;
      if (distKm >= 0) {
        if (distKm < Stats.minKm) Stats.minKm = distKm;
        if (distKm > Stats.maxKm) Stats.maxKm = distKm;
        if (Stats.avgKm == 0.0f && Stats.strikes == 1) Stats.avgKm = distKm;
        else Stats.avgKm = Stats.avgKm + (distKm - Stats.avgKm) / (float)Stats.strikes;
      }
      Stats.lastEnergy = energy;
      Stats.lastMs = nowMs;
      cntLightning++;
      pushEvent(nowMs, energy, distKm);
      lastReportMs = nowMs;

      // Blink LED once
      digitalWrite(PIN_LED, HIGH); delay(20); digitalWrite(PIN_LED, LOW);

      waitForIrqRelease();
    }
    else if (intsrc & INT_DISTURBER) {
      cntDisturber++;
      waitForIrqRelease();
    }
    else if (intsrc & INT_NOISE) {
      cntNoise++;
      waitForIrqRelease();
    }
    else {
      waitForIrqRelease();
    }
  }
}
