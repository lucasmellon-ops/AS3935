#include <SPI.h>

// ===== Pins (Arduino Uno/Nano) =====
const int PIN_CS  = 10;   // -> CS
const int PIN_IRQ = 2;    // -> IRQ

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

// ===== IRQ flag & edge =====
volatile bool lightningIRQ = false;
void IRAM_ATTR isrIRQ() { lightningIRQ = true; }
volatile bool irqOnRising = true;   // toggled by 'r'

// ===== AS3935 Registers / Masks (per datasheet) =====
#define REG_AFE_GAIN       0x00 // [5:1] AFE_GB (indoor/outdoor preset)
#define REG_NOISE_WDTH     0x01 // [6:4] NF_LEV, [3:0] WDTH
#define REG_SREJ_MNL       0x02 // [5:4] MIN_NUM_LIGH, [3:0] SREJ
#define REG_INT            0x03 // [7:6] LCO_FDIV, [5] MASK_DIST, [3:0] INT
#define REG_ENERGY_H       0x04 // [4:0] MSBs of energy
#define REG_ENERGY_M       0x05
#define REG_ENERGY_L       0x06
#define REG_DISTANCE       0x07 // [5:0] km (0x3F => unknown/out-of-range)
#define REG_TUN_CAP_LCO    0x08 // [7] DISP_LCO, [3:0] TUN_CAP
#define REG_CALIB          0x3D // Direct command: 0x96 => CALIB_RCO

// INT flags (reg 0x03 bits [3:0])
#define INT_NOISE        0x01
#define INT_DISTURBER    0x04
#define INT_LIGHTNING    0x08

// ===== User filters (tunable at runtime) =====
uint32_t minEnergyToReport = 200000; // via 'e<num>'
bool countZeroEnergy = false;        // via 'z0/1'

// ===== Debounce / lockout =====
unsigned long lastIrqMicros = 0;
const unsigned long irqMinSpacingUs = 10000;  // raw edge debounce 10 ms
const unsigned long reportLockoutMs = 150;    // ignore spark "trains" for 150 ms
unsigned long lastReportMs = 0;

// ===== Basic Stats =====
struct {
  uint32_t strikes = 0;
  int minKm = 999;
  int maxKm = -1;
  float avgKm = 0.0f;
  uint32_t lastEnergy = 0;
  unsigned long lastMs = 0;
} Stats;

void resetStats() {
  Stats.strikes = 0;
  Stats.minKm = 999;
  Stats.maxKm = -1;
  Stats.avgKm = 0.0f;
  Stats.lastEnergy = 0;
  Stats.lastMs = 0;
}

// ===== Extended Metrics =====
struct Event {
  uint32_t tms;
  uint32_t energy;
  int8_t   distKm; // -1 = unknown
};

const uint8_t RING_N = 32;
Event ringBuf[RING_N];
uint8_t ringHead = 0, ringCount = 0;

uint32_t cntLightning = 0, cntDisturber = 0, cntNoise = 0;

// Energy histogram (8 bins)
const uint32_t EH[8] = { 50000UL, 100000UL, 200000UL, 400000UL, 800000UL, 1600000UL, 3200000UL, 0xFFFFFFFF };
uint32_t eHist[8] = {0};

// Rolling strike rate per minute (avg over 5 minutes)
const uint8_t RATE_WINDOWS = 5;
uint16_t rateBuckets[RATE_WINDOWS] = {0};
uint8_t  rateIdx = 0;
unsigned long rateBucketStartMs = 0;
const unsigned long RATE_BUCKET_MS = 60000UL;

uint32_t lastStrikeMs = 0;
uint32_t longestQuietMs = 0;

void pushEvent(uint32_t tms, uint32_t e, int d) {
  ringBuf[ringHead] = {tms, e, (int8_t)(d > 127 ? -1 : d)};
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
  uint32_t sum = 0;
  for (uint8_t i=0;i<RATE_WINDOWS;i++) sum += rateBuckets[i];
  return sum / (float)RATE_WINDOWS;
}

// ===== SPI helpers =====
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

// ===== Convenience =====
uint32_t as3935ReadEnergy() {
  // L -> M -> H order latches energy on many clones; 20-bit value
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

// Indoor/Outdoor presets per datasheet
void as3935SetIndoor()  { as3935UpdateBits(REG_AFE_GAIN, 0b00111110, 1, 0b10010); } // 0x12
void as3935SetOutdoor() { as3935UpdateBits(REG_AFE_GAIN, 0b00111110, 1, 0b01110); } // 0x0E

// NoiseFloor [6:4] (0..7), WDTH [3:0] (0..15)
void as3935SetNoiseFloor(uint8_t lvl) {
  if (lvl > 7) lvl = 7;
  as3935UpdateBits(REG_NOISE_WDTH, 0b01110000, 4, lvl);
}
void as3935SetWDTH(uint8_t wd) {
  if (wd > 15) wd = 15;
  as3935UpdateBits(REG_NOISE_WDTH, 0b00001111, 0, wd);
}

// Spike rejection SREJ [3:0] (0..15), Min strikes MIN_NUM_LIGH [5:4] (0..3 => 1/5/9/16)
void as3935SetSREJ(uint8_t s) {
  if (s > 15) s = 15;
  as3935UpdateBits(REG_SREJ_MNL, 0b00001111, 0, s);
}
void as3935SetMinStrikes(uint8_t m) {
  if (m > 3) m = 3; // 0:1, 1:5, 2:9, 3:16
  as3935UpdateBits(REG_SREJ_MNL, 0b00110000, 4, m);
}

void as3935EnableDisturberReject(bool en) {
  uint8_t r = as3935ReadReg(REG_INT);
  if (en) r |=  (1 << 5);  // MASK_DIST = 1 => mask disturber IRQs
  else    r &= ~(1 << 5);
  as3935WriteReg(REG_INT, r);
}

// Antenna tuning: LCO on IRQ, divider, tuning caps
void as3935SetLCO(bool enable, uint8_t fdivSel /*0:÷16, 1:÷32, 2:÷64*/) {
  // Set DISP_LCO
  uint8_t r8 = as3935ReadReg(REG_TUN_CAP_LCO);
  if (enable) r8 |=  (1 << 7);
  else        r8 &= ~(1 << 7);
  as3935WriteReg(REG_TUN_CAP_LCO, r8);

  // Set LCO_FDIV (bits [7:6] of REG_INT)
  if (fdivSel > 2) fdivSel = 2;
  as3935UpdateBits(REG_INT, 0b11000000, 6, fdivSel);
}

void as3935SetTuningCap(uint8_t steps /*0..15*/) {
  if (steps > 15) steps = 15;
  as3935UpdateBits(REG_TUN_CAP_LCO, 0b00001111, 0, steps);
}

// ===== Printing helpers =====
void printStats() {
  Serial.println(F("=== Lightning Stats ==="));
  Serial.print(F("Strikes: "));   Serial.println(Stats.strikes);
  Serial.print(F("Min km: "));    Serial.println(Stats.minKm == 999 ? -1 : Stats.minKm);
  Serial.print(F("Max km: "));    Serial.println(Stats.maxKm);
  Serial.print(F("Avg km: "));    Serial.println(Stats.strikes ? Stats.avgKm : 0.0f);
  Serial.print(F("Last E: "));    Serial.println(Stats.lastEnergy);
  Serial.print(F("Last ms: "));   Serial.println(Stats.lastMs);
  Serial.print(F("MinEnergy: ")); Serial.println(minEnergyToReport);
  Serial.print(F("CountZeroE: "));Serial.println(countZeroEnergy ? F("ON") : F("OFF"));
  Serial.print(F("IRQ edge: "));  Serial.println(irqOnRising ? F("RISING") : F("FALLING"));
  Serial.println(F("======================="));
}

void printExtendedSummary() {
  Serial.println(F("=== Extended Summary ==="));
  Serial.print(F("Lightning: "));  Serial.println(cntLightning);
  Serial.print(F("Disturbers: ")); Serial.println(cntDisturber);
  Serial.print(F("Noise: "));      Serial.println(cntNoise);
  Serial.print(F("Strike rate (per min, 5-min avg): "));
  Serial.println(strikeRatePerMin(), 2);
  Serial.print(F("Time since last (s): "));
  Serial.println(lastStrikeMs ? (millis()-lastStrikeMs)/1000.0f : -1);
  Serial.print(F("Longest quiet (s): "));
  Serial.println(longestQuietMs/1000.0f);
  Serial.println(F("========================"));
}

void printHistogram() {
  Serial.println(F("Energy histogram (<= thresholds):"));
  for (uint8_t i=0;i<8;i++){
    Serial.print(EH[i]); Serial.print(F(": ")); Serial.println(eHist[i]);
  }
}

void printRecentEvents() {
  Serial.println(F("Recent events (newest first): t(ms)  energy  dist(km)"));
  int idx = (ringHead + RING_N - 1) % RING_N;
  for (uint8_t i=0;i<ringCount;i++) {
    const Event &ev = ringBuf[idx];
    Serial.print(ev.tms); Serial.print('\t');
    Serial.print(ev.energy); Serial.print('\t');
    Serial.println((int)ev.distKm);
    idx = (idx + RING_N - 1) % RING_N;
  }
}

void dumpCoreRegs() {
  Serial.println(F("Registers:"));
  for (uint8_t r=0; r<=0x08; r++){
    uint8_t v = as3935ReadReg(r);
    Serial.print(F("R[")); Serial.print(r, HEX); Serial.print(F("]=0x")); Serial.println(v, HEX);
  }
  uint8_t c = as3935ReadReg(REG_CALIB);
  Serial.print(F("R[3D]=0x")); Serial.println(c, HEX);
}

// ===== IRQ helpers =====
void attachIrq(bool rising) {
  detachInterrupt(digitalPinToInterrupt(PIN_IRQ));
  attachInterrupt(digitalPinToInterrupt(PIN_IRQ), isrIRQ, rising ? RISING : FALLING);
}

// Wait until IRQ returns to idle (idle = LOW on AS3935 until event; line releases HIGH on event and falls after read)
// We just wait until it is no longer "active" per chosen edge heuristic, with generous timeout.
void waitForIrqRelease() {
  unsigned long t0 = micros();
  // After we read INT reg, IRQ should fall back. Just wait for a change or timeout.
  while ((micros() - t0) < 100000UL) {
    if (digitalRead(PIN_IRQ) == LOW) break;
  }
}

// ===== Profiles (macros) =====
void applyStormProfile() {
  as3935SetOutdoor();
  as3935EnableDisturberReject(true);
  as3935SetNoiseFloor(3);
  // Leave WDTH/SREJ defaults unless needed:
  // as3935SetWDTH(2); as3935SetSREJ(2);
  as3935SetMinStrikes(0); // 0: report first strike
  minEnergyToReport = 200000;
  countZeroEnergy = false;
  Serial.println(F("STORM profile: OUTDOOR, d=ON, noise=3, minE=200000"));
}

void applyDemoProfile() {
  as3935SetIndoor();
  as3935EnableDisturberReject(false);
  as3935SetNoiseFloor(2);
  // as3935SetWDTH(2); as3935SetSREJ(2);
  as3935SetMinStrikes(0);
  minEnergyToReport = 100000;
  countZeroEnergy = false;
  Serial.println(F("DEMO profile: INDOOR, d=OFF, noise=2, minE=100000"));
}

// ===== Menu =====
void handleMenu() {
  static char buf[16];
  static uint8_t idx = 0;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { idx = 0; continue; }

    if (c == 'i') { as3935SetIndoor();  Serial.println(F("Mode: INDOOR"));  }
    else if (c == 'o') { as3935SetOutdoor(); Serial.println(F("Mode: OUTDOOR")); }
    else if (c == 's') { printStats(); }
    else if (c == 'S') { printExtendedSummary(); }
    else if (c == 'h') { printHistogram(); }
    else if (c == 'l') { printRecentEvents(); }
    else if (c == 'c') { resetStats(); Serial.println(F("Stats cleared.")); }
    else if (c == 't') { applyDemoProfile(); Serial.println(F("Test mode (DEMO profile)")); }
    else if (c == 'X') { applyStormProfile(); }
    else if (c == 'Y') { applyDemoProfile(); }
    else if (c == 'p') { dumpCoreRegs(); }
    else if (c == 'd') {
      while (!Serial.available()) {}
      char v = Serial.read();
      bool on = (v != '0');
      as3935EnableDisturberReject(on);
      Serial.print(F("Disturber reject: ")); Serial.println(on ? F("ON") : F("OFF"));
    }
    else if (c == 'n') {
      while (!Serial.available()) {}
      char lvl = Serial.read();
      if (lvl >= '0' && lvl <= '7') {
        as3935SetNoiseFloor(lvl - '0');
        Serial.print(F("Noise floor set to ")); Serial.println(lvl - '0');
      }
    }
    else if (c == 'w') { // WDTH 0..15
      while (!Serial.available()) {}
      char v = Serial.read();
      if (v >= '0' && v <= '9') { as3935SetWDTH(v - '0'); Serial.print(F("WDTH=")); Serial.println(v - '0'); }
      else if (v >= 'A' && v <= 'F') { as3935SetWDTH(10 + v - 'A'); Serial.print(F("WDTH=")); Serial.println(10 + v - 'A'); }
      else if (v >= 'a' && v <= 'f') { as3935SetWDTH(10 + v - 'a'); Serial.print(F("WDTH=")); Serial.println(10 + v - 'a'); }
    }
    else if (c == 'r') {
      irqOnRising = !irqOnRising;
      attachIrq(irqOnRising);
      Serial.print(F("IRQ edge: ")); Serial.println(irqOnRising ? F("RISING") : F("FALLING"));
    }
    else if (c == 'j') { // SREJ 0..15
      while (!Serial.available()) {}
      char v = Serial.read();
      if (v >= '0' && v <= '9') { as3935SetSREJ(v - '0'); Serial.print(F("SREJ=")); Serial.println(v - '0'); }
      else if (v >= 'A' && v <= 'F') { as3935SetSREJ(10 + v - 'A'); Serial.print(F("SREJ=")); Serial.println(10 + v - 'A'); }
      else if (v >= 'a' && v <= 'f') { as3935SetSREJ(10 + v - 'a'); Serial.print(F("SREJ=")); Serial.println(10 + v - 'a'); }
    }
    else if (c == 'm') { // MIN_NUM_LIGH 0..3 (0:1,1:5,2:9,3:16)
      while (!Serial.available()) {}
      char v = Serial.read();
      if (v >= '0' && v <= '3') { as3935SetMinStrikes(v - '0'); Serial.print(F("Min strikes index=")); Serial.println(v - '0'); }
    }
    else if (c == 'L') { // Toggle LCO on IRQ and set divider
      while (!Serial.available()) {}
      char v = Serial.read(); // '0' off, '1' on
      bool on = (v == '1');
      uint8_t divSel = 0; // default ÷16
      if (on) {
        // Optional: read another char for divider: '1'->16, '2'->32, '3'->64
        while (Serial.available() == 0) {}
        char d = Serial.read();
        if (d == '1') divSel = 0;
        else if (d == '2') divSel = 1;
        else if (d == '3') divSel = 2;
      }
      as3935SetLCO(on, divSel);
      Serial.print(F("LCO on IRQ: ")); Serial.print(on ? F("ON") : F("OFF"));
      if (on) { Serial.print(F("  Divider: ")); Serial.println(divSel==0?16:(divSel==1?32:64)); } else Serial.println();
    }
    else if (c == 'U') { // TUN_CAP 0..15
      while (!Serial.available()) {}
      char v = Serial.read();
      uint8_t cap = 0xFF;
      if (v >= '0' && v <= '9') cap = v - '0';
      else if (v >= 'A' && v <= 'F') cap = 10 + v - 'A';
      else if (v >= 'a' && v <= 'f') cap = 10 + v - 'a';
      if (cap != 0xFF) {
        as3935SetTuningCap(cap);
        Serial.print(F("TUN_CAP=")); Serial.println(cap);
      }
    }
    else if (c == 'z') {
      while (!Serial.available()) {}
      char v = Serial.read();
      countZeroEnergy = (v != '0');
      Serial.print(F("Zero-energy events: ")); Serial.println(countZeroEnergy ? F("COUNTED") : F("IGNORED"));
    }
    else if (c == 'e') {
      // read numeric value into buf
      idx = 0;
      while (idx < sizeof(buf)-1) {
        while (!Serial.available()) {}
        char d = Serial.read();
        if (d == '\n' || d == '\r' || d == ' ') break;
        if (d < '0' || d > '9') continue;
        buf[idx++] = d;
      }
      buf[idx] = 0;
      uint32_t v = 0;
      for (uint8_t i=0;i<idx;i++) v = v*10 + (buf[i]-'0');
      minEnergyToReport = v;
      Serial.print(F("Min energy set to ")); Serial.println(minEnergyToReport);
      idx = 0;
    }
    else if (c == 'R') {
      dumpCoreRegs();
    }
  }
}

// ===== Setup / Loop =====
void setup() {
  Serial.begin(115200);

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);

  pinMode(PIN_IRQ, INPUT_PULLUP);   // IRQ is open-drain on most boards; pull-up needed

  SPI.begin(); // D13=SCK ("SCL" on some PCBs), D11=MOSI, D12=MISO
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0)); // conservative SPI

  attachIrq(true); // start on RISING (IRQ goes HIGH on event)

  // Power-up / calibration sequence
  as3935CalibrateOsc();         // datasheet: calibrate RCOs at startup
  as3935SetIndoor();            // default: indoor (higher gain)
  as3935SetNoiseFloor(3);       // moderate
  as3935SetWDTH(2);             // default-ish
  as3935SetSREJ(2);             // default-ish
  as3935SetMinStrikes(0);       // report first strike
  as3935EnableDisturberReject(true);

  (void)as3935ReadReg(REG_INT); // read INT once to clear any stale IRQ

  Serial.println(F("AS3935 init complete. Waiting for IRQs..."));
  Serial.println(F(
    "Menu: i/o (indoor/outdoor)  n0..n7 (noise)  w0..F (WDTH)  j0..F (SREJ)  m0..3 (min strikes)\n"
    "      d0/1 (disturber OFF/ON)  e<num> (minEnergy)  z0/1 (zeroE OFF/ON)  r (IRQ edge)\n"
    "      L1<1|2|3>/L0 (LCO on ÷16/32/64 or off)  U0..F (TUN_CAP)  R (regs)  s/S/h/l (stats)\n"
    "      X=STORM  Y=DEMO  t=DEMO"
  ));

  rateBucketStartMs = millis();
}

void loop() {
  handleMenu();

  if (lightningIRQ) {
    unsigned long nowUs = micros();
    if (nowUs - lastIrqMicros < irqMinSpacingUs) {
      lightningIRQ = false;
      return; // raw edge debounce
    }
    lastIrqMicros = nowUs;
    lightningIRQ = false;

    // Let INT latch and distance compute per datasheet
    delay(2); // ~2 ms is safe for event data to settle

    uint8_t reg3 = as3935ReadReg(REG_INT);
    uint8_t intsrc = reg3 & 0x0F;

    if (intsrc & INT_LIGHTNING) {
      // "train" lockout: ignore events too soon after a report
      unsigned long nowMs = millis();
      if (nowMs - lastReportMs < reportLockoutMs) {
        waitForIrqRelease();
        return;
      }

      // Read energy and distance AFTER settle
      uint32_t energy = as3935ReadEnergy();
      int distKm = as3935ReadDistanceKm();

      // Filter: ignore energy==0 unless enabled, and small energies below threshold
      if ((energy == 0 && !countZeroEnergy) || (minEnergyToReport && energy < minEnergyToReport)) {
        waitForIrqRelease();
        return;
      }

      // Update basic stats
      Stats.strikes++;
      if (distKm >= 0) {
        if (distKm < Stats.minKm) Stats.minKm = distKm;
        if (distKm > Stats.maxKm) Stats.maxKm = distKm;
        if (Stats.avgKm == 0.0f && Stats.strikes == 1) Stats.avgKm = distKm;
        else Stats.avgKm = Stats.avgKm + (distKm - Stats.avgKm) / (float)Stats.strikes;
      }
      Stats.lastEnergy = energy;
      Stats.lastMs = nowMs;

      // Extended metrics
      cntLightning++;
      pushEvent(nowMs, energy, distKm);

      lastReportMs = nowMs;

      Serial.print(F("⚡ Lightning!  Energy=")); Serial.print(energy);
      Serial.print(F("  Distance(km)=")); Serial.println(distKm < 0 ? -1 : distKm);

      waitForIrqRelease();
    }
    else if (intsrc & INT_DISTURBER) {
      cntDisturber++;
      Serial.println(F("~ Disturber detected (ignored)."));
      waitForIrqRelease();
    }
    else if (intsrc & INT_NOISE) {
      cntNoise++;
      Serial.println(F("~ Noise too high; increase noise floor (n3..n5)."));
      waitForIrqRelease();
    }
    else {
      // No latched reason; likely a stray edge (or we read too soon)
      waitForIrqRelease();
    }
  }

  delay(2);
}
