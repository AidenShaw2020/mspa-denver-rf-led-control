#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// ESP32-WROOM-32 / DevKit. Never connect the CC1101 to 5 V.
static constexpr int PIN_GDO0 = 17;
static constexpr int PIN_CSN = 5;
static constexpr int PIN_SCK = 18;
static constexpr int PIN_MOSI = 23;
static constexpr int PIN_MISO = 19;
static constexpr int PIN_GDO2 = 16;
static constexpr uint32_t XOSC_HZ = 26000000UL; // Change if your module has another crystal.
static constexpr uint32_t SPI_HZ = 4000000UL;
static constexpr uint16_t MAX_EDGES = 16384;

// CC1101 config/status registers and strobes.
static constexpr uint8_t IOCFG2 = 0x00, IOCFG1 = 0x01, IOCFG0 = 0x02;
static constexpr uint8_t PKTCTRL1 = 0x07, PKTCTRL0 = 0x08;
static constexpr uint8_t FREQ2 = 0x0D, FREQ1 = 0x0E, FREQ0 = 0x0F;
static constexpr uint8_t MDMCFG4 = 0x10, MDMCFG3 = 0x11, MDMCFG2 = 0x12;
static constexpr uint8_t MCSM1 = 0x17, MCSM0 = 0x18, FREND0 = 0x22;
static constexpr uint8_t FOCCFG = 0x19;
static constexpr uint8_t AGCCTRL2 = 0x1B, AGCCTRL1 = 0x1C, AGCCTRL0 = 0x1D;
static constexpr uint8_t PARTNUM = 0x30, VERSION = 0x31, RSSI_REG = 0x34, MARCSTATE = 0x35, TXBYTES = 0x3A;
static constexpr uint8_t SRES = 0x30, SRX = 0x34, STX = 0x35, SIDLE = 0x36, SFTX = 0x3B;
static constexpr uint8_t PATABLE = 0x3E, TXFIFO = 0x3F;

static volatile uint32_t edgeTime[MAX_EDGES]; // relative microseconds
static volatile uint8_t edgeFlags[MAX_EDGES];  // bit0: GDO0 after edge; bit1: GDO2 at edge
static volatile uint16_t edgeCount = 0;
static volatile bool capturing = false;
static volatile bool overflowed = false;
static portMUX_TYPE captureMux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t captureStartUs = 0, captureEndUs = 0, windowMs = 3000;
static uint8_t startLevel = 0, endCarrier = 0;
static bool hasCapture = false, radioReady = false, rssiMonitor = false;
static bool useGdo1 = true;
static volatile int dataPin = PIN_MISO;
static const char *stopReason = "none";
static uint32_t lastRssiMs = 0;

static char commandLine[80];
static size_t commandLength = 0;

// Requested values; readback registers are printed too.
static double wantedFreqMHz = 433.973;
static double wantedBwKHz = 101.6;
static double wantedRateKBaud = 4.8;
// This board measured 433.9722 MHz on air at a programmed 434.0254 MHz.
// The TX offset is board-specific; retune with txfreq when using another module.
static double wantedTxFreqMHz = 434.0254;
static uint8_t wantedAgc2 = 0x04;
static uint8_t wantedAgc0 = 0x92;

// Last five data symbols of each 25-mark period, S=0 and L=1.
// The 26th symbol in the receive fingerprint is the NEXT period's sync mark.
static constexpr struct { const char *name; uint8_t code; } BUTTONS[] = {
    {"power", 1}, {"mode_plus", 5}, {"speed_minus", 7}, {"demo", 8},
    {"speed_plus", 9}, {"color_plus", 10}, {"mode_minus", 11},
    {"bright_plus", 12}, {"color_minus", 13}, {"bright_minus", 15},
    {"white", 14}, {"red", 16}, {"green", 17}, {"blue", 18},
    {"yellow", 19}, {"cyan", 20}, {"pink", 21},
};

struct TxBits {
  uint8_t data[2048]{};
  size_t bits = 0;
  bool overflow = false;

  void append(bool high, uint32_t us) {
    // 40 kBaud OOK bitstream, one FIFO bit per 25 us.
    const uint32_t chips = (us + 12) / 25;
    for (uint32_t n = 0; n < chips; ++n) {
      if (bits >= sizeof(data) * 8) { overflow = true; return; }
      if (high) data[bits / 8] |= uint8_t(0x80 >> (bits % 8));
      ++bits;
    }
  }
  size_t bytes() const { return (bits + 7) / 8; }
};

static TxBits txWave;

void IRAM_ATTR onDataEdge() {
  const uint32_t now = micros();
  const uint8_t flags = (digitalRead(dataPin) ? 1 : 0) |
                        (digitalRead(PIN_GDO2) ? 2 : 0);
  portENTER_CRITICAL_ISR(&captureMux);
  if (capturing) {
    if (edgeCount < MAX_EDGES) {
      const uint16_t i = edgeCount;
      edgeTime[i] = now - captureStartUs;
      edgeFlags[i] = flags;
      edgeCount = i + 1;
    } else {
      overflowed = true;
      capturing = false;
      captureEndUs = now;
    }
  }
  portEXIT_CRITICAL_ISR(&captureMux);
}

bool selectRadio() {
  digitalWrite(PIN_CSN, LOW);
  const uint32_t beginUs = micros();
  while (digitalRead(PIN_MISO) == HIGH) {
    if ((uint32_t)(micros() - beginUs) > 5000) {
      digitalWrite(PIN_CSN, HIGH);
      return false;
    }
  }
  return true;
}

void deselectRadio() { digitalWrite(PIN_CSN, HIGH); }

bool strobe(uint8_t cmd) {
  SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
  if (!selectRadio()) { SPI.endTransaction(); return false; }
  SPI.transfer(cmd);
  deselectRadio();
  SPI.endTransaction();
  return true;
}

bool writeReg(uint8_t addr, uint8_t value) {
  SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
  if (!selectRadio()) { SPI.endTransaction(); return false; }
  SPI.transfer(addr);
  SPI.transfer(value);
  deselectRadio();
  SPI.endTransaction();
  return true;
}

bool readReg(uint8_t addr, uint8_t &value, bool status = false) {
  SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
  if (!selectRadio()) { SPI.endTransaction(); return false; }
  SPI.transfer(addr | (status ? 0xC0 : 0x80));
  value = SPI.transfer(0);
  deselectRadio();
  SPI.endTransaction();
  return true;
}

bool resetRadio() {
  deselectRadio(); delayMicroseconds(10);
  digitalWrite(PIN_CSN, LOW); delayMicroseconds(10);
  deselectRadio(); delayMicroseconds(50);
  SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
  if (!selectRadio()) { SPI.endTransaction(); return false; }
  SPI.transfer(SRES);
  const uint32_t beginUs = micros();
  while (digitalRead(PIN_MISO) == HIGH) {
    if ((uint32_t)(micros() - beginUs) > 10000) {
      deselectRadio(); SPI.endTransaction(); return false;
    }
  }
  deselectRadio();
  SPI.endTransaction();
  return true;
}

// f = (FREQ * fXOSC) / 2^16.
uint32_t frequencyWord(double mhz) {
  return (uint32_t)llround(mhz * 1000000.0 * 65536.0 / XOSC_HZ);
}
double frequencyMHz(uint32_t word) {
  return (double)word * XOSC_HZ / 65536.0 / 1000000.0;
}

// RX filter BW = fXOSC / [8 * (4 + M) * 2^E].
uint8_t nearestBw(double khz) {
  double bestError = 1e30;
  uint8_t best = 0;
  for (uint8_t e = 0; e < 4; ++e) {
    for (uint8_t m = 0; m < 4; ++m) {
      const double actual = (double)XOSC_HZ / (8.0 * (4 + m) * (1 << e)) / 1000.0;
      const double error = fabs(actual - khz);
      if (error < bestError) { bestError = error; best = (e << 6) | (m << 4); }
    }
  }
  return best;
}
double bandwidthKHz(uint8_t mdmcfg4) {
  const uint8_t e = (mdmcfg4 >> 6) & 3, m = (mdmcfg4 >> 4) & 3;
  return (double)XOSC_HZ / (8.0 * (4 + m) * (1 << e)) / 1000.0;
}

// Rdata = (256 + DRATE_M) * 2^DRATE_E * fXOSC / 2^28.
uint16_t nearestRate(double kbaud) {
  double bestError = 1e30;
  uint16_t best = 0;
  for (uint8_t e = 0; e < 16; ++e) {
    for (uint16_t m = 0; m < 256; ++m) {
      const double actual = (256.0 + m) * (1UL << e) * XOSC_HZ / 268435456.0 / 1000.0;
      const double error = fabs(actual - kbaud);
      if (error < bestError) { bestError = error; best = ((uint16_t)e << 8) | m; }
    }
  }
  return best;
}
double dataRateKBaud(uint8_t mdmcfg4, uint8_t mdmcfg3) {
  return (256.0 + mdmcfg3) * (1UL << (mdmcfg4 & 15)) * XOSC_HZ / 268435456.0 / 1000.0;
}

bool programRx() {
  const uint32_t fw = frequencyWord(wantedFreqMHz);
  const uint8_t bw = nearestBw(wantedBwKHz);
  const uint16_t rate = nearestRate(wantedRateKBaud);
  if (!strobe(SIDLE)) return false;
  delay(2);
  // GDO0 or shared SO/GDO1 = async data, GDO2 = carrier sense.
  if (!writeReg(IOCFG0, useGdo1 ? 0x2F : 0x0D) ||
      !writeReg(IOCFG1, useGdo1 ? 0x0D : 0x2E) ||
      !writeReg(IOCFG2, 0x0E) ||
      !writeReg(PKTCTRL1, 0x00) || !writeReg(PKTCTRL0, 0x30) ||
      !writeReg(FREQ2, fw >> 16) || !writeReg(FREQ1, fw >> 8) ||
      !writeReg(FREQ0, fw) ||
      !writeReg(MDMCFG4, bw | (rate >> 8)) ||
      !writeReg(MDMCFG3, rate & 0xFF) || !writeReg(MDMCFG2, 0x30) ||
      !writeReg(MCSM0, 0x18) || // auto-calibrate on IDLE -> RX
      !writeReg(FOCCFG, 0x16) || // FOC limit disabled for OOK.
      !writeReg(AGCCTRL2, wantedAgc2) || !writeReg(AGCCTRL1, 0x00) ||
      !writeReg(AGCCTRL0, wantedAgc0) || !strobe(SRX)) return false;
  delay(3);
  uint8_t state = 0;
  return readReg(MARCSTATE, state, true) && (state & 0x1F) == 0x0D;
}

bool writePaTable() {
  SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
  if (!selectRadio()) { SPI.endTransaction(); return false; }
  SPI.transfer(PATABLE | 0x40);
  SPI.transfer(0x00); // OOK off
  SPI.transfer(0x34); // approximately -10 dBm at 433 MHz
  deselectRadio();
  SPI.endTransaction();
  return true;
}

bool writeTxBytes(const uint8_t *data, size_t count) {
  SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
  if (!selectRadio()) { SPI.endTransaction(); return false; }
  SPI.transfer(TXFIFO | 0x40);
  for (size_t i = 0; i < count; ++i) SPI.transfer(data[i]);
  deselectRadio();
  SPI.endTransaction();
  return true;
}

void appendTxPeriod(TxBits &wave, uint8_t command) {
  // Measured 51.1 ms period: sync, 12.6 ms to first data mark,
  // then 24 data marks spaced 1.6 ms apart. The last gap is 1.7 ms.
  static constexpr char PREFIX[] = "SLSLLSLLSLSSLSSLSSSS";
  wave.append(true, 475);
  wave.append(false, 12125);
  for (uint8_t i = 0; i < 24; ++i) {
    const bool longMark = i < 19 ? PREFIX[i + 1] == 'L' :
                                    (command & (1U << (23 - i))) != 0;
    const uint32_t onUs = longMark ? 1275 : 475;
    wave.append(true, onUs);
    wave.append(false, (i == 23 ? 1700 : 1600) - onUs);
  }
}

bool sendButton(const char *name, uint8_t command) {
  if (!radioReady || capturing) return false;
  memset(txWave.data, 0, sizeof(txWave.data));
  txWave.bits = 0;
  txWave.overflow = false;
  txWave.append(false, 5000);
  for (uint8_t n = 0; n < 4; ++n) appendTxPeriod(txWave, command);
  txWave.append(false, 20000);
  if (txWave.overflow) { Serial.println("TX error: waveform buffer overflow"); return false; }

  detachInterrupt(digitalPinToInterrupt(dataPin));
  bool sent = false;
  const uint32_t txWord = frequencyWord(wantedTxFreqMHz);
  const uint8_t txBw = nearestBw(wantedBwKHz);
  const uint16_t txRate = nearestRate(40.0);
  do {
    pinMode(PIN_GDO0, INPUT); // FIFO OOK, no direct-GPIO modulation
    if (!strobe(SIDLE) || !writeReg(IOCFG0, 0x2E) ||
        !writePaTable() || !writeReg(FREND0, 0x11) ||
        !writeReg(FREQ2, txWord >> 16) || !writeReg(FREQ1, txWord >> 8) ||
        !writeReg(FREQ0, txWord) ||
        !writeReg(MDMCFG4, txBw | (txRate >> 8)) ||
        !writeReg(MDMCFG3, txRate & 0xFF) || !writeReg(MDMCFG2, 0x30) ||
        !writeReg(PKTCTRL0, 0x02) || !writeReg(MCSM1, 0x00) ||
        !strobe(SFTX)) break;
    const size_t initial = min(txWave.bytes(), size_t(64));
    if (!writeTxBytes(txWave.data, initial) || !strobe(STX)) break;
    size_t queued = initial;
    const uint32_t deadline = millis() + 1500;
    while ((int32_t)(millis() - deadline) < 0) {
      uint8_t occupied = 0;
      if (!readReg(TXBYTES, occupied, true)) break;
      if (occupied & 0x80) { sent = queued == txWave.bytes(); break; }
      occupied &= 0x7F;
      if (queued < txWave.bytes() && occupied <= 32) {
        const size_t count = min(size_t(32), txWave.bytes() - queued);
        if (!writeTxBytes(txWave.data + queued, count)) break;
        queued += count;
      }
      if (queued == txWave.bytes() && occupied == 0) { sent = true; break; }
      delay(1);
    }
  } while (false);

  // A reset restores all packet and modem registers before RX is rearmed.
  radioReady = strobe(SIDLE) && resetRadio() && programRx();
  if (radioReady) attachInterrupt(digitalPinToInterrupt(dataPin), onDataEdge, CHANGE);
  else Serial.println("TX error: RX restore failed; reset ESP32");
  if (sent && radioReady)
    Serial.printf("TX %s: four 51.1 ms OOK periods queued, RX restored; on-air waveform unverified\n", name);
  else
    Serial.printf("TX %s: failed, sent=%u RX=%u\n", name, sent, radioReady);
  return sent && radioReady;
}

void printConfig() {
  uint8_t f2, f1, f0, m4, m3, state;
  if (!readReg(FREQ2, f2) || !readReg(FREQ1, f1) || !readReg(FREQ0, f0) ||
      !readReg(MDMCFG4, m4) || !readReg(MDMCFG3, m3) ||
      !readReg(MARCSTATE, state, true)) { Serial.println("ERROR: SPI read"); return; }
  const uint32_t fw = ((uint32_t)f2 << 16) | ((uint32_t)f1 << 8) | f0;
  Serial.printf("CONFIG freq=%.6f MHz bw=%.3f kHz rate=%.5f kBaud state=0x%02X\n",
                frequencyMHz(fw), bandwidthKHz(m4), dataRateKBaud(m4, m3), state & 0x1F);
}

void stopCapture(const char *why) {
  uint16_t n;
  bool over;
  portENTER_CRITICAL(&captureMux);
  capturing = false;
  if (!overflowed) captureEndUs = micros();
  n = edgeCount;
  over = overflowed;
  portEXIT_CRITICAL(&captureMux);
  endCarrier = digitalRead(PIN_GDO2);
  stopReason = why;
  hasCapture = true;
  Serial.printf("STOP reason=%s edges=%u overflow=%u elapsed_us=%lu\n",
                why, n, over, (unsigned long)(captureEndUs - captureStartUs));
}

void startCapture(uint32_t durationMs) {
  if (capturing) { Serial.println("ERROR: capture already active"); return; }
  Serial.printf("CAPTURE start window_ms=%lu\n", (unsigned long)durationMs);
  // Align the boundary with the GPIO snapshot as closely as possible.
  portENTER_CRITICAL(&captureMux);
  edgeCount = 0;
  overflowed = false;
  startLevel = digitalRead(dataPin);
  captureStartUs = micros();
  windowMs = durationMs;
  hasCapture = false;
  capturing = true;
  portEXIT_CRITICAL(&captureMux);
}

void dumpCapture() {
  if (capturing) { Serial.println("ERROR: stop capture first"); return; }
  if (!hasCapture) { Serial.println("ERROR: no completed capture"); return; }
  uint8_t f2, f1, f0, m4, m3;
  if (!readReg(FREQ2, f2) || !readReg(FREQ1, f1) || !readReg(FREQ0, f0) ||
      !readReg(MDMCFG4, m4) || !readReg(MDMCFG3, m3)) {
    Serial.println("ERROR: SPI read"); return;
  }
  const uint32_t fw = ((uint32_t)f2 << 16) | ((uint32_t)f1 << 8) | f0;
  const uint32_t elapsed = captureEndUs - captureStartUs;
  const uint16_t n = edgeCount;
  Serial.printf("# raw-v1 freq_mhz=%.6f bw_khz=%.3f rate_kbaud=%.5f\n",
                frequencyMHz(fw), bandwidthKHz(m4), dataRateKBaud(m4, m3));
  Serial.printf("# edges=%u overflow=%u elapsed_us=%lu reason=%s\n",
                n, overflowed, (unsigned long)elapsed, stopReason);
  Serial.printf("# source=%s gpio=%d\n", useGdo1 ? "gdo1" : "gdo0", dataPin);
  Serial.println("index,t_start_us,level,duration_us,cs_at_end,boundary");
  uint32_t previous = 0;
  uint8_t level = startLevel;
  for (uint16_t i = 0; i < n; ++i) {
    const uint32_t at = edgeTime[i];
    const uint8_t flags = edgeFlags[i];
    Serial.printf("%u,%lu,%u,%lu,%u,%u\n", i, (unsigned long)previous,
                  level, (unsigned long)(at - previous), (flags >> 1) & 1, i == 0);
    previous = at;
    level = flags & 1;
  }
  Serial.printf("%u,%lu,%u,%lu,%u,1\n", n, (unsigned long)previous,
                level, (unsigned long)(elapsed - previous), endCarrier);
  Serial.println("# END");
}

void printRssi() {
  uint8_t raw, state;
  if (!readReg(RSSI_REG, raw, true) || !readReg(MARCSTATE, state, true)) {
    Serial.println("ERROR: SPI read"); return;
  }
  // Datasheet's approximate RSSI conversion for 433 MHz.
  const float dbm = (int8_t)raw / 2.0f - 74.0f;
  Serial.printf("RSSI raw=0x%02X approx=%.1f dBm GDO2_CS=%u state=0x%02X\n",
                raw, dbm, digitalRead(PIN_GDO2), state & 0x1F);
}

void printHelp() {
  Serial.println("help | start [100..60000 ms] | stop | dump | status | regs | pins");
  Serial.println("source gdo0 | source gdo1 (shared SPI MISO; no SPI during capture)");
  Serial.println("agc2 3..7 | agc0 91|92 (hex OOK threshold profile)");
  Serial.println("freq 433.973 | txfreq 434.0254 | bw 101.6 [kHz] | rate 4.8 [kBaud]");
  Serial.println("rssi | rssi on | rssi off | send <button|all> (manual TX only)");
}

void testPins() {
  if (capturing) { Serial.println("ERROR: stop capture first"); return; }
  uint8_t original0, original1, original2;
  if (!readReg(IOCFG0, original0) || !readReg(IOCFG1, original1) ||
      !readReg(IOCFG2, original2)) {
    Serial.println("ERROR: SPI read"); return;
  }
  bool ok = true;
  // 0x2F = forced low, 0x6F = forced high via GDOx_INV.
  if (!writeReg(IOCFG0, 0x2F) || !writeReg(IOCFG2, 0x2F)) ok = false;
  delay(2);
  const int low0 = digitalRead(PIN_GDO0), low2 = digitalRead(PIN_GDO2);
  if (!writeReg(IOCFG0, 0x6F) || !writeReg(IOCFG2, 0x6F)) ok = false;
  delay(2);
  const int high0 = digitalRead(PIN_GDO0), high2 = digitalRead(PIN_GDO2);
  if (!writeReg(IOCFG0, original0) || !writeReg(IOCFG2, original2)) ok = false;
  if (!writeReg(IOCFG1, 0x2F)) ok = false;
  delay(2);
  const int low1 = digitalRead(PIN_MISO);
  if (!writeReg(IOCFG1, 0x6F)) ok = false;
  delay(2);
  const int high1 = digitalRead(PIN_MISO);
  if (!writeReg(IOCFG1, original1)) ok = false;
  Serial.printf("PINS GDO0(GPIO17) low=%d high=%d GDO1(GPIO19) low=%d high=%d GDO2(GPIO16) low=%d high=%d SPI=%s\n",
                low0, high0, low1, high1, low2, high2, ok ? "ok" : "error");
  Serial.println("Expected: low=0 high=1 on all connected pins");
}

bool parseDouble(const char *s, double &value) {
  if (!s || !*s) return false;
  char *end = nullptr;
  value = strtod(s, &end);
  return end != s && *end == '\0' && isfinite(value);
}

void processCommand(char *line) {
  char *cmd = strtok(line, " \t\r\n");
  if (!cmd) return;
  char *arg = strtok(nullptr, " \t\r\n");
  char *extra = strtok(nullptr, " \t\r\n");
  if (extra) { Serial.println("ERROR: too many arguments"); return; }
  if (!strcmp(cmd, "help")) { printHelp(); return; }
  if (!strcmp(cmd, "pins")) { testPins(); return; }
  if (!strcmp(cmd, "source")) {
    if (capturing) { Serial.println("ERROR: stop capture first"); return; }
    if (!arg || (strcmp(arg, "gdo0") && strcmp(arg, "gdo1"))) {
      Serial.println("ERROR: source gdo0|gdo1"); return;
    }
    detachInterrupt(digitalPinToInterrupt(dataPin));
    useGdo1 = !strcmp(arg, "gdo1");
    dataPin = useGdo1 ? PIN_MISO : PIN_GDO0;
    hasCapture = false;
    radioReady = programRx();
    if (!radioReady) { Serial.println("ERROR: RX reconfiguration failed; reset ESP"); return; }
    attachInterrupt(digitalPinToInterrupt(dataPin), onDataEdge, CHANGE);
    Serial.printf("SOURCE %s GPIO%d\n", useGdo1 ? "gdo1" : "gdo0", dataPin);
    return;
  }
  if (!strcmp(cmd, "start")) {
    double d = 3000;
    if (arg && (!parseDouble(arg, d) || d < 100 || d > 60000 || floor(d) != d)) {
      Serial.println("ERROR: start [100..60000 ms]"); return;
    }
    startCapture((uint32_t)d); return;
  }
  if (!strcmp(cmd, "stop")) {
    if (capturing) stopCapture("manual");
    else Serial.println("ERROR: no active capture");
    return;
  }
  if (!strcmp(cmd, "dump")) { dumpCapture(); return; }
  if (!strcmp(cmd, "status")) {
    if (capturing) { Serial.println("ERROR: stop capture first"); return; }
    printConfig();
    Serial.printf("capture=%u saved=%u edges=%u overflow=%u source=%s GPIO%d=%u GDO2_CS=%u\n",
                  capturing, hasCapture, edgeCount, overflowed,
                  useGdo1 ? "gdo1" : "gdo0", dataPin,
                  digitalRead(dataPin), digitalRead(PIN_GDO2));
    return;
  }
  if (!strcmp(cmd, "regs")) {
    if (capturing) { Serial.println("ERROR: stop capture first"); return; }
    for (uint8_t a = 0; a <= 0x2E; ++a) {
      uint8_t v;
      if (!readReg(a, v)) { Serial.println("ERROR: SPI read"); return; }
      Serial.printf("%02X=%02X%s", a, v, ((a & 7) == 7 || a == 0x2E) ? "\n" : " ");
    }
    return;
  }
  if (!strcmp(cmd, "rssi")) {
    if (!arg) { if (!capturing) printRssi(); else Serial.println("ERROR: capture active"); }
    else if (!strcmp(arg, "on")) { rssiMonitor = true; Serial.println("RSSI monitor on"); }
    else if (!strcmp(arg, "off")) { rssiMonitor = false; Serial.println("RSSI monitor off"); }
    else Serial.println("ERROR: rssi [on|off]");
    return;
  }
  if (!strcmp(cmd, "send")) {
    if (capturing) { Serial.println("ERROR: stop capture first"); return; }
    if (!arg) { Serial.println("ERROR: send <button|all>"); return; }
    if (!strcmp(arg, "all")) {
      Serial.printf("TX sequence: %u buttons, four periods each, 1 s between buttons\n",
                    unsigned(sizeof(BUTTONS) / sizeof(BUTTONS[0])));
      for (size_t i = 0; i < sizeof(BUTTONS) / sizeof(BUTTONS[0]); ++i) {
        if (!sendButton(BUTTONS[i].name, BUTTONS[i].code)) break;
        if (i + 1 < sizeof(BUTTONS) / sizeof(BUTTONS[0])) delay(1000);
      }
      Serial.println("TX sequence ended");
      return;
    }
    for (const auto &button : BUTTONS) {
      if (!strcmp(arg, button.name)) { sendButton(button.name, button.code); return; }
    }
    Serial.println("ERROR: unknown button");
    return;
  }
  if (!strcmp(cmd, "agc2") || !strcmp(cmd, "agc0")) {
    if (capturing) { Serial.println("ERROR: stop capture first"); return; }
    char *end = nullptr;
    const long value = arg ? strtol(arg, &end, 16) : -1;
    if (!arg || end == arg || *end != '\0' ||
        (!strcmp(cmd, "agc2") ? (value < 3 || value > 7) : (value != 0x91 && value != 0x92))) {
      Serial.println("ERROR: agc2 3..7 | agc0 91|92 (hex)"); return;
    }
    if (!strcmp(cmd, "agc2")) wantedAgc2 = (uint8_t)value;
    else wantedAgc0 = (uint8_t)value;
    hasCapture = false;
    radioReady = programRx();
    if (radioReady) Serial.printf("AGC agc2=0x%02X agc0=0x%02X\n", wantedAgc2, wantedAgc0);
    else Serial.println("ERROR: RX reconfiguration failed; reset ESP");
    return;
  }
  if (!strcmp(cmd, "freq") || !strcmp(cmd, "txfreq") || !strcmp(cmd, "bw") || !strcmp(cmd, "rate")) {
    if (capturing) { Serial.println("ERROR: stop capture first"); return; }
    double value;
    if (!parseDouble(arg, value)) { Serial.println("ERROR: numeric argument required"); return; }
    if (!strcmp(cmd, "txfreq")) {
      if (value < 433.05 || value > 434.79) { Serial.println("ERROR: txfreq range 433.05..434.79 MHz"); return; }
      wantedTxFreqMHz = value;
      Serial.printf("TX frequency set to %.6f MHz (programmed; verify on air)\n", value);
      return;
    } else if (!strcmp(cmd, "freq")) {
      if (value < 433.05 || value > 434.79) { Serial.println("ERROR: freq range 433.05..434.79 MHz"); return; }
      wantedFreqMHz = value;
    } else if (!strcmp(cmd, "bw")) {
      if (value < 58 || value > 812) { Serial.println("ERROR: bw range 58..812 kHz"); return; }
      wantedBwKHz = value;
    } else {
      if (value < 0.6 || value > 20) { Serial.println("ERROR: rate range 0.6..20 kBaud"); return; }
      wantedRateKBaud = value;
    }
    hasCapture = false;
    radioReady = programRx();
    if (radioReady) printConfig(); else Serial.println("ERROR: RX reconfiguration failed; reset ESP");
    return;
  }
  Serial.println("ERROR: unknown command; type help");
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("CC1101 raw RX and manual FIFO OOK TX test");
  pinMode(PIN_CSN, OUTPUT);
  deselectRadio();
  pinMode(PIN_MISO, INPUT);
  pinMode(PIN_GDO0, INPUT);
  pinMode(PIN_GDO2, INPUT);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CSN);
  if (!resetRadio()) { Serial.println("ERROR: CC1101 SPI/reset failed; check wiring"); return; }
  uint8_t part = 0, version = 0;
  if (!readReg(PARTNUM, part, true) || !readReg(VERSION, version, true) ||
      part != 0x00 || version == 0x00 || version == 0xFF) {
    Serial.printf("ERROR: CC1101 identification PARTNUM=0x%02X VERSION=0x%02X\n", part, version);
    return;
  }
  Serial.printf("PARTNUM=0x%02X VERSION=0x%02X\n", part, version);
  radioReady = programRx();
  if (!radioReady) { Serial.println("ERROR: RX initialization failed; check module/crystal"); return; }
  printConfig();
  attachInterrupt(digitalPinToInterrupt(dataPin), onDataEdge, CHANGE);
  Serial.println("READY; type help");
}

void loop() {
  if (!radioReady) { delay(1000); return; }
  if (capturing) {
    if ((uint32_t)(micros() - captureStartUs) >= windowMs * 1000UL)
      stopCapture("timeout");
  } else if (overflowed && !hasCapture) {
    stopCapture("overflow");
  }
  if (!capturing && rssiMonitor && (uint32_t)(millis() - lastRssiMs) >= 250) {
    lastRssiMs = millis();
    printRssi();
  }
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (commandLength) {
        commandLine[commandLength] = '\0';
        processCommand(commandLine);
        commandLength = 0;
      }
    } else if (commandLength < sizeof(commandLine) - 1) {
      commandLine[commandLength++] = c;
    } else {
      commandLength = 0;
      Serial.println("ERROR: command too long");
    }
  }
}
