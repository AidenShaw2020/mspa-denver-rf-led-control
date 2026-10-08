#include "mspa_led_rf.h"

#include <cmath>
#include <cstring>

#include "esphome/core/log.h"

namespace esphome::mspa_led_rf {

static const char *const TAG = "mspa_led_rf";
MspaLedRfComponent *MspaLedRfComponent::instance_ = nullptr;

namespace {
constexpr int PIN_GDO0 = 17, PIN_CSN = 5, PIN_SCK = 18;
constexpr int PIN_MOSI = 23, PIN_MISO = 19, PIN_GDO2 = 16;
constexpr uint32_t XOSC_HZ = 26000000UL, SPI_HZ = 4000000UL;
constexpr uint8_t IOCFG2 = 0x00, IOCFG1 = 0x01, IOCFG0 = 0x02;
constexpr uint8_t PKTCTRL1 = 0x07, PKTCTRL0 = 0x08;
constexpr uint8_t FREQ2 = 0x0D, FREQ1 = 0x0E, FREQ0 = 0x0F;
constexpr uint8_t MDMCFG4 = 0x10, MDMCFG3 = 0x11, MDMCFG2 = 0x12;
constexpr uint8_t MCSM1 = 0x17, MCSM0 = 0x18, FOCCFG = 0x19, FREND0 = 0x22;
constexpr uint8_t AGCCTRL2 = 0x1B, AGCCTRL1 = 0x1C, AGCCTRL0 = 0x1D;
constexpr uint8_t PARTNUM = 0x30, VERSION = 0x31, MARCSTATE = 0x35, TXBYTES = 0x3A;
constexpr uint8_t SRES = 0x30, SRX = 0x34, STX = 0x35, SIDLE = 0x36, SFTX = 0x3B;
constexpr uint8_t PATABLE = 0x3E, TXFIFO = 0x3F;

constexpr struct { const char *name; uint8_t code; } BUTTONS[] = {
    {"power", 1}, {"mode_plus", 5}, {"speed_minus", 7}, {"demo", 8},
    {"speed_plus", 9}, {"color_plus", 10}, {"mode_minus", 11},
    {"bright_plus", 12}, {"color_minus", 13}, {"bright_minus", 15},
    {"white", 14}, {"red", 16}, {"green", 17}, {"blue", 18},
    {"yellow", 19}, {"cyan", 20}, {"pink", 21},
};

const char *name_for_code(uint8_t code) {
  for (const auto &button : BUTTONS)
    if (button.code == code) return button.name;
  return nullptr;
}

uint32_t frequency_word(double mhz) {
  return static_cast<uint32_t>(llround(mhz * 1000000.0 * 65536.0 / XOSC_HZ));
}

uint8_t nearest_bw(double khz) {
  double best_error = 1e30;
  uint8_t best = 0;
  for (uint8_t e = 0; e < 4; ++e)
    for (uint8_t m = 0; m < 4; ++m) {
      const double actual = double(XOSC_HZ) / (8.0 * (4 + m) * (1 << e)) / 1000.0;
      const double error = fabs(actual - khz);
      if (error < best_error) { best_error = error; best = (e << 6) | (m << 4); }
    }
  return best;
}

uint16_t nearest_rate(double kbaud) {
  double best_error = 1e30;
  uint16_t best = 0;
  for (uint8_t e = 0; e < 16; ++e)
    for (uint16_t m = 0; m < 256; ++m) {
      const double actual = (256.0 + m) * (1UL << e) * XOSC_HZ / 268435456.0 / 1000.0;
      const double error = fabs(actual - kbaud);
      if (error < best_error) { best_error = error; best = (uint16_t(e) << 8) | m; }
    }
  return best;
}

struct TxBits {
  uint8_t data[2048]{};
  size_t bits{0};
  bool overflow{false};
  void append(bool high, uint32_t us) {
    const uint32_t chips = (us + 12) / 25;
    for (uint32_t n = 0; n < chips; ++n) {
      if (bits >= sizeof(data) * 8) { overflow = true; return; }
      if (high) data[bits / 8] |= uint8_t(0x80 >> (bits % 8));
      ++bits;
    }
  }
  size_t bytes() const { return (bits + 7) / 8; }
};

void append_period(TxBits &wave, uint8_t command) {
  static constexpr char PREFIX[] = "SLSLLSLLSLSSLSSLSSSS";
  wave.append(true, 475);
  wave.append(false, 12125);
  for (uint8_t i = 0; i < 24; ++i) {
    const bool long_mark = i < 19 ? PREFIX[i + 1] == 'L' :
                                     (command & (1U << (23 - i))) != 0;
    const uint32_t on_us = long_mark ? 1275 : 475;
    wave.append(true, on_us);
    wave.append(false, (i == 23 ? 1700 : 1600) - on_us);
  }
}
}  // namespace

void IRAM_ATTR MspaLedRfComponent::on_edge_() {
  auto *self = instance_;
  if (self == nullptr) return;
  const uint16_t next = (self->head_ + 1) & RING_MASK;
  if (next == self->tail_) { self->ring_overflow_ = true; return; }
  self->ring_[self->head_].at = micros();
  self->ring_[self->head_].level = digitalRead(PIN_MISO);
  self->head_ = next;
}

bool MspaLedRfComponent::select_() {
  digitalWrite(PIN_CSN, LOW);
  const uint32_t started = micros();
  while (digitalRead(PIN_MISO) == HIGH) {
    if (uint32_t(micros() - started) > 5000) { deselect_(); return false; }
  }
  return true;
}
void MspaLedRfComponent::deselect_() { digitalWrite(PIN_CSN, HIGH); }
bool MspaLedRfComponent::strobe_(uint8_t command) {
  SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
  if (!select_()) { SPI.endTransaction(); return false; }
  SPI.transfer(command);
  deselect_(); SPI.endTransaction(); return true;
}
bool MspaLedRfComponent::write_reg_(uint8_t address, uint8_t value) {
  SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
  if (!select_()) { SPI.endTransaction(); return false; }
  SPI.transfer(address); SPI.transfer(value);
  deselect_(); SPI.endTransaction(); return true;
}
bool MspaLedRfComponent::read_reg_(uint8_t address, uint8_t &value, bool status) {
  SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
  if (!select_()) { SPI.endTransaction(); return false; }
  SPI.transfer(address | (status ? 0xC0 : 0x80));
  value = SPI.transfer(0);
  deselect_(); SPI.endTransaction(); return true;
}
bool MspaLedRfComponent::reset_radio_() {
  deselect_(); delayMicroseconds(10);
  digitalWrite(PIN_CSN, LOW); delayMicroseconds(10);
  deselect_(); delayMicroseconds(50);
  SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
  if (!select_()) { SPI.endTransaction(); return false; }
  SPI.transfer(SRES);
  const uint32_t started = micros();
  while (digitalRead(PIN_MISO) == HIGH) {
    if (uint32_t(micros() - started) > 10000) {
      deselect_(); SPI.endTransaction(); return false;
    }
  }
  deselect_(); SPI.endTransaction(); return true;
}

bool MspaLedRfComponent::program_rx_() {
  const uint32_t fw = frequency_word(rx_frequency_mhz_);
  const uint16_t rate = nearest_rate(4.8);
  if (!strobe_(SIDLE)) return false;
  delay(2);
  // SO/GDO1 (shared with SPI MISO) carries asynchronous OOK data in RX.
  if (!write_reg_(IOCFG0, 0x2F) || !write_reg_(IOCFG1, 0x0D) ||
      !write_reg_(IOCFG2, 0x0E) || !write_reg_(PKTCTRL1, 0) ||
      !write_reg_(PKTCTRL0, 0x30) || !write_reg_(FREQ2, fw >> 16) ||
      !write_reg_(FREQ1, fw >> 8) || !write_reg_(FREQ0, fw) ||
      !write_reg_(MDMCFG4, nearest_bw(101.6) | (rate >> 8)) ||
      !write_reg_(MDMCFG3, rate) || !write_reg_(MDMCFG2, 0x30) ||
      !write_reg_(MCSM0, 0x18) || !write_reg_(FOCCFG, 0x16) ||
      !write_reg_(AGCCTRL2, 0x04) || !write_reg_(AGCCTRL1, 0x00) ||
      !write_reg_(AGCCTRL0, 0x92) || !strobe_(SRX)) return false;
  delay(3);
  uint8_t state = 0;
  return read_reg_(MARCSTATE, state, true) && (state & 0x1F) == 0x0D;
}

bool MspaLedRfComponent::write_pa_() {
  SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
  if (!select_()) { SPI.endTransaction(); return false; }
  SPI.transfer(PATABLE | 0x40);
  SPI.transfer(0x00); SPI.transfer(0x34);  // off / about -10 dBm
  deselect_(); SPI.endTransaction(); return true;
}
bool MspaLedRfComponent::write_fifo_(const uint8_t *data, size_t count) {
  SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
  if (!select_()) { SPI.endTransaction(); return false; }
  SPI.transfer(TXFIFO | 0x40);
  for (size_t i = 0; i < count; ++i) SPI.transfer(data[i]);
  deselect_(); SPI.endTransaction(); return true;
}

void MspaLedRfComponent::setup() {
  pinMode(PIN_CSN, OUTPUT); deselect_();
  pinMode(PIN_MISO, INPUT); pinMode(PIN_GDO0, INPUT); pinMode(PIN_GDO2, INPUT);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CSN);
  uint8_t part = 0, version = 0;
  if (!reset_radio_() || !read_reg_(PARTNUM, part, true) ||
      !read_reg_(VERSION, version, true) || part != 0 ||
      version == 0 || version == 0xFF || !program_rx_()) {
    ESP_LOGE(TAG, "CC1101 setup failed; check 3.3V, wiring and crystal");
    mark_failed(); return;
  }
  instance_ = this;
  ready_ = true;
  attachInterrupt(digitalPinToInterrupt(PIN_MISO), on_edge_, CHANGE);
  ESP_LOGI(TAG, "CC1101 ready, RX %.6f MHz, TX programmed %.6f MHz, GDO1 GPIO19",
           rx_frequency_mhz_, tx_frequency_mhz_);
}

void MspaLedRfComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "MSpa Denver LED RF");
  ESP_LOGCONFIG(TAG, "  RX %.6f MHz; programmed TX %.6f MHz", rx_frequency_mhz_, tx_frequency_mhz_);
  ESP_LOGCONFIG(TAG, "  TX power about -10 dBm; no automatic transmission");
}

void MspaLedRfComponent::reset_decoder_() { mark_count_ = 0; }
void MspaLedRfComponent::consume_edge_(uint32_t at, bool level) {
  if (level) {
    rise_us_ = at;
    have_rise_ = true;
  } else if (have_rise_) {
    have_rise_ = false;
    consume_mark_(rise_us_, at);
    previous_fall_us_ = at;
  }
}
void MspaLedRfComponent::consume_mark_(uint32_t rise, uint32_t fall) {
  const uint32_t width = fall - rise;
  const char symbol = width >= 250 && width <= 750 ? 'S' :
                      width >= 1000 && width <= 1500 ? 'L' : '?';
  const uint32_t start_delta = rise - previous_rise_us_;
  const uint32_t idle_gap = rise - previous_fall_us_;
  previous_rise_us_ = rise;
  const char prior_symbol = previous_symbol_;
  previous_symbol_ = symbol;
  if (symbol == '?') { reset_decoder_(); return; }
  if (mark_count_ == 0) {
    if (prior_symbol == 'S' && start_delta >= 12000 && start_delta <= 13200) {
      marks_[0] = 'S'; marks_[1] = symbol; mark_count_ = 2;
    } else if (symbol == 'S' && (previous_fall_us_ == 0 || idle_gap >= 8000)) {
      marks_[0] = 'S'; mark_count_ = 1;
    }
    return;
  }
  const bool timing_ok = mark_count_ == 1 ? start_delta >= 12000 && start_delta <= 13200 :
                                           start_delta >= 1400 && start_delta <= 1800;
  if (!timing_ok) {
    reset_decoder_();
    if (prior_symbol == 'S' && start_delta >= 12000 && start_delta <= 13200) {
      marks_[0] = 'S'; marks_[1] = symbol; mark_count_ = 2;
    } else if (symbol == 'S' && idle_gap >= 8000) {
      marks_[0] = 'S'; mark_count_ = 1;
    }
    return;
  }
  marks_[mark_count_++] = symbol;
  if (mark_count_ == 25) { finish_frame_(); reset_decoder_(); }
}
void MspaLedRfComponent::finish_frame_() {
  static constexpr char PREFIX[] = "SLSLLSLLSLSSLSSLSSSS";
  if (memcmp(marks_, PREFIX, 20) != 0) return;
  uint8_t command = 0;
  for (uint8_t i = 20; i < 25; ++i)
    command = (command << 1) | (marks_[i] == 'L' ? 1 : 0);
  const char *name = name_for_code(command);
  if (name == nullptr) return;
  const uint32_t now = millis();
  if (command == last_command_ && uint32_t(now - last_command_ms_) < 350) return;
  last_command_ = command;
  last_command_ms_ = now;
  ESP_LOGD(TAG, "RX remote: %s (command %u)", name, command);
  if (remote_event_) remote_event_->trigger(name);
  if (last_button_) last_button_->publish_state(name);
}

void MspaLedRfComponent::loop() {
  if (!ready_) return;
  if (ring_overflow_) {
    noInterrupts(); tail_ = head_; ring_overflow_ = false; interrupts();
    reset_decoder_(); ESP_LOGW(TAG, "RX edge buffer overflow");
  }
  uint16_t processed = 0;
  while (tail_ != head_ && processed++ < 256) {
    const Edge edge{ring_[tail_].at, ring_[tail_].level};
    tail_ = (tail_ + 1) & RING_MASK;
    consume_edge_(edge.at, edge.level);
  }
}

bool MspaLedRfComponent::send_button(const std::string &name) {
  if (!ready_) { ESP_LOGW(TAG, "TX rejected: radio not ready"); return false; }
  int command = -1;
  for (const auto &button : BUTTONS)
    if (name == button.name) { command = button.code; break; }
  if (command < 0) { ESP_LOGW(TAG, "Unknown button: %s", name.c_str()); return false; }
  TxBits wave;
  wave.append(false, 5000);
  for (uint8_t i = 0; i < 4; ++i) append_period(wave, command);
  wave.append(false, 20000);
  if (wave.overflow) { ESP_LOGE(TAG, "TX waveform overflow"); return false; }

  detachInterrupt(digitalPinToInterrupt(PIN_MISO));
  tail_ = head_;
  reset_decoder_();
  bool sent = false;
  const uint32_t fw = frequency_word(tx_frequency_mhz_);
  const uint16_t rate = nearest_rate(40.0);
  do {
    if (!strobe_(SIDLE) || !write_reg_(IOCFG0, 0x2E) || !write_pa_() ||
        !write_reg_(FREND0, 0x11) || !write_reg_(FREQ2, fw >> 16) ||
        !write_reg_(FREQ1, fw >> 8) || !write_reg_(FREQ0, fw) ||
        !write_reg_(MDMCFG4, nearest_bw(101.6) | (rate >> 8)) ||
        !write_reg_(MDMCFG3, rate) || !write_reg_(MDMCFG2, 0x30) ||
        !write_reg_(PKTCTRL0, 0x02) || !write_reg_(MCSM1, 0x00) ||
        !strobe_(SFTX)) break;
    const size_t initial = wave.bytes() < 64 ? wave.bytes() : 64;
    if (!write_fifo_(wave.data, initial) || !strobe_(STX)) break;
    size_t queued = initial;
    const uint32_t deadline = millis() + 1500;
    while (int32_t(millis() - deadline) < 0) {
      uint8_t occupied = 0;
      if (!read_reg_(TXBYTES, occupied, true)) break;
      if (occupied & 0x80) break;  // FIFO underflow.
      occupied &= 0x7F;
      if (queued < wave.bytes() && occupied <= 32) {
        const size_t count = wave.bytes() - queued < 32 ? wave.bytes() - queued : 32;
        if (!write_fifo_(wave.data + queued, count)) break;
        queued += count;
      }
      if (queued == wave.bytes() && occupied == 0) { sent = true; break; }
      delay(1);
    }
  } while (false);
  ready_ = strobe_(SIDLE) && reset_radio_() && program_rx_();
  tail_ = head_;
  reset_decoder_();
  if (ready_) attachInterrupt(digitalPinToInterrupt(PIN_MISO), on_edge_, CHANGE);
  else mark_failed();
  if (!sent || !ready_) {
    ESP_LOGE(TAG, "TX %s failed (queued=%d, RX restored=%d)", name.c_str(), sent, ready_);
    return false;
  }
  ESP_LOGI(TAG, "TX %s: four OOK periods queued; RX restored; LED response unverified", name.c_str());
  if (last_tx_button_) last_tx_button_->publish_state(name);
  return true;
}

}  // namespace esphome::mspa_led_rf
