#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <stdint.h>
#include <string>

#include "esphome/components/event/event.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/component.h"

namespace esphome::mspa_led_rf {

class MspaLedRfComponent : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  void set_rx_frequency(double mhz) { rx_frequency_mhz_ = mhz; }
  void set_tx_frequency(double mhz) { tx_frequency_mhz_ = mhz; }
  void set_remote_event(event::Event *value) { remote_event_ = value; }
  void set_last_button(text_sensor::TextSensor *value) { last_button_ = value; }
  void set_last_tx_button(text_sensor::TextSensor *value) { last_tx_button_ = value; }
  bool send_button(const std::string &name);

 protected:
  struct Edge { uint32_t at; uint8_t level; };
  static constexpr uint16_t RING_SIZE = 1024;
  static constexpr uint16_t RING_MASK = RING_SIZE - 1;
  static MspaLedRfComponent *instance_;
  static void IRAM_ATTR on_edge_();
  volatile Edge ring_[RING_SIZE]{};
  volatile uint16_t head_{0};
  volatile uint16_t tail_{0};
  volatile bool ring_overflow_{false};
  bool ready_{false};
  double rx_frequency_mhz_{433.973};
  double tx_frequency_mhz_{434.0254};
  event::Event *remote_event_{nullptr};
  text_sensor::TextSensor *last_button_{nullptr};
  text_sensor::TextSensor *last_tx_button_{nullptr};

  uint32_t rise_us_{0};
  uint32_t previous_rise_us_{0};
  uint32_t previous_fall_us_{0};
  char previous_symbol_{'?'};
  bool have_rise_{false};
  uint8_t mark_count_{0};
  char marks_[26]{};
  uint8_t last_command_{0};
  uint32_t last_command_ms_{0};

  bool select_();
  void deselect_();
  bool strobe_(uint8_t command);
  bool write_reg_(uint8_t address, uint8_t value);
  bool read_reg_(uint8_t address, uint8_t &value, bool status = false);
  bool reset_radio_();
  bool program_rx_();
  bool write_pa_();
  bool write_fifo_(const uint8_t *data, size_t count);
  void consume_edge_(uint32_t at, bool level);
  void consume_mark_(uint32_t rise, uint32_t fall);
  void finish_frame_();
  void reset_decoder_();
};

}  // namespace esphome::mspa_led_rf
