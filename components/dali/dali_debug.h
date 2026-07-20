#pragma once

#ifdef USE_DALI_DEBUG

#include <cstdint>
#include <string>

#include "esphome/core/component.h"
#include "esphome/core/entity_base.h"
#include "esphome/core/helpers.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/button/button.h"
#include "esphome/components/number/number.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

#include "dali.h"
#include "dali_phy.h"

namespace esphome {
namespace dali {

/// Decode a 16-bit DALI forward frame into a human-readable string.
std::string decode_forward_frame(uint8_t address, uint8_t data);

/// Decode a backward frame / PHY result into a short string.
std::string decode_backward_frame(uint8_t value, uint8_t phy_result);

const char *phy_result_name(uint8_t result);

enum class DaliDebugAction : uint8_t {
  QUERY_PRESENT = 0,
  QUERY_STATUS,
  QUERY_ACTUAL_LEVEL,
  RECALL_MAX,
  RECALL_MIN,
  OFF,
  DAPC_100,
  DAPC_50,
  DAPC_0,
  IDENTIFY,
  SCAN_SHORT,
  DUMP_STATUS,
  BLINK,
  COMPARE_PROBE,
  RESET_BUS,
  TERMINATE,
};

class DaliBusComponent;

class DaliDebugButton : public button::Button {
 public:
  void set_parent(DaliBusComponent *parent) { parent_ = parent; }
  void set_action(DaliDebugAction action) { action_ = action; }

 protected:
  void press_action() override;

  DaliBusComponent *parent_{nullptr};
  DaliDebugAction action_{DaliDebugAction::QUERY_PRESENT};
};

class DaliDebugAddressNumber : public number::Number {
 public:
  void set_parent(DaliBusComponent *parent) { parent_ = parent; }

 protected:
  void control(float value) override;

  DaliBusComponent *parent_{nullptr};
};

/// Runtime debug entities + traffic sniffer for a DALI bus.
class DaliDebugHub {
 public:
  void setup(DaliBusComponent *bus);
  void loop();

  void on_tx(uint8_t address, uint8_t data, uint8_t phy_result);
  void on_rx(uint8_t value, uint8_t phy_result);

  short_addr_t target_addr() const { return target_addr_; }
  void set_target_addr(short_addr_t addr) { target_addr_ = addr; }

  void run_action(DaliDebugAction action);

 private:
  void publish_present_(bool present, bool got_reply);
  void append_log_(const char *line);
  void refresh_bus_level_();

  DaliBusComponent *bus_{nullptr};
  short_addr_t target_addr_{ADDR_BROADCAST};

  text_sensor::TextSensor *tx_hex_{nullptr};
  text_sensor::TextSensor *tx_decoded_{nullptr};
  text_sensor::TextSensor *rx_hex_{nullptr};
  text_sensor::TextSensor *rx_decoded_{nullptr};
  text_sensor::TextSensor *bus_log_{nullptr};
  text_sensor::TextSensor *scan_result_{nullptr};

  sensor::Sensor *tx_count_{nullptr};
  sensor::Sensor *rx_reply_count_{nullptr};
  sensor::Sensor *rx_noreply_count_{nullptr};
  sensor::Sensor *tx_error_count_{nullptr};
  sensor::Sensor *last_rx_value_{nullptr};
  sensor::Sensor *last_tx_result_{nullptr};
  sensor::Sensor *devices_found_{nullptr};
  sensor::Sensor *last_actual_level_{nullptr};

  binary_sensor::BinarySensor *gear_present_{nullptr};
  binary_sensor::BinarySensor *rx_idle_{nullptr};
  binary_sensor::BinarySensor *last_got_reply_{nullptr};

  DaliDebugAddressNumber *target_number_{nullptr};

  uint32_t tx_count_val_{0};
  uint32_t rx_reply_count_val_{0};
  uint32_t rx_noreply_count_val_{0};
  uint32_t tx_error_count_val_{0};
  uint32_t last_bus_level_ms_{0};
  std::string bus_log_val_;
};

}  // namespace dali
}  // namespace esphome

#endif  // USE_DALI_DEBUG
