#include "esphome/core/defines.h"
#include "dali_debug.h"

#ifdef USE_DALI_DEBUG

#include "esphome_dali.h"

#include "esphome/core/application.h"

#include <cstdio>
#include <cstring>

namespace esphome {
namespace dali {

namespace {

const char *command_name(uint8_t cmd) {
  switch (static_cast<DaliCommand>(cmd)) {
    case DaliCommand::OFF: return "OFF";
    case DaliCommand::UP: return "UP";
    case DaliCommand::DOWN: return "DOWN";
    case DaliCommand::STEP_UP: return "STEP_UP";
    case DaliCommand::STEP_DOWN: return "STEP_DOWN";
    case DaliCommand::RECALL_MAX_LEVEL: return "RECALL_MAX";
    case DaliCommand::RECALL_MIN_LEVEL: return "RECALL_MIN";
    case DaliCommand::STEP_DOWN_AND_OFF: return "STEP_DOWN_AND_OFF";
    case DaliCommand::ON_AND_STEP_UP: return "ON_AND_STEP_UP";
    case DaliCommand::ENABLE_DAPC_SEQUENCE: return "ENABLE_DAPC_SEQ";
    case DaliCommand::GO_TO_LAST_ACTIVE_LEVEL: return "GO_TO_LAST_ACTIVE";
    case DaliCommand::CONTINUOUS_UP: return "CONTINUOUS_UP";
    case DaliCommand::CONTINUOUS_DOWN: return "CONTINUOUS_DOWN";
    case DaliCommand::DALI_RESET: return "RESET";
    case DaliCommand::STORE_ACTUAL_LEVEL_IN_DTR0: return "STORE_LEVEL_DTR0";
    case DaliCommand::SAVE_PERSISTENT_VARIABLES: return "SAVE_PERSISTENT";
    case DaliCommand::IDENTIFY_DEVICE: return "IDENTIFY";
    case DaliCommand::SET_MAX_LEVEL_DTR0: return "SET_MAX_DTR0";
    case DaliCommand::SET_MIN_LEVEL_DTR0: return "SET_MIN_DTR0";
    case DaliCommand::SET_POWER_ON_LEVEL_DTR0: return "SET_POWER_ON_DTR0";
    case DaliCommand::SET_FADE_TIME_DTR0: return "SET_FADE_TIME_DTR0";
    case DaliCommand::SET_FADE_RATE_DTR0: return "SET_FADE_RATE_DTR0";
    case DaliCommand::SET_SHORT_ADDRESS_DTR0: return "SET_SHORT_ADDR_DTR0";
    case DaliCommand::QUERY_STATUS: return "QUERY_STATUS";
    case DaliCommand::QUERY_CONTROL_GEAR_PRESENT: return "QUERY_GEAR_PRESENT";
    case DaliCommand::QUERY_LAMP_FAILURE: return "QUERY_LAMP_FAILURE";
    case DaliCommand::QUERY_LAMP_POWER_ON: return "QUERY_LAMP_POWER_ON";
    case DaliCommand::QUERY_LIMIT_ERROR: return "QUERY_LIMIT_ERROR";
    case DaliCommand::QUERY_RESET_STATE: return "QUERY_RESET_STATE";
    case DaliCommand::QUERY_MISSING_SHORT_ADDRESS: return "QUERY_MISSING_SHORT_ADDR";
    case DaliCommand::QUERY_VERSION_NUMBER: return "QUERY_VERSION";
    case DaliCommand::QUERY_CONTENT_DTR0: return "QUERY_DTR0";
    case DaliCommand::QUERY_DEVICE_TYPE: return "QUERY_DEVICE_TYPE";
    case DaliCommand::QUERY_PHYSICAL_MINIMUM: return "QUERY_PHYS_MIN";
    case DaliCommand::QUERY_POWER_FAILURE: return "QUERY_POWER_FAILURE";
    case DaliCommand::QUERY_CONTENT_DTR1: return "QUERY_DTR1";
    case DaliCommand::QUERY_CONTENT_DTR2: return "QUERY_DTR2";
    case DaliCommand::QUERY_OPERATING_MODE: return "QUERY_OP_MODE";
    case DaliCommand::QUERY_LIGHT_SOURCE_TYPE: return "QUERY_LIGHT_TYPE";
    case DaliCommand::QUERY_ACTUAL_LEVEL: return "QUERY_ACTUAL_LEVEL";
    case DaliCommand::QUERY_MAX_LEVEL: return "QUERY_MAX_LEVEL";
    case DaliCommand::QUERY_MIN_LEVEL: return "QUERY_MIN_LEVEL";
    case DaliCommand::QUERY_POWER_ON_LEVEL: return "QUERY_POWER_ON_LEVEL";
    case DaliCommand::QUERY_SYSTEM_FAILURE_LEVEL: return "QUERY_SYS_FAIL_LEVEL";
    case DaliCommand::QUERY_FADE_TIME_FADE_RATE: return "QUERY_FADE_TIME_RATE";
    case DaliCommand::QUERY_RANDOM_ADDRESS_H: return "QUERY_RANDOM_H";
    case DaliCommand::QUERY_RANDOM_ADDRESS_M: return "QUERY_RANDOM_M";
    case DaliCommand::QUERY_RANDOM_ADDRESS_L: return "QUERY_RANDOM_L";
    case DaliCommand::QUERY_GROUPS_0_7: return "QUERY_GROUPS_0_7";
    case DaliCommand::QUERY_GROUPS_8_15: return "QUERY_GROUPS_8_15";
    default:
      break;
  }
  if (cmd >= 0x10 && cmd <= 0x1F)
    return "GO_TO_SCENE";
  if (cmd >= 0x40 && cmd <= 0x4F)
    return "SET_SCENE";
  if (cmd >= 0x60 && cmd <= 0x6F)
    return "ADD_TO_GROUP";
  if (cmd >= 0x70 && cmd <= 0x7F)
    return "REMOVE_FROM_GROUP";
  return nullptr;
}

const char *special_command_name(uint8_t address) {
  switch (static_cast<DaliSpecialCommand>(address)) {
    case DaliSpecialCommand::TERMINATE: return "TERMINATE";
    case DaliSpecialCommand::DTR0_DATA: return "DTR0";
    case DaliSpecialCommand::INITIALISE: return "INITIALISE";
    case DaliSpecialCommand::RANDOMIZE: return "RANDOMIZE";
    case DaliSpecialCommand::COMPARE: return "COMPARE";
    case DaliSpecialCommand::WITHDRAW: return "WITHDRAW";
    case DaliSpecialCommand::PING: return "PING";
    case DaliSpecialCommand::SEARCH_ADDRH: return "SEARCHADDR_H";
    case DaliSpecialCommand::SEARCH_ADDRM: return "SEARCHADDR_M";
    case DaliSpecialCommand::SEARCH_ADDRL: return "SEARCHADDR_L";
    case DaliSpecialCommand::PROGRAM_SHORT_ADDRESS: return "PROGRAM_SHORT_ADDR";
    case DaliSpecialCommand::VERIFY_SHORT_ADDRESS: return "VERIFY_SHORT_ADDR";
    case DaliSpecialCommand::QUERY_SHORT_ADDRESS: return "QUERY_SHORT_ADDR";
    case DaliSpecialCommand::ENABLE_DEVICE_TYPE: return "ENABLE_DEVICE_TYPE";
    case DaliSpecialCommand::DTR1_DATA: return "DTR1";
    case DaliSpecialCommand::DTR2_DATA: return "DTR2";
    case DaliSpecialCommand::WRITE_MEMORY_LOCATION: return "WRITE_MEMORY";
    case DaliSpecialCommand::WRITE_MEMORY_LOCATION_NO_REPLY: return "WRITE_MEMORY_NR";
    default:
      return nullptr;
  }
}

bool is_special_address(uint8_t address) {
  // Special commands use odd address bytes in 0xA1..0xC9 range (IEC 62386-102).
  return (address & 0x01) && address >= 0xA1 && address <= 0xC9;
}

}  // namespace

const char *phy_result_name(uint8_t result) {
  switch (result) {
    case DALI_PHY_OK: return "OK";
    case DALI_PHY_RESULT_BUS_NOT_IDLE: return "BUS_NOT_IDLE";
    case DALI_PHY_RESULT_FRAME_TOO_LONG: return "FRAME_TOO_LONG";
    case DALI_PHY_RESULT_COLLISION: return "COLLISION";
    case DALI_PHY_RESULT_TRANSMITTING: return "TRANSMITTING";
    case DALI_PHY_RESULT_TIMEOUT: return "TIMEOUT";
    case DALI_PHY_RESULT_NO_REPLY: return "NO_REPLY";
    case DALI_PHY_RESULT_INVALID_REPLY: return "INVALID_REPLY";
    default: return "UNKNOWN";
  }
}

std::string decode_forward_frame(uint8_t address, uint8_t data) {
  char buf[96];

  if (is_special_address(address)) {
    const char *name = special_command_name(address);
    if (name != nullptr) {
      snprintf(buf, sizeof(buf), "Special %s data=0x%02X (%u)", name, data, data);
    } else {
      snprintf(buf, sizeof(buf), "Special 0x%02X data=0x%02X", address, data);
    }
    return buf;
  }

  const bool is_command = (address & 0x01) != 0;
  const uint8_t addr7 = address >> 1;
  const char *dest;
  char dest_buf[24];
  if (addr7 == ADDR_BROADCAST) {
    dest = "Broadcast";
  } else if ((addr7 & ADDR_GROUP_MASK) == ADDR_GROUP) {
    snprintf(dest_buf, sizeof(dest_buf), "Group %u", addr7 & 0x0F);
    dest = dest_buf;
  } else {
    snprintf(dest_buf, sizeof(dest_buf), "Short %u", addr7);
    dest = dest_buf;
  }

  if (!is_command) {
    if (data == 0xFF) {
      snprintf(buf, sizeof(buf), "%s DAPC STOP", dest);
    } else if (data == 0x00) {
      snprintf(buf, sizeof(buf), "%s DAPC OFF/MASK", dest);
    } else {
      snprintf(buf, sizeof(buf), "%s DAPC level=%u", dest, data);
    }
    return buf;
  }

  const char *cname = command_name(data);
  if (cname != nullptr) {
    snprintf(buf, sizeof(buf), "%s CMD %s (0x%02X)", dest, cname, data);
  } else {
    snprintf(buf, sizeof(buf), "%s CMD 0x%02X", dest, data);
  }
  return buf;
}

std::string decode_backward_frame(uint8_t value, uint8_t phy_result) {
  char buf[64];
  if (phy_result == DALI_PHY_OK) {
    if (value == 0xFF) {
      snprintf(buf, sizeof(buf), "YES/FF (0x%02X)", value);
    } else {
      snprintf(buf, sizeof(buf), "DATA 0x%02X (%u)", value, value);
    }
    return buf;
  }
  snprintf(buf, sizeof(buf), "%s", phy_result_name(phy_result));
  return buf;
}

void DaliDebugButton::press_action() {
  if (this->parent_ != nullptr) {
    this->parent_->run_debug_action(this->action_);
  }
}

void DaliDebugAddressNumber::control(float value) {
  if (this->parent_ == nullptr)
    return;
  auto addr = static_cast<short_addr_t>(value);
  if (addr > ADDR_SHORT_MAX && addr != ADDR_BROADCAST)
    addr = ADDR_BROADCAST;
  this->parent_->set_debug_target_addr(addr);
  this->publish_state(addr);
}

namespace {

uint32_t entity_hash_(const char *object_id) {
  return fnv1_hash_object_id(object_id, std::strlen(object_id));
}

uint32_t diag_fields_() {
  return static_cast<uint32_t>(ENTITY_CATEGORY_DIAGNOSTIC) << ENTITY_FIELD_ENTITY_CATEGORY_SHIFT;
}

uint32_t config_fields_() {
  return static_cast<uint32_t>(ENTITY_CATEGORY_CONFIG) << ENTITY_FIELD_ENTITY_CATEGORY_SHIFT;
}

}  // namespace

void DaliDebugHub::setup(DaliBusComponent *bus) {
  this->bus_ = bus;

  auto make_text = [](const char *name, const char *object_id) {
    auto *s = new text_sensor::TextSensor();
    App.register_text_sensor(s, name, entity_hash_(object_id), diag_fields_());
    return s;
  };
  this->tx_hex_ = make_text("DALI TX Hex", "dali_tx_hex");
  this->tx_decoded_ = make_text("DALI TX Decoded", "dali_tx_decoded");
  this->rx_hex_ = make_text("DALI RX Hex", "dali_rx_hex");
  this->rx_decoded_ = make_text("DALI RX Decoded", "dali_rx_decoded");
  this->bus_log_ = make_text("DALI Bus Log", "dali_bus_log");
  this->scan_result_ = make_text("DALI Scan Result", "dali_scan_result");

  auto make_sensor = [](const char *name, const char *object_id) {
    auto *s = new sensor::Sensor();
    s->set_accuracy_decimals(0);
    App.register_sensor(s, name, entity_hash_(object_id), diag_fields_());
    return s;
  };
  this->tx_count_ = make_sensor("DALI TX Count", "dali_tx_count");
  this->rx_reply_count_ = make_sensor("DALI RX Replies", "dali_rx_replies");
  this->rx_noreply_count_ = make_sensor("DALI RX No Reply", "dali_rx_noreply");
  this->tx_error_count_ = make_sensor("DALI TX Errors", "dali_tx_errors");
  this->last_rx_value_ = make_sensor("DALI Last RX Value", "dali_last_rx_value");
  this->last_tx_result_ = make_sensor("DALI Last TX Result", "dali_last_tx_result");
  this->devices_found_ = make_sensor("DALI Devices Found", "dali_devices_found");
  this->last_actual_level_ = make_sensor("DALI Actual Level", "dali_actual_level");

  auto make_bin = [](const char *name, const char *object_id) {
    auto *s = new binary_sensor::BinarySensor();
    App.register_binary_sensor(s, name, entity_hash_(object_id), diag_fields_());
    return s;
  };
  this->gear_present_ = make_bin("DALI Gear Present", "dali_gear_present");
  this->rx_idle_ = make_bin("DALI RX Idle High", "dali_rx_idle");
  this->last_got_reply_ = make_bin("DALI Last Got Reply", "dali_last_got_reply");

  this->target_number_ = new DaliDebugAddressNumber();
  this->target_number_->set_parent(bus);
  this->target_number_->traits.set_min_value(0);
  this->target_number_->traits.set_max_value(127);
  this->target_number_->traits.set_step(1);
  this->target_number_->traits.set_mode(number::NUMBER_MODE_BOX);
  App.register_number(this->target_number_, "DALI Target Address", entity_hash_("dali_target_address"),
                      config_fields_());
  this->target_number_->publish_state(ADDR_BROADCAST);

  struct BtnDef {
    const char *name;
    const char *object_id;
    DaliDebugAction action;
  };
  static const BtnDef BUTTONS[] = {
      {"DALI Query Present", "dali_btn_query_present", DaliDebugAction::QUERY_PRESENT},
      {"DALI Query Status", "dali_btn_query_status", DaliDebugAction::QUERY_STATUS},
      {"DALI Query Actual Level", "dali_btn_query_level", DaliDebugAction::QUERY_ACTUAL_LEVEL},
      {"DALI Recall Max", "dali_btn_recall_max", DaliDebugAction::RECALL_MAX},
      {"DALI Recall Min", "dali_btn_recall_min", DaliDebugAction::RECALL_MIN},
      {"DALI Off", "dali_btn_off", DaliDebugAction::OFF},
      {"DALI DAPC 100%", "dali_btn_dapc_100", DaliDebugAction::DAPC_100},
      {"DALI DAPC 50%", "dali_btn_dapc_50", DaliDebugAction::DAPC_50},
      {"DALI DAPC 0%", "dali_btn_dapc_0", DaliDebugAction::DAPC_0},
      {"DALI Identify", "dali_btn_identify", DaliDebugAction::IDENTIFY},
      {"DALI Scan Short Addrs", "dali_btn_scan", DaliDebugAction::SCAN_SHORT},
      {"DALI Dump Status", "dali_btn_dump", DaliDebugAction::DUMP_STATUS},
      {"DALI Blink", "dali_btn_blink", DaliDebugAction::BLINK},
      {"DALI COMPARE Probe", "dali_btn_compare", DaliDebugAction::COMPARE_PROBE},
      {"DALI Reset Bus", "dali_btn_reset_bus", DaliDebugAction::RESET_BUS},
      {"DALI Terminate Init", "dali_btn_terminate", DaliDebugAction::TERMINATE},
  };

  for (const auto &def : BUTTONS) {
    auto *btn = new DaliDebugButton();
    btn->set_parent(bus);
    btn->set_action(def.action);
    App.register_button(btn, def.name, entity_hash_(def.object_id), diag_fields_());
  }

  this->tx_count_->publish_state(0);
  this->rx_reply_count_->publish_state(0);
  this->rx_noreply_count_->publish_state(0);
  this->tx_error_count_->publish_state(0);
  this->bus_log_->publish_state("(empty)");
  this->scan_result_->publish_state("(not scanned)");
  this->tx_hex_->publish_state("--");
  this->tx_decoded_->publish_state("--");
  this->rx_hex_->publish_state("--");
  this->rx_decoded_->publish_state("--");

  DALI_LOGI("DALI debug entities registered");
}

void DaliDebugHub::loop() {
  uint32_t now = millis();
  if (now - this->last_bus_level_ms_ >= 500) {
    this->last_bus_level_ms_ = now;
    this->refresh_bus_level_();
  }
}

void DaliDebugHub::refresh_bus_level_() {
  if (this->bus_ == nullptr || this->rx_idle_ == nullptr)
    return;
  this->rx_idle_->publish_state(this->bus_->debug_rx_is_high());
}

void DaliDebugHub::append_log_(const char *line) {
  if (this->bus_log_ == nullptr)
    return;
  if (!this->bus_log_val_.empty())
    this->bus_log_val_ += " | ";
  this->bus_log_val_ += line;
  while (this->bus_log_val_.size() > 240) {
    auto pos = this->bus_log_val_.find(" | ");
    if (pos == std::string::npos) {
      this->bus_log_val_.erase(0, this->bus_log_val_.size() - 240);
      break;
    }
    this->bus_log_val_.erase(0, pos + 3);
  }
  this->bus_log_->publish_state(this->bus_log_val_);
}

void DaliDebugHub::on_tx(uint8_t address, uint8_t data, uint8_t phy_result) {
  this->tx_count_val_++;
  if (this->tx_count_ != nullptr)
    this->tx_count_->publish_state(this->tx_count_val_);
  if (this->last_tx_result_ != nullptr)
    this->last_tx_result_->publish_state(phy_result);
  if (phy_result != DALI_PHY_OK) {
    this->tx_error_count_val_++;
    if (this->tx_error_count_ != nullptr)
      this->tx_error_count_->publish_state(this->tx_error_count_val_);
  }

  char hex[16];
  snprintf(hex, sizeof(hex), "%02X %02X", address, data);
  auto decoded = decode_forward_frame(address, data);
  if (this->tx_hex_ != nullptr)
    this->tx_hex_->publish_state(hex);
  if (this->tx_decoded_ != nullptr)
    this->tx_decoded_->publish_state(decoded);

  char log_line[128];
  if (phy_result == DALI_PHY_OK) {
    snprintf(log_line, sizeof(log_line), "TX %s %s", hex, decoded.c_str());
  } else {
    snprintf(log_line, sizeof(log_line), "TX %s %s [%s]", hex, decoded.c_str(), phy_result_name(phy_result));
  }
  this->append_log_(log_line);
  DALI_LOGD("%s", log_line);
}

void DaliDebugHub::on_rx(uint8_t value, uint8_t phy_result) {
  const bool ok = phy_result == DALI_PHY_OK;
  if (ok) {
    this->rx_reply_count_val_++;
    if (this->rx_reply_count_ != nullptr)
      this->rx_reply_count_->publish_state(this->rx_reply_count_val_);
    if (this->last_rx_value_ != nullptr)
      this->last_rx_value_->publish_state(value);
  } else {
    this->rx_noreply_count_val_++;
    if (this->rx_noreply_count_ != nullptr)
      this->rx_noreply_count_->publish_state(this->rx_noreply_count_val_);
  }

  if (this->last_got_reply_ != nullptr)
    this->last_got_reply_->publish_state(ok);

  char hex[8];
  if (ok) {
    snprintf(hex, sizeof(hex), "%02X", value);
  } else {
    snprintf(hex, sizeof(hex), "--");
  }
  auto decoded = decode_backward_frame(value, phy_result);
  if (this->rx_hex_ != nullptr)
    this->rx_hex_->publish_state(hex);
  if (this->rx_decoded_ != nullptr)
    this->rx_decoded_->publish_state(decoded);

  char log_line[64];
  snprintf(log_line, sizeof(log_line), "RX %s", decoded.c_str());
  this->append_log_(log_line);
  DALI_LOGD("%s", log_line);
}

void DaliDebugHub::publish_present_(bool present, bool got_reply) {
  if (this->gear_present_ != nullptr)
    this->gear_present_->publish_state(present);
  if (this->last_got_reply_ != nullptr)
    this->last_got_reply_->publish_state(got_reply);
}

void DaliDebugHub::run_action(DaliDebugAction action) {
  if (this->bus_ == nullptr)
    return;

  auto &dali = this->bus_->dali;
  const short_addr_t addr = this->target_addr_;

  switch (action) {
    case DaliDebugAction::QUERY_PRESENT: {
      uint8_t rv = 0;
      uint8_t st = this->bus_->send_query_debug(addr, DaliCommand::QUERY_CONTROL_GEAR_PRESENT, &rv);
      bool present = (st == DALI_PHY_OK && rv != 0);
      this->publish_present_(present, st == DALI_PHY_OK);
      DALI_LOGI("Debug QUERY_GEAR_PRESENT @%u -> %s (0x%02X, %s)", addr, present ? "YES" : "NO", rv,
                phy_result_name(st));
      break;
    }
    case DaliDebugAction::QUERY_STATUS: {
      uint8_t rv = 0;
      uint8_t st = this->bus_->send_query_debug(addr, DaliCommand::QUERY_STATUS, &rv);
      DALI_LOGI("Debug QUERY_STATUS @%u -> 0x%02X (%s)", addr, rv, phy_result_name(st));
      break;
    }
    case DaliDebugAction::QUERY_ACTUAL_LEVEL: {
      uint8_t rv = 0;
      uint8_t st = this->bus_->send_query_debug(addr, DaliCommand::QUERY_ACTUAL_LEVEL, &rv);
      if (st == DALI_PHY_OK && this->last_actual_level_ != nullptr)
        this->last_actual_level_->publish_state(rv);
      DALI_LOGI("Debug QUERY_ACTUAL_LEVEL @%u -> %u (%s)", addr, rv, phy_result_name(st));
      break;
    }
    case DaliDebugAction::RECALL_MAX:
      dali.lamp.fadeToMaximum(addr);
      DALI_LOGI("Debug RECALL_MAX @%u", addr);
      break;
    case DaliDebugAction::RECALL_MIN:
      dali.lamp.fadeToMinimum(addr);
      DALI_LOGI("Debug RECALL_MIN @%u", addr);
      break;
    case DaliDebugAction::OFF:
      dali.lamp.turnOff(addr);
      DALI_LOGI("Debug OFF @%u", addr);
      break;
    case DaliDebugAction::DAPC_100:
      dali.lamp.setBrightness(addr, 254);
      DALI_LOGI("Debug DAPC 254 @%u", addr);
      break;
    case DaliDebugAction::DAPC_50:
      dali.lamp.setBrightness(addr, 127);
      DALI_LOGI("Debug DAPC 127 @%u", addr);
      break;
    case DaliDebugAction::DAPC_0:
      dali.lamp.setBrightness(addr, 0);
      DALI_LOGI("Debug DAPC 0 @%u", addr);
      break;
    case DaliDebugAction::IDENTIFY:
      dali.identifyDevice(addr);
      DALI_LOGI("Debug IDENTIFY @%u", addr);
      break;
    case DaliDebugAction::SCAN_SHORT: {
      char result[200] = "";
      size_t used = 0;
      uint8_t found = 0;
      for (short_addr_t a = 0; a <= ADDR_SHORT_MAX; a++) {
        uint8_t rv = 0;
        uint8_t st = this->bus_->send_query_debug(a, DaliCommand::QUERY_CONTROL_GEAR_PRESENT, &rv);
        if (st == DALI_PHY_OK && rv != 0) {
          found++;
          if (used + 8 < sizeof(result)) {
            used += snprintf(result + used, sizeof(result) - used, used ? ",%u" : "%u", a);
          }
          DALI_LOGI("  Short %u present", a);
        }
        delay(2);
        yield();
      }
      if (found == 0)
        snprintf(result, sizeof(result), "(none)");
      if (this->scan_result_ != nullptr)
        this->scan_result_->publish_state(result);
      if (this->devices_found_ != nullptr)
        this->devices_found_->publish_state(found);
      this->publish_present_(found > 0, found > 0);
      DALI_LOGI("Debug scan done: %u device(s): %s", found, result);
      break;
    }
    case DaliDebugAction::DUMP_STATUS: {
      if (addr <= ADDR_SHORT_MAX) {
        dali.dumpStatusForDevice(addr);
      } else {
        for (short_addr_t a = 0; a <= ADDR_SHORT_MAX; a++) {
          uint8_t rv = 0;
          uint8_t st = this->bus_->send_query_debug(a, DaliCommand::QUERY_CONTROL_GEAR_PRESENT, &rv);
          if (st == DALI_PHY_OK && rv != 0) {
            dali.dumpStatusForDevice(a);
          }
          delay(2);
          yield();
        }
      }
      break;
    }
    case DaliDebugAction::BLINK:
      dali.lamp.fadeToMaximum(addr);
      delay(400);
      dali.lamp.turnOff(addr);
      delay(400);
      dali.lamp.fadeToMaximum(addr);
      DALI_LOGI("Debug BLINK @%u", addr);
      break;
    case DaliDebugAction::COMPARE_PROBE: {
      dali.bus_manager.initialize(ASSIGN_ALL);
      bool ok = dali.bus_manager.compareSearchAddress(0xFFFFFF);
      dali.bus_manager.terminate();
      if (this->scan_result_ != nullptr) {
        this->scan_result_->publish_state(ok ? "COMPARE ok (gear in init)" : "COMPARE failed (no reply)");
      }
      this->publish_present_(ok, ok);
      DALI_LOGI("Debug COMPARE 0xFFFFFF -> %s", ok ? "YES" : "NO");
      break;
    }
    case DaliDebugAction::RESET_BUS:
      this->bus_->resetBus();
      DALI_LOGI("Debug RESET_BUS done");
      break;
    case DaliDebugAction::TERMINATE:
      dali.bus_manager.terminate();
      DALI_LOGI("Debug TERMINATE sent");
      break;
  }
}

}  // namespace dali
}  // namespace esphome

#endif  // USE_DALI_DEBUG
