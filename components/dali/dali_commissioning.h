#pragma once

#ifdef USE_DALI_COMMISSIONING

#include <cstdint>
#include <string>

#include "esphome/core/component.h"
#include "esphome/core/entity_base.h"
#include "esphome/core/helpers.h"
#include "esphome/components/button/button.h"
#include "esphome/components/number/number.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

#include "dali.h"
#include "dali_phy.h"

namespace esphome {
namespace dali {

enum class DaliCommissioningAction : uint8_t {
  ADD_TO_GROUP = 0,
  REMOVE_FROM_GROUP,
  QUERY_GROUPS,
  QUERY_FADE,
  SET_FADE_TIME,
  SET_FADE_RATE,
  STORE_SCENE,
  REMOVE_FROM_SCENE,
  QUERY_SCENE_LEVEL,
  GOTO_SCENE,
};

class DaliBusComponent;

class DaliCommissioningButton : public button::Button {
 public:
  void set_parent(DaliBusComponent *parent) { parent_ = parent; }
  void set_action(DaliCommissioningAction action) { action_ = action; }

 protected:
  void press_action() override;

  DaliBusComponent *parent_{nullptr};
  DaliCommissioningAction action_{DaliCommissioningAction::QUERY_GROUPS};
};

class DaliCommissioningNumber : public number::Number {
 public:
  enum class Kind : uint8_t {
    TARGET_ADDR = 0,
    GROUP,
    SCENE,
    FADE_TIME,
    FADE_RATE,
  };

  void set_parent(DaliBusComponent *parent) { parent_ = parent; }
  void set_kind(Kind kind) { kind_ = kind; }

 protected:
  void control(float value) override;

  DaliBusComponent *parent_{nullptr};
  Kind kind_{Kind::TARGET_ADDR};
};

/// Home Assistant entities for DALI group / fade / scene commissioning.
class DaliCommissioningHub {
 public:
  void setup(DaliBusComponent *bus);

  short_addr_t target_addr() const { return target_addr_; }
  void set_target_addr(short_addr_t addr) { target_addr_ = addr; }

  uint8_t group() const { return group_; }
  void set_group(uint8_t group) { group_ = group & 0x0F; }

  uint8_t scene() const { return scene_; }
  void set_scene(uint8_t scene) { scene_ = scene & 0x0F; }

  uint8_t fade_time() const { return fade_time_; }
  void set_fade_time(uint8_t fade_time) { fade_time_ = fade_time & 0x0F; }

  uint8_t fade_rate() const { return fade_rate_; }
  void set_fade_rate(uint8_t fade_rate) {
    fade_rate_ = fade_rate;
    if (fade_rate_ < 1)
      fade_rate_ = 1;
    if (fade_rate_ > 15)
      fade_rate_ = 15;
  }

  void run_action(DaliCommissioningAction action);

 private:
  void publish_groups_(uint16_t mask);
  void publish_scene_level_(uint8_t level, bool got_reply);
  void publish_fade_(uint8_t packed, bool got_reply);

  DaliBusComponent *bus_{nullptr};
  short_addr_t target_addr_{ADDR_BROADCAST};
  uint8_t group_{0};
  uint8_t scene_{0};
  uint8_t fade_time_{0};
  uint8_t fade_rate_{7};

  text_sensor::TextSensor *groups_result_{nullptr};
  text_sensor::TextSensor *scene_level_result_{nullptr};

  sensor::Sensor *fade_time_result_{nullptr};
  sensor::Sensor *fade_rate_result_{nullptr};

  DaliCommissioningNumber *target_number_{nullptr};
  DaliCommissioningNumber *group_number_{nullptr};
  DaliCommissioningNumber *scene_number_{nullptr};
  DaliCommissioningNumber *fade_time_number_{nullptr};
  DaliCommissioningNumber *fade_rate_number_{nullptr};
};

}  // namespace dali
}  // namespace esphome

#endif  // USE_DALI_COMMISSIONING
