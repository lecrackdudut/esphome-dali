#include "esphome/core/defines.h"
#include "dali_commissioning.h"

#ifdef USE_DALI_COMMISSIONING

#include "esphome_dali.h"

#include "esphome/core/application.h"

#include <cstdio>
#include <cstring>

namespace esphome {
namespace dali {

void DaliCommissioningButton::press_action() {
  if (this->parent_ != nullptr) {
    this->parent_->run_commissioning_action(this->action_);
  }
}

void DaliCommissioningNumber::control(float value) {
  if (this->parent_ == nullptr)
    return;

  switch (this->kind_) {
    case Kind::TARGET_ADDR: {
      auto addr = static_cast<short_addr_t>(value);
      if (addr > ADDR_SHORT_MAX && addr != ADDR_BROADCAST)
        addr = ADDR_BROADCAST;
      this->parent_->set_commissioning_target_addr(addr);
      this->publish_state(addr);
      break;
    }
    case Kind::GROUP:
      this->parent_->set_commissioning_group(static_cast<uint8_t>(value));
      this->publish_state(static_cast<uint8_t>(value) & 0x0F);
      break;
    case Kind::SCENE:
      this->parent_->set_commissioning_scene(static_cast<uint8_t>(value));
      this->publish_state(static_cast<uint8_t>(value) & 0x0F);
      break;
    case Kind::FADE_TIME:
      this->parent_->set_commissioning_fade_time(static_cast<uint8_t>(value));
      this->publish_state(static_cast<uint8_t>(value) & 0x0F);
      break;
    case Kind::FADE_RATE: {
      uint8_t rate = static_cast<uint8_t>(value);
      this->parent_->set_commissioning_fade_rate(rate);
      if (rate < 1)
        rate = 1;
      if (rate > 15)
        rate = 15;
      this->publish_state(rate);
      break;
    }
  }
}

namespace {

uint32_t entity_hash_(const char *object_id) {
  return fnv1_hash_object_id(object_id, std::strlen(object_id));
}

uint32_t config_fields_() {
  return static_cast<uint32_t>(ENTITY_CATEGORY_CONFIG) << ENTITY_FIELD_ENTITY_CATEGORY_SHIFT;
}

}  // namespace

void DaliCommissioningHub::setup(DaliBusComponent *bus) {
  this->bus_ = bus;

  auto make_text = [](const char *name, const char *object_id) {
    auto *s = new text_sensor::TextSensor();
    App.register_text_sensor(s, name, entity_hash_(object_id), config_fields_());
    return s;
  };
  this->groups_result_ = make_text("DALI Comm Groups", "dali_comm_groups");
  this->scene_level_result_ = make_text("DALI Comm Scene Level", "dali_comm_scene_level");

  auto make_sensor = [](const char *name, const char *object_id) {
    auto *s = new sensor::Sensor();
    s->set_accuracy_decimals(0);
    App.register_sensor(s, name, entity_hash_(object_id), config_fields_());
    return s;
  };
  this->fade_time_result_ = make_sensor("DALI Comm Fade Time", "dali_comm_fade_time");
  this->fade_rate_result_ = make_sensor("DALI Comm Fade Rate", "dali_comm_fade_rate");

  auto make_number = [bus](const char *name, const char *object_id, DaliCommissioningNumber::Kind kind, float min,
                           float max, float initial) {
    auto *n = new DaliCommissioningNumber();
    n->set_parent(bus);
    n->set_kind(kind);
    n->traits.set_min_value(min);
    n->traits.set_max_value(max);
    n->traits.set_step(1);
    n->traits.set_mode(number::NUMBER_MODE_BOX);
    App.register_number(n, name, entity_hash_(object_id), config_fields_());
    n->publish_state(initial);
    return n;
  };

  this->target_number_ =
      make_number("DALI Comm Target Address", "dali_comm_target_address", DaliCommissioningNumber::Kind::TARGET_ADDR,
                  0, 127, ADDR_BROADCAST);
  this->group_number_ =
      make_number("DALI Comm Group", "dali_comm_group", DaliCommissioningNumber::Kind::GROUP, 0, 15, 0);
  this->scene_number_ =
      make_number("DALI Comm Scene", "dali_comm_scene", DaliCommissioningNumber::Kind::SCENE, 0, 15, 0);
  this->fade_time_number_ =
      make_number("DALI Comm Fade Time Set", "dali_comm_fade_time_set", DaliCommissioningNumber::Kind::FADE_TIME, 0, 15,
                  0);
  this->fade_rate_number_ =
      make_number("DALI Comm Fade Rate Set", "dali_comm_fade_rate_set", DaliCommissioningNumber::Kind::FADE_RATE, 1, 15,
                  7);

  struct BtnDef {
    const char *name;
    const char *object_id;
    DaliCommissioningAction action;
  };
  static const BtnDef BUTTONS[] = {
      {"DALI Comm Add To Group", "dali_comm_btn_add_group", DaliCommissioningAction::ADD_TO_GROUP},
      {"DALI Comm Remove From Group", "dali_comm_btn_remove_group", DaliCommissioningAction::REMOVE_FROM_GROUP},
      {"DALI Comm Query Groups", "dali_comm_btn_query_groups", DaliCommissioningAction::QUERY_GROUPS},
      {"DALI Comm Query Fade", "dali_comm_btn_query_fade", DaliCommissioningAction::QUERY_FADE},
      {"DALI Comm Set Fade Time", "dali_comm_btn_set_fade_time", DaliCommissioningAction::SET_FADE_TIME},
      {"DALI Comm Set Fade Rate", "dali_comm_btn_set_fade_rate", DaliCommissioningAction::SET_FADE_RATE},
      {"DALI Comm Store Scene", "dali_comm_btn_store_scene", DaliCommissioningAction::STORE_SCENE},
      {"DALI Comm Remove From Scene", "dali_comm_btn_remove_scene", DaliCommissioningAction::REMOVE_FROM_SCENE},
      {"DALI Comm Query Scene Level", "dali_comm_btn_query_scene", DaliCommissioningAction::QUERY_SCENE_LEVEL},
      {"DALI Comm Goto Scene", "dali_comm_btn_goto_scene", DaliCommissioningAction::GOTO_SCENE},
  };

  for (const auto &def : BUTTONS) {
    auto *btn = new DaliCommissioningButton();
    btn->set_parent(bus);
    btn->set_action(def.action);
    App.register_button(btn, def.name, entity_hash_(def.object_id), config_fields_());
  }

  this->groups_result_->publish_state("(not queried)");
  this->scene_level_result_->publish_state("(not queried)");

  DALI_LOGI("DALI commissioning entities registered");
}

void DaliCommissioningHub::publish_groups_(uint16_t mask) {
  if (this->groups_result_ == nullptr)
    return;
  char buf[64];
  size_t used = 0;
  buf[0] = '\0';
  for (uint8_t g = 0; g < 16; g++) {
    if (mask & (1u << g)) {
      if (used + 4 < sizeof(buf)) {
        used += snprintf(buf + used, sizeof(buf) - used, used ? ",%u" : "%u", g);
      }
    }
  }
  if (used == 0)
    snprintf(buf, sizeof(buf), "(none)");
  this->groups_result_->publish_state(buf);
}

void DaliCommissioningHub::publish_scene_level_(uint8_t level, bool got_reply) {
  if (this->scene_level_result_ == nullptr)
    return;
  char buf[32];
  if (!got_reply) {
    snprintf(buf, sizeof(buf), "(no reply)");
  } else if (level == 0xFF) {
    snprintf(buf, sizeof(buf), "MASK");
  } else {
    snprintf(buf, sizeof(buf), "%u", level);
  }
  this->scene_level_result_->publish_state(buf);
}

void DaliCommissioningHub::publish_fade_(uint8_t packed, bool got_reply) {
  if (!got_reply) {
    DALI_LOGI("Commissioning QUERY_FADE: no reply");
    return;
  }
  uint8_t fade_time = (packed >> 4) & 0x0F;
  uint8_t fade_rate = packed & 0x0F;
  if (this->fade_time_result_ != nullptr)
    this->fade_time_result_->publish_state(fade_time);
  if (this->fade_rate_result_ != nullptr)
    this->fade_rate_result_->publish_state(fade_rate);
  // Keep set numbers in sync with what the gear reports
  this->fade_time_ = fade_time;
  if (fade_rate >= 1 && fade_rate <= 15)
    this->fade_rate_ = fade_rate;
  if (this->fade_time_number_ != nullptr)
    this->fade_time_number_->publish_state(fade_time);
  if (this->fade_rate_number_ != nullptr && fade_rate >= 1)
    this->fade_rate_number_->publish_state(fade_rate);
}

void DaliCommissioningHub::run_action(DaliCommissioningAction action) {
  if (this->bus_ == nullptr)
    return;

  auto &dali = this->bus_->dali;
  const short_addr_t addr = this->target_addr_;
  const uint8_t group = this->group_;
  const uint8_t scene = this->scene_;

  switch (action) {
    case DaliCommissioningAction::ADD_TO_GROUP:
      dali.scene.addToGroup(addr, group);
      DALI_LOGI("Commissioning ADD_TO_GROUP %u @%u", group, addr);
      break;
    case DaliCommissioningAction::REMOVE_FROM_GROUP:
      dali.scene.removeFromGroup(addr, group);
      DALI_LOGI("Commissioning REMOVE_FROM_GROUP %u @%u", group, addr);
      break;
    case DaliCommissioningAction::QUERY_GROUPS: {
      uint8_t lo = 0, hi = 0;
      uint8_t st_lo = this->bus_->send_query_commissioning(addr, DaliCommand::QUERY_GROUPS_0_7, &lo);
      uint8_t st_hi = this->bus_->send_query_commissioning(addr, DaliCommand::QUERY_GROUPS_8_15, &hi);
      bool ok = (st_lo == DALI_PHY_OK && st_hi == DALI_PHY_OK);
      uint16_t mask = static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
      if (ok) {
        this->publish_groups_(mask);
      } else if (this->groups_result_ != nullptr) {
        this->groups_result_->publish_state("(no reply)");
      }
      DALI_LOGI("Commissioning QUERY_GROUPS @%u -> 0x%04X (%s)", addr, mask, ok ? "ok" : "fail");
      break;
    }
    case DaliCommissioningAction::QUERY_FADE: {
      uint8_t rv = 0;
      uint8_t st = this->bus_->send_query_commissioning(addr, DaliCommand::QUERY_FADE_TIME_FADE_RATE, &rv);
      this->publish_fade_(rv, st == DALI_PHY_OK);
      DALI_LOGI("Commissioning QUERY_FADE @%u -> 0x%02X (%s)", addr, rv,
                st == DALI_PHY_OK ? "ok" : "fail");
      break;
    }
    case DaliCommissioningAction::SET_FADE_TIME:
      dali.lamp.setFadeTime(addr, this->fade_time_);
      DALI_LOGI("Commissioning SET_FADE_TIME %u @%u", this->fade_time_, addr);
      break;
    case DaliCommissioningAction::SET_FADE_RATE:
      dali.lamp.setFadeRate(addr, this->fade_rate_);
      DALI_LOGI("Commissioning SET_FADE_RATE %u @%u", this->fade_rate_, addr);
      break;
    case DaliCommissioningAction::STORE_SCENE:
      // STORE_ACTUAL_LEVEL_IN_DTR0 + SET_SCENE are both send-twice config commands
      dali.scene.storeScene(addr, scene);
      DALI_LOGI("Commissioning STORE_SCENE %u @%u", scene, addr);
      break;
    case DaliCommissioningAction::REMOVE_FROM_SCENE:
      dali.scene.removeScene(addr, scene);
      DALI_LOGI("Commissioning REMOVE_FROM_SCENE %u @%u", scene, addr);
      break;
    case DaliCommissioningAction::QUERY_SCENE_LEVEL: {
      DaliCommand cmd =
          static_cast<DaliCommand>((uint8_t) DaliCommand::QUERY_SCENE_LEVEL | (scene & 0x0F));
      uint8_t rv = 0;
      uint8_t st = this->bus_->send_query_commissioning(addr, cmd, &rv);
      this->publish_scene_level_(rv, st == DALI_PHY_OK);
      DALI_LOGI("Commissioning QUERY_SCENE_LEVEL %u @%u -> 0x%02X (%s)", scene, addr, rv,
                st == DALI_PHY_OK ? "ok" : "fail");
      break;
    }
    case DaliCommissioningAction::GOTO_SCENE:
      dali.scene.goToScene(addr, scene);
      DALI_LOGI("Commissioning GOTO_SCENE %u @%u", scene, addr);
      break;
  }
}

}  // namespace dali
}  // namespace esphome

#endif  // USE_DALI_COMMISSIONING
