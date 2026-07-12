/*###########################################################################
        copyright qqqlab.com / github.com/qqqlab
        Adapted for ESPHome DALI component

        RMT physical layer based on Espressif esp-iot-solution DALI driver
        (Apache-2.0).

        This program is free software: you can redistribute it and/or modify
        it under the terms of the GNU General Public License as published by
        the Free Software Foundation, either version 3 of the License, or
        (at your option) any later version.
###########################################################################*/
#include "dali_phy.h"

#include <string.h>

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

namespace dali_phy {

namespace {

static const char *const TAG = "dali.phy";

static constexpr uint32_t DALI_RMT_RESOLUTION_HZ = 1000000U;
static constexpr uint32_t DALI_TE_US = 416U;
static constexpr uint32_t DALI_IFG_MS = 20;
static constexpr uint32_t DALI_BF_TIMEOUT_MS = 25;
static constexpr uint32_t DALI_DEFAULT_MEM_BLOCK = 48;

static inline uint32_t us_to_rmt_ticks(uint32_t us) { return us * (DALI_RMT_RESOLUTION_HZ / 1000000U); }
static inline uint32_t us_to_ns(uint32_t us) { return us * 1000U; }

static inline rmt_symbol_word_t make_rmt_symbol(uint16_t dur0, uint8_t lvl0, uint16_t dur1, uint8_t lvl1) {
  rmt_symbol_word_t sym{};
  sym.duration0 = dur0;
  sym.level0 = lvl0;
  sym.duration1 = dur1;
  sym.level1 = lvl1;
  return sym;
}

static const uint16_t DALI_TE_TICKS = static_cast<uint16_t>(us_to_rmt_ticks(DALI_TE_US));

static rmt_symbol_word_t make_sym_one_normal() {
  return make_rmt_symbol(DALI_TE_TICKS, 0, DALI_TE_TICKS, 1);
}
static rmt_symbol_word_t make_sym_zero_normal() {
  return make_rmt_symbol(DALI_TE_TICKS, 1, DALI_TE_TICKS, 0);
}
static rmt_symbol_word_t make_sym_stop_normal() {
  return make_rmt_symbol(DALI_TE_TICKS * 2, 0, DALI_TE_TICKS * 2, 0);
}
static rmt_symbol_word_t make_sym_one_invert() {
  return make_rmt_symbol(DALI_TE_TICKS, 1, DALI_TE_TICKS, 0);
}
static rmt_symbol_word_t make_sym_zero_invert() {
  return make_rmt_symbol(DALI_TE_TICKS, 0, DALI_TE_TICKS, 1);
}
static rmt_symbol_word_t make_sym_stop_invert() {
  return make_rmt_symbol(DALI_TE_TICKS * 2, 0, DALI_TE_TICKS * 2, 0);
}

static bool IRAM_ATTR dali_rx_done_cb(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data) {
  BaseType_t hp_woken = pdFALSE;
  xQueueSendFromISR(static_cast<QueueHandle_t>(user_data), edata, &hp_woken);
  return hp_woken == pdTRUE;
}

static esp_err_t decode_backward_frame_byte(const rmt_symbol_word_t *symbols, size_t num_symbols, uint8_t *out_byte,
                                            uint8_t *out_bits_decoded) {
  const uint32_t te_ticks = us_to_rmt_ticks(DALI_TE_US);
  constexpr size_t max_te = 40;

  uint8_t te_levels[max_te];
  memset(te_levels, 0, max_te * sizeof(uint8_t));
  size_t te_len = 0;

  for (size_t i = 0; i < num_symbols && te_len < max_te; i++) {
    uint32_t n0 = (symbols[i].duration0 + (te_ticks / 2)) / te_ticks;
    uint32_t n1 = (symbols[i].duration1 + (te_ticks / 2)) / te_ticks;
    if (n0 == 0) {
      n0 = 1;
    } else if (n0 > 8) {
      n0 = 8;
    }
    if (n1 == 0) {
      n1 = 1;
    } else if (n1 > 8) {
      n1 = 8;
    }

    for (uint32_t k = 0; k < n0 && te_len < max_te; k++) {
      te_levels[te_len++] = static_cast<uint8_t>(symbols[i].level0 & 0x01U);
    }
    for (uint32_t k = 0; k < n1 && te_len < max_te; k++) {
      te_levels[te_len++] = static_cast<uint8_t>(symbols[i].level1 & 0x01U);
    }
  }

  if (te_len > 26) {
    *out_bits_decoded = 0;
    return ESP_ERR_INVALID_STATE;
  }

  size_t start = 0;
  while (start + 1 < te_len && te_levels[start] == te_levels[start + 1]) {
    start++;
  }
  if (start + 1 >= te_len) {
    *out_bits_decoded = 0;
    return ESP_ERR_INVALID_STATE;
  }

  const uint8_t one_a = te_levels[start];
  const uint8_t one_b = te_levels[start + 1];
  if (one_a == one_b) {
    *out_bits_decoded = 0;
    return ESP_ERR_INVALID_STATE;
  }

  uint8_t frame = 0;
  uint8_t bits = 0;
  size_t idx = start + 2;
  while (bits < 8 && (idx + 1) < te_len) {
    const uint8_t a = te_levels[idx];
    const uint8_t b = te_levels[idx + 1];
    if (a == one_a && b == one_b) {
      frame = static_cast<uint8_t>((frame << 1) | 1U);
      bits++;
    } else if (a == one_b && b == one_a) {
      frame = static_cast<uint8_t>(frame << 1);
      bits++;
    } else {
      break;
    }
    idx += 2;
  }

  *out_byte = frame;
  *out_bits_decoded = bits;
  return (bits == 8) ? ESP_OK : ESP_ERR_INVALID_STATE;
}

}  // namespace

const char *phy_result_name(uint8_t code) {
  switch (code) {
    case DALI_PHY_OK:
      return "OK";
    case DALI_PHY_RESULT_BUS_NOT_IDLE:
      return "BUS_NOT_IDLE";
    case DALI_PHY_RESULT_FRAME_TOO_LONG:
      return "FRAME_TOO_LONG";
    case DALI_PHY_RESULT_COLLISION:
      return "COLLISION";
    case DALI_PHY_RESULT_TRANSMITTING:
      return "TRANSMITTING";
    case DALI_PHY_RESULT_NO_REPLY:
      return "NO_REPLY";
    case DALI_PHY_RESULT_TIMEOUT:
      return "TIMEOUT";
    case DALI_PHY_RESULT_INVALID_REPLY:
      return "INVALID_REPLY";
    default:
      return "UNKNOWN";
  }
}

const char *backward_result_name(BackwardResultType type) {
  switch (type) {
    case BACKWARD_TIMEOUT:
      return "timeout";
    case BACKWARD_DECODE_ERROR:
      return "decode_error";
    case BACKWARD_REPLY:
      return "reply";
    default:
      return "unknown";
  }
}

DaliPhy::~DaliPhy() { this->end(); }

bool DaliPhy::begin(int tx_gpio, int rx_gpio, bool invert_tx, bool invert_rx, uint8_t (*bus_is_high)()) {
  this->end();

  if (tx_gpio < 0 || rx_gpio < 0) {
    ESP_LOGE(TAG, "Invalid GPIO: tx=%d rx=%d", tx_gpio, rx_gpio);
    return false;
  }

  this->tx_gpio_ = tx_gpio;
  this->rx_gpio_ = rx_gpio;
  this->invert_tx_ = invert_tx;
  this->invert_rx_ = invert_rx;
  this->bus_is_high_ = bus_is_high;

  if (invert_tx) {
    this->symbol_one_ = make_sym_one_invert();
    this->symbol_zero_ = make_sym_zero_invert();
    this->symbol_stop_ = make_sym_stop_invert();
  } else {
    this->symbol_one_ = make_sym_one_normal();
    this->symbol_zero_ = make_sym_zero_normal();
    this->symbol_stop_ = make_sym_stop_normal();
  }

  gpio_set_direction(static_cast<gpio_num_t>(rx_gpio), GPIO_MODE_INPUT);

  if (!this->ensure_channels_created_()) {
    this->end();
    return false;
  }

  if (!this->enable_channels_()) {
    this->end();
    return false;
  }

  this->last_ifg_end_ms_ = 0;
  this->tx_count_ = 0;
  this->last_tx_err_ = 0;
  this->last_tx_bus_active_ = 0;
  this->last_backward_type_ = BACKWARD_TIMEOUT;
  this->last_backward_data_ = 0;
  this->initialized_ = true;

  ESP_LOGI(TAG, "RMT PHY ready (TX GPIO %d%s, RX GPIO %d%s, mem=%u sym)", tx_gpio, invert_tx ? " inv" : "", rx_gpio,
           invert_rx ? " inv" : "", static_cast<unsigned>(DALI_DEFAULT_MEM_BLOCK));
  return true;
}

void DaliPhy::end() {
  this->destroy_channels_();
  this->initialized_ = false;
  this->channels_enabled_ = false;
  this->channel_state_ = ChannelState::OFF;
}

void DaliPhy::destroy_channels_() {
  if (this->tx_channel_ != nullptr) {
    rmt_tx_wait_all_done(this->tx_channel_, 0);
  }

  this->disable_channels_();

  if (this->tx_encoder_ != nullptr) {
    const esp_err_t err = rmt_del_encoder(this->tx_encoder_);
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "rmt_del_encoder failed: %s", esp_err_to_name(err));
    }
    this->tx_encoder_ = nullptr;
  }
  if (this->tx_channel_ != nullptr) {
    const esp_err_t err = rmt_del_channel(this->tx_channel_);
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "rmt_del_channel TX failed: %s", esp_err_to_name(err));
    }
    this->tx_channel_ = nullptr;
  }
  if (this->rx_channel_ != nullptr) {
    const esp_err_t err = rmt_del_channel(this->rx_channel_);
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "rmt_del_channel RX failed: %s", esp_err_to_name(err));
    }
    this->rx_channel_ = nullptr;
  }
  if (this->rx_queue_ != nullptr) {
    vQueueDelete(this->rx_queue_);
    this->rx_queue_ = nullptr;
  }
}

bool DaliPhy::ensure_channels_created_() {
  if (this->rx_channel_ != nullptr && this->tx_channel_ != nullptr && this->tx_encoder_ != nullptr) {
    return true;
  }

  this->destroy_channels_();
  this->rx_queue_ = xQueueCreate(4, sizeof(rmt_rx_done_event_data_t));
  if (this->rx_queue_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create RX queue");
    return false;
  }

  // Create TX before RX: on ESP32-S3 both share one RMT memory pool. RX at block 4
  // can block a 2-block TX allocation at block 3; allocate TX first when slots are free.
  rmt_tx_channel_config_t tx_channel_cfg = {};
  tx_channel_cfg.gpio_num = static_cast<gpio_num_t>(this->tx_gpio_);
  tx_channel_cfg.clk_src = RMT_CLK_SRC_DEFAULT;
  tx_channel_cfg.resolution_hz = DALI_RMT_RESOLUTION_HZ;
  tx_channel_cfg.mem_block_symbols = DALI_DEFAULT_MEM_BLOCK;
  tx_channel_cfg.trans_queue_depth = 1;
  esp_err_t err = rmt_new_tx_channel(&tx_channel_cfg, &this->tx_channel_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create TX channel on GPIO %d: %s", this->tx_gpio_, esp_err_to_name(err));
    this->destroy_channels_();
    return false;
  }

  const rmt_copy_encoder_config_t enc_cfg = {};
  err = rmt_new_copy_encoder(&enc_cfg, &this->tx_encoder_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create TX copy encoder: %s", esp_err_to_name(err));
    this->destroy_channels_();
    return false;
  }

  rmt_rx_channel_config_t rx_channel_cfg = {};
  rx_channel_cfg.gpio_num = static_cast<gpio_num_t>(this->rx_gpio_);
  rx_channel_cfg.clk_src = RMT_CLK_SRC_DEFAULT;
  rx_channel_cfg.resolution_hz = DALI_RMT_RESOLUTION_HZ;
  rx_channel_cfg.mem_block_symbols = DALI_DEFAULT_MEM_BLOCK;
  rx_channel_cfg.flags.invert_in = this->invert_rx_ ? 1U : 0U;
  err = rmt_new_rx_channel(&rx_channel_cfg, &this->rx_channel_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create RX channel on GPIO %d: %s", this->rx_gpio_, esp_err_to_name(err));
    this->destroy_channels_();
    return false;
  }

  rmt_rx_event_callbacks_t rx_cbs = {
      .on_recv_done = dali_rx_done_cb,
  };
  err = rmt_rx_register_event_callbacks(this->rx_channel_, &rx_cbs, this->rx_queue_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to register RX callbacks: %s", esp_err_to_name(err));
    this->destroy_channels_();
    return false;
  }

  this->rx_cfg_.signal_range_min_ns = us_to_ns(2);
  this->rx_cfg_.signal_range_max_ns = us_to_ns(2000);

  return true;
}

bool DaliPhy::enable_channels_() {
  if (this->channels_enabled_) {
    return true;
  }
  if (this->tx_channel_ == nullptr || this->rx_channel_ == nullptr) {
    return false;
  }

  esp_err_t err = rmt_enable(this->tx_channel_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to enable TX channel: %s", esp_err_to_name(err));
    return false;
  }
  err = rmt_enable(this->rx_channel_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to enable RX channel: %s", esp_err_to_name(err));
    rmt_disable(this->tx_channel_);
    return false;
  }

  this->channels_enabled_ = true;
  return true;
}

void DaliPhy::disable_channels_() {
  if (this->tx_channel_ != nullptr) {
    const esp_err_t err = rmt_disable(this->tx_channel_);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
      ESP_LOGW(TAG, "rmt_disable TX failed: %s", esp_err_to_name(err));
    }
  }
  if (this->rx_channel_ != nullptr) {
    const esp_err_t err = rmt_disable(this->rx_channel_);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
      ESP_LOGW(TAG, "rmt_disable RX failed: %s", esp_err_to_name(err));
    }
  }
  if (this->rx_queue_ != nullptr) {
    xQueueReset(this->rx_queue_);
  }
  this->channels_enabled_ = false;
  this->channel_state_ = ChannelState::OFF;
}

void DaliPhy::set_tx_idle_level_() const {
  if (this->tx_gpio_ < 0) {
    return;
  }
  // Inverted opto: GPIO low releases the bus; non-inverted: GPIO high is idle.
  gpio_set_level(static_cast<gpio_num_t>(this->tx_gpio_), this->invert_tx_ ? 0 : 1);
}

void DaliPhy::pulse_bus_reset() {
  if (!this->initialized_) {
    return;
  }

  ESP_LOGD(TAG, "Bus reset pulse start (1s active)");

  if (this->tx_channel_ != nullptr) {
    rmt_tx_wait_all_done(this->tx_channel_, 0);
  }
  this->disable_channels_();

  gpio_set_direction(static_cast<gpio_num_t>(this->tx_gpio_), GPIO_MODE_OUTPUT);
  gpio_set_level(static_cast<gpio_num_t>(this->tx_gpio_), this->invert_tx_ ? 1 : 0);
  vTaskDelay(pdMS_TO_TICKS(1000));
  this->set_tx_idle_level_();

  this->last_ifg_end_ms_ = 0;
  this->channel_state_ = ChannelState::OFF;

  if (!this->enable_channels_()) {
    ESP_LOGE(TAG, "Failed to re-enable RMT channels after bus reset pulse");
  }

  ESP_LOGD(TAG, "Bus reset pulse done");
}

void DaliPhy::recover_tx_channel_() {
  if (this->tx_channel_ == nullptr || this->tx_encoder_ == nullptr) {
    return;
  }

  rmt_tx_wait_all_done(this->tx_channel_, 0);
  if (this->channels_enabled_) {
    const esp_err_t dis = rmt_disable(this->tx_channel_);
    if (dis == ESP_OK) {
      const esp_err_t en = rmt_enable(this->tx_channel_);
      if (en != ESP_OK) {
        ESP_LOGW(TAG, "TX channel re-enable failed: %s", esp_err_to_name(en));
      }
    } else {
      ESP_LOGW(TAG, "TX channel disable failed: %s", esp_err_to_name(dis));
    }
  }
  rmt_encoder_reset(this->tx_encoder_);
}

bool DaliPhy::arm_rx_() {
  if (!this->initialized_ || this->rx_channel_ == nullptr) {
    return false;
  }

  xQueueReset(this->rx_queue_);
  if (rmt_receive(this->rx_channel_, this->rx_raw_, sizeof(this->rx_raw_), &this->rx_cfg_) != ESP_OK) {
    ESP_LOGE(TAG, "rmt_receive failed");
    return false;
  }
  this->channel_state_ = ChannelState::ARMED;
  return true;
}

bool DaliPhy::wait_ifg_(uint32_t timeout_ms) {
  const uint32_t start_ms = this->milli_();
  while (true) {
    const uint32_t now_ms = this->milli_();
    if (this->last_ifg_end_ms_ == 0 || now_ms - this->last_ifg_end_ms_ >= DALI_IFG_MS) {
      return true;
    }
    if (now_ms - start_ms > timeout_ms) {
      return false;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

uint32_t DaliPhy::milli_() const { return static_cast<uint32_t>(esp_timer_get_time() / 1000LL); }

uint8_t DaliPhy::read_rx_level_() const {
  if (this->bus_is_high_ != nullptr) {
    return this->bus_is_high_();
  }
  if (this->rx_gpio_ >= 0) {
    return gpio_get_level(static_cast<gpio_num_t>(this->rx_gpio_));
  }
  return 0;
}

size_t DaliPhy::build_forward_symbols_(const uint8_t *data, uint8_t byte_count, rmt_symbol_word_t *out) const {
  size_t idx = 0;
  out[idx++] = this->symbol_one_;
  for (uint8_t b = 0; b < byte_count; b++) {
    for (int mask = 0x80; mask != 0; mask >>= 1) {
      out[idx++] = (data[b] & mask) ? this->symbol_one_ : this->symbol_zero_;
    }
  }
  out[idx++] = this->symbol_stop_;
  return idx;
}

bool DaliPhy::transmit_forward_(uint8_t *data, uint8_t bitlen, uint32_t timeout_ms) {
  if (!this->initialized_) {
    this->last_tx_err_ = ESP_ERR_INVALID_STATE;
    return false;
  }
  if (bitlen > 32) {
    this->last_tx_err_ = ESP_ERR_INVALID_ARG;
    return false;
  }

  if (!this->channels_enabled_ && !this->enable_channels_()) {
    this->last_tx_err_ = ESP_ERR_INVALID_STATE;
    ESP_LOGE(TAG, "TX aborted: RMT channels not enabled");
    return false;
  }

  const uint8_t byte_count = static_cast<uint8_t>((bitlen + 7U) / 8U);
  rmt_symbol_word_t symbols[20];
  const size_t symbol_count = this->build_forward_symbols_(data, byte_count, symbols);

  this->channel_state_ = ChannelState::TX;
  const rmt_transmit_config_t tx_cfg = {
      .loop_count = 0,
      .flags =
          {
              .eot_level = this->invert_tx_ ? 0U : 1U,
          },
  };

  rmt_encoder_reset(this->tx_encoder_);
  esp_err_t tx_err =
      rmt_transmit(this->tx_channel_, this->tx_encoder_, symbols, symbol_count * sizeof(rmt_symbol_word_t), &tx_cfg);
  if (tx_err == ESP_OK) {
    tx_err = rmt_tx_wait_all_done(this->tx_channel_, 200);
  }

  if (tx_err != ESP_OK) {
    this->last_tx_err_ = tx_err;
    ESP_LOGE(TAG, "TX failed: %s (symbols=%u)", esp_err_to_name(tx_err), static_cast<unsigned>(symbol_count));
    this->recover_tx_channel_();
    this->channel_state_ = ChannelState::OFF;
    return false;
  }

  this->last_tx_err_ = 0;
  this->tx_count_++;
  this->last_tx_bus_active_ = this->read_rx_level_() == 0 ? 1U : 0U;

  if (!this->arm_rx_()) {
    this->channel_state_ = ChannelState::OFF;
    return false;
  }

  return true;
}

bool DaliPhy::poll_rx_event_(uint32_t wait_ms, uint8_t *out_byte, bool *out_decode_error) {
  if (this->rx_queue_ == nullptr) {
    return false;
  }

  const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(wait_ms);
  while (true) {
    const TickType_t now = xTaskGetTickCount();
    if (now >= deadline) {
      return false;
    }

    rmt_rx_done_event_data_t rx_data;
    if (xQueueReceive(this->rx_queue_, &rx_data, deadline - now) != pdTRUE) {
      return false;
    }

    uint8_t frame = 0;
    uint8_t bit_count = 0;
    const esp_err_t dec_err =
        decode_backward_frame_byte(rx_data.received_symbols, rx_data.num_symbols, &frame, &bit_count);
    if (dec_err == ESP_OK) {
      *out_byte = frame;
      if (out_decode_error != nullptr) {
        *out_decode_error = false;
      }
      return true;
    }

    if (out_decode_error != nullptr) {
      *out_decode_error = true;
    }

    if (rmt_receive(this->rx_channel_, this->rx_raw_, sizeof(this->rx_raw_), &this->rx_cfg_) != ESP_OK) {
      ESP_LOGE(TAG, "rmt_receive re-arm failed");
      return false;
    }
  }
}

void DaliPhy::apply_ifg_() {
  this->channel_state_ = ChannelState::OFF;
  vTaskDelay(pdMS_TO_TICKS(DALI_IFG_MS));
  this->last_ifg_end_ms_ = this->milli_();
}

uint8_t DaliPhy::tx_wait(uint8_t *data, uint8_t bitlen, uint32_t timeout_ms) {
  if (bitlen > 32) {
    return DALI_PHY_RESULT_FRAME_TOO_LONG;
  }
  if (!this->initialized_) {
    return DALI_PHY_RESULT_TIMEOUT;
  }

  if (!this->wait_ifg_(timeout_ms)) {
    return DALI_PHY_RESULT_BUS_NOT_IDLE;
  }

  if (!this->transmit_forward_(data, bitlen, timeout_ms)) {
    return DALI_PHY_RESULT_TIMEOUT;
  }

  return DALI_PHY_OK;
}

BackwardResult DaliPhy::receive_backward_detailed(uint32_t timeout_ms) {
  BackwardResult result{BACKWARD_TIMEOUT, 0};
  if (!this->initialized_ || this->channel_state_ != ChannelState::ARMED) {
    this->last_backward_type_ = BACKWARD_TIMEOUT;
    this->last_backward_data_ = 0;
    this->apply_ifg_();
    return result;
  }

  const uint32_t wait_ms = timeout_ms < DALI_BF_TIMEOUT_MS ? timeout_ms : DALI_BF_TIMEOUT_MS;
  uint8_t reply = 0;
  bool decode_error = false;

  if (this->poll_rx_event_(wait_ms, &reply, &decode_error)) {
    result.type = BACKWARD_REPLY;
    result.data = reply;
    this->last_backward_type_ = BACKWARD_REPLY;
    this->last_backward_data_ = reply;
    this->apply_ifg_();
    return result;
  }

  if (decode_error) {
    result.type = BACKWARD_DECODE_ERROR;
    this->last_backward_type_ = BACKWARD_DECODE_ERROR;
    this->last_backward_data_ = 0;
    this->apply_ifg_();
    return result;
  }

  this->last_backward_type_ = BACKWARD_TIMEOUT;
  this->last_backward_data_ = 0;
  this->apply_ifg_();
  return result;
}

uint8_t DaliPhy::receive_backward(uint32_t timeout_ms) {
  const BackwardResult result = this->receive_backward_detailed(timeout_ms);
  if (result.type == BACKWARD_REPLY) {
    return result.data;
  }
  return 0;
}

PhySnapshot DaliPhy::get_snapshot() const {
  PhySnapshot snap{};
  snap.busstate = static_cast<uint8_t>(this->channel_state_);
  const uint32_t now_ms = this->milli_();
  if (this->last_ifg_end_ms_ == 0) {
    snap.idle_ms = 255U;
  } else {
    const uint32_t idle = now_ms - this->last_ifg_end_ms_;
    snap.idle_ms = idle > 255U ? 255U : static_cast<uint8_t>(idle);
  }
  snap.txcollision = 0;
  snap.rx_gpio_level = this->read_rx_level_();
  snap.tx_count = this->tx_count_;
  snap.last_tx_err = this->last_tx_err_;
  snap.last_tx_bus_active = this->last_tx_bus_active_;
  snap.last_backward_type = this->last_backward_type_;
  snap.last_backward_data = this->last_backward_data_;
  return snap;
}

}  // namespace dali_phy
