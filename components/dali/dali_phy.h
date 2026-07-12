/*###########################################################################
        copyright qqqlab.com / github.com/qqqlab
        Adapted for ESPHome DALI component

        RMT physical layer based on Espressif esp-iot-solution DALI driver
        (Apache-2.0). Replaces the previous 9600 Hz timer ISR implementation.

        This program is free software: you can redistribute it and/or modify
        it under the terms of the GNU General Public License as published by
        the Free Software Foundation, either version 3 of the License, or
        (at your option) any later version.
###########################################################################*/
#pragma once

#include <stdint.h>

#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

namespace dali_phy {

#define DALI_PHY_OK 0
#define DALI_PHY_RESULT_BUS_NOT_IDLE 1
#define DALI_PHY_RESULT_FRAME_TOO_LONG 2
#define DALI_PHY_RESULT_COLLISION 3
#define DALI_PHY_RESULT_TRANSMITTING 4
#define DALI_PHY_RESULT_TIMEOUT 102
#define DALI_PHY_RESULT_NO_REPLY 101
#define DALI_PHY_RESULT_INVALID_REPLY 105

enum PhyBusState : uint8_t {
  PHY_BUSSTATE_IDLE = 0,
  PHY_BUSSTATE_RX_ARMED = 1,
  PHY_BUSSTATE_TX = 3,
};

/// Result of a backward-frame receive attempt.
enum BackwardResultType : uint8_t {
  BACKWARD_TIMEOUT = 0,
  BACKWARD_DECODE_ERROR = 1,
  BACKWARD_REPLY = 2,
};

struct BackwardResult {
  BackwardResultType type;
  uint8_t data;
};

/// Snapshot of PHY state for diagnostics (read from main thread only).
struct PhySnapshot {
  uint8_t busstate;
  uint8_t idle_ms;
  uint8_t txcollision;
  uint8_t rx_gpio_level;
  uint32_t tx_count;
  uint8_t last_tx_bus_active;
  BackwardResultType last_backward_type;
  uint8_t last_backward_data;
};

const char *phy_result_name(uint8_t code);
const char *backward_result_name(BackwardResultType type);

/// Low-level DALI physical layer using ESP32 RMT for Manchester TX/RX.
class DaliPhy {
 public:
  DaliPhy() = default;
  ~DaliPhy();

  DaliPhy(const DaliPhy &) = delete;
  DaliPhy &operator=(const DaliPhy &) = delete;

  /// Initialize RMT TX/RX channels on the given GPIOs.
  /// @param bus_is_high Optional RX level callback for diagnostics (may be nullptr).
  bool begin(int tx_gpio, int rx_gpio, bool invert_tx, bool invert_rx, uint8_t (*bus_is_high)() = nullptr);

  void end();

  /// Drive the bus low for 1s then release, without destroying RMT channels.
  void pulse_bus_reset();

  /// Blocking transmit; waits for inter-frame gap, then sends Manchester frame.
  uint8_t tx_wait(uint8_t *data, uint8_t bitlen, uint32_t timeout_ms = 500);

  /// Poll receiver after a forward frame; returns reply byte or 0 on timeout/NACK.
  uint8_t receive_backward(uint32_t timeout_ms = 100);

  /// Poll receiver with explicit result classification.
  BackwardResult receive_backward_detailed(uint32_t timeout_ms = 100);

  /// Read PHY fields and current RX GPIO level (main thread only).
  PhySnapshot get_snapshot() const;

  /// True if the last completed TX was observed modulating the bus (RX saw line low).
  uint8_t get_last_tx_bus_active() const { return this->last_tx_bus_active_; }

 private:
  enum class ChannelState : uint8_t { OFF = 0, ARMED = 1, TX = 3 };

  bool ensure_channels_created_();
  void destroy_channels_();
  void disable_channels_();
  bool enable_channels_();
  bool arm_rx_();
  bool wait_ifg_(uint32_t timeout_ms);
  bool transmit_forward_(uint8_t *data, uint8_t bitlen, uint32_t timeout_ms);
  bool poll_rx_event_(uint32_t wait_ms, uint8_t *out_byte, bool *out_decode_error);
  void apply_ifg_();
  void set_tx_idle_level_() const;
  uint32_t milli_() const;
  uint8_t read_rx_level_() const;

  int tx_gpio_{-1};
  int rx_gpio_{-1};
  bool invert_tx_{true};
  bool invert_rx_{false};
  uint8_t (*bus_is_high_)(){nullptr};

  rmt_channel_handle_t rx_channel_{nullptr};
  rmt_channel_handle_t tx_channel_{nullptr};
  rmt_encoder_handle_t tx_encoder_{nullptr};
  QueueHandle_t rx_queue_{nullptr};
  rmt_receive_config_t rx_cfg_{};
  rmt_symbol_word_t rx_raw_[32]{};
  void *phy_ctx_{nullptr};

  const rmt_symbol_word_t *symbol_one_{nullptr};
  const rmt_symbol_word_t *symbol_zero_{nullptr};
  const rmt_symbol_word_t *symbol_stop_{nullptr};

  ChannelState channel_state_{ChannelState::OFF};
  uint32_t last_ifg_end_ms_{0};
  uint32_t tx_count_{0};
  uint8_t last_tx_bus_active_{0};
  BackwardResultType last_backward_type_{BACKWARD_TIMEOUT};
  uint8_t last_backward_data_{0};
  bool initialized_{false};
  bool channels_enabled_{false};
};

}  // namespace dali_phy
