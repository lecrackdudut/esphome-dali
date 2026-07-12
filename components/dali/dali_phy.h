/*###########################################################################
        copyright qqqlab.com / github.com/qqqlab
        Adapted for ESPHome DALI component

        This program is free software: you can redistribute it and/or modify
        it under the terms of the GNU General Public License as published by
        the Free Software Foundation, either version 3 of the License, or
        (at your option) any later version.
###########################################################################*/
#pragma once

#include <stdint.h>

#include "esp_attr.h"

namespace dali_phy {

#define DALI_PHY_OK 0
#define DALI_PHY_RESULT_BUS_NOT_IDLE 1
#define DALI_PHY_RESULT_FRAME_TOO_LONG 2
#define DALI_PHY_RESULT_COLLISION 3
#define DALI_PHY_RESULT_TRANSMITTING 4
#define DALI_PHY_RESULT_TIMEOUT 102
#define DALI_PHY_RESULT_NO_REPLY 101
#define DALI_PHY_RESULT_INVALID_REPLY 105

#define DALI_PHY_TX_COLLISION_AUTO 0
#define DALI_PHY_TX_COLLISION_OFF 1
#define DALI_PHY_TX_COLLISION_ON 2

#define DALI_PHY_RX_BUF_SIZE 40

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
  uint8_t idlecnt;
  uint8_t txcollision;
  uint8_t rx_gpio_level;
  uint32_t isr_ticks;
  uint8_t last_tx_bus_active;
  BackwardResultType last_backward_type;
  uint8_t last_backward_data;
};

const char *phy_result_name(uint8_t code);
const char *backward_result_name(BackwardResultType type);

/// Low-level DALI physical layer: 9600 Hz oversampled TX/RX with Manchester decode.
/// Ported from qqqlab DALI_Lib (ESP32-S3-Pico-DALI reference).
class DaliPhy {
 public:
  void begin(uint8_t (*bus_is_high)(), void (*bus_set_low)(), void (*bus_set_high)());
  void IRAM_ATTR timer();

  uint8_t tx(uint8_t *data, uint8_t bitlen);
  uint8_t rx(uint8_t *data);
  uint8_t tx_state();
  uint32_t milli();

  /// Blocking transmit; waits for bus idle, retries on collision.
  uint8_t tx_wait(uint8_t *data, uint8_t bitlen, uint32_t timeout_ms = 500);

  /// Poll receiver after a forward frame; returns reply byte or 0 on timeout/NACK.
  uint8_t receive_backward(uint32_t timeout_ms = 100);

  /// Poll receiver with explicit result classification.
  BackwardResult receive_backward_detailed(uint32_t timeout_ms = 100);

  /// Read volatile PHY fields and current RX GPIO level (main thread only).
  PhySnapshot get_snapshot() const;

  /// True if the last completed TX was observed modulating the bus (RX saw line low).
  uint8_t get_last_tx_bus_active() const { return this->last_tx_bus_active; }

  uint8_t txcollisionhandling{DALI_PHY_TX_COLLISION_AUTO};

 private:
  enum rx_stateEnum { EMPTY, RECEIVING, COMPLETED };

  volatile uint8_t busstate{0};
  volatile uint8_t idlecnt{0};

  volatile rx_stateEnum rxstate{EMPTY};
  volatile uint8_t rxdata[DALI_PHY_RX_BUF_SIZE]{};
  volatile uint8_t rxpos{0};
  volatile uint8_t rxbyte{0};
  volatile uint8_t rxbitcnt{0};
  volatile uint8_t rxidle{0};

  volatile uint8_t txhbdata[9]{};
  volatile uint8_t txhblen{0};
  volatile uint8_t txhbcnt{0};
  volatile uint8_t txspcnt{0};
  volatile uint8_t txhigh{0};
  volatile uint8_t txcollision{0};
  volatile uint32_t isr_ticks{0};
  volatile uint8_t tx_bus_active{0};
  volatile uint8_t last_tx_bus_active{0};
  volatile uint8_t last_backward_type{BACKWARD_TIMEOUT};
  volatile uint8_t last_backward_data{0};

  uint8_t (*bus_is_high)(){nullptr};
  void (*bus_set_low)(){nullptr};
  void (*bus_set_high)(){nullptr};

  void _set_busstate_idle();
  void _init();
  void _tx_push_2hb(uint8_t hb);

  uint8_t _man_weight(uint8_t i);
  uint8_t _man_sample(volatile uint8_t *edata, uint16_t bitpos, uint8_t *stop_coll);
  uint8_t _man_decode(volatile uint8_t *edata, uint8_t ebitlen, uint8_t *ddata);
};

}  // namespace dali_phy
