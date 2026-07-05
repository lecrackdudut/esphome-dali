/*###########################################################################
        copyright qqqlab.com / github.com/qqqlab
        Adapted for ESPHome DALI component
###########################################################################*/
#include "dali_phy.h"

#include "esp_timer.h"

namespace dali_phy {

// Require this many idle samples before transmit (~1.35 ms at 9600 Hz).
static constexpr uint8_t BEFORE_CMD_IDLE_SAMPLES = 13;

#define IDLE 0
#define RX 1
#define TX 3
#define COLLISION_TX 4

void DaliPhy::begin(uint8_t (*bus_is_high_fn)(), void (*bus_set_low_fn)(), void (*bus_set_high_fn)()) {
  this->bus_is_high = bus_is_high_fn;
  this->bus_set_low = bus_set_low_fn;
  this->bus_set_high = bus_set_high_fn;
  this->_init();
}

void DaliPhy::_set_busstate_idle() {
  this->bus_set_high();
  this->idlecnt = 0;
  this->busstate = IDLE;
}

void DaliPhy::_init() {
  this->_set_busstate_idle();
  this->rxstate = EMPTY;
  this->txcollision = 0;
}

uint32_t DaliPhy::milli() { return static_cast<uint32_t>(esp_timer_get_time() / 1000LL); }

void DaliPhy::timer() {
  uint8_t busishigh = (this->bus_is_high() ? 1 : 0);

  switch (this->busstate) {
    case IDLE:
      if (busishigh) {
        if (this->idlecnt != 0xff)
          this->idlecnt++;
        break;
      }
      this->rxpos = 0;
      this->rxbitcnt = 0;
      this->rxidle = 0;
      this->rxstate = RECEIVING;
      this->busstate = RX;
      // fall through
    case RX:
      this->rxbyte = (this->rxbyte << 1) | busishigh;
      this->rxbitcnt++;
      if (this->rxbitcnt == 8) {
        this->rxdata[this->rxpos] = this->rxbyte;
        this->rxpos++;
        if (this->rxpos > DALI_PHY_RX_BUF_SIZE - 1)
          this->rxpos = DALI_PHY_RX_BUF_SIZE - 1;
        this->rxbitcnt = 0;
      }
      if (busishigh) {
        this->rxidle++;
        if (this->rxidle >= 16) {
          this->rxdata[this->rxpos] = 0xFF;
          this->rxpos++;
          this->rxstate = COMPLETED;
          this->_set_busstate_idle();
          break;
        }
      } else {
        this->rxidle = 0;
      }
      break;
    case TX:
      if (this->txhbcnt >= this->txhblen) {
        this->_set_busstate_idle();
      } else {
        if (((this->txcollisionhandling == DALI_PHY_TX_COLLISION_ON) ||
             (this->txcollisionhandling == DALI_PHY_TX_COLLISION_AUTO && this->txhblen != 2 + 8 + 4)) &&
            (this->txhigh && !busishigh) && (this->txspcnt == 1 || this->txspcnt == 2)) {
          if (this->txcollision != 0xFF)
            this->txcollision++;
          this->txspcnt = 0;
          this->busstate = COLLISION_TX;
          return;
        }

        if (this->txspcnt == 0) {
          uint8_t pos = this->txhbcnt >> 3;
          uint8_t bitmask = 1 << (7 - (this->txhbcnt & 0x7));
          if (this->txhbdata[pos] & bitmask) {
            this->bus_set_low();
            this->txhigh = 0;
          } else {
            this->bus_set_high();
            this->txhigh = 1;
          }
          this->txhbcnt++;
          this->txspcnt = 4;
        }
        this->txspcnt--;
      }
      break;
    case COLLISION_TX:
      this->bus_set_low();
      this->txspcnt++;
      if (this->txspcnt >= 16)
        this->_set_busstate_idle();
      break;
  }
}

void DaliPhy::_tx_push_2hb(uint8_t hb) {
  uint8_t pos = this->txhblen >> 3;
  uint8_t shift = 6 - (this->txhblen & 0x7);
  this->txhbdata[pos] |= hb << shift;
  this->txhblen += 2;
}

uint8_t DaliPhy::tx(uint8_t *data, uint8_t bitlen) {
  if (bitlen > 32)
    return DALI_PHY_RESULT_FRAME_TOO_LONG;
  if (this->busstate != IDLE)
    return DALI_PHY_RESULT_BUS_NOT_IDLE;

  for (uint8_t i = 0; i < 9; i++)
    this->txhbdata[i] = 0;

  this->txhblen = 0;
  this->_tx_push_2hb(0x2);
  for (uint8_t i = 0; i < bitlen; i++) {
    uint8_t pos = i >> 3;
    uint8_t mask = 1 << (7 - (i & 0x7));
    this->_tx_push_2hb(data[pos] & mask ? 0x2 : 0x1);
  }
  this->_tx_push_2hb(0x0);
  this->_tx_push_2hb(0x0);

  this->txhbcnt = 0;
  this->txspcnt = 0;
  this->txcollision = 0;
  this->rxstate = EMPTY;
  this->busstate = TX;
  return DALI_PHY_OK;
}

uint8_t DaliPhy::tx_state() {
  if (this->txcollision) {
    this->txcollision = 0;
    return DALI_PHY_RESULT_COLLISION;
  }
  if (this->busstate == TX)
    return DALI_PHY_RESULT_TRANSMITTING;
  return DALI_PHY_OK;
}

uint8_t DaliPhy::_man_weight(uint8_t i) {
  int8_t w = 0;
  w += ((i >> 7) & 1) ? 1 : -1;
  w += ((i >> 6) & 1) ? 2 : -2;
  w += ((i >> 5) & 1) ? 2 : -2;
  w += ((i >> 4) & 1) ? 1 : -1;
  w -= ((i >> 3) & 1) ? 1 : -1;
  w -= ((i >> 2) & 1) ? 2 : -2;
  w -= ((i >> 1) & 1) ? 2 : -2;
  w -= ((i >> 0) & 1) ? 1 : -1;
  w *= 2;
  if (w < 0)
    w = -w + 1;
  return static_cast<uint8_t>(w);
}

uint8_t DaliPhy::_man_sample(volatile uint8_t *edata, uint16_t bitpos, uint8_t *stop_coll) {
  uint8_t pos = bitpos >> 3;
  uint8_t shift = bitpos & 0x7;
  uint8_t sample = (edata[pos] << shift) | (edata[pos + 1] >> (8 - shift));

  if (sample == 0xFF)
    *stop_coll = 1;
  if (sample == 0x00)
    *stop_coll = 2;

  return sample;
}

uint8_t DaliPhy::_man_decode(volatile uint8_t *edata, uint8_t ebitlen, uint8_t *ddata) {
  uint8_t dbitlen = 0;
  uint16_t ebitpos = 1;
  while (ebitpos + 1 < ebitlen) {
    uint8_t stop_coll = 0;
    uint8_t sample = this->_man_sample(edata, ebitpos, &stop_coll);
    uint8_t weightmax = this->_man_weight(sample);
    uint8_t pmax = 8;

    sample = this->_man_sample(edata, ebitpos - 1, &stop_coll);
    uint8_t w = this->_man_weight(sample);
    if (weightmax < w) {
      weightmax = w;
      pmax = 7;
    }

    sample = this->_man_sample(edata, ebitpos + 1, &stop_coll);
    w = this->_man_weight(sample);
    if (weightmax < w) {
      weightmax = w;
      pmax = 9;
    }

    if (stop_coll == 1)
      break;
    if (stop_coll == 2)
      return 0;

    if (dbitlen > 0) {
      uint8_t bytepos = (dbitlen - 1) >> 3;
      uint8_t bitpos = (dbitlen - 1) & 0x7;
      if (bitpos == 0)
        ddata[bytepos] = 0;
      ddata[bytepos] = (ddata[bytepos] << 1) | (weightmax & 1);
    }
    dbitlen++;
    ebitpos += pmax;
  }
  if (dbitlen > 1)
    dbitlen--;
  return dbitlen;
}

uint8_t DaliPhy::rx(uint8_t *ddata) {
  switch (this->rxstate) {
    case EMPTY:
      return 0;
    case RECEIVING:
      return 1;
    case COMPLETED:
      this->rxstate = EMPTY;
      {
        uint8_t dlen = this->_man_decode(this->rxdata, this->rxpos * 8, ddata);
        if (dlen < 3)
          return 2;
        return dlen;
      }
    default:
      return 0;
  }
}

uint8_t DaliPhy::tx_wait(uint8_t *data, uint8_t bitlen, uint32_t timeout_ms) {
  if (bitlen > 32)
    return DALI_PHY_RESULT_FRAME_TOO_LONG;
  uint32_t start_ms = this->milli();
  while (true) {
    while (this->idlecnt < BEFORE_CMD_IDLE_SAMPLES) {
      if (this->milli() - start_ms > timeout_ms)
        return DALI_PHY_RESULT_TIMEOUT;
    }
    while (this->tx(data, bitlen) != DALI_PHY_OK) {
      if (this->milli() - start_ms > timeout_ms)
        return DALI_PHY_RESULT_TIMEOUT;
    }
    uint8_t rv;
    while (true) {
      rv = this->tx_state();
      if (rv != DALI_PHY_RESULT_TRANSMITTING)
        break;
      if (this->milli() - start_ms > timeout_ms)
        return DALI_PHY_RESULT_TIMEOUT;
    }
    if (rv == DALI_PHY_OK) {
      int64_t start_us = esp_timer_get_time();
      while (esp_timer_get_time() - start_us < 1000)
        __asm__ __volatile__("nop");
      return DALI_PHY_OK;
    }
  }
}

uint8_t DaliPhy::receive_backward(uint32_t timeout_ms) {
  uint8_t data[4];
  uint32_t rx_start_ms = this->milli();
  uint32_t rx_timeout_ms = timeout_ms < 10 ? timeout_ms : 10;

  while (true) {
    uint8_t rv = this->rx(data);
    switch (rv) {
      case 0:
        break;
      case 1:
        rx_timeout_ms = timeout_ms < 25 ? timeout_ms : 25;
        break;
      case 2:
        return 0;
      default:
        if (rv == 8)
          return data[0];
        return 0;
    }
    if (this->milli() - rx_start_ms > rx_timeout_ms)
      return 0;
  }
}

}  // namespace dali_phy
