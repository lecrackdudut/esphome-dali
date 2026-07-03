#pragma once

#include <vector>
#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/components/light/light_state.h"
#include "dali.h"

namespace esphome {
namespace dali {

enum class DaliInitMode {
    DiscoverOnly,
    InitializeUnassigned,
    InitializeAll
};

/// @brief State shared between the RX edge-interrupt handler and the main code.
/// The ISR decodes the bi-phase (Manchester) encoded backward frame into
/// a stream of 2-bit symbols stored in `frame`.
struct DaliInterruptState {
    volatile uint32_t frame;
    volatile uint32_t bitcount;
    volatile uint32_t timestamp;

    ISRInternalGPIOPin rx_pin;

    static void gpio_intr(DaliInterruptState* state);
    void reset();
};

class DaliBusComponent : public Component, public DaliPort {
public:
    DaliBusComponent()
        : Component { }
        , dali { *this }
    { }

    void setup() override;
    void loop() override;
    void dump_config() override;

    void set_tx_pin(GPIOPin* tx_pin) { m_txPin = tx_pin; }
    void set_rx_pin(InternalGPIOPin* rx_pin) { m_rxPin = rx_pin; }

    /// @brief Perform automatic device discovery on setup.
    /// Light components will automatically be created and appear in HomeAssistant
    void do_device_discovery() { m_discovery = true; }

    /// @brief Initialize long and short addresses for devices on the bus.
    /// @param mode 
    //          InitializeUnassigned - only devices that do not yet have an assigned short address
    ///         InitializeAll - all devices on the bus
    /// @note
    void do_initialize_addresses(DaliInitMode mode = DaliInitMode::InitializeUnassigned) { m_initialize_addresses = mode; }

    // NOTE: Must have a higher priority number than the components that depend on this.
    // ie, this must be initialized first.
    float get_setup_priority() const override { return setup_priority::HARDWARE; }

    void register_static_addr(short_addr_t short_addr) {
        if (short_addr < ADDR_SHORT_MAX) {
            m_addresses[short_addr] = 0xFFFFFF;
        }
    }

    DaliMaster dali;

public: // DaliPort
    void resetBus() override;
    void sendForwardFrame(uint8_t address, uint8_t data) override;
    uint8_t receiveBackwardFrame(unsigned long timeout_ms = 100) override;

private:
    void writeBit(bool bit);
    void writeByte(uint8_t b);

    void armInterrupt();
    void disarmInterrupt();

    void create_light_component(short_addr_t short_addr, uint32_t long_addr);

    InternalGPIOPin* m_rxPin;
    GPIOPin* m_txPin;
    uint32_t m_last_rx_ts = 0;

    DaliInterruptState m_interrupt_state;

    bool m_discovery = false;
    DaliInitMode m_initialize_addresses = DaliInitMode::DiscoverOnly;
    uint32_t m_addresses[ADDR_SHORT_MAX+1] = {0};

    // Dynamic lights created during discovery are not in ESPHome's looping_components_
    // (that list is fixed at compile time). We drive their loop() manually.
    std::vector<light::LightState*> m_dynamic_lights;
};

}  // namespace dali
}  // namespace esphome
