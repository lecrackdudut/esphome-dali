#pragma once

#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/components/light/light_state.h"
#include <vector>
#include "dali.h"
#include "dali_phy.h"

namespace esphome {
namespace dali {

enum class DaliInitMode {
    DiscoverOnly,
    InitializeUnassigned,
    InitializeAll
};

class DaliBusComponent : public esphome::Component, public ::DaliPort {
public:
    DaliBusComponent()
        : esphome::Component { }
        , dali { *this }
    { }

    void setup() override;
    void loop() override;
    void dump_config() override;
    void on_shutdown() override;

    void set_tx_pin(esphome::GPIOPin* tx_pin) { m_txPin = tx_pin; }
    void set_rx_pin(esphome::GPIOPin* rx_pin) { m_rxPin = rx_pin; }

    /// @brief Perform automatic device discovery on setup.
    /// Light components will automatically be created and appear in HomeAssistant
    void do_device_discovery() { m_discovery = true; }

    /// @brief Initialize long and short addresses for devices on the bus.
    /// @param mode 
    //          InitializeUnassigned - only devices that do not yet have an assigned short address
    ///         InitializeAll - all devices on the bus
    /// @note
    void do_initialize_addresses(DaliInitMode mode = DaliInitMode::InitializeUnassigned) { m_initialize_addresses = mode; }

    /// @brief Send TERMINATE at boot to exit INITIALISE mode on control gear (e.g. after interrupted discovery).
    void set_terminate_on_boot(bool terminate_on_boot) { m_terminate_on_boot = terminate_on_boot; }

    void set_boot_delay(uint32_t boot_delay_ms) { m_boot_delay_ms = boot_delay_ms; }

    /// @brief True once terminate/discovery has finished (or was not configured).
    bool is_bus_init_done() const { return m_deferred_init_done; }

    // NOTE: Must have a higher priority number than the components that depend on this.
    // ie, this must be initialized first.
    float get_setup_priority() const override { return esphome::setup_priority::HARDWARE; }

    void register_static_addr(short_addr_t short_addr) {
        if (short_addr <= ADDR_SHORT_MAX && m_addresses[short_addr] == 0) {
            m_addresses[short_addr] = 0xFFFFFF;
        }
    }

    void register_discovered_addr(short_addr_t short_addr, uint32_t long_addr) {
        if (short_addr <= ADDR_SHORT_MAX) {
            m_addresses[short_addr] = long_addr;
        }
    }

    bool is_addr_reserved_yaml(short_addr_t short_addr) const {
        return short_addr <= ADDR_SHORT_MAX && m_addresses[short_addr] == 0xFFFFFF;
    }

    short_addr_t find_free_short_addr(const bool is_discovered[ADDR_SHORT_MAX + 1]) const {
        for (short_addr_t i = 0; i <= ADDR_SHORT_MAX; i++) {
            if (!is_discovered[i] && !this->is_addr_reserved_yaml(i)) {
                return i;
            }
        }
        return 0xFF;
    }

    DaliMaster dali;

public: // DaliPort
    void resetBus() override;
    void sendForwardFrame(uint8_t address, uint8_t data) override;
    uint8_t receiveBackwardFrame(unsigned long timeout_ms = 100) override;

private:
    void init_phy();
    void start_phy_timer();
    void stop_phy_timer();
    void create_light_component(short_addr_t short_addr, uint32_t long_addr);
    void run_deferred_bus_init();
    void log_bus_diagnostics_();
    bool is_network_ready_() const;

    dali_phy::DaliPhy m_phy;
    void* m_timer{nullptr};

    esphome::GPIOPin* m_rxPin{nullptr};
    esphome::GPIOPin* m_txPin{nullptr};

    bool m_discovery = false;
    bool m_terminate_on_boot = false;
    bool m_deferred_init_pending = false;
    bool m_deferred_init_done = false;
    uint32_t m_boot_delay_ms = 5000;
    uint32_t m_deferred_init_start_ms = 0;
    DaliInitMode m_initialize_addresses = DaliInitMode::DiscoverOnly;
    uint32_t m_addresses[ADDR_SHORT_MAX+1] = {0};

    uint8_t m_discovery_devices_found{0};
    uint8_t m_discovery_lights_created{0};
    bool m_discovery_compare_ok{false};

    // Dynamic lights created during discovery are not in ESPHome's looping_components_
    // (that list is fixed at compile time). We drive their loop() manually.
    std::vector<esphome::light::LightState*> m_dynamic_lights;
};

}  // namespace dali
}  // namespace esphome
