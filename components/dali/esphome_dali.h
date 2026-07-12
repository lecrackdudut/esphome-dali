#pragma once

#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/components/light/light_state.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
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

class DaliBusComponent;

/// Periodic health check exposed as a Home Assistant binary sensor.
class DaliBusStatusBinarySensor : public binary_sensor::BinarySensor, public Component {
public:
    void set_parent(DaliBusComponent *parent) { parent_ = parent; }
    void set_update_interval(uint32_t interval_ms) { update_interval_ms_ = interval_ms; }

    float get_setup_priority() const override { return setup_priority::DATA; }
    void setup() override;
    void loop() override;

private:
    DaliBusComponent *parent_{nullptr};
    uint32_t update_interval_ms_{30000};
    uint32_t last_update_ms_{0};
};

class DaliBusComponent : public esphome::Component, public ::DaliPort {
public:
    static constexpr uint8_t BUS_STATUS_FAILURE_THRESHOLD = 3;
    static constexpr uint32_t PHY_LOG_INTERVAL_MS = 60000;

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
    void set_debug_tx_rx(bool enabled) { m_debug_tx_rx = enabled; }
    void set_bus_status_sensor(DaliBusStatusBinarySensor *sensor) { m_bus_status_sensor = sensor; }

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

    bool check_bus_health();
    void attempt_bus_recovery();
    void log_phy_snapshot(bool force = false);

    DaliMaster dali;

    uint32_t get_tx_error_count() const { return m_tx_error_count; }

public: // DaliPort
    void resetBus() override;
    void sendForwardFrame(uint8_t address, uint8_t data) override;
    uint8_t receiveBackwardFrame(unsigned long timeout_ms = 100) override;

private:
    void init_phy();
    void start_phy_timer();
    void stop_phy_timer();
    void create_light_component(short_addr_t short_addr, uint32_t long_addr);
    void ensure_bus_mutex();
    bool lock_bus(uint32_t timeout_ms = 600);
    void unlock_bus();
    void update_bus_status_sensor(bool bus_ok);
    static const char *phy_busstate_name(uint8_t busstate);

    dali_phy::DaliPhy m_phy;
    void* m_timer{nullptr};
    void* m_bus_mutex{nullptr};

    esphome::GPIOPin* m_rxPin{nullptr};
    esphome::GPIOPin* m_txPin{nullptr};
    DaliBusStatusBinarySensor *m_bus_status_sensor{nullptr};

    bool m_discovery = false;
    bool m_debug_tx_rx = false;
    DaliInitMode m_initialize_addresses = DaliInitMode::DiscoverOnly;
    uint32_t m_addresses[ADDR_SHORT_MAX+1] = {0};

    uint8_t m_discovery_devices_found{0};
    uint8_t m_discovery_lights_created{0};
    bool m_discovery_compare_ok{false};

    uint32_t m_tx_error_count{0};
    uint8_t m_consecutive_bus_failures{0};
    uint32_t m_last_phy_log_ms{0};
    uint8_t m_last_logged_busstate{0xFF};

    // Dynamic lights created during discovery are not in ESPHome's looping_components_
    // (that list is fixed at compile time). We drive their loop() manually.
    std::vector<esphome::light::LightState*> m_dynamic_lights;
};

}  // namespace dali
}  // namespace esphome
