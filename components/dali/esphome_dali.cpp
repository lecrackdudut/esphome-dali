#include <esphome.h>
#include <esp_task_wdt.h>
#include <driver/gpio.h>
#include "esphome_dali.h"
#include "esphome_dali_light.h"
#include "port.h"

#if defined(USE_ESP32) && defined(ARDUINO)
#include <esp32-hal-timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#endif

using namespace esphome;
using namespace esphome::dali;

namespace {

class AppRegistrationAccessor : public esphome::Application {
public:
    using Application::register_component_;
};

class DynamicDaliLightState : public esphome::light::LightState {
public:
    using esphome::light::LightState::LightState;

    void configure_dynamic_entity(const char* name, const char* object_id, bool disabled_by_default) {
        uint32_t entity_fields = (static_cast<uint32_t>(disabled_by_default) << esphome::ENTITY_FIELD_DISABLED_BY_DEFAULT_SHIFT);
        this->configure_entity_(name, esphome::fnv1_hash_object_id(object_id, std::strlen(object_id)), entity_fields);
    }
};

gpio_num_t s_rx_gpio = GPIO_NUM_NC;
gpio_num_t s_tx_gpio = GPIO_NUM_NC;
dali_phy::DaliPhy* s_phy = nullptr;

gpio_num_t pin_to_gpio(esphome::GPIOPin* pin) {
    return static_cast<gpio_num_t>(static_cast<esphome::InternalGPIOPin*>(pin)->get_pin());
}

uint8_t IRAM_ATTR phy_bus_is_high() {
    return gpio_get_level(s_rx_gpio);
}

void IRAM_ATTR phy_bus_assert() {
    gpio_set_level(s_tx_gpio, 1);
}

void IRAM_ATTR phy_bus_release() {
    gpio_set_level(s_tx_gpio, 0);
}

#if defined(USE_ESP32) && defined(ARDUINO)
void ARDUINO_ISR_ATTR on_dali_timer() {
    if (s_phy != nullptr) {
        s_phy->timer();
    }
}
#endif

}  // namespace

void DaliBusComponent::ensure_bus_mutex() {
#if defined(USE_ESP32) && defined(ARDUINO)
    if (this->m_bus_mutex == nullptr) {
        this->m_bus_mutex = xSemaphoreCreateRecursiveMutex();
    }
#endif
}

bool DaliBusComponent::lock_bus(uint32_t timeout_ms) {
#if defined(USE_ESP32) && defined(ARDUINO)
    this->ensure_bus_mutex();
    if (this->m_bus_mutex == nullptr) {
        return true;
    }
    return xSemaphoreTakeRecursive(static_cast<SemaphoreHandle_t>(this->m_bus_mutex), pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
#else
    return true;
#endif
}

void DaliBusComponent::unlock_bus() {
#if defined(USE_ESP32) && defined(ARDUINO)
    if (this->m_bus_mutex != nullptr) {
        xSemaphoreGiveRecursive(static_cast<SemaphoreHandle_t>(this->m_bus_mutex));
    }
#endif
}

const char *DaliBusComponent::phy_busstate_name(uint8_t busstate) {
    switch (busstate) {
        case 0: return "IDLE";
        case 1: return "RX";
        case 3: return "TX";
        case 4: return "COLLISION_TX";
        default: return "UNKNOWN";
    }
}

void DaliBusComponent::log_phy_snapshot(bool force) {
    const uint32_t now = millis();
    dali_phy::PhySnapshot snap = this->m_phy.get_snapshot();

    if (!force) {
        if (now - this->m_last_phy_log_ms < PHY_LOG_INTERVAL_MS &&
            snap.busstate == this->m_last_logged_busstate) {
            return;
        }
    }

    this->m_last_phy_log_ms = now;
    this->m_last_logged_busstate = snap.busstate;

    DALI_LOGD("PHY state=%s idlecnt=%u rx=%u collisions=%u tx_errors=%u",
              phy_busstate_name(snap.busstate),
              snap.idlecnt,
              snap.rx_gpio_level,
              snap.txcollision,
              this->m_tx_error_count);
}

bool DaliBusComponent::check_bus_health() {
    if (!this->lock_bus()) {
        DALI_LOGW("Bus health check skipped: could not acquire bus lock");
        return false;
    }

    dali.bus_manager.terminate();
    delay(10);
    const bool present = dali.bus_manager.isControlGearPresent();
    this->unlock_bus();

    this->log_phy_snapshot(false);

    if (present) {
        this->m_consecutive_bus_failures = 0;
    } else {
        this->m_consecutive_bus_failures++;
        DALI_LOGW("Bus health check failed (%u consecutive)", this->m_consecutive_bus_failures);
        if (this->m_consecutive_bus_failures >= BUS_STATUS_FAILURE_THRESHOLD) {
            this->attempt_bus_recovery();
        }
    }

    this->update_bus_status_sensor(present);
    return present;
}

void DaliBusComponent::attempt_bus_recovery() {
    DALI_LOGW("Attempting DALI bus recovery (reset + terminate)");
    if (!this->lock_bus()) {
        DALI_LOGW("Bus recovery skipped: could not acquire bus lock");
        return;
    }

    this->resetBus();
    esp_task_wdt_reset();
    dali.bus_manager.terminate();
    delay(50);

    const bool present = dali.bus_manager.isControlGearPresent();
    this->unlock_bus();

    this->log_phy_snapshot(true);

    if (present) {
        DALI_LOGI("Bus recovery succeeded");
        this->m_consecutive_bus_failures = 0;
    } else {
        DALI_LOGE("Bus recovery failed");
    }

    this->update_bus_status_sensor(present);
}

void DaliBusComponent::update_bus_status_sensor(bool bus_ok) {
    if (this->m_bus_status_sensor != nullptr) {
        this->m_bus_status_sensor->publish_state(bus_ok);
    }
}

void DaliBusStatusBinarySensor::setup() {
    if (this->parent_ == nullptr) {
        return;
    }
    this->last_update_ms_ = millis();
    this->parent_->check_bus_health();
}

void DaliBusStatusBinarySensor::loop() {
    if (this->parent_ == nullptr) {
        return;
    }

    const uint32_t now = millis();
    if (now - this->last_update_ms_ < this->update_interval_ms_) {
        return;
    }
    this->last_update_ms_ = now;
    this->parent_->check_bus_health();
}

void DaliBusComponent::init_phy() {
    s_rx_gpio = pin_to_gpio(this->m_rxPin);
    s_tx_gpio = pin_to_gpio(this->m_txPin);
    s_phy = &this->m_phy;

    this->m_txPin->pin_mode(gpio::Flags::FLAG_OUTPUT);
    this->m_rxPin->pin_mode(gpio::Flags::FLAG_INPUT);
    this->m_txPin->digital_write(false);

    // Inverted opto TX: assert = GPIO high, release = GPIO low (same as WS_DALI swapped begin()).
    this->m_phy.begin(phy_bus_is_high, phy_bus_assert, phy_bus_release);
}

void DaliBusComponent::start_phy_timer() {
#if defined(USE_ESP32) && defined(ARDUINO)
    auto* timer = timerBegin(9600000);
    timerAttachInterrupt(timer, &on_dali_timer);
    timerAlarm(timer, 1000, true, 0);
    this->m_timer = timer;
#else
    ESP_LOGE("dali", "Sampled DALI PHY requires ESP32 Arduino framework");
#endif
}

void DaliBusComponent::stop_phy_timer() {
#if defined(USE_ESP32) && defined(ARDUINO)
    if (this->m_timer != nullptr) {
        auto* timer = static_cast<hw_timer_t*>(this->m_timer);
        timerEnd(timer);
        this->m_timer = nullptr;
    }
#endif
    if (s_phy == &this->m_phy) {
        s_phy = nullptr;
    }
}

void DaliBusComponent::setup() {
    this->ensure_bus_mutex();
    this->init_phy();
    this->start_phy_timer();
    DALI_LOGI("DALI bus ready (sampled PHY @ 9600 Hz)");

    this->resetBus();
    esp_task_wdt_reset();
    dali.bus_manager.terminate();
    delay(50);

    if (m_discovery) {
        if (dali.bus_manager.isControlGearPresent()) {
            DALI_LOGD("Detected control gear on bus");
        } else {
            DALI_LOGE("No DALI control gear detected on bus!");
            return;
        }

        if (this->m_initialize_addresses != DaliInitMode::DiscoverOnly) {
            if (this->m_initialize_addresses == DaliInitMode::InitializeAll) {
                DALI_LOGI("Randomizing addresses for *all* DALI devices");
                dali.bus_manager.initialize(ASSIGN_ALL);
            }
            else if (this->m_initialize_addresses == DaliInitMode::InitializeUnassigned) {
                DALI_LOGI("Randomizing addresses for unassigned DALI devices");
                dali.bus_manager.initialize(ASSIGN_UNINITIALIZED);
            }

            dali.bus_manager.randomize();
            dali.bus_manager.terminate();

            delay(50);
        }

        DALI_LOGI("Begin device discovery...");
        dali.bus_manager.startAddressScan();

        // COMPARE probe (devices must be in INITIALISE mode from startAddressScan)
        this->m_discovery_compare_ok = dali.bus_manager.compareSearchAddress(0xFFFFFF);

        bool duplicate_detected = false;
        bool is_discovered[ADDR_SHORT_MAX+1];
        for (int i = 0; i <= ADDR_SHORT_MAX; i++) {
            is_discovered[i] = false;
        }

        uint8_t count = 0;
        short_addr_t short_addr = 0xFF;
        uint32_t long_addr = 0;
        while (dali.bus_manager.findNextAddress(short_addr, long_addr)) {
            count++;
            this->m_discovery_devices_found = count;
            delay(1);
            esp_task_wdt_reset();

            if (short_addr <= ADDR_SHORT_MAX) {
                DALI_LOGI("  Device %.6x @ %.2x", long_addr, short_addr);

                if (is_discovered[short_addr]) {
                    if (m_initialize_addresses == DaliInitMode::DiscoverOnly) {
                        DALI_LOGW("  WARNING: Duplicate short address detected!");
                        duplicate_detected = true;
                    }
                    else {
                        short_addr++;
                        DALI_LOGD("  Duplicate short address detected, assigning a new address: %.2x", short_addr);

                        if (!dali.bus_manager.programShortAddress(short_addr)) {
                            DALI_LOGE("  Could not program short address");
                            dali.bus_manager.withdrawCurrentDevice();
                            short_addr = 0xFF;
                            continue;
                        }
                    }
                }
                else {
                    is_discovered[short_addr] = true;
                }

                dali.bus_manager.withdrawCurrentDevice();

                if (this->is_addr_reserved_yaml(short_addr)) {
                    DALI_LOGD("  Ignoring, already defined in YAML");
                }
                else if (m_addresses[short_addr] != 0) {
                    DALI_LOGD("  Ignoring, already discovered");
                }
                else {
                    this->register_discovered_addr(short_addr, long_addr);
                    create_light_component(short_addr, long_addr);
                    this->m_discovery_lights_created++;
                }
            }
            else if (short_addr == 0xFF) {
                if (m_initialize_addresses == DaliInitMode::DiscoverOnly) {
                    DALI_LOGI("  Device %.6x @ --", long_addr);
                    DALI_LOGW("  No short address assigned!");
                    dali.bus_manager.withdrawCurrentDevice();
                    continue;
                }
                else {
                    short_addr = count;
                    DALI_LOGI("  Assigning short address: %.2x", short_addr);

                    if (!dali.bus_manager.programShortAddress(short_addr)) {
                        DALI_LOGE("  Could not program short address");
                        dali.bus_manager.withdrawCurrentDevice();
                        short_addr = 0xFF;
                        continue;
                    }

                    dali.bus_manager.withdrawCurrentDevice();

                    DALI_LOGI("  Device %.6x @ %.2x", long_addr, short_addr);
                    if (!this->is_addr_reserved_yaml(short_addr) && m_addresses[short_addr] == 0) {
                        this->register_discovered_addr(short_addr, long_addr);
                        create_light_component(short_addr, long_addr);
                        this->m_discovery_lights_created++;
                    }
                }
            }
        }

        DALI_LOGD("No more devices found!");
        dali.bus_manager.endAddressScan();

        if (duplicate_detected) {
            DALI_LOGW("Duplicate short addresses detected on the bus!");
            DALI_LOGW("  Devices may report inconsistent capabilities.");
            DALI_LOGW("  You should fix your address assignments.");
        }
    }

    if (m_dynamic_lights.empty()) {
        this->disable_loop();
    }
}

void DaliBusComponent::on_shutdown() {
    this->stop_phy_timer();
}

void DaliBusComponent::create_light_component(short_addr_t short_addr, uint32_t long_addr) {
#ifdef USE_LIGHT
    DaliLight* dali_light = new DaliLight { this };
    dali_light->set_address(short_addr, false);

    const int MAX_STR_LEN = 32;
    char* name = new char[MAX_STR_LEN];
    char* id = new char[MAX_STR_LEN];
    snprintf(name, MAX_STR_LEN, "DALI %.6x", static_cast<unsigned>(long_addr));
    snprintf(id, MAX_STR_LEN, "dali_%.6x", static_cast<unsigned>(long_addr));

    auto* light_state = new DynamicDaliLightState { dali_light };
    light_state->configure_dynamic_entity(name, id, false);

    const size_t lights_before = App.get_lights().size();
    App.register_light(light_state);
    if (App.get_lights().size() == lights_before) {
        DALI_LOGE("Failed to register light '%s': entity slot limit reached (recompile with higher max_discovered_lights)", name);
        delete light_state;
        delete[] name;
        delete[] id;
        delete dali_light;
        return;
    }

    static_cast<AppRegistrationAccessor&>(App).register_component_(light_state);

    light_state->set_restore_mode(light::LIGHT_RESTORE_DEFAULT_ON);
    light_state->add_effects({});

    // setup_state() runs when the LightState component starts (API/MQTT ready).
    // Do not call it here — 18 immediate bus queries during bus setup can block
    // entity registration and trip the watchdog.

    if (m_dynamic_lights.empty()) {
        this->enable_loop();
    }
    m_dynamic_lights.push_back(light_state);

    DALI_LOGI("Created light '%s' (%s) @ short %.2x", name, id, short_addr);
#else
    DALI_LOGE("Not compiled with light component. Add `light:` to YAML.");
#endif
}

void DaliBusComponent::loop() {
    this->log_phy_snapshot(false);

    for (auto* light : m_dynamic_lights) {
        light->loop();
    }
}

void DaliBusComponent::dump_config() {
    static const char *const TAG = "dali";
    ESP_LOGCONFIG(TAG, "DALI Bus:");
    LOG_PIN("  TX Pin: ", m_txPin);
    LOG_PIN("  RX Pin: ", m_rxPin);
    ESP_LOGCONFIG(TAG, "  PHY: sampled (9600 Hz), driver v2");
    ESP_LOGCONFIG(TAG, "  Discovery: %s", m_discovery ? "enabled" : "disabled");
    if (m_discovery) {
        const char *init_mode = "discover only";
        if (m_initialize_addresses == DaliInitMode::InitializeAll) {
            init_mode = "initialize all";
        } else if (m_initialize_addresses == DaliInitMode::InitializeUnassigned) {
            init_mode = "initialize unassigned";
        }
        ESP_LOGCONFIG(TAG, "  Initialize addresses: %s", init_mode);
        ESP_LOGCONFIG(TAG, "  COMPARE probe: %s", m_discovery_compare_ok ? "ok" : "failed");
        ESP_LOGCONFIG(TAG, "  Devices found: %u", m_discovery_devices_found);
        ESP_LOGCONFIG(TAG, "  Lights created: %u", m_discovery_lights_created);
    }
    ESP_LOGCONFIG(TAG, "  Debug TX/RX: %s", m_debug_tx_rx ? "enabled" : "disabled");
    ESP_LOGCONFIG(TAG, "  TX errors: %u", m_tx_error_count);
    dali_phy::PhySnapshot snap = this->m_phy.get_snapshot();
    ESP_LOGCONFIG(TAG, "  PHY state: %s, idlecnt=%u, rx=%u",
                  phy_busstate_name(snap.busstate), snap.idlecnt, snap.rx_gpio_level);
    ESP_LOGCONFIG(TAG, "  Control Gear: %s", dali.bus_manager.isControlGearPresent() ? "present" : "not present");
    bool any = false;
    for (int i = 0; i <= ADDR_SHORT_MAX; i++) {
        if (m_addresses[i] == 0xFFFFFF) {
            if (!any) {
                ESP_LOGCONFIG(TAG, "  Addresses (YAML):");
                any = true;
            }
            ESP_LOGCONFIG(TAG, "    %.2u = yaml", i);
        } else if (m_addresses[i] > 0) {
            if (!any) {
                ESP_LOGCONFIG(TAG, "  Addresses:");
                any = true;
            }
            ESP_LOGCONFIG(TAG, "    %.2u = %.6x", i, m_addresses[i]);
        }
    }
}

void DaliBusComponent::resetBus() {
    DALI_LOGD("Resetting bus");
    this->m_txPin->digital_write(true);
    delay(1000);
    this->m_txPin->digital_write(false);
    this->m_phy.begin(phy_bus_is_high, phy_bus_assert, phy_bus_release);
}

void DaliBusComponent::sendForwardFrame(uint8_t address, uint8_t data) {
    if (!this->lock_bus()) {
        this->m_tx_error_count++;
        DALI_LOGW("TX dropped (lock timeout): %02x %02x", address, data);
        return;
    }

    if (this->m_debug_tx_rx) {
        DALI_LOGD("TX: %02x %02x", address, data);
    }

    uint8_t frame[2] = { address, data };
    const uint8_t result = this->m_phy.tx_wait(frame, 16, 500);
    if (result != DALI_PHY_OK) {
        this->m_tx_error_count++;
        DALI_LOGW("TX failed (%s): %02x %02x", dali_phy::phy_result_name(result), address, data);
    }

    this->unlock_bus();
}

uint8_t DaliBusComponent::receiveBackwardFrame(unsigned long timeout_ms) {
    if (!this->lock_bus()) {
        DALI_LOGW("RX skipped (lock timeout)");
        return 0;
    }

    const uint8_t data = this->m_phy.receive_backward(timeout_ms);

    if (this->m_debug_tx_rx) {
        DALI_LOGD("RX: %02x", data);
    }

    this->unlock_bus();
    return data;
}
