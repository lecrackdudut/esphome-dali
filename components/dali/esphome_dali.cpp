#include <esphome.h>
#include <esp_task_wdt.h>
#include <driver/gpio.h>
#include "esphome_dali.h"
#include "esphome_dali_light.h"
#include "port.h"

#if defined(USE_ESP32) && defined(ARDUINO)
#include <esp32-hal-timer.h>
#endif

static const bool DEBUG_LOG_RXTX = false;

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
    this->init_phy();
    this->start_phy_timer();
    DALI_LOGI("DALI bus ready (sampled PHY @ 9600 Hz)");

    if (m_discovery) {
        if (false) {
            this->resetBus();
            esp_task_wdt_reset();
        }

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

                if (m_addresses[short_addr]) {
                    DALI_LOGD("  Ignoring, already defined");
                }
                else {
                    m_addresses[short_addr] = long_addr;
                    create_light_component(short_addr, long_addr);
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
    dali_light->set_address(short_addr);

    const int MAX_STR_LEN = 20;
    char* name = new char[MAX_STR_LEN];
    char* id = new char[MAX_STR_LEN];
    snprintf(name, MAX_STR_LEN, "DALI Light %d", short_addr);
    snprintf(id, MAX_STR_LEN, "dali_light_%.6x", long_addr);

    auto* light_state = new DynamicDaliLightState { dali_light };
    light_state->configure_dynamic_entity(name, id, false);
    App.register_light(light_state);
    static_cast<AppRegistrationAccessor&>(App).register_component_(light_state);

    light_state->set_restore_mode(light::LIGHT_RESTORE_DEFAULT_ON);
    light_state->add_effects({});

    dali_light->setup_state(light_state);

    if (m_dynamic_lights.empty()) {
        this->enable_loop();
    }
    m_dynamic_lights.push_back(light_state);

    DALI_LOGI("Created light component '%s' (%s)", name, id);
#else
    DALI_LOGE("Not compiled with light component. Add `light:` to YAML.");
#endif
}

void DaliBusComponent::loop() {
    for (auto* light : m_dynamic_lights) {
        light->loop();
    }
}

void DaliBusComponent::dump_config() {
    static const char *const TAG = "dali";
    ESP_LOGCONFIG(TAG, "DALI Bus:");
    LOG_PIN("  TX Pin: ", m_txPin);
    LOG_PIN("  RX Pin: ", m_rxPin);
    ESP_LOGCONFIG(TAG, "  PHY: sampled (9600 Hz)");
    ESP_LOGCONFIG(TAG, "  Discovery: %s", m_discovery ? "enabled" : "disabled");
    ESP_LOGCONFIG(TAG, "  Control Gear: %s", dali.bus_manager.isControlGearPresent() ? "present" : "not present");
    bool any = false;
    for (int i = 0; i <= ADDR_SHORT_MAX; i++) {
        if (m_addresses[i] > 0) {
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
    if (DEBUG_LOG_RXTX) {
        DALI_LOGD("TX: %02x %02x", address, data);
    }

    uint8_t frame[2] = { address, data };
    this->m_phy.tx_wait(frame, 16, 500);
}

uint8_t DaliBusComponent::receiveBackwardFrame(unsigned long timeout_ms) {
    uint8_t data = this->m_phy.receive_backward(timeout_ms);

    if (DEBUG_LOG_RXTX) {
        DALI_LOGD("RX: %02x", data);
    }

    return data;
}
