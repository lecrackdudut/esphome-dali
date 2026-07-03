#include <esphome.h>
#include <esp_task_wdt.h>
#include "esphome_dali.h"
#include "esphome_dali_light.h"
#include "port.h"

//static const char *const TAG = "dali";
static const bool DEBUG_LOG_RXTX = false; // NOTE: Will probably trigger WDT

using namespace esphome;
using namespace dali;

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

}  // namespace

// Interrupt-driven bi-phase receiver, ported from sehraf/esphome-dali
// (branch gpio_interrupt), itself based on petrinm/ESP32Dali.
//
// One bit on the DALI bus is 833.33us (1200 baud), i.e. 2 TE half-bit periods.

#define TE (417)                                // half bit time in us

// Relaxed receive timing (strict spec would be TE +/- 42us, 2TE +/- 83us)
#define MIN_TE      (300)                       // minimum half bit time
#define MAX_TE      (550)                       // maximum half bit time
#define MIN_2TE     (2*TE - (2*(TE/5)))         // minimum full bit time
#define MAX_2TE     (2*TE + (2*(TE/5)))         // maximum full bit time

// Each received half-bit level is stored as a 2-bit symbol in DaliInterruptState::frame
#define BI_PHASE_HIGH   0b01
#define BI_PHASE_LOW    0b10
#define BI_PHASE_MASK   0b11

#define BACKWARD_FRAME_BIT_LENGTH 18            // (1 start bit + 8 data bits) * 2 symbols per bit

void IRAM_ATTR DaliInterruptState::gpio_intr(DaliInterruptState* state) {
    if (state->bitcount >= BACKWARD_FRAME_BIT_LENGTH) {
        // Already received enough bits
        return;
    }

    // Read pin state and timestamp
    uint32_t ts = micros();
    bool level = state->rx_pin.digital_read();

    if (state->timestamp == 0) {
        // First edge: start bit (should be low)
        state->frame <<= 1;
        state->frame |= level ? BI_PHASE_HIGH : 0;
        state->bitcount++;
    } else {
        // Ongoing reception: classify the pulse width as TE or 2TE
        uint32_t diff = ts - state->timestamp;

        if (MIN_2TE < diff && diff < MAX_2TE) {
            // 2TE pulse: shift two identical symbols
            state->frame <<= 2;
            state->frame |= level ? BI_PHASE_HIGH : BI_PHASE_LOW;
            state->bitcount += 2;
        } else if (MIN_TE < diff && diff < MAX_TE) {
            // TE pulse: shift one symbol matching the current level
            state->frame <<= 1;
            if (level) {
                state->frame |= BI_PHASE_HIGH;
            }
            state->bitcount += 1;
        } else {
            // Not a valid DALI bi-phase pulse: abort reception.
            // receiveBackwardFrame() will reject the zeroed frame.
            state->bitcount = BACKWARD_FRAME_BIT_LENGTH;
            state->frame = 0;
        }
    }

    // Save timestamp for next edge
    state->timestamp = ts;
}

void DaliInterruptState::reset() {
    this->frame = 0;
    this->bitcount = 0;
    this->timestamp = 0;
}

// Prints the DALI QUERY_STATUS (0x90) response in a human-readable format
static void print_dali_status(uint8_t status, uint8_t address) {
    DALI_LOGI("DALI[%.2d] Status: %s | %s | %s | %s | %s | %s | %s | %s",
        address,
        (status & STATUS_BALLAST_OK) ? "Gear Failed" : "Gear OK",
        (status & STATUS_LAMP_FAILURE) ? "Lamp Failed" : "Lamp OK",
        (status & STATUS_LAMP_ON) ? "Lamp On" : "Lamp Off",
        (status & STATUS_LIMIT_ERROR) ? "Limit Error" : "Level OK",
        (status & STATUS_FADE_STATE) ? "Fading" : "Not Fading",
        (status & STATUS_RESET_STATE) ? "Reset State" : "Not Reset",
        (status & STATUS_MISSING_SHORT_ADDRESS) ? "No Address" : "Address OK",
        (status & STATUS_POWER_FAILURE) ? "Power Failure" : "Power OK");
}

void DaliBusComponent::setup() {
    m_txPin->pin_mode(gpio::Flags::FLAG_OUTPUT);
    m_rxPin->pin_mode(gpio::Flags::FLAG_INPUT);

    m_interrupt_state.rx_pin = m_rxPin->to_isr();
    m_interrupt_state.reset();
    this->armInterrupt();

    DALI_LOGI("DALI bus ready");

    if (m_discovery) {
        // Optional: reset devices on the bus so we are in a known-good state.
        // Can help if devices are not responding to anything.
        if (false) {
            this->resetBus();
            esp_task_wdt_reset();
        }

        if (dali.bus_manager.isControlGearPresent()) {
            DALI_LOGD("Detected control gear on bus");
        } else {
            DALI_LOGE("No DALI control gear detected on bus!");
            return; // Unlikely to get anything from discovery if no one responds to this
        }

        if (this->m_initialize_addresses != DaliInitMode::DiscoverOnly) {
            if (this->m_initialize_addresses == DaliInitMode::InitializeAll) {
                DALI_LOGI("Randomizing addresses for *all* DALI devices");
                dali.bus_manager.initialize(ASSIGN_ALL); 
            } 
            else if (this->m_initialize_addresses == DaliInitMode::InitializeUnassigned) {
                // Only randomize devices without an assigned short address
                DALI_LOGI("Randomizing addresses for unassigned DALI devices");
                dali.bus_manager.initialize(ASSIGN_UNINITIALIZED); 
            }

            dali.bus_manager.randomize();
            dali.bus_manager.terminate();

            // Seem to need a delay to allow time for devices to randomize...
            delay(50);
        }

        DALI_LOGI("Begin device discovery...");
        dali.bus_manager.startAddressScan(); // All devices

        // Keep track of short addresses to detect duplicates
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
            delay(1); // yield to ESP stack
            esp_task_wdt_reset();

            if (short_addr <= ADDR_SHORT_MAX) {
                DALI_LOGI("  Device %.6x @ %.2x", long_addr, short_addr);

                // Duplicate detection
                if (is_discovered[short_addr]) {
                    if (m_initialize_addresses == DaliInitMode::DiscoverOnly) {
                        DALI_LOGW("  WARNING: Duplicate short address detected!");
                        duplicate_detected = true;
                    }
                    else {
                        // Assign a new address for this
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

                // Withdraw after address is confirmed (spec order: find → program → withdraw)
                dali.bus_manager.withdrawCurrentDevice();

                uint8_t status = dali.port.sendQueryCommand(short_addr, DaliCommand::QUERY_STATUS);
                print_dali_status(status, short_addr);

                // Dynamic component creation (if not defined in YAML)
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

                    // Withdraw after successful address programming
                    dali.bus_manager.withdrawCurrentDevice();

                    DALI_LOGI("  Device %.6x @ %.2x", long_addr, short_addr);

                    uint8_t status = dali.port.sendQueryCommand(short_addr, DaliCommand::QUERY_STATUS);
                    print_dali_status(status, short_addr);
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

    // If no dynamic lights were discovered, disable loop() to avoid unnecessary CPU cycles.
    if (m_dynamic_lights.empty()) {
        this->disable_loop();
    }
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
    // NOTE: Not freeing these strings, they will be owned by LightState.

    auto* light_state = new DynamicDaliLightState { dali_light };
    // set_component_source is codegen-only since ESPHome 2026.4 (uint8_t index into PROGMEM table);
    // dynamic components cannot participate, and it was cosmetic (log source tagging) only.
    light_state->configure_dynamic_entity(name, id, false);
    App.register_light(light_state);
    static_cast<AppRegistrationAccessor&>(App).register_component_(light_state);

    light_state->set_restore_mode(light::LIGHT_RESTORE_DEFAULT_ON);
    light_state->add_effects({});

    // Initialize the DaliLight with the LightState:
    // queries device capabilities (min/max level, color temperature support)
    dali_light->setup_state(light_state);

    // Track for manual loop() driving — dynamic lights are not in ESPHome's
    // compile-time looping_components_ list so their loop() won't be called automatically.
    if (m_dynamic_lights.empty()) {
        this->enable_loop();  // only enable loop when we have lights to drive
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

void DaliBusComponent::armInterrupt() {
    m_rxPin->attach_interrupt(DaliInterruptState::gpio_intr, &m_interrupt_state, gpio::INTERRUPT_ANY_EDGE);
}

void DaliBusComponent::disarmInterrupt() {
    m_rxPin->detach_interrupt();
}

#define QUARTER_BIT_PERIOD 208
#define HALF_BIT_PERIOD 416
#define BIT_PERIOD 833

void DaliBusComponent::writeBit(bool bit) {
    // Output is inverted: HIGH pulls the bus to 0V.
    m_txPin->digital_write(bit ? HIGH : LOW);
    delayMicroseconds(HALF_BIT_PERIOD - 6);
    m_txPin->digital_write(bit ? LOW : HIGH);
    delayMicroseconds(HALF_BIT_PERIOD - 6);
}

void DaliBusComponent::writeByte(uint8_t b) {
    for (int i = 0; i < 8; i++) {
        writeBit(b & 0x80);
        b <<= 1;
    }
}

void DaliBusComponent::resetBus() {
    DALI_LOGD("Resetting bus");
    m_txPin->digital_write(HIGH);
    delay(1000);
    m_txPin->digital_write(LOW);
}

void DaliBusComponent::sendForwardFrame(uint8_t address, uint8_t data) {
    if (DEBUG_LOG_RXTX) {
        DALI_LOGD("TX: %02x %02x", address, data);
        delayMicroseconds(BIT_PERIOD*8);
    }

    // Minimum idle time since the previous frame before we may transmit
    // (>= 22 TE between two forward frames, ~9.2ms)
    const uint32_t min_idle_ms = ((HALF_BIT_PERIOD * 22) / 1000) + 1;
    uint32_t elapsed_ms = millis() - m_last_rx_ts;
    if (elapsed_ms < min_idle_ms) {
        delayMilliseconds(min_idle_ms - elapsed_ms);
    }

    // Don't receive our own transmission
    this->disarmInterrupt();
    m_interrupt_state.reset();

    {
        // This is timing critical
        InterruptLock lock;

        writeBit(1); // START bit
        writeByte(address);
        writeByte(data);
        m_txPin->digital_write(LOW);
    }

    // Non critical delay
    delayMicroseconds(HALF_BIT_PERIOD*2);
    this->armInterrupt();
    m_last_rx_ts = millis();
    delayMicroseconds(BIT_PERIOD*4); // Optional, for clarity in scope trace
}

uint8_t DaliBusComponent::receiveBackwardFrame(unsigned long timeout_ms) {
    uint8_t data = 0;

    // Wait until the RX interrupt has captured a complete backward frame
    uint32_t start_time = millis();
    while (m_interrupt_state.bitcount < BACKWARD_FRAME_BIT_LENGTH) {
        delayMilliseconds(1);
        if (millis() - start_time > timeout_ms) {
            if (DEBUG_LOG_RXTX) {
                DALI_LOGD("RX: timeout (NACK)");
            }
            return 0;
        }
    }

    uint32_t raw_data = m_interrupt_state.frame;
    m_interrupt_state.reset();

    if (DEBUG_LOG_RXTX) {
        DALI_LOGD("RX raw: %05x", raw_data);
    }

    // Decode the 2-bit bi-phase symbols into a byte (LSB symbol pair first)
    for (int i = 0; i < 8; i++) {
        data >>= 1;
        switch (raw_data & BI_PHASE_MASK) {
        case BI_PHASE_HIGH:
            data |= 0x80;
            break;
        case BI_PHASE_LOW:
            break;
        default:
            // Invalid symbol (aborted or corrupted frame)
            return 0;
        }
        raw_data >>= 2;
    }

    if (DEBUG_LOG_RXTX) {
        DALI_LOGD("RX: %02x", data);
    }

    m_last_rx_ts = millis();

    // Minimum time before we can send another forward frame
    delayMicroseconds(BIT_PERIOD*7);
    return data;
}
