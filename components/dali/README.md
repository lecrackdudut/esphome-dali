This component is dual purpose:
- PlatformIO-compatible Arduino library
- ESPHome custom component

The PlatformIO library only builds the core dali functionality and ignores the esphome component cpp files, while ESPHome uses all cpp code in this folder.

## Physical layer (RMT)

The ESPHome component uses the ESP32 **RMT peripheral** for Manchester-encoded DALI at 1200 baud (Te = 416 µs). This replaces the previous 9600 Hz timer ISR approach, which was sensitive to jitter from Ethernet, WiFi, and other system load.

```yaml
dali:
  id: dali_bus
  tx_pin: 32
  rx_pin: 33
  invert_tx: true   # required for Waveshare Pico-DALI opto interface (default: true)
  invert_rx: false
  boot_delay: 30s
  discovery: true
  bus_status:
    name: "DALI Bus OK"
    update_interval: 30s
  diag:
    name: "DALI Diag"
    update_interval: 10s
```

- **`invert_tx`**: Set to `true` when using an inverting opto driver (Waveshare Pico-DALI2). GPIO high asserts the bus (pulls line low).
- **`invert_rx`**: Set to `true` if the RX opto inverts the sensed bus level.

## Bus diagnostics

The DALI bus exposes optional runtime diagnostics for troubleshooting unavailable buses:

- **`bus_status`** publishes a Home Assistant binary sensor that polls `QUERY_CONTROL_GEAR_PRESENT` (broadcast). It turns off after 3 consecutive failures and triggers automatic bus recovery (`resetBus` + `terminate`) with a 5-minute cooldown.
- **`diag`** publishes live diagnostics: PHY state (`IDLE` / `RX_ARMED` / `TX`), RX GPIO level, idle time since last frame, TX count, TX bus activity (`tx_act=1` means Manchester was observed on the bus during TX), last backward result (`timeout` / `decode_error` / `reply`), failure and error counters.
- **Inter-frame timing**: the RMT PHY waits 20 ms between transactions (IEC 62386 minimum ~9.2 ms).
- **Logs** (tag `dali`): failed transmissions are logged with PHY error codes (`BUS_NOT_IDLE`, `TIMEOUT`, …). PHY state is logged periodically at DEBUG level.
- **`dump_config`** reports PHY state, TX error count, and control gear presence at boot.

### Interpreting diagnostics

| Observation | Likely cause |
|---|---|
| `tx_act=0` during failure | TX not modulating bus (opto/driver hardware) |
| `tx_act=1`, `rx_last=timeout` | Bus modulated but no device reply |
| `rx_last=decode_error` | Collision or corrupted backward frame |
| `state=TX` stuck | RMT channel error (check logs) |
| `rx=0` permanently | Stuck bus line low |

## Address initialization

Use `initialize_addresses: true` only while commissioning new devices. Once short addresses are stable, set:

```yaml
initialize_addresses: false
```

Re-running address randomization on every boot can destabilize a bus over time (devices left in INITIALISE mode after interrupted scans, duplicate addresses).
