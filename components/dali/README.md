This component is dual purpose:
- PlatformIO-compatible Arduino library
- ESPHome custom component

The PlatformIO library only builds the core dali functionality and ignores the esphome component cpp files, while ESPHome uses all cpp code in this folder.

## Bus diagnostics

The DALI bus exposes optional runtime diagnostics for troubleshooting unavailable buses:

```yaml
dali:
  id: dali_bus
  tx_pin: 32
  rx_pin: 33
  discovery: true
  debug_tx_rx: false  # log every TX/RX frame (verbose)
  bus_status:
    name: "DALI Bus OK"
    update_interval: 30s
  diag:
    name: "DALI Diag"
    update_interval: 10s
```

- **`bus_status`** publishes a Home Assistant binary sensor that polls `QUERY_CONTROL_GEAR_PRESENT` (broadcast). It turns off after 3 consecutive failures and triggers automatic bus recovery (`resetBus` + `terminate`) with a 5-minute cooldown.
- **`diag`** publishes live diagnostics: ISR tick rate (~9600/s), PHY state, RX GPIO level, TX bus activity (`tx_act=1` means Manchester was observed on the bus during TX), last backward result (`timeout` / `decode_error` / `reply`), failure and error counters.
- **Inter-frame timing**: the PHY now waits ~9.4 ms of bus idle before transmitting (DALI spec minimum ~9.2 ms).
- **Logs** (tag `dali`): failed transmissions are logged with PHY error codes (`BUS_NOT_IDLE`, `TIMEOUT`, `COLLISION`, …). PHY state is logged periodically at DEBUG level.
- **`dump_config`** reports PHY state, TX error count, and control gear presence at boot.

### Interpreting diagnostics

| Observation | Likely cause |
|---|---|
| `tx_act=0` during failure | TX not modulating bus (opto/driver hardware) |
| `tx_act=1`, `rx_last=timeout` | Bus modulated but no device reply |
| `rx_last=decode_error` | Collision or corrupted backward frame |
| `isr` far from 9600 | Timer ISR disrupted (CPU load, Ethernet) |
| `rx=0` permanently | Stuck bus line low |

## Address initialization

Use `initialize_addresses: true` only while commissioning new devices. Once short addresses are stable, set:

```yaml
initialize_addresses: false
```

Re-running address randomization on every boot can destabilize a bus over time (devices left in INITIALISE mode after interrupted scans, duplicate addresses).
