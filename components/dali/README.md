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
```

- **`bus_status`** publishes a Home Assistant binary sensor that polls `QUERY_CONTROL_GEAR_PRESENT` (broadcast). It turns off after 3 consecutive failures and triggers an automatic bus recovery (`resetBus` + `terminate`).
- **Logs** (tag `dali`): failed transmissions are logged with PHY error codes (`BUS_NOT_IDLE`, `TIMEOUT`, `COLLISION`, …). PHY state is logged periodically at DEBUG level.
- **`dump_config`** reports PHY state, TX error count, and control gear presence at boot.

### Interpreting diagnostics

| Observation | Likely cause |
|---|---|
| `rx=0` permanently (GPIO low) | Stuck bus line: failed driver, short, or TX stuck |
| `rx=1`, bus OK off, `BUS_NOT_IDLE` | Parasitic activity / another master |
| `rx=1`, bus OK off, `TIMEOUT` | Missing bus power or all drivers offline |
| bus OK on, lights unresponsive | Duplicate or wrong short addresses |

## Address initialization

Use `initialize_addresses: true` only while commissioning new devices. Once short addresses are stable, set:

```yaml
initialize_addresses: false
```

Re-running address randomization on every boot can destabilize a bus over time (devices left in INITIALISE mode after interrupted scans, duplicate addresses).
