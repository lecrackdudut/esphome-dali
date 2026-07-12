This component is dual purpose:
- PlatformIO-compatible Arduino library
- ESPHome custom component

The PlatformIO library only builds the core dali functionality and ignores the esphome component cpp files, while ESPHome uses all cpp code in this folder.

## Discovery and addressing

With `discovery: true`, each DALI control gear found on the bus gets a Home Assistant light entity named by **short address** (decimal 0–63), e.g. `DALI 5` / `dali_5`.

Commissioning follows the IEC 62386-102 sequence when `initialize_addresses: true`:

1. INITIALISE (unassigned devices)
2. RANDOMISE
3. Binary search (SEARCHADDR + COMPARE)
4. PROGRAM SHORT ADDRESS + VERIFY
5. WITHDRAW (repeat until no devices respond)

After short addresses are programmed into your drivers and stable across power cycles, set `initialize_addresses: false` to discover existing addresses without re-randomizing.

## Recovering from interrupted discovery

If discovery was interrupted (watchdog, power loss), control gear may remain in INITIALISE mode and stop responding to normal commands. Enable:

```yaml
dali:
  terminate_on_boot: true
```

This sends TERMINATE once at boot, before discovery or manual light setup. Safe to leave enabled when using manual YAML lights without discovery.

## Home Assistant migration

Firmware that used long-address entity IDs (`dali_a1b2c3`) will register **new** entities after upgrading. Delete the old orphaned entities in Home Assistant once the new `dali_0` … `dali_N` entities are verified.
