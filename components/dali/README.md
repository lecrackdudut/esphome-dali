This component is dual purpose:
- PlatformIO-compatible Arduino library
- ESPHome custom component

The PlatformIO library only builds the core dali functionality and ignores the esphome component cpp files, while ESPHome uses all cpp code in this folder.

## Discovery and addressing

With `discovery: true`, each DALI control gear found on the bus gets a Home Assistant light entity named by **short address** (decimal 0–63), e.g. `DALI 5` / `dali_5`.

## Debug mode

Set `debug: true` on the bus to create diagnostic entities in Home Assistant:

- **Traffic sniffer**: TX/RX hex + decoded command names, rolling bus log
- **Counters**: TX frames, RX replies, no-reply, TX errors
- **Bus health**: gear present, RX idle (bus high), last got reply
- **Target address** number: `0–63` short address, `127` = broadcast
- **Buttons**: Query present/status/level, Recall max/min, Off, DAPC 0/50/100%, Identify, Scan short addrs, Dump status, Blink, COMPARE probe, Reset bus, Terminate

Useful when `Control Gear: not present` — press Query Present / Scan / COMPARE and watch whether replies appear on RX.

## Commissioning mode

Set `commissioning: true` on the bus to create **config** entities for ballast programming (separate from debug):

- **Numbers**: Comm Target Address (`0–63` / `127`), Group (`0–15`), Scene (`0–15`), Fade Time Set (`0–15`), Fade Rate Set (`1–15`)
- **Buttons**: Add/Remove group, Query groups, Query/Set fade time & rate, Store/Remove/Query/Goto scene
- **Results**: Comm Groups (e.g. `0,2,5`), Comm Scene Level (`0–254` or `MASK`), Comm Fade Time / Fade Rate sensors

Typical flow: set target address → set group/scene/fade numbers → press the action button → read result sensors.

**Group control at runtime** does not need commissioning entities — declare a light with a group address (`64–79` = groups `0–15`, i.e. `0x40 | group`):

```yaml
light:
  - platform: dali
    name: "DALI Group 0"
    address: 64  # group 0
```

**Scenes** are activated with the **Goto Scene** button (not as `light` entities). Store Scene saves the ballast's current level into the selected scene slot.

## Address assignment

Short-address commissioning follows the IEC 62386-102 sequence when `initialize_addresses` is enabled:

| Value | Behaviour |
|---|---|
| `false` (default) | Discover existing short addresses only |
| `true` / `unassigned` | Assign short addresses to devices that have none |
| `all` | **Factory-style reset**: re-randomize and reassign every device to free slots (0,1,2…, skipping YAML-reserved addresses) |

Sequence for `unassigned` / `all`:

1. INITIALISE (unassigned only, or all devices)
2. RANDOMISE
3. Binary search (SEARCHADDR + COMPARE)
4. PROGRAM SHORT ADDRESS + VERIFY
5. WITHDRAW (repeat until no devices respond)

**Warning:** `initialize_addresses: all` rewrites every short address on the bus each boot. Use once to clean up a messy bus, then set `false` or `unassigned`.

After short addresses are programmed into your drivers and stable across power cycles, set `initialize_addresses: false` to discover existing addresses without re-randomizing.

## Home Assistant migration

Firmware that used long-address entity IDs (`dali_a1b2c3`) will register **new** entities after upgrading. Delete the old orphaned entities in Home Assistant once the new `dali_0` … `dali_N` entities are verified.
