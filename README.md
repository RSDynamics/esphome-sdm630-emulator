# sdm630_emulator

An [ESPHome](https://esphome.io) external component that makes an ESP32 behave like an
**Eastron SDM630 Modbus TCP energy meter**, fed from your own ESPHome sensors (for example
the `dsmr` platform reading a Dutch/Belgian P1 smart meter).

Some EV chargers (tested target: **Peblar**, "Eastron / SDM630" over Modbus TCP) use such a
meter as the grid measurement for dynamic load balancing and solar charging. This lets a P1
reader stand in for the meter, locally and without cloud.

> **Status: untested against real hardware.** The protocol logic is unit tested on a PC
> (`test/test_protocol.cpp`), but the ESPHome socket glue has only been syntax-checked.
> Test with the charger's maximum current set low and no car attached first.

## Installation

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/RSDynamics/esphome-sdm630-emulator
      ref: main
    components: [sdm630_emulator]
```

Or, for a first compile, copy the `components/` folder next to your YAML and use:

```yaml
external_components:
  - source:
      type: local
      path: components
```

## Example

The dsmr sensors need an `id`. Power must be in **watts** (DSMR reports kW, hence the multiply).

```yaml
sensor:
  - platform: dsmr
    voltage_l1:        { id: voltage_l1_sensor, name: "Voltage Phase 1" }
    power_delivered_l1:
      id: power_delivered_l1_sensor
      name: "Power Consumed Phase 1"
      unit_of_measurement: "W"
      filters: [{ multiply: 1000 }]
    power_returned_l1:
      id: power_returned_l1_sensor
      name: "Power Returned Phase 1"
      unit_of_measurement: "W"
      filters: [{ multiply: 1000 }]

sdm630_emulator:
  id: sdm630
  port: 502                # default
  unit_id: 1               # Modbus slave address, default 1
  max_age: 10s             # see "Safety"
  nominal_voltage: 230     # used when a voltage sensor has no value
  voltage_l1: voltage_l1_sensor
  power_delivered_l1: power_delivered_l1_sensor
  power_returned_l1: power_returned_l1_sensor
  # voltage_l2 / power_delivered_l2 / power_returned_l2, and the same for l3, are optional
```

On the charger, choose Modbus TCP, brand **Eastron**, type **SDM630**, the IP address of the
ESP, port 502 and slave address 1.

## What it serves

Function code 4 (input registers) and 3 (holding registers, same data), unit ID as configured
(or 0xFF). Values are big-endian IEEE-754 float32, two registers each:

| Address | Value |
| --- | --- |
| `0x0000` `0x0002` `0x0004` | Voltage L1, L2, L3 (V) |
| `0x0006` `0x0008` `0x000A` | Current L1, L2, L3 (A), signed |
| `0x000C` `0x000E` `0x0010` | Active power L1, L2, L3 (W), signed |
| `0x0034` | Total system power (W) |
| `0x0046` | Frequency (50 Hz) |

Everything else below `0x0200` reads 0, above that returns *illegal data address*.

Sign convention: **import positive, export negative**. Power per phase is
`power_delivered - power_returned`. Current is *computed* as power / voltage, because DSMR
current sensors are unsigned.

The register layout comes from a log of a Peblar charger polling a Modbus logging server:
function code 4, unit 1, three reads of 6 registers at addresses 0, 6 and 12, repeated
continuously.

## Safety

If the L1 import sensor has not updated for `max_age` (default 10 s), every read is answered
with a Modbus *slave device failure* exception instead of old numbers. The charger then falls
back to its own configured fallback current. Set that low (the Peblar default is 6 A).

Also keep the charger's "maximum grid connection" at or below your main fuse.

## Tests

```
g++ -std=c++17 -Wall -Wextra -I components/sdm630_emulator -o test_protocol test/test_protocol.cpp && ./test_protocol
```

## License

MIT, see LICENSE.
