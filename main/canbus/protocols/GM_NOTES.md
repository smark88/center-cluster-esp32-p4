# GM CAN notes

Source: [commaai/opendbc](https://github.com/commaai/opendbc) and the
[BogGyver/opendbc](https://github.com/BogGyver/opendbc) fork, `gm_global_a_*.dbc`.

GM Global A splits the data across **two physically different buses**, and that
split is the whole story here.

| | High speed (HS) | Low speed (GMLAN LS) |
|---|---|---|
| Bitrate | 500 kbit/s | 33.3 kbit/s |
| Wiring | twisted pair, standard HS transceiver | **single wire**, needs a SW-CAN transceiver (TH8056 / NCV7356) |
| ID format | 11-bit standard | 29-bit extended |
| Protocol file | `gm.json` | `gm_lowspeed.json` |
| Works today | **yes** | **no — see blockers** |

## gm.json (HS 500k) — working

| Dash value | Frame | Signal | Decode |
|---|---|---|---|
| rpm | `0x0C9` ECMEngineStatus | EngineRPM | byte 1, 2 bytes BE, x0.25 |
| speed | `0x3E9` ECMVehicleSpeed | VehicleSpeed | byte 0, 2 bytes BE, x0.01609344 |
| water temp | `0x4C1` ECMEngineCoolantTemp | EngineCoolantTemp | byte 2, 1 byte, x1.8 offset -40 |

Unit notes:
- `speed` is consumed as **KPH** by `can_mapping_task`, and the DBC signal is in
  mph (x0.01), so the scale folds in the mph->km/h conversion (0.01 x 1.609344).
- Temps are converted straight to **degF** in the scale: the raw byte is degC
  with a -40 offset, and `(raw-40) * 1.8 + 32` simplifies to `raw * 1.8 - 40`.

### EngineRPM bit-width discrepancy — check this first if RPM reads wrong

The two DBC sources disagree:

- **commaai (current):** `EngineRPM : 15|16@0+` — byte aligned, bytes 1-2
- **BogGyver (older fork):** `EngineRPM : 13|14@0+` — starts mid-byte, 14 bits

`gm.json` uses the **16-bit** version, because 14 bits x 0.25 caps out at
4095 RPM, which no engine this cluster targets would fit under. If your car
shows wrong or pinned RPM, the upper two bits of byte 1 are carrying something
else on your model year and the field really is 14 bits — that needs a bitmask,
which the decoder does not currently support (see limitations below).

## gm_lowspeed.json (GMLAN 33.3k) — blocked

This is where the gauge data actually lives. Every signal below is byte
aligned, so the decoder logic itself would handle them fine.

| Dash value | Frame | Signal | Decode |
|---|---|---|---|
| oil pressure | `0x802E0000` Analog_Values_Slow_LS | EngOilPrs | byte 1, x4 kPa -> x0.580152 psi |
| trans temp | `0x802E0000` | TrnOilTmp | byte 3, x1.8 offset -40 (degF) |
| oil temp | `0x802E0000` | EngOilTmp | byte 5, x1.8 offset -40 (degF) |
| intake air temp | `0x802E0000` | EngIntAirTmp | byte 6, x1.8 offset -40 (degF) |
| water temp | `0x802E0000` | EngCltTmp | byte 7, x1.8 offset -40 (degF) |
| rpm | `0x802CA000` Engine_Information_1_LS | EngSpd | byte 2, 2 bytes BE, x0.25 |
| ethanol % | `0x80806000` Alternative_Fuel_Information_LS | FuelAlcoholComp | byte 4, x0.392157 |

### Four blockers, in order of effort

1. **`CAN_ID_MAX` is 2048** (`protocol_loader.h`). `frame_lookup` is a flat
   array indexed by CAN id, so anything >= 0x800 is dropped. `0x802E0000` is
   2.15 billion — a flat array cannot work. Needs a hash map or a small linear
   scan over the active protocol's frames.
2. **Extended frames are discarded.** `canbus_task` filters with
   `if (!message.extd && !message.rtr)`, and the bitrate probe skips
   `msg.extd` too. Both need to accept 29-bit ids.
3. **33333 is not a supported bitrate.** `get_timing()` handles 1M/500k/250k/125k
   and silently falls back to 500k. ESP-IDF's TWAI can do 33.3k but the timing
   config has to be added.
4. **Hardware.** GMLAN low speed is single-wire. The high-speed transceiver this
   project uses cannot read it — that needs a SW-CAN part (TH8056, NCV7356) on a
   separate transceiver, and the ESP32-P4 has one TWAI controller, so reading
   both buses at once means a second CAN interface or an external MCU.

## Not available on GM CAN at all

Confirmed absent across all seven `gm_global_a_*.dbc` files:

- **Fuel level %** — only `FuelLvlLwIO` (a low-fuel *bit*) and `VehFuelRngCalc`
  (range in km). No percentage or volume anywhere. The fuel arc has to stay on
  the analog sender, or be derived from range.
- **Boost / MAP** — only `EngBstPrsInd`, a 0-100% indicator, not a pressure.
- **AFR** — nothing.
- **Fuel pressure** — zero hits.

## OBD-II / Mode 22 fallback

The missing values are reachable by *polling* rather than listening:

| Value | Request | Notes |
|---|---|---|
| Fuel level % | Mode 01 PID `0x2F` | standard SAE J1979 |
| Engine oil temp | Mode 01 PID `0x5C` | standard, not on all GM |
| Oil pressure | Mode 22 PID `0x1470` | GM enhanced |
| Oil temperature | Mode 22 PID `0x1154` | GM enhanced |
| Trans fluid temp | Mode 22 PID `0x1940` | GM enhanced |

**This does not fit the current firmware.** PIDs are request/response: you send
a query to `0x7DF` and read the reply from `0x7E8`. `canbus.c` only ever calls
`twai_receive` — there is no transmit path, no ISO-TP multi-frame handling and
no per-PID state machine. Adding OBD polling is a separate feature, not a
protocol json.

Mode 22 PIDs are also manufacturer proprietary and vary by model year, so any
of the three above may simply not answer on a given car.

Sources: [Chevy SS Forum PID thread](https://www.ssforums.com/threads/obd2-pid-codes.41346/),
[Colorado/Canyon GM PID list](https://www.coloradofans.com/threads/gm-pids-for-scantools-obd-tools-torque-app-xgauge-etc.294922/),
[OBDLink trans temp guide](https://support.obdlink.com/support/solutions/articles/43000708843-display-transmission-temperature-gauge).

## Protocol detection caveat

`protocol_detect()` locks onto the first protocol to match 2 frames. `0x3E9`
also appears in `link_g4x.json`, but GM will win on a GM bus because `0x0C9`
broadcasts at a high rate. Worth knowing if detection ever picks wrong.
