# Wilduino — Hardware Reference

Status: **draft**. Everything marked ⚠ is unverified and must be confirmed on
a physical board before firmware depends on it.

This document records what the hardware actually is, as opposed to what the
design intent was. Most of it is derived by reading `PCB/NEU-Logger.net`
(the KiCad netlist) rather than from any datasheet or vendor page, because
the Anarduino vendor site is no longer reachable.

---

## 1. Overview

Wilduino is an animal-borne environmental logger built as a custom shield
("NEU-Logger") on an Anarduino MiniWireless carrier board.

Two firmware variants are planned from one codebase:

| Variant | Purpose | Sensors | Power |
|---|---|---|---|
| **Tag** | Animal-borne. Vertical position via barometry, coarse position via GPS. | HP206C, GPS, MPU-9255 (accelerometer as wake source only) | Battery, deep sleep between samples |
| **Station** | Stationary reference. Pressure reference for the tag, plus lightscape measurement. | HP206C, TSL2591, GPS (time + position) | Mains or large battery |

The scientific design is *differential*: the tag's absolute pressure is
meaningless on its own, because synoptic weather moves pressure by far more
than the animal does. Height is recovered by differencing tag against
station.

---

## 2. Carrier board — Anarduino MiniWireless

| Property | Value | Source |
|---|---|---|
| MCU | ATmega328P | PlatformIO board manifest |
| Clock | 16 MHz | PlatformIO board manifest |
| Flash | 31.50 KB usable | PlatformIO board manifest |
| SRAM | **2 KB** | PlatformIO board manifest |
| Logic level | 3.3 V | vendor |
| Onboard SPI flash | S25FL127S, 128 Mbit = **16 MB** | project email |
| Onboard radio | RFM69 **or** RFM98W (433 MHz LoRa) ⚠ | project email, ambiguous |

Board identity was confirmed two ways: the PlatformIO manifest, and the
shield footprint (`PCB/NEU.pretty/anarduino_mini_wireless.kicad_mod`) having
26 pads over roughly 20.3 × 30.5 mm — a classic 328P pinout. A 1284P variant
would not fit this footprint.

**⚠ Open: which radio is fitted.** This changes the band (868 vs 433 MHz),
the duty-cycle regime, the achievable range, and the driver library. Resolve
by reading the marking on the module can.

`31.50 KB` of flash is the binding constraint on feature scope. Measure
compiled size at every milestone.

---

## 3. Pin map

Derived from `PCB/NEU-Logger.net`. `M2` is the MiniWireless; pad numbering
(`1A`…`13A`, `1B`…`13B`) comes from the KiCad symbol in
`NEU-Logger-cache.lib`.

| Arduino pin | Netlist pad | Net name | Connects to | Notes |
|---|---|---|---|---|
| A0 | M2 1A | `V_SENSE` | R1/R2 divider from V_BAT | See §6 — always-on drain |
| A1 | M2 2A | `1SEC` | MCP7940 MFP | **Primary wake source.** PCINT9 |
| A2 | M2 3A | `GPS-TX` | GPS TX, jumper J9/J11 | SoftwareSerial RX in v1.4 |
| A3 | M2 4A | `GPS-RX` | GPS RX, jumper J10/J12 | SoftwareSerial TX in v1.4 |
| A4 | M2 5A | `SDA/SDI` | MPU-9255, HP206C, P2 pin 9 | I²C, 10 kΩ pull-up (R3) |
| A5 | M2 6A | `SCL/SCK` | MPU-9255, HP206C, P2 pin 10 | I²C, 10 kΩ pull-up (R4) |
| D0 | M2 9A | via J11 | GPS TX | Hardware UART RX — **preferred** |
| D1 | M2 10A | via J12 | GPS RX | Hardware UART TX — **preferred** |
| D2 | M2 12B | shared | HP206C INT (J13), TSL2591 INT (J14), radio DIO0 | **Oversubscribed — see §5** |
| D3 | M2 11B | `INT-MPU` | MPU-9255 INT | INT1 / PCINT19 |
| D4–D7, D9 | — | unconnected on shield | free | Candidates for onboard flash CS ⚠ |
| D8 | M2 6B | `SS` | microSD CS | ⚠ possible clash with flash CS |
| D10 | M2 4B | unconnected on shield | radio CS (on MiniWireless) | Leave alone |
| D11 | M2 3B | `MOSI` | microSD, radio, flash | SPI bus |
| D12 | M2 2B | `MISO` | microSD, radio, flash | SPI bus |
| D13 | M2 1B | `SCK` | microSD, radio, flash | SPI bus |

### External header P2 (`CONN_01X10`)

The light sensor and GPS are **off-board**, on this header.

| P2 pin | Net |
|---|---|
| 1 | `GPS-RX` |
| 2 | `GPS-TX` |
| 3 | GND |
| 4 | +3V3 |
| 5 | `1SEC` |
| 6 | +3V3 |
| 7 | GND |
| 8 | `INT-2591` |
| 9 | SDA |
| 10 | SCL |

**Note what is absent:** there is no GPS power-enable line and no separate
backup rail. GPS VCC is on the permanent +3V3 net. This forecloses hardware
power-gating of the GPS — see §6.

---

## 4. Components

| Ref | Part | Bus | Notes |
|---|---|---|---|
| U1 | MPU-9255 | I²C | 9DOF. AD0 address select via jumper J8. EOL part. |
| U2 | HP206C | I²C | Barometer + temperature. Datasheet in repo. |
| U4 | microSD socket | SPI | CS = D8 |
| M2 | Anarduino MiniWireless | — | See §2 |
| P2 | 10-pin header | — | GPS + TSL2591 breakout |
| P1/SW1 | reed switch + battery | — | Magnet-activated power switch |
| R1, R2 | 10 kΩ | — | Battery divider — see §6 |
| R3, R4 | 10 kΩ | — | I²C pull-ups |

### Off-board

| Part | Bus | Notes |
|---|---|---|
| u-blox **PAM-7Q-0-000** | UART | u-blox 7, TCXO version, integrated patch antenna. Has V_BCKP, but it is not broken out on P2. |
| TSL2591 | I²C | Ambient light. Station variant only. |

### Solder jumpers

| Jumper | Selects | Recommendation |
|---|---|---|
| J8 | MPU-9255 AD0 (I²C address) | Either; record which |
| J9 / J11 | GPS TX → A2 or D0 | **D0** (hardware UART) |
| J10 / J12 | GPS RX → A3 or D1 | **D1** (hardware UART) |
| J13 | HP206C INT → D2 | **Leave unfitted** — see §5 |
| J14 | TSL2591 INT → D2 | **Leave unfitted** — see §5 |

⚠ Jumper fitment on the existing prototypes has not been inspected. Check
with a loupe or continuity meter before assuming.

---

## 5. Known conflicts and traps

### 5.1 D2 is oversubscribed

Three interrupt sources want D2: the HP206C (via J13), the TSL2591 (via
J14), and the radio's DIO0 on the MiniWireless. Only one can have it.

**Resolution (no hardware change):** leave J13 and J14 unfitted, reserve D2
for the radio, and cover sensor conversion times with a timed sleep instead
of an interrupt. The HP206C conversion at OSR 1024 is 32.8 ms; a ~40 ms
watchdog sleep covers it. The cost is negligible next to GPS.

### 5.2 INT0 cannot wake from power-down on an edge

On the ATmega328P, INT0/INT1 can only wake from power-down when configured
as **low-level** triggered — edge detection needs the I/O clock, which is
stopped. Pin-change interrupts (PCINT) are asynchronous and wake on either
edge.

The HP206C's interrupts are **active-high**, so a low-level INT0 wake can
never fire from it. If the HP206C interrupt is ever used, it must be via
**PCINT18**, not INT0.

The MCP7940 alarm on A1 is PCINT9 and is fine. Its polarity is configurable
via the ALMPOL bit.

### 5.3 ⚠ Possible flash / SD chip-select collision

The shield puts microSD CS on **D8**. The MiniWireless carries the
S25FL127S on the same SPI bus with its own CS, and on some boards in this
family that CS is also D8.

Could not be confirmed from documentation (vendor site down). **Resolve
empirically** with `01_flash_probe.ino`, which walks a CS across D4–D9 and
issues JEDEC RDID (`0x9F`). The S25FL127S should answer `0x01 0x20 0x18`.

If they collide, the flash-buffered storage architecture (§7) is not
available without rework.

### 5.4 SD card back-powering

If SD power is ever gated, MOSI/SCK/CS must be driven **low** at the same
time, or the card back-powers itself through its ESD protection diodes and
continues to draw current.

---

## 6. Power

### Always-on loads that firmware cannot fix

Nothing on the shield is power-switched: MPU-9255, HP206C, microSD and the
GPS all sit on the permanent +3V3 net.

| Load | Current | Comment |
|---|---|---|
| ATmega328P power-down, BOD off | ~1.5 µA | |
| MiniWireless regulator quiescent | ~3 µA | |
| MCP7940 timekeeping | ~1.5 µA | |
| HP206C standby | <0.1 µA | datasheet |
| RFM6x/RFM9x sleep | ~0.1 µA | **only if explicitly commanded** |
| Onboard SPI flash | ~10–25 µA | **only if deep power-down commanded** |
| **microSD idle** | **~100–1000 µA** ⚠ | **highly card-dependent — measure** |
| **R1/R2 divider** | **185 µA** | 10 k + 10 k across V_BAT at 3.7 V |

Current policy is **no hardware changes** until the existing boards are
proven. Consequences:

- The 185 µA divider stays. Costs roughly a quarter of deployment life.
- SD idle current cannot be gated. **Card selection is therefore a power
  decision** — choosing a low-idle card is a purchasing choice, not a
  hardware modification, and may be the single cheapest win available.
- Radio and SPI flash *must* be explicitly commanded to sleep in `setup()`.
  This is free and is a well-known trap on Moteino-family boards.

Deferred to any future respin: series MOSFET or 1 MΩ divider (185 µA →
1.85 µA); SD power gating; GPS power gating.

### GPS power management

GPS dominates the budget — roughly 95% of average current in any realistic
duty cycle. Since VCC cannot be gated, the only route is **software backup
mode via `UBX-RXM-PMREQ`**.

This is fortunate rather than limiting: because VCC stays applied, VCC
continues to supply the module's RTC and battery-backed RAM, so ephemeris is
retained and subsequent starts are **hot starts** (seconds) rather than cold
starts (~30 s). The PAM-7Q's TCXO further helps weak-signal acquisition under
canopy.

Wake from PMREQ backup is by UART RX activity — no extra pin needed.

The v1.4 sketch already sends a PMREQ, but only once daily and with a
hand-pasted hex string. It needs to become per-fix with a computed duration.

---

## 7. Storage architecture

**Flash-first, SD as archive.**

At 0.2 Hz with ~5-byte packed barometric records the tag generates roughly
86 kB/day; GPS at 96 fixes/day adds ~2 kB. Against 16 MB of onboard flash
that is **about six months** of data.

Rationale for buffering in flash rather than writing straight to SD:

1. **Corruption resistance.** A brownout during a FAT metadata write can
   destroy the whole filesystem. A circular log in raw NOR flash has no
   filesystem to corrupt. Mounting the SD weekly rather than per-sample cuts
   exposure by roughly three orders of magnitude.
2. **Write energy.** A 256-byte NOR page program is a few ms at ~20 mA. An
   SD write is 512 bytes plus FAT updates at ~100 mA, after tens of ms of
   card wake latency.
3. **No SD in the per-sample timing path.**

Caveat: the card remains *powered*, so its idle draw persists regardless.
Flash buffering saves write energy, not idle current.

⚠ Contingent on §5.3 being resolved favourably.

---

## 8. Data format principles

- **Binary, fixed-width records.** Not CSV. Roughly 3× less write volume,
  and write volume is current draw.
- **Implicit timestamps** where the sample interval is fixed; periodic
  absolute time records for resynchronisation.
- **UTC only.** No local-time conversion on the device. (v1.4 hard-coded an
  AEST offset with a hand-rolled leap-year routine.)
- **Log failures, not just successes.** GPS fix attempts that failed must be
  recorded with timestamp and satellite count. Canopy blocks GPS, and canopy
  use *is* the response variable, so the missingness is informative and must
  be analysable.
- **Raw sensor counts, not derived quantities.** For the TSL2591, log CH0/CH1
  ADC counts plus gain and integration setting — never computed lux. The
  vendor lux equations are empirical fits for indoor illumination and are
  untrustworthy at moonlight levels. Conversion belongs in R.
- **Version the record format.** A format byte in the file header.
- Decoder lives in R, in this repository, tested against the C++ encoder.

---

## 9. Clock synchronisation

The tag/station difference is a *time-series* difference, so clock alignment
between units is critical.

- Both units set RTC from GPS at startup.
- Log a GPS timestamp alongside RTC time at every fix, so RTC drift can be
  measured and corrected post hoc in R.
- Co-locate tag and station before and after deployment to fit sensor offset
  (see §10).

---

## 10. Calibration required before deployment

| What | Why | How |
|---|---|---|
| Barometer offset, per unit | Per-device offset, ±1.5 mbar/year drift, ±0.5 mbar after reflow | Co-locate tag + station several hours, before and after deployment |
| Barometer temperature response, per unit | Error triples across the temperature range and is **not** common-mode — the tag is warmed by the animal | Characterise each sensor across a temperature range |
| TSL2591 dark current vs temperature | At ~0.001 lux the dark offset is comparable to signal | Dark box, across temperature; correct using HP206C temperature |
| TSL2591 gain-step ratios, per unit | Nominal 1×/25×/428×/9876× deviate by several percent; uncorrected, every range change injects a step correlated with moonrise | Bracketed readings at range changes (implemented in `02_tsl2591_autorange.ino`) |
| Cross-unit light comparability | Only if running multiple stations | Co-locate before and after |

---

## 11. Measurement limits

Pressure gradient near sea level ≈ **8.4 m per mbar** (the HP206C datasheet
uses 8.326 in its own offset table).

| Height | Δ pressure |
|---|---|
| 5 m | 0.6 mbar |
| 15 m | 1.8 mbar |
| 30 m | 3.6 mbar |
| 40 m | 4.8 mbar |

HP206C **relative** accuracy: ±0.5 mbar at 25 °C, ±1.5 mbar over 0–50 °C.
The 0.01 mbar / 0.1 m figures are *resolution*, not accuracy — do not plan
around them.

**Conclusion:** ground-vs-canopy contrast at 20–40 m is comfortably
resolvable. Discriminating strata *within* the canopy at ~5 m granularity is
not.

Error sources surviving the tag−station difference, worst first:

1. **Temperature** — different thermal environments, not common-mode.
2. **Per-unit offset and drift** — fit by co-location.
3. **Wind** — dynamic pressure at 5 m/s ≈ 0.15 mbar ≈ 1.3 m. Canopy is
   windier than the forest floor, so this is a *systematic bias aligned with
   the response variable*. Needs a baffled, membrane-covered vent.
4. **Thermal stratification** of the air column, e.g. nocturnal inversions.

---

## 12. Open questions

| # | Question | Blocks |
|---|---|---|
| 1 | Which radio is fitted — RFM69 or RFM98W? | Radio driver, band, duty cycle, range |
| 2 | Flash CS pin; does it collide with D8? | Storage architecture (§7) |
| 3 | microSD idle current, on the actual card | Power budget, card selection |
| 4 | Jumper fitment on existing prototypes | Pin assignment |
| 5 | Canopy height and focal species | Whether §11 residual error is acceptable |
| 6 | Deployment duration; tag recoverable? | Duty cycle; calibration strategy |

---

## Sources

- `PCB/NEU-Logger.net`, `PCB/NEU-Logger-cache.lib` (this repository)
- `HP206C/HP206C_DataSheet_EN_V2.0.pdf` (this repository)
- PlatformIO board manifest: <https://docs.platformio.org/en/latest/boards/atmelavr/miniwireless.html>
- u-blox PAM-7Q data sheet and hardware integration manual
- ATmega328P datasheet (wake-source behaviour, §5.2)
- Project specification email, hardware inventory
