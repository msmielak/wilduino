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

### ⚠ No RTC on the Moteino R6

**The MCP7940 RTC is on the Anarduino MiniWireless, not on the shield.** The
shield's `1SEC` net merely routes it from carrier pad 2A (A1) out to P2 pin 5.

The Moteino R6 has no RTC. Migrating to it therefore *loses* the alarm wake
source that the whole low-power scheduling design assumed. Options:

| Option | Notes |
|---|---|
| **External RTC on P2** | P2 already carries SDA, SCL, 3V3, GND and `1SEC` to A1. A DS3231 or MCP7940 module plugs in almost directly, INT/SQW to P2 pin 5. **Preferred.** DS3231 is TCXO-compensated (±2 ppm) — far better than the MCP7940 for tag/station time alignment. |
| Watchdog timer only | Works, no parts. But WDT drift is ±10% and temperature-dependent — unacceptable for differencing two time series (§9). |
| Periodic GPS sync | Necessary regardless, but insufficient alone: it cannot wake the MCU. |
| MoteinoM0 | SAMD21 has an internal RTC that runs in standby. Removes the problem entirely — see §14. |

Note this is *not* a regression introduced by the Moteino: the prototypes'
MCP7940 is not on the shields, so any board without an RTC has the same gap.

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

- The 185 µA divider stays (see §6.1). Costs ~15% of deployment life at a
  15-minute GPS interval, but proportionally more as GPS use is reduced.
- SD idle current cannot be gated. **Card selection is therefore a power
  decision** — choosing a low-idle card is a purchasing choice, not a
  hardware modification, and may be the single cheapest win available.
- Radio and SPI flash *must* be explicitly commanded to sleep in `setup()`.
  This is free and is a well-known trap on Moteino-family boards.

Deferred to any future respin: series MOSFET or 1 MΩ divider (185 µA →
1.85 µA); SD power gating; GPS power gating.

### 6.1 The battery sense divider

R1 and R2 (10 kΩ each) form a resistive divider from V_BAT to GND, tapped at
`V_SENSE` and read on A0. It is a shield component, not part of the
MiniWireless.

It sits *downstream* of the reed switch SW1, so it does not drain the battery
in storage — only while the logger is running.

Because there is no switch in the divider path, current flows continuously:

    I = V_BAT / (R1 + R2) = 3.7 V / 20 kΩ = 185 µA        ⚠ assumes ~3.7 V

That is more than 100x the sleeping MCU (~1.5 µA). Cost over a 30-day
deployment is ~133 mAh.

| Scenario | Without divider | With divider | Loss |
|---|---|---|---|
| GPS every 15 min (~1050 µA) | ~40 days | ~34 days | 15% |
| GPS hourly (~300 µA) | ~139 days | ~86 days | 38% |

Note the trend: the divider is a *fixed* drain, so its relative cost grows as
everything else is optimised. It is not currently the limiting factor; it
would become one if GPS duty were reduced substantially.

Future respin options, in order of preference:

1. Series MOSFET on the divider, gated from a spare GPIO (D4–D7, D9 free).
   Sample hourly, otherwise off. Keeps 10 kΩ so the ADC is happy.
2. 1 MΩ + 1 MΩ (1.85 µA) **plus a 100 nF cap across R2**. The ATmega328P ADC
   specifies source impedance <=10 kΩ for the sample-and-hold to settle; the
   cap acts as a local charge reservoir. Discard the first conversion.
3. 100 kΩ + 100 kΩ (18.5 µA) as a lazier compromise.

### 6.2 microSD card selection is a power decision

**Measured on the bench: smaller, older, lower-capacity cards draw
substantially less idle current than new high-capacity ones.**

Mechanism: idle draw is dominated by the card's controller, not its NAND.
Large modern cards have more complex flash translation layers with larger
mapping tables in controller SRAM, more NAND planes held alive, and
controllers clocked for sequential throughput rather than idle efficiency.
High speed-class ratings make this worse and buy nothing here.

Since the shield cannot gate SD power, **card choice is the only available
lever** — and it is a purchasing decision, not a hardware modification.

Guidance:
- Prefer old, small, low-speed-class cards. 512 MB – 2 GB is ample.
- A 30-day deployment needs roughly 90 MB. Capacity beyond ~1 GB is pure
  power cost.
- Measure idle current on each candidate card and record it below. Cards
  vary by an order of magnitude, and vary between production runs of the
  same model number — so measure the actual cards being deployed, and buy
  spares from the same batch.

| Card | Capacity | Class | Idle current | Write energy | Notes |
|---|---|---|---|---|---|
| *(record measurements here)* | | | | | |

**Cards in hand: ~120 MB (nominal 128 MB), bulk purchase.** Ample — see
§7 for the volume arithmetic.

Secondary benefit: cards of this vintage are likely **SLC or early MLC
NAND**, with endurance in the tens of thousands of write cycles per block,
versus a few hundred to ~1,000 for the TLC/QLC used in modern high-capacity
cards. For a logger making continuous small appends this is a real
reliability advantage on top of the power saving.

#### Two figures of merit, not one

Idle current is only half the cost. Energy *per write* also matters, and
slow cards can lose there:

    slow card:  30 mA x 60 ms = 1.8 mA.s per write
    fast card: 100 mA x  8 ms = 0.8 mA.s per write

Minimise `idle + (writes_per_day x energy_per_write)`. For the base station,
where writes are infrequent, idle dominates and old cards win outright. For
any high-rate variant, re-check.

#### Verification before deployment — mandatory, every card

Cheap old cards are a classic counterfeit category: relabelled, capacity-
faked, or dead stock. A fake reports the advertised size but wraps writes
around, silently overwriting earlier data — a failure that would only
surface after a field season.

1. **Capacity/integrity test every card**, not a sample: `f3write`/`f3read`
   (Linux/macOS) or `h2testw` (Windows). Minutes per card at this size.
2. **Retention check**: write, leave one week unpowered, read back. NAND
   retention is specified around ten years and a card made in ~2008 has
   already spent most of that.
3. **Keep tested cards together and buy spares from the same batch.** Cards
   from different production runs of the same model number behave
   differently; a replacement bought later is not equivalent.

#### Formatting

Cards <=2 GB are **SDSC**, not SDHC — byte-addressed rather than
block-addressed, typically FAT16 or FAT12. SdFat handles all of these.

- Format **FAT16 with a 32 KB allocation unit**. Larger clusters mean fewer
  FAT updates per unit of data written: less power, less corruption
  exposure.
- See §7 for pre-allocation, which matters more.

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

### 7.1 Flash-first is probably NOT available

The Moteino family places the onboard SPI flash chip select on **D8** — the
same pin the shield uses for microSD chip select. Confirmed for the
MoteinoM0 from its schematic (PA06, FLASH-MEM SS); ⚠ strongly suspected for
the MiniWireless, which is a Moteino clone.

If confirmed by `01_flash_probe`, the two devices cannot coexist and buffering
to onboard flash is unavailable. This also explains why the 2017 firmware
wrote directly to SD.

The Moteino R6's optional flash is in any case only 4 Mbit (512 KB) — about
six days of data — versus 16 MB on the MiniWireless. **Order Moteinos
without the flash option**; it is $3 for a chip that cannot be addressed.

Flash-first can be restored later via the adapter board (§14.6).

### 7.2 Therefore: SD-primary, with a pre-allocated contiguous file

Volume, base station, ~10-byte records (pressure, temperature, CH0, CH1):

| Rate | Per day | 120 MB holds |
|---|---|---|
| 0.2 Hz | ~173 kB | **~2 years** |
| 1 Hz | ~864 kB | ~4 months |

Capacity is a non-issue. Robustness is the whole problem.

**The design rule: pre-allocate one large contiguous file at first boot
using SdFat's `preAllocate()`, then write into it at fixed offsets.**

Rationale: the dominant way to lose a deployment is a brownout during a FAT
metadata write, which can take the whole filesystem with it. With a
pre-allocated file, the directory entry and the allocation table **never
change while logging** — only data blocks are written. There is no metadata
window to be interrupted.

Consequences for the firmware:

- Maintain a write cursor in the file header, updated infrequently, with a
  checksum. On boot, scan forward from the last recorded cursor to find the
  true end of data.
- Records must be self-identifying (magic byte + checksum) so a partial
  final record is detectable and skippable.
- Never rely on file size for record count.
- Flush deliberately, not per record. Batch to whole 512-byte blocks.

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

## 9.1 Timing error budget — decision: no RTC, GPS-disciplined watchdog

**Decision: no external RTC.** Time is derived from a monotonic sample
counter, anchored to GPS absolute time at each successful fix, and
interpolated in R. Rationale below.

### The oscillator

In power-down the 16 MHz crystal and all timers stop. The only running
oscillator is the **internal watchdog RC**, nominal **128 kHz at 3 V and
25 °C**. The datasheet states its frequency is voltage-dependent and refers
to Typical Characteristics — i.e. **graphs, not guaranteed limits**. There is
no specified tolerance. Reported real-world spread is about **±12%** on a
nominal 8 s period.

### Three error terms

| Term | Magnitude | Survives GPS anchoring? |
|---|---|---|
| Absolute rate error | ±10–12% | **No** — pure scale factor, calibrated out |
| Voltage dependence | small **because LiFePO4** | mostly no |
| Temperature dependence | ~0.05–0.15 %/°C ⚠ assumed | **Yes** — dominant residual |

LFP holds 3.2–3.3 V across most of its discharge (~0.1 V swing) versus LiPo's
4.2→3.0 V. The cell chemistry choice largely removes the voltage term. This
was not why LFP was chosen, but it matters.

### Residual after anchoring

Only *variation in rate within an interval* survives. For a roughly linear
drift between anchors, peak error at the midpoint is `Δr · T / 8`.
Assuming 0.1 %/°C:

| Interval between fixes | ΔT | Δr | Peak timing error |
|---|---|---|---|
| 1 h | 5 °C | 0.5% | 2 s |
| 8 h | 15 °C | 1.5% | 54 s |
| 12 h | 20 °C | 2.0% | 1.8 min |
| 24 h | 25 °C | 2.5% | 4.5 min |

### Converting to metres

Timing error only matters through the rate of pressure change:

    height error = (dP/dt × ε) / 11.8 Pa/m

At a typical 1 mbar/hour:

| Timing error | Apparent height error |
|---|---|
| 1 min | 0.14 m |
| 5 min | 0.7 m |
| 30 min | 4.2 m |
| 60 min | 8.5 m |

Sensor relative accuracy is already ±0.5 mbar ≈ **±4.2 m** (§11). Anything
under ~5 minutes of timing error is buried in sensor noise. During rapid
frontal passage (3 mbar/h) divide the tolerance by three — still ~10 min.

**Conclusion: GPS-disciplined watchdog timing is adequate. A DS3231 would buy
seconds where minutes are tolerable.** Revisit only for tag deployments where
canopy causes fix gaps beyond ~24 h.

### The actual risk is counter integrity, not oscillator physics

A missed wake, brownout reset, or counter overflow decouples the counter from
time, and no amount of anchoring recovers it. Firmware requirements:

- Increment the counter on **every** wake, whether or not the sample succeeded
- Persist the counter in the file header periodically, checksummed
- Write a reset-marker record on every boot
- Log temperature with every sample (free from the HP206C) so the drift curve
  can be fitted in R rather than assumed linear
- Log failed GPS attempts explicitly — they define the interpolation gaps

### Worth measuring (deferred bench test)

If the GPS breakout exposes **TIMEPULSE (PPS)**, count watchdog periods
against PPS edges while logging HP206C temperature. Half a day in a fridge
and on a windowsill converts the assumed 0.1 %/°C into a measured per-device
curve. ⚠ Whether the breakout routes TIMEPULSE is unverified.

Note P2 pin 5 (`1SEC` → A1) is already a suitable route for PPS.

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
| 3 | ~~microSD idle current~~ **PARTLY RESOLVED** | Older/smaller cards measurably better; see §6.2. Per-card figures still to be tabulated. |
| 4 | Jumper fitment on existing prototypes | Pin assignment |
| 5 | Canopy height and focal species | Whether §11 residual error is acceptable |
| 6 | Deployment duration; tag recoverable? | Duty cycle; calibration strategy |
| 7 | ~~Battery chemistry~~ **RESOLVED** | LiFePO4wered/USB, 550 mAh LFP, 2.0–3.6 V. See §13. Power arithmetic elsewhere assuming 3.7 V LiPo needs revisiting. |
| 8 | 3.3 V rail voltage vs cell voltage ⚠ | Whether PAM-7Q (2.7 V min) browns out late in discharge. See §13. |
| 9 | RTC choice for Moteino R6 ⚠ | Wake source and clock stability. DS3231 on P2 preferred. See §3. |
| 10 | Card verification results | Counterfeit risk; per-card idle and write energy. See §6.2. |

---

## 13. Power source (base station test unit)

**Silicognition LiFePO4wered/USB**, tabbed 550 mAh 3.2 V LiFePO4 cell.

| Property | Value |
|---|---|
| Chemistry | LiFePO4 (LFP) |
| Nominal | 3.2 V, flat for most of discharge |
| Full range | 2.0 V (empty) – 3.6 V (max charge) |
| Capacity | 550 mAh |
| Charger reverse leakage | 0.5 µA (USB disconnected) |
| Discharge temp range | -20 to +60 °C |

The module outputs **raw cell voltage** — it is designed to power 3.3 V
circuits directly, with no regulator, precisely to avoid an LDO's quiescent
draw dominating a sleeping system.

**Consequence for wiring.** The shield routes P1 "BATT IN" → SW1 → V_BAT →
pad 12A (VIN), i.e. into the carrier's MCP1703 LDO. With a LiFePO4 source at
3.2 V the LDO sits in permanent dropout. It does not fail — dropout at
sleep-level currents is only tens of mV — but the 3.3 V rail then tracks the
cell rather than being regulated.

Minimum supply voltages of everything on the rail:

| Part | Min VDD | Note |
|---|---|---|
| **u-blox PAM-7Q** | **2.7 V** | **first to fail as cell depletes** |
| MPU-9255 | 2.4 V | |
| microSD | 2.7 V | card-dependent |
| HP206C | 1.8 V | |
| TSL2591 | 1.8 V | |
| RFM95 | 1.8 V | |
| ATmega328P @16 MHz | 3.3 V in practice | outside official speed grade |
| SAMD21 | 1.62 V | |

Since LFP holds 3.2–3.3 V for most of its discharge, this is workable. The
risk window is the tail end of the curve.

⚠ **Measure the 3.3 V rail against cell voltage early**, and log battery
voltage from the first deployment so the failure threshold is observed rather
than guessed.

Cleaner alternative (no hardware change, just different wiring): feed the
cell to pad **11A (VCC/3V3)** directly instead of 12A, bypassing the LDO
entirely. Standard practice for LFP. Trade-off: no reverse protection, and
the LDO output is back-driven — generally safe with VIN unpowered, but
verify before adopting.

---

## 14. Forward look — MoteinoM0 migration

**Not current work.** Recorded so the analysis is not lost. Current plan is
Moteino R6 (see §15).

Established from the MoteinoM0 R1 schematic and quick reference
(© LowPowerLab 2018).

### 14.1 Why it is attractive

| | ATmega328P | SAMD21G18A |
|---|---|---|
| SRAM | 2 KB | **32 KB** |
| Flash | 31.5 KB | **256 KB** |
| Clock | 16 MHz | 48 MHz |
| ADC | 10-bit | 12-bit |
| RTC | external MCP7940 | **internal, runs in standby** |
| Standby | ~1.5 µA MCU | ~6 µA standalone, +1 µA flash, +1 µA radio |
| UARTs | 1 | 2 + native USB |

The RAM figure is the one that matters: 2 KB is the binding constraint on
everything, and 32 KB removes it.

### 14.2 Pin alignment — does NOT drop in

Geometry matches (rows 20.32 mm apart, 2.54 mm pitch) but the M0 has **16
positions per row, not 13**, and groups pins by function block rather than
port order.

Brute-forced across both row assignments, both rotations, and all four
sliding offsets. Best case:

    5 of 18 required signals align  (A0, A1, A2, A3, GND)
    13 need rerouting: SCK MISO MOSI SDA SCL D8 D0 D1 VCC VIN D2 D3 RST

An adapter PCB is therefore required. 13 flying leads on an animal-borne
device is not acceptable.

### 14.3 M0 pins already committed

| M0 pin | SAMD21 | Committed to |
|---|---|---|
| A2 | PB09 | RFM95 NSS |
| D9 | PA07 | RFM95 DIO0 |
| **D8** | PA06 | **FLASH-MEM SS** — same D8 collision as AVR |
| A5 | PB02 | VMON divider (defeatable via VMON_EN A/B) |
| D13 | PA17 | LED_BUILTIN |
| SDA/SCL | PA22/PA23 | dedicated I²C (not A4/A5) |
| MISO/SCK/MOSI | PA12/PB11/PB10 | dedicated SPI (not D11-13) |
| D0/D1 | PA11/PA10 | Serial1 |
| PB22/PB23 | | second hardware UART — free debug console |

Free after commitments: A0, A1, A3, A4, D2–D7, D10–D12, PB22/PB23.

### 14.4 The battery divider fixes itself

The M0 already carries a **1 MΩ + 1 MΩ** monitor on A5 (~1.85 µA), with
VMON_EN jumpers.

So on migration: **cut the shield's R1/R2 divider out entirely** (one trace
cut, no wire, no added part) and read the battery on the M0's own A5. The
185 µA problem disappears without a MOSFET.

### 14.5 Power input

M0 power chain: LiPo JST → VBAT → Q1 → VIN → MCP1703 → 3.3 V.

- `VUSB` **is** on the header (row 2 pin 1), accepts 3.6–6 V, but sits behind
  a Schottky drop — with a sagging LFP cell this browns out.
- **Feed the JST/VBAT pads instead** — the intended low-drop battery path.
- Note the M0's MCP73831 charger is for LiPo, **not LiFePO4**. Do not use the
  M0's charging path with an LFP cell. Charge externally via the
  LiFePO4wered/USB module.

### 14.6 Proposed adapter design

Small 2-layer PCB, ~42 × 25 mm: 2×13 male underside (into shield), 2×16
female topside (M0). Estimated $2–5/unit in a batch of 20.

Signal routing (shield pad → M0 pin):

| Shield | Signal | → M0 |
|---|---|---|
| 1A | A0 V_SENSE | *unused* — use M0 A5 VMON instead |
| 2A | A1 1SEC | *unused* — MCP7940 is on the Anarduino, not the shield; use SAMD21 internal RTC |
| 3A/9A | GPS-TX | D0/RXI (Serial1) |
| 4A/10A | GPS-RX | D1/TXO (Serial1) |
| 5A | SDA | SDA (PA22) |
| 6A | SCL | SCL (PA23) |
| 7A | *(free)* | spare route |
| 8A | *(free)* | **SD VDD** after shield cut — to load switch |
| 11A | VCC 3V3 | 3.3V |
| 12A | VIN | → M0 VBAT/JST |
| 13A | GND | GND |
| 1B/2B/3B | SCK/MISO/MOSI | SPI_SCK / SPI_MISO / SPI_MOSI |
| 6B | SD CS | **D10** (moved off D8) |
| 11B | MPU INT | D3 |
| 12B | sensor/radio INT | D2 |
| 13B | RST | !RESET |

Added on the adapter:

1. **W25Q128 16 MB SPI flash**, CS on **D8** (M0's native FLASH-MEM SS, so
   the OTA bootloader still works). Order the M0 **without** its 4 Mbit
   flash. Restores the flash-first storage architecture at 32× capacity.
2. **TPS22860 load switch** for SD VDD, enable on **D4**. Kills the
   100–1000 µA card idle draw. Remember to drive MOSI/SCK/CS low when the
   card is unpowered or it back-powers through its ESD diodes.
3. Decoupling: 100 nF per device, 10 µF bulk.

### 14.7 Required shield modifications

Per shield: **two trace cuts, one wire.**

1. Cut R1/R2 divider from V_BAT (§14.4). No wire needed.
2. Cut SD card VDD from the shield 3V3 net; wire it to unconnected pad
   **8A (A7)**.

### 14.8 Variant worth considering

The same adapter with a 2×13 MCU-side footprint would give a **Moteino R6**
the identical power fixes (SD gating, bigger flash) without the pin
translation. One design, two targets, populate as needed.

### 14.9 Still to verify before committing

- M0 physical dimensions vs the shield outline (mechanical clash?)
- M0 LDO current capacity vs GPS burst (~50 mA) + SD write (~100 mA)
- Whether the M0's D2/D3 are EIC-capable for wake-from-standby
- LFP compatibility of the whole M0 power path

---

## 15. Provenance

The shield was commissioned as a custom design and drawn by a third party.
From the netlist header:

    (source "C:/Users/Kyle/Google Drive/EdenWorth/neu-logger/PCB/NEU-Logger.sch")
    (date   "3/12/2015 10:52:44 PM")
    (tool   "Eeschema 4.0.0-rc2-stable")

The designer is **not contactable**. Design *rationale* is therefore
unrecoverable, and the netlist is the authoritative record of intent.

Practical consequences:

- Where this document and the physical board disagree, **the board wins**.
  Verify empirically rather than seeking clarification.
- Undocumented choices should be assumed deliberate-but-unexplained, not
  errors. The 10k/10k divider, for instance, is standard practice; it is
  suboptimal here only because multi-week sleep was probably not stated as a
  requirement in 2015.
- The firmware in `Main_script/` (2017, R. Farrell) postdates the board by
  about two years and was written by a different person again. It should not
  be treated as evidence of design intent.

---

## Sources

- `PCB/NEU-Logger.net`, `PCB/NEU-Logger-cache.lib` (this repository)
- `HP206C/HP206C_DataSheet_EN_V2.0.pdf` (this repository)
- PlatformIO board manifest: <https://docs.platformio.org/en/latest/boards/atmelavr/miniwireless.html>
- u-blox PAM-7Q data sheet and hardware integration manual
- ATmega328P datasheet (wake-source behaviour, §5.2)
- Project specification email, hardware inventory
