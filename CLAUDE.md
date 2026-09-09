# Wilduino — project instructions

Animal-borne environmental logger. ATmega328P firmware, C++ / Arduino
framework, built with PlatformIO.

Read `docs/HARDWARE.md` before touching anything hardware-related. It is the
authoritative pin map and is derived from the KiCad netlist, not from vendor
documentation (the vendor site is offline).

---

## Hard constraints — do not violate

**SRAM is 2048 bytes. Flash is 31.5 KB.** These are the binding constraints
on every design decision.

- Report flash and RAM figures after any build. If a change costs more than
  ~200 bytes of either, say so unprompted.
- No `String`. Ever. Use `char[]` and the `F()` macro for literals.
- No `new` / `malloc`. Static allocation only — heap fragmentation on a 2 KB
  device is a field failure, not a warning.
- No floating point in the logging path. Store raw integer sensor counts;
  convert in R.
- Watch out for the 512-byte SdFat block buffer. It is a quarter of RAM.

## Pin map — the traps

Full map in `docs/HARDWARE.md`. The three that bite:

- **D2 is contested** by HP206C INT (J13), TSL2591 INT (J14) and radio DIO0.
  Policy: J13 and J14 stay unfitted, D2 is reserved for the radio, sensor
  conversions are covered by a timed sleep. Do not propose using D2 for a
  sensor interrupt.
- **INT0 cannot wake from power-down on an edge** — only on a low level.
  The HP206C interrupt is active-high, so INT0 will never fire from it. Use
  PCINT if a sensor wake is ever needed.
- **D8 is microSD CS**, and may collide with the onboard SPI flash CS. ⚠
  Unresolved. See `docs/HARDWARE.md` §5.3.

## Power

The device sleeps almost all the time. GPS dominates the average current at
roughly 95% of the budget; optimise there and nowhere else. Micro-optimising
anything that is not GPS is wasted effort.

- Radio and onboard SPI flash **must** be explicitly commanded to sleep in
  `setup()`. Easy to forget, costs tens of µA.
- GPS is power-managed in software via `UBX-RXM-PMREQ`, never by gating VCC
  (there is no gate). Keeping VCC applied preserves ephemeris and gives hot
  starts — this is deliberate, not an oversight.
- No hardware modifications are in scope. Work with the board as built.

## Robustness — this is unattended field hardware

A tag that hangs is a lost deployment and a lost animal-handling event. That
is expensive in a way a desktop bug is not.

- **Every** wait on external hardware needs a timeout. The 2017 code has
  `while(!gpsSerial.available()){}` with no escape; that pattern must never
  reappear.
- Enable the watchdog.
- Fail toward "keep logging". Losing GPS must not stop barometric logging.
- Assume brownouts. The storage layer must survive power loss mid-write.

## Data

- Binary, fixed-width records. Never CSV on the device.
- UTC only. No timezone conversion on the device.
- Log failures explicitly — a GPS fix attempt that failed is data, because
  canopy blocks GPS and canopy use is the response variable.
- Raw sensor counts, never derived quantities. TSL2591: log CH0/CH1 plus
  gain and integration code, never computed lux.
- Version the record format in the file header.
- The R decoder lives in `R/` and is tested against the C++ encoder in the
  `native` environment. Changing the record format means changing both, in
  the same commit.

## Build

    pio run -e tag              # animal-borne firmware
    pio run -e station          # stationary reference firmware
    pio test -e native          # host-side unit tests, no hardware
    pio check                   # cppcheck

Keep logic testable in `native`: hardware access goes behind interfaces that
can be stubbed. Pure logic — record packing, gain ratios, pressure
conversion — must be unit-tested.

## Working style

- Michał is an ecologist and R programmer, not an embedded developer.
  Explain embedded-specific reasoning; don't explain programming basics.
- UK English in all documentation and comments.
- Be concise and cite sources for hardware claims.
- Say plainly when something is untested or unverifiable rather than
  presenting it as working. Mark unverified hardware claims with ⚠.
- Prefer explicit register access over a library when the library's cost in
  flash or RAM is unclear.
