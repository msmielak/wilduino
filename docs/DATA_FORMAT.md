# Wilduino — Data Format Specification

**Format version: 1**

Authoritative specification for the on-SD binary format. The C++ encoder
(`src/records.h`) and the R decoder (`R/decode_wilduino.R`) must both conform
to this document, and changes to any of the three require changes to all
three **in the same commit**.

---

## 1. Design constraints

Derived from `HARDWARE.md`:

- **2 KB SRAM.** No buffering of more than one block. Records are encoded and
  written as they are produced.
- **Brownouts are expected.** A torn write must cost one record, not the file.
- **No filesystem writes during logging.** One pre-allocated contiguous file;
  the FAT and directory entry never change while recording (§7.2).
- **Time is a counter, not a timestamp.** Absolute time is reconstructed in R
  by interpolating between GPS anchors (§9.1).
- **Raw sensor counts only.** No unit conversion, no floating point, no
  derived quantities on the device.

---

## 2. File layout

    offset 0      : header block (512 bytes, one full SD block)
    offset 512    : first record
    offset 512+16n: record n
    ...            up to the pre-allocated file length

Fixed 16-byte records give exactly **32 records per 512-byte block**, so a
block write never splits a record.

All multi-byte fields are **little-endian** (native AVR order, so encoding is
a straight memory copy and the decoder does the work).

---

## 3. Header block

Written once at file creation, then rewritten only when the cursor is
persisted. Occupies block 0; unused bytes are zero.

| Offset | Size | Field | Notes |
|---|---|---|---|
| 0 | 8 | `magic` | ASCII `WILDUINO` |
| 8 | 1 | `format_version` | 1 |
| 9 | 1 | `variant` | 0 = tag, 1 = station |
| 10 | 1 | `record_size` | 16 |
| 11 | 1 | `fw_version` | firmware build id |
| 12 | 4 | `device_id` | unique per unit, set at provisioning |
| 16 | 4 | `created_unix` | UTC from first GPS fix; 0 if never fixed |
| 20 | 2 | `sample_interval_s` | nominal seconds between SAMPLE records |
| 22 | 2 | `boot_count` | increments every power-up |
| 24 | 4 | `write_cursor` | byte offset of next record to write |
| 28 | 4 | `cursor_seq` | increments on every cursor persist |
| 32 | 1 | `header_crc` | CRC-8 over bytes 0..31 |
| 33 | 479 | *(zero)* | |

### Cursor recovery on boot

`write_cursor` is persisted infrequently (rewriting block 0 costs a write, and
it is the only block whose contents change), so it lags reality. On boot:

1. Read and CRC-check the header. If invalid, the file is unusable — start a
   new one rather than guessing.
2. Seek to `write_cursor` and **scan forward**, validating each record, until
   one fails. That offset is the true end of data.
3. Write a `BOOT` record there.

This makes the cursor a hint that bounds the scan, never a source of truth.

---

## 4. Record frame

Every record is exactly 16 bytes with the same frame:

| Offset | Size | Field |
|---|---|---|
| 0 | 1 | `sync` — always `0xA5` |
| 1 | 1 | `type` |
| 2 | 3 | `counter` — u24, monotonic wake counter |
| 5 | 10 | `payload` — interpretation depends on `type` |
| 15 | 1 | `crc` — CRC-8 over bytes 0..14 |

### Validity

A record is valid iff `sync == 0xA5` **and** the CRC matches. Both checks are
needed: sync alone would accept corrupted payloads, CRC alone would
occasionally validate erased flash (`0xFF` fill).

Erased/unwritten space reads as `0xFF`, which fails the sync check
immediately — so scanning terminates cleanly at end of data.

### The counter

- u24, so 16,777,215 wakes. At the station's 5 s interval that is **970 days**.
- Increments on **every wake**, whether or not sampling succeeded. It counts
  wake events, not successful samples — this is what makes it monotonic and
  gap-free.
- Wraps to 0 on overflow. The decoder must detect and unwrap.
- Resets to 0 on reboot; the `BOOT` record marks the discontinuity.

---

## 5. Record types

### `0x01` SAMPLE

| Offset | Size | Type | Field | Units |
|---|---|---|---|---|
| 5 | 3 | u24 | `pressure` | raw HP206C, 1 Pa (= 0.01 mbar) |
| 8 | 2 | i16 | `temperature` | raw HP206C, 0.01 °C |
| 10 | 2 | u16 | `light_ch0` | raw TSL2591 full spectrum |
| 12 | 2 | u16 | `light_ch1` | raw TSL2591 infrared |
| 14 | 1 | u8 | `range` | `gain << 4 \| atime` |

Pressure range 300–1200 mbar is 30,000–120,000 in raw units — comfortably
inside u24.

`range` is mandatory: light counts are meaningless without the gain and
integration setting in force. Gain codes 0–3 (1×/25×/428×/9876× nominal),
atime codes 0–5 (100–600 ms).

On the tag variant the light fields are 0 and `range` is `0xFF`.

### `0x02` GPS_TIME

| Offset | Size | Type | Field |
|---|---|---|---|
| 5 | 4 | u32 | `unix_time` — UTC seconds |
| 9 | 2 | u16 | `h_acc_m` — horizontal accuracy estimate, metres |
| 11 | 1 | u8 | `num_sv` — satellites used |
| 12 | 1 | u8 | `fix_type` |
| 13 | 2 | — | *(reserved)* |

**These are the time anchors.** Every GPS_TIME record pins its `counter` value
to absolute UTC; R interpolates between consecutive anchors.

### `0x03` GPS_POS

Always written immediately after a GPS_TIME record from the same fix.

| Offset | Size | Type | Field | Units |
|---|---|---|---|---|
| 5 | 4 | i32 | `latitude` | 1e-7 degrees |
| 9 | 4 | i32 | `longitude` | 1e-7 degrees |
| 13 | 2 | i16 | `altitude_m` | metres above ellipsoid |

Split from GPS_TIME because time plus position exceeds a 10-byte payload, and
a fixed frame size is worth more than packing density.

### `0x04` GPS_FAIL

| Offset | Size | Type | Field |
|---|---|---|---|
| 5 | 2 | u16 | `duration_s` — how long acquisition was attempted |
| 7 | 1 | u8 | `num_sv` — satellites visible at timeout (0 if none) |
| 8 | 7 | — | *(reserved)* |

**Not optional.** Canopy blocks GPS and canopy use is the response variable,
so fix failure is informative data. It also defines the interpolation gaps,
without which the decoder cannot bound its own timing uncertainty.

### `0x05` STATUS

| Offset | Size | Type | Field | Units |
|---|---|---|---|---|
| 5 | 2 | u16 | `rail_mv` | measured 3.3 V rail |
| 7 | 2 | u16 | `cell_mv` | battery |
| 9 | 2 | i16 | `temperature` | 0.01 °C |
| 11 | 4 | — | *(reserved)* |

Both voltages are logged because the LFP cell feeds an LDO in dropout, so the
rail tracks the cell (§13). The relationship must be observed, not assumed.

### `0x06` BOOT

| Offset | Size | Type | Field |
|---|---|---|---|
| 5 | 2 | u16 | `boot_count` |
| 7 | 1 | u8 | `reset_flags` — MCUSR at startup |
| 8 | 1 | u8 | `fw_version` |
| 9 | 6 | — | *(reserved)* |

Marks a counter discontinuity. `reset_flags` distinguishes power-on from
brownout from watchdog reset — a brownout reset in the field is a finding,
not noise.

---

## 6. CRC-8

CRC-8/MAXIM (Dallas/1-Wire): polynomial `0x31` reflected as `0x8C`, initial
value `0x00`, no final XOR.

Chosen over CRC-16 because it fits the frame in 16 bytes. Combined with the
sync byte, a single-byte CRC is adequate for detecting torn writes, which is
the actual failure mode — not bit rot.

---

## 7. Decoder requirements

The R decoder must:

1. Validate the header CRC and refuse to proceed on failure.
2. Stop at the first invalid record; do not skip and continue. A gap means
   something is wrong that silent skipping would hide.
3. Unwrap counter overflow and segment at BOOT records.
4. Reconstruct time by **linear interpolation between GPS_TIME anchors**, and
   report per-sample timing uncertainty derived from anchor spacing.
5. Refuse to extrapolate beyond the last anchor without an explicit flag.
6. Return raw counts alongside converted values, never converted alone.

---

## 8. Change log

| Version | Change |
|---|---|
| 1 | Initial format. |
