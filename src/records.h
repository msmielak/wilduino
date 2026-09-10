/*
 * records.h  --  Wilduino on-SD binary record format
 *
 * Implements docs/DATA_FORMAT.md version 1.
 *
 * DELIBERATELY FREE OF HARDWARE DEPENDENCIES.
 * This header includes only <stdint.h> and <string.h> so that it compiles
 * in the PlatformIO `native` environment and can be unit-tested on the host
 * without a board. Anything that touches SPI, I2C or Arduino APIs belongs
 * elsewhere. Keep it that way -- this is the code most expensive to get
 * wrong and the only code that can be tested exhaustively.
 *
 * Wilduino -- GPL-3.0
 */

#ifndef WILDUINO_RECORDS_H
#define WILDUINO_RECORDS_H

#include <stdint.h>
#include <string.h>

/* ================= constants ================= */

#define WILD_FORMAT_VERSION   1
#define WILD_RECORD_SIZE      16
#define WILD_HEADER_SIZE      512
#define WILD_SYNC             0xA5

#define WILD_VARIANT_TAG      0
#define WILD_VARIANT_STATION  1

/* Record types */
#define WILD_REC_SAMPLE       0x01
#define WILD_REC_GPS_TIME     0x02
#define WILD_REC_GPS_POS      0x03
#define WILD_REC_GPS_FAIL     0x04
#define WILD_REC_STATUS       0x05
#define WILD_REC_BOOT         0x06

/* Sentinel for "no light sensor fitted" in the SAMPLE range byte */
#define WILD_RANGE_NONE       0xFF

/* Counter is u24 */
#define WILD_COUNTER_MAX      0x00FFFFFFUL


/* ================= CRC-8/MAXIM ================= */
/*
 * Polynomial 0x31 reflected (0x8C), init 0x00, no final XOR.
 * Bitwise rather than table-driven: a 256-byte table would be an eighth of
 * total SRAM if held in RAM, and the PROGMEM version costs code space for
 * a saving that does not matter at 0.2 Hz.
 */
static inline uint8_t wild_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t b = data[i];
        for (uint8_t j = 0; j < 8; j++) {
            uint8_t mix = (uint8_t)((crc ^ b) & 0x01);
            crc >>= 1;
            if (mix) crc ^= 0x8C;
            b >>= 1;
        }
    }
    return crc;
}


/* ================= little-endian helpers ================= */
/*
 * Explicit byte-by-byte rather than memcpy of native types. AVR is
 * little-endian so memcpy would work, but the native test host might not be,
 * and a format that silently depends on host endianness is a trap. These
 * compile to almost nothing on AVR.
 */

static inline void wild_put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}

static inline void wild_put_u24(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
}

static inline void wild_put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

static inline uint16_t wild_get_u16(const uint8_t *p)
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static inline uint32_t wild_get_u24(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16);
}

static inline uint32_t wild_get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* Signed accessors go through unsigned then cast, so the conversion is
 * implementation-defined in exactly one place rather than scattered. */
static inline void wild_put_i16(uint8_t *p, int16_t v)
{
    wild_put_u16(p, (uint16_t)v);
}
static inline int16_t wild_get_i16(const uint8_t *p)
{
    return (int16_t)wild_get_u16(p);
}
static inline void wild_put_i32(uint8_t *p, int32_t v)
{
    wild_put_u32(p, (uint32_t)v);
}
static inline int32_t wild_get_i32(const uint8_t *p)
{
    return (int32_t)wild_get_u32(p);
}


/* ================= frame ================= */

/* Start a record: sync, type, counter. Leaves payload untouched. */
static inline void wild_frame_begin(uint8_t *rec, uint8_t type, uint32_t counter)
{
    memset(rec, 0, WILD_RECORD_SIZE);   /* reserved bytes must be zero */
    rec[0] = WILD_SYNC;
    rec[1] = type;
    wild_put_u24(&rec[2], counter & WILD_COUNTER_MAX);
}

/* Finish a record: compute CRC over bytes 0..14 into byte 15. */
static inline void wild_frame_end(uint8_t *rec)
{
    rec[15] = wild_crc8(rec, 15);
}

/* A record is valid iff sync matches AND CRC matches.
 *
 * Both checks matter. Sync alone accepts a corrupted payload. CRC alone
 * would occasionally validate a run of erased bytes. Erased SD/flash space
 * reads 0xFF, which fails sync immediately -- so a forward scan terminates
 * cleanly at end of data rather than needing a separate length. */
static inline uint8_t wild_record_valid(const uint8_t *rec)
{
    if (rec[0] != WILD_SYNC) return 0;
    return (uint8_t)(rec[15] == wild_crc8(rec, 15));
}

static inline uint8_t  wild_record_type(const uint8_t *rec)    { return rec[1]; }
static inline uint32_t wild_record_counter(const uint8_t *rec) { return wild_get_u24(&rec[2]); }


/* ================= encoders ================= */
/*
 * Each writes exactly WILD_RECORD_SIZE bytes into `rec`.
 * All values are RAW sensor counts -- no unit conversion happens on the
 * device. See DATA_FORMAT.md section 1.
 */

/* pressure: raw HP206C, 1 Pa units.  temperature: raw HP206C, 0.01 C.
 * ch0/ch1: raw TSL2591.  gain 0-3, atime 0-5.
 * For the tag variant pass ch0=ch1=0 and range=WILD_RANGE_NONE. */
static inline void wild_encode_sample(uint8_t *rec, uint32_t counter,
                                      uint32_t pressure, int16_t temperature,
                                      uint16_t ch0, uint16_t ch1, uint8_t range)
{
    wild_frame_begin(rec, WILD_REC_SAMPLE, counter);
    wild_put_u24(&rec[5],  pressure);
    wild_put_i16(&rec[8],  temperature);
    wild_put_u16(&rec[10], ch0);
    wild_put_u16(&rec[12], ch1);
    rec[14] = range;
    wild_frame_end(rec);
}

/* Pack the TSL2591 gain and integration codes into the SAMPLE range byte. */
static inline uint8_t wild_pack_range(uint8_t gain, uint8_t atime)
{
    return (uint8_t)(((gain & 0x0F) << 4) | (atime & 0x0F));
}
static inline uint8_t wild_range_gain(uint8_t range)  { return (uint8_t)(range >> 4); }
static inline uint8_t wild_range_atime(uint8_t range) { return (uint8_t)(range & 0x0F); }

/* The time anchor. Every one of these pins a counter value to absolute UTC;
 * R interpolates between consecutive anchors. */
static inline void wild_encode_gps_time(uint8_t *rec, uint32_t counter,
                                        uint32_t unix_time, uint16_t h_acc_m,
                                        uint8_t num_sv, uint8_t fix_type)
{
    wild_frame_begin(rec, WILD_REC_GPS_TIME, counter);
    wild_put_u32(&rec[5], unix_time);
    wild_put_u16(&rec[9], h_acc_m);
    rec[11] = num_sv;
    rec[12] = fix_type;
    wild_frame_end(rec);
}

/* Always written immediately after a GPS_TIME from the same fix. */
static inline void wild_encode_gps_pos(uint8_t *rec, uint32_t counter,
                                       int32_t lat_1e7, int32_t lon_1e7,
                                       int16_t alt_m)
{
    wild_frame_begin(rec, WILD_REC_GPS_POS, counter);
    wild_put_i32(&rec[5], lat_1e7);
    wild_put_i32(&rec[9], lon_1e7);
    wild_put_i16(&rec[13], alt_m);
    wild_frame_end(rec);
}

/* Failed fixes are data, not noise -- they define the interpolation gaps and
 * they correlate with canopy, which is the response variable. */
static inline void wild_encode_gps_fail(uint8_t *rec, uint32_t counter,
                                        uint16_t duration_s, uint8_t num_sv)
{
    wild_frame_begin(rec, WILD_REC_GPS_FAIL, counter);
    wild_put_u16(&rec[5], duration_s);
    rec[7] = num_sv;
    wild_frame_end(rec);
}

/* Both voltages: the LFP cell feeds an LDO sitting in dropout, so the rail
 * tracks the cell rather than being regulated. Observe, do not assume. */
static inline void wild_encode_status(uint8_t *rec, uint32_t counter,
                                      uint16_t rail_mv, uint16_t cell_mv,
                                      int16_t temperature)
{
    wild_frame_begin(rec, WILD_REC_STATUS, counter);
    wild_put_u16(&rec[5], rail_mv);
    wild_put_u16(&rec[7], cell_mv);
    wild_put_i16(&rec[9], temperature);
    wild_frame_end(rec);
}

/* reset_flags is MCUSR captured at startup. A brownout reset in the field is
 * a finding, not noise -- which is why it is recorded rather than cleared. */
static inline void wild_encode_boot(uint8_t *rec, uint32_t counter,
                                    uint16_t boot_count, uint8_t reset_flags,
                                    uint8_t fw_version)
{
    wild_frame_begin(rec, WILD_REC_BOOT, counter);
    wild_put_u16(&rec[5], boot_count);
    rec[7] = reset_flags;
    rec[8] = fw_version;
    wild_frame_end(rec);
}


/* ================= file header ================= */

typedef struct {
    uint8_t  format_version;
    uint8_t  variant;
    uint8_t  record_size;
    uint8_t  fw_version;
    uint32_t device_id;
    uint32_t created_unix;
    uint16_t sample_interval_s;
    uint16_t boot_count;
    uint32_t write_cursor;
    uint32_t cursor_seq;
} wild_header_t;

/* Serialise into a full 512-byte block. Caller supplies the buffer -- on a
 * 2 KB device the caller almost certainly wants to reuse the SD block buffer
 * rather than have one allocated here. */
static inline void wild_encode_header(uint8_t *blk, const wild_header_t *h)
{
    memset(blk, 0, WILD_HEADER_SIZE);
    memcpy(blk, "WILDUINO", 8);
    blk[8]  = h->format_version;
    blk[9]  = h->variant;
    blk[10] = h->record_size;
    blk[11] = h->fw_version;
    wild_put_u32(&blk[12], h->device_id);
    wild_put_u32(&blk[16], h->created_unix);
    wild_put_u16(&blk[20], h->sample_interval_s);
    wild_put_u16(&blk[22], h->boot_count);
    wild_put_u32(&blk[24], h->write_cursor);
    wild_put_u32(&blk[28], h->cursor_seq);
    blk[32] = wild_crc8(blk, 32);
}

/* Returns 1 on success, 0 if magic or CRC is wrong.
 *
 * On failure the correct response is to start a NEW file, not to guess at
 * the contents. A header we cannot trust makes every offset in the file
 * meaningless. */
static inline uint8_t wild_decode_header(const uint8_t *blk, wild_header_t *h)
{
    if (memcmp(blk, "WILDUINO", 8) != 0) return 0;
    if (blk[32] != wild_crc8(blk, 32))   return 0;

    h->format_version    = blk[8];
    h->variant           = blk[9];
    h->record_size       = blk[10];
    h->fw_version        = blk[11];
    h->device_id         = wild_get_u32(&blk[12]);
    h->created_unix      = wild_get_u32(&blk[16]);
    h->sample_interval_s = wild_get_u16(&blk[20]);
    h->boot_count        = wild_get_u16(&blk[22]);
    h->write_cursor      = wild_get_u32(&blk[24]);
    h->cursor_seq        = wild_get_u32(&blk[28]);
    return 1;
}

#endif /* WILDUINO_RECORDS_H */
