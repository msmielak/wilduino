/*
 * test_records.cpp  --  host-side unit tests for the record format
 *
 * Runs in the PlatformIO `native` environment:  pio test -e native
 * No hardware required. This is why we use PlatformIO rather than the
 * Arduino IDE.
 *
 * These tests exist because the record format is the most expensive thing in
 * the project to get wrong: a format bug is not discovered until data comes
 * back from the field, by which point the deployment is spent.
 *
 * Wilduino -- GPL-3.0
 */

#include <unity.h>
#include "../src/records.h"

/* ---- CRC ---- */

void test_crc8_known_vector(void)
{
    /* CRC-8/MAXIM of "123456789" is 0xA1 -- the standard check value.
     * If this fails, the polynomial or reflection is wrong and every
     * record written so far is undecodable. */
    const uint8_t v[] = "123456789";
    TEST_ASSERT_EQUAL_UINT8(0xA1, wild_crc8(v, 9));
}

void test_crc8_detects_single_bit_flip(void)
{
    uint8_t a[8] = {1,2,3,4,5,6,7,8};
    uint8_t b[8] = {1,2,3,4,5,6,7,8};
    b[3] ^= 0x01;
    TEST_ASSERT_NOT_EQUAL(wild_crc8(a,8), wild_crc8(b,8));
}

/* ---- endianness ---- */

void test_u24_roundtrip(void)
{
    uint8_t buf[3];
    wild_put_u24(buf, 0x123456UL);
    TEST_ASSERT_EQUAL_UINT8(0x56, buf[0]);   /* little-endian on the wire */
    TEST_ASSERT_EQUAL_UINT8(0x34, buf[1]);
    TEST_ASSERT_EQUAL_UINT8(0x12, buf[2]);
    TEST_ASSERT_EQUAL_UINT32(0x123456UL, wild_get_u24(buf));
}

void test_negative_i16_roundtrip(void)
{
    /* -40.00 C as raw HP206C hundredths. Sign handling is the classic
     * place a format silently breaks -- only in winter. */
    uint8_t buf[2];
    wild_put_i16(buf, -4000);
    TEST_ASSERT_EQUAL_INT16(-4000, wild_get_i16(buf));
}

void test_negative_i32_roundtrip(void)
{
    /* Southern hemisphere latitude: NSW is about -30.5 degrees. */
    uint8_t buf[4];
    wild_put_i32(buf, -305000000L);
    TEST_ASSERT_EQUAL_INT32(-305000000L, wild_get_i32(buf));
}

/* ---- frame ---- */

void test_sample_roundtrip(void)
{
    uint8_t rec[WILD_RECORD_SIZE];
    wild_encode_sample(rec, 12345UL, 101325UL, 2152, 60000, 30000,
                       wild_pack_range(3, 5));

    TEST_ASSERT_TRUE(wild_record_valid(rec));
    TEST_ASSERT_EQUAL_UINT8(WILD_REC_SAMPLE, wild_record_type(rec));
    TEST_ASSERT_EQUAL_UINT32(12345UL, wild_record_counter(rec));
    TEST_ASSERT_EQUAL_UINT32(101325UL, wild_get_u24(&rec[5]));
    TEST_ASSERT_EQUAL_INT16(2152, wild_get_i16(&rec[8]));
    TEST_ASSERT_EQUAL_UINT16(60000, wild_get_u16(&rec[10]));
    TEST_ASSERT_EQUAL_UINT16(30000, wild_get_u16(&rec[12]));
    TEST_ASSERT_EQUAL_UINT8(3, wild_range_gain(rec[14]));
    TEST_ASSERT_EQUAL_UINT8(5, wild_range_atime(rec[14]));
}

void test_pressure_extremes_fit_u24(void)
{
    /* HP206C range is 300-1200 mbar = 30000-120000 raw. */
    uint8_t rec[WILD_RECORD_SIZE];
    wild_encode_sample(rec, 0, 30000UL, 0, 0, 0, WILD_RANGE_NONE);
    TEST_ASSERT_EQUAL_UINT32(30000UL, wild_get_u24(&rec[5]));
    wild_encode_sample(rec, 0, 120000UL, 0, 0, 0, WILD_RANGE_NONE);
    TEST_ASSERT_EQUAL_UINT32(120000UL, wild_get_u24(&rec[5]));
}

void test_counter_masks_to_u24(void)
{
    uint8_t rec[WILD_RECORD_SIZE];
    wild_encode_sample(rec, 0x01FFFFFFUL, 0, 0, 0, 0, WILD_RANGE_NONE);
    /* Top byte must be discarded, not corrupt the payload. */
    TEST_ASSERT_EQUAL_UINT32(0x00FFFFFFUL, wild_record_counter(rec));
    TEST_ASSERT_TRUE(wild_record_valid(rec));
}

void test_gps_records_roundtrip(void)
{
    uint8_t rec[WILD_RECORD_SIZE];

    wild_encode_gps_time(rec, 100, 1757462400UL, 12, 9, 3);
    TEST_ASSERT_TRUE(wild_record_valid(rec));
    TEST_ASSERT_EQUAL_UINT32(1757462400UL, wild_get_u32(&rec[5]));
    TEST_ASSERT_EQUAL_UINT8(9, rec[11]);

    wild_encode_gps_pos(rec, 100, -305000000L, 1518000000L, 1050);
    TEST_ASSERT_TRUE(wild_record_valid(rec));
    TEST_ASSERT_EQUAL_INT32(-305000000L, wild_get_i32(&rec[5]));
    TEST_ASSERT_EQUAL_INT32(1518000000L, wild_get_i32(&rec[9]));
    TEST_ASSERT_EQUAL_INT16(1050, wild_get_i16(&rec[13]));
}

/* ---- corruption detection: the whole point ---- */

void test_erased_media_is_invalid(void)
{
    /* Unwritten SD/flash reads 0xFF. A forward scan must stop here. */
    uint8_t rec[WILD_RECORD_SIZE];
    memset(rec, 0xFF, sizeof(rec));
    TEST_ASSERT_FALSE(wild_record_valid(rec));
}

void test_zeroed_media_is_invalid(void)
{
    uint8_t rec[WILD_RECORD_SIZE];
    memset(rec, 0x00, sizeof(rec));
    TEST_ASSERT_FALSE(wild_record_valid(rec));
}

void test_every_single_byte_corruption_detected(void)
{
    /* Exhaustive: flip every bit of every byte and confirm the record is
     * rejected. 16 bytes x 8 bits = 128 cases, free on the host and
     * impossible to check by inspection. */
    uint8_t good[WILD_RECORD_SIZE];
    wild_encode_sample(good, 5000UL, 98765UL, -1234, 4096, 2048,
                       wild_pack_range(2, 3));

    for (uint8_t byte = 0; byte < WILD_RECORD_SIZE; byte++) {
        for (uint8_t bit = 0; bit < 8; bit++) {
            uint8_t bad[WILD_RECORD_SIZE];
            memcpy(bad, good, sizeof(bad));
            bad[byte] ^= (uint8_t)(1u << bit);
            TEST_ASSERT_FALSE_MESSAGE(wild_record_valid(bad),
                "single-bit corruption went undetected");
        }
    }
}

void test_truncated_record_is_invalid(void)
{
    /* A torn write leaves a partial record followed by erased media. */
    uint8_t rec[WILD_RECORD_SIZE];
    wild_encode_sample(rec, 77, 100000UL, 2000, 100, 50, 0x00);
    memset(&rec[8], 0xFF, WILD_RECORD_SIZE - 8);
    TEST_ASSERT_FALSE(wild_record_valid(rec));
}

/* ---- header ---- */

void test_header_roundtrip(void)
{
    uint8_t blk[WILD_HEADER_SIZE];
    wild_header_t out, in = {
        WILD_FORMAT_VERSION, WILD_VARIANT_STATION, WILD_RECORD_SIZE, 1,
        0xDEADBEEFUL, 1757462400UL, 5, 42, 512UL + 16UL * 1000UL, 7
    };

    wild_encode_header(blk, &in);
    TEST_ASSERT_TRUE(wild_decode_header(blk, &out));
    TEST_ASSERT_EQUAL_UINT32(0xDEADBEEFUL, out.device_id);
    TEST_ASSERT_EQUAL_UINT32(512UL + 16UL * 1000UL, out.write_cursor);
    TEST_ASSERT_EQUAL_UINT16(42, out.boot_count);
    TEST_ASSERT_EQUAL_UINT8(WILD_VARIANT_STATION, out.variant);
}

void test_header_rejects_bad_magic(void)
{
    uint8_t blk[WILD_HEADER_SIZE];
    wild_header_t out, in = {1,1,16,1, 1,1,5,1, 512,0};
    wild_encode_header(blk, &in);
    blk[0] = 'X';
    TEST_ASSERT_FALSE(wild_decode_header(blk, &out));
}

void test_header_rejects_bad_crc(void)
{
    uint8_t blk[WILD_HEADER_SIZE];
    wild_header_t out, in = {1,1,16,1, 1,1,5,1, 512,0};
    wild_encode_header(blk, &in);
    blk[20] ^= 0x01;                     /* corrupt sample_interval_s */
    TEST_ASSERT_FALSE(wild_decode_header(blk, &out));
}

/* ---- layout invariants ---- */

void test_records_divide_block_evenly(void)
{
    /* 32 records per 512-byte block means a block write never splits a
     * record. If someone changes WILD_RECORD_SIZE, this fails loudly. */
    TEST_ASSERT_EQUAL_INT(0, 512 % WILD_RECORD_SIZE);
    TEST_ASSERT_EQUAL_INT(0, WILD_HEADER_SIZE % 512);
}


int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_crc8_known_vector);
    RUN_TEST(test_crc8_detects_single_bit_flip);
    RUN_TEST(test_u24_roundtrip);
    RUN_TEST(test_negative_i16_roundtrip);
    RUN_TEST(test_negative_i32_roundtrip);
    RUN_TEST(test_sample_roundtrip);
    RUN_TEST(test_pressure_extremes_fit_u24);
    RUN_TEST(test_counter_masks_to_u24);
    RUN_TEST(test_gps_records_roundtrip);
    RUN_TEST(test_erased_media_is_invalid);
    RUN_TEST(test_zeroed_media_is_invalid);
    RUN_TEST(test_every_single_byte_corruption_detected);
    RUN_TEST(test_truncated_record_is_invalid);
    RUN_TEST(test_header_roundtrip);
    RUN_TEST(test_header_rejects_bad_magic);
    RUN_TEST(test_header_rejects_bad_crc);
    RUN_TEST(test_records_divide_block_evenly);
    return UNITY_END();
}
