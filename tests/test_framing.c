/*
 * test_framing.c - Unit tests for src/framing.c
 *
 * Covers: crc8_xor, pack_i32_le, pack_f32_le, unpack_i32_le, unpack_f32_le,
 *         framing_state_init, framing_feed (FSM — happy path, CRC mismatch,
 *         resync after garbage, SOF bytes embedded in payload).
 *
 * Compile (MinGW / MSYS2 UCRT64 from project root):
 *   gcc -O0 -Wall -Wextra -I src -o build/test_framing \
 *       tests/test_framing.c src/framing.c -lm
 *   build/test_framing && echo PASS
 *
 * Or via Make:
 *   make test-framing
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "framing.h"
#include "protocol.h"

/* ---- Soft assertion — logs failure and increments counter, does not abort - */

static int _check_failures = 0;

#define CHECK(expr)                                                          \
    do {                                                                     \
        if (!(expr)) {                                                       \
            fprintf(stderr, "  FAIL: %s  (%s:%d)\n", #expr, __FILE__, __LINE__); \
            _check_failures++;                                               \
            return;                                                          \
        }                                                                    \
    } while (0)

/* ---- Capture state for dispatch callback --------------------------------- */

static int     g_pkt_count   = 0;
static uint8_t g_pkt_type    = 0;
static uint8_t g_pkt_len     = 0;
static uint8_t g_pkt_payload[255];

static void capture(uint8_t t, uint8_t l, const uint8_t *p, void *ctx)
{
    (void)ctx;
    g_pkt_type = t;
    g_pkt_len  = l;
    if (l) memcpy(g_pkt_payload, p, l);
    g_pkt_count++;
}

static void reset_capture(void)
{
    g_pkt_count = 0;
    g_pkt_type  = 0;
    g_pkt_len   = 0;
    memset(g_pkt_payload, 0, sizeof(g_pkt_payload));
}

/* ---- Frame builders ------------------------------------------------------ */

/* Build a complete 13-byte TYPE_POINT frame. */
static void build_point_frame(uint8_t buf[13], int32_t r, float theta)
{
    buf[0] = SOF_BYTE_1;
    buf[1] = SOF_BYTE_2;
    buf[2] = TYPE_POINT;
    buf[3] = POINT_LEN;
    pack_i32_le(&buf[4], r);
    pack_f32_le(&buf[8], theta);
    buf[12] = crc8_xor(&buf[2], 2 + POINT_LEN);
}

/* Build a 5-byte TYPE_END frame. */
static void build_end_frame(uint8_t buf[5])
{
    buf[0] = SOF_BYTE_1;
    buf[1] = SOF_BYTE_2;
    buf[2] = TYPE_END;
    buf[3] = 0x00;
    buf[4] = crc8_xor(&buf[2], 2);
}

/* Feed an entire byte buffer into the FSM. */
static void feed_all(FrameState *fs, const uint8_t *buf, size_t len)
{
    for (size_t i = 0; i < len; i++)
        framing_feed(fs, buf[i], capture, NULL);
}

/* ---- Test runner bookkeeping --------------------------------------------- */

static int tests_run    = 0;
static int tests_failed = 0;

#define RUN(fn)                                                     \
    do {                                                            \
        reset_capture();                                            \
        int _before = _check_failures;                              \
        printf("  %-55s", #fn "...");                              \
        fn();                                                       \
        tests_run++;                                                \
        if (_check_failures > _before) {                            \
            tests_failed++;                                         \
            printf("FAIL\n");                                       \
        } else {                                                    \
            printf("PASS\n");                                       \
        }                                                           \
    } while (0)

/* ============================================================= */
/*  CRC tests                                                    */
/* ============================================================= */

static void test_crc_empty(void)
{
    uint8_t dummy[1] = {0};
    CHECK(crc8_xor(dummy, 0) == 0);
}

static void test_crc_single_byte(void)
{
    uint8_t b = 0x42;
    CHECK(crc8_xor(&b, 1) == 0x42);
}

static void test_crc_known_vector(void)
{
    /* TYPE_POINT(0x01) ^ LEN(0x08) ^ 8 zero payload bytes = 0x09 */
    uint8_t v[10] = {0x01, 0x08, 0,0,0,0, 0,0,0,0};
    CHECK(crc8_xor(v, 10) == 0x09);
}

static void test_crc_self_check(void)
{
    /* Appending the CRC to data should XOR-sum to 0. */
    uint8_t data[] = {0x01, 0x08, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
    uint8_t crc    = crc8_xor(data, sizeof(data));
    uint8_t full[11];
    memcpy(full, data, sizeof(data));
    full[sizeof(data)] = crc;
    CHECK(crc8_xor(full, sizeof(full)) == 0);
}

static void test_crc_idempotent_after_xor_pair(void)
{
    /* XOR-CRC of the same byte twice cancels out. */
    uint8_t v[2] = {0xAB, 0xAB};
    CHECK(crc8_xor(v, 2) == 0);
}

/* ============================================================= */
/*  pack / unpack tests                                          */
/* ============================================================= */

static void test_pack_unpack_i32_zero(void)
{
    uint8_t buf[4];
    pack_i32_le(buf, 0);
    CHECK(unpack_i32_le(buf) == 0);
}

static void test_pack_unpack_i32_positive(void)
{
    uint8_t buf[4];
    pack_i32_le(buf, 33000);
    CHECK(unpack_i32_le(buf) == 33000);
}

static void test_pack_unpack_i32_negative(void)
{
    uint8_t buf[4];
    pack_i32_le(buf, -1);
    CHECK(unpack_i32_le(buf) == -1);
}

static void test_pack_unpack_i32_min(void)
{
    uint8_t buf[4];
    pack_i32_le(buf, INT32_MIN);
    CHECK(unpack_i32_le(buf) == INT32_MIN);
}

static void test_pack_unpack_i32_max(void)
{
    uint8_t buf[4];
    pack_i32_le(buf, INT32_MAX);
    CHECK(unpack_i32_le(buf) == INT32_MAX);
}

static void test_pack_i32_le_byte_order(void)
{
    /* 0x01020304 → [0x04, 0x03, 0x02, 0x01] in little-endian */
    uint8_t buf[4];
    pack_i32_le(buf, 0x01020304);
    CHECK(buf[0] == 0x04);
    CHECK(buf[1] == 0x03);
    CHECK(buf[2] == 0x02);
    CHECK(buf[3] == 0x01);
}

static void test_pack_unpack_f32_zero(void)
{
    uint8_t buf[4];
    pack_f32_le(buf, 0.0f);
    CHECK(unpack_f32_le(buf) == 0.0f);
}

static void test_pack_unpack_f32_roundtrip(void)
{
    const float vals[] = {1.0f, -1.0f, 180.0f, 359.9999f, 1e-6f, 45.123f};
    uint8_t buf[4];
    for (size_t i = 0; i < sizeof(vals)/sizeof(vals[0]); i++) {
        pack_f32_le(buf, vals[i]);
        float got = unpack_f32_le(buf);
        /* Require bit-identical round-trip — no lossy conversion allowed. */
        CHECK(memcmp(&got, &vals[i], 4) == 0);
    }
}

static void test_pack_f32_le_byte_order(void)
{
    /* IEEE 754: 1.0f = 0x3F800000 → LE bytes [0x00, 0x00, 0x80, 0x3F] */
    uint8_t buf[4];
    pack_f32_le(buf, 1.0f);
    CHECK(buf[0] == 0x00);
    CHECK(buf[1] == 0x00);
    CHECK(buf[2] == 0x80);
    CHECK(buf[3] == 0x3F);
}

/* ============================================================= */
/*  FSM tests                                                    */
/* ============================================================= */

static void test_fsm_init_state(void)
{
    FrameState fs;
    framing_state_init(&fs);
    CHECK(fs.st == FS_SOF1);
}

static void test_fsm_happy_path_point(void)
{
    uint8_t frame[13];
    build_point_frame(frame, 16500, 45.0f);
    FrameState fs;
    framing_state_init(&fs);
    feed_all(&fs, frame, 13);
    CHECK(g_pkt_count == 1);
    CHECK(g_pkt_type  == TYPE_POINT);
    CHECK(g_pkt_len   == POINT_LEN);
    CHECK(unpack_i32_le(g_pkt_payload) == 16500);  /* payload round-trips */
}

static void test_fsm_happy_path_end(void)
{
    uint8_t frame[5];
    build_end_frame(frame);
    FrameState fs;
    framing_state_init(&fs);
    feed_all(&fs, frame, 5);
    CHECK(g_pkt_count == 1);
    CHECK(g_pkt_type  == TYPE_END);
    CHECK(g_pkt_len   == 0);
}

static void test_fsm_crc_mismatch_no_dispatch(void)
{
    uint8_t frame[13];
    build_point_frame(frame, 16500, 45.0f);
    frame[12] ^= 0xFF;   /* corrupt CRC */
    FrameState fs;
    framing_state_init(&fs);
    feed_all(&fs, frame, 13);
    CHECK(g_pkt_count == 0);          /* bad CRC → no dispatch */
    CHECK(fs.st       == FS_SOF1);    /* FSM resynced to hunt state */
}

static void test_fsm_crc_recovery_then_valid(void)
{
    /* bad frame immediately followed by a good frame — only good must dispatch */
    uint8_t bad[13];
    build_point_frame(bad, 100, 0.0f);
    bad[12] ^= 0xFF;

    uint8_t good[13];
    build_point_frame(good, 200, 90.0f);

    FrameState fs;
    framing_state_init(&fs);
    feed_all(&fs, bad,  13);
    feed_all(&fs, good, 13);

    CHECK(g_pkt_count == 1);
    CHECK(unpack_i32_le(g_pkt_payload) == 200);
}

static void test_fsm_resync_after_garbage(void)
{
    /* Leading junk bytes (including a partial/false SOF) then a valid frame. */
    uint8_t junk[] = {0x12, 0x34, SOF_BYTE_1, 0x12, 0xFF, 0x00, 0xAB};
    uint8_t frame[13];
    build_point_frame(frame, 33000, 90.0f);

    FrameState fs;
    framing_state_init(&fs);
    feed_all(&fs, junk,  sizeof(junk));
    feed_all(&fs, frame, 13);

    CHECK(g_pkt_count == 1);
    CHECK(g_pkt_type  == TYPE_POINT);
}

static void test_fsm_sof_bytes_in_payload(void)
{
    /*
     * r_um = 0x0055AA00 → LE bytes [0x00, 0xAA, 0x55, 0x00].
     * Payload positions 1 and 2 contain SOF_BYTE_1 (0xAA) followed by
     * SOF_BYTE_2 (0x55).  A buggy FSM might resync here.
     * The correct FSM ignores payload bytes in FS_PAYLOAD state.
     */
    int32_t r = 0x0055AA00;
    uint8_t frame[13];
    build_point_frame(frame, r, 0.0f);

    FrameState fs;
    framing_state_init(&fs);
    feed_all(&fs, frame, 13);

    CHECK(g_pkt_count == 1);
    CHECK(unpack_i32_le(g_pkt_payload) == r);
}

static void test_fsm_two_consecutive_frames(void)
{
    uint8_t f1[13], f2[13];
    build_point_frame(f1, 1000, 0.0f);
    build_point_frame(f2, 2000, 180.0f);

    FrameState fs;
    framing_state_init(&fs);

    feed_all(&fs, f1, 13);
    CHECK(g_pkt_count == 1);
    CHECK(unpack_i32_le(g_pkt_payload) == 1000);

    reset_capture();
    feed_all(&fs, f2, 13);
    CHECK(g_pkt_count == 1);
    CHECK(unpack_i32_le(g_pkt_payload) == 2000);
}

static void test_fsm_partial_sof_then_valid(void)
{
    /* SOF_BYTE_1 followed by the wrong SOF_BYTE_2 → FSM resets to SOF1.
     * A complete valid frame fed next must still be parsed. */
    uint8_t frame[13];
    build_point_frame(frame, 5000, 0.0f);

    FrameState fs;
    framing_state_init(&fs);

    framing_feed(&fs, SOF_BYTE_1, capture, NULL);
    framing_feed(&fs, 0xFF,       capture, NULL);  /* wrong SOF2 */
    CHECK(fs.st == FS_SOF1);                      /* FSM reset */

    feed_all(&fs, frame, 13);
    CHECK(g_pkt_count == 1);
}

static void test_fsm_bad_then_bad_then_good(void)
{
    /* Two corrupted frames; FSM must accept the third valid frame. */
    uint8_t bad[13], good[13];
    build_point_frame(bad,  1, 0.0f);  bad[12]  ^= 0x01;
    build_point_frame(good, 999, 45.0f);

    FrameState fs;
    framing_state_init(&fs);
    feed_all(&fs, bad,  13);
    feed_all(&fs, bad,  13);
    feed_all(&fs, good, 13);

    CHECK(g_pkt_count == 1);
    CHECK(unpack_i32_le(g_pkt_payload) == 999);
}

/* ============================================================= */
/*  main                                                         */
/* ============================================================= */

int main(void)
{
    printf("=== test_framing ===\n\n");

    /* CRC */
    RUN(test_crc_empty);
    RUN(test_crc_single_byte);
    RUN(test_crc_known_vector);
    RUN(test_crc_self_check);
    RUN(test_crc_idempotent_after_xor_pair);

    /* pack / unpack */
    RUN(test_pack_unpack_i32_zero);
    RUN(test_pack_unpack_i32_positive);
    RUN(test_pack_unpack_i32_negative);
    RUN(test_pack_unpack_i32_min);
    RUN(test_pack_unpack_i32_max);
    RUN(test_pack_i32_le_byte_order);
    RUN(test_pack_unpack_f32_zero);
    RUN(test_pack_unpack_f32_roundtrip);
    RUN(test_pack_f32_le_byte_order);

    /* FSM */
    RUN(test_fsm_init_state);
    RUN(test_fsm_happy_path_point);
    RUN(test_fsm_happy_path_end);
    RUN(test_fsm_crc_mismatch_no_dispatch);
    RUN(test_fsm_crc_recovery_then_valid);
    RUN(test_fsm_resync_after_garbage);
    RUN(test_fsm_sof_bytes_in_payload);
    RUN(test_fsm_two_consecutive_frames);
    RUN(test_fsm_partial_sof_then_valid);
    RUN(test_fsm_bad_then_bad_then_good);

    printf("\n%d / %d tests passed.\n", tests_run - tests_failed, tests_run);
    return tests_failed ? 1 : 0;
}
