/*
 * framing.h - RAPID packet encoding, decoding, and serial transport (PC side).
 *
 * Covers:
 *   - XOR-CRC8 calculation
 *   - Little-endian integer / float pack & unpack
 *   - Windows serial port open/write
 *   - Outgoing packet builders (point frame, end frame)
 */
#pragma once
#include <windows.h>
#include <stdint.h>
#include <stddef.h>

/* ---- CRC ------------------------------------------------------------ */
uint8_t crc8_xor(const uint8_t *data, size_t len);

/* ---- Little-endian serialisation ------------------------------------ */
void    pack_i32_le(uint8_t out[4], int32_t v);
void    pack_f32_le(uint8_t out[4], float v);
int32_t unpack_i32_le(const uint8_t b[4]);
float   unpack_f32_le(const uint8_t b[4]);

/* ---- Serial transport ----------------------------------------------- */

/**
 * Open a COM port for bidirectional 8N1 communication at the given baud rate.
 * Returns INVALID_HANDLE_VALUE on failure.
 */
HANDLE open_serial(const char *com_name, int baud);

/* ---- Inbound framing FSM -------------------------------------------- */

/*
 * FrameState — opaque FSM state for incremental frame parsing.
 * Initialise with framing_state_init() before use.
 * Feed bytes one at a time with framing_feed(); it calls on_packet
 * whenever a complete, CRC-valid frame is assembled.
 */
typedef enum {
    FS_SOF1, FS_SOF2, FS_TYPE, FS_LEN, FS_PAYLOAD, FS_CRC
} FsmState;

typedef struct {
    FsmState st;
    uint8_t  type;
    uint8_t  len;
    uint8_t  payload[255];
    uint8_t  pay_i;
} FrameState;

/** Callback invoked once per valid received packet. */
typedef void (*PacketHandler)(uint8_t type, uint8_t len,
                              const uint8_t *payload, void *ctx);

/** Reset a FrameState to the initial SOF-hunt state. */
void framing_state_init(FrameState *fs);

/**
 * framing_feed — push one byte into the FSM.
 * On CRC-valid frame completion, on_pkt is called synchronously.
 * CRC mismatches are logged to stdout and the FSM resyncs on the next SOF.
 */
void framing_feed(FrameState *fs, uint8_t b,
                  PacketHandler on_pkt, void *ctx);

/* ---- Outgoing packet builders --------------------------------------- */

/** Frame and send a TYPE_POINT packet.  Returns 1 on success, 0 on error. */
int send_polar_point(HANDLE h, double r_um, double theta_deg);

/** Frame and send a TYPE_END packet.  Returns 1 on success, 0 on error. */
int send_end_packet(HANDLE h);
