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

/** Write all len bytes of buf to h.  Returns 1 on success, 0 on error. */
int write_all(HANDLE h, const uint8_t *buf, size_t len);

/* ---- Outgoing packet builders --------------------------------------- */

/** Frame and send a TYPE_POINT packet.  Returns 1 on success, 0 on error. */
int send_polar_point(HANDLE h, double r_um, double theta_deg);

/** Frame and send a TYPE_END packet.  Returns 1 on success, 0 on error. */
int send_end_packet(HANDLE h);
