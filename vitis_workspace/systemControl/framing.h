/*
 * framing.h - RAPID packet framing for the bare-metal FPGA side.
 *
 * Covers the Vitis / Zynq PS equivalents of the PC-side framing layer:
 *   - XOR-CRC8
 *   - Little-endian integer unpack
 *   - Polled UART send / receive primitives
 *   - Outgoing frame builder  (send_frame)
 *   - Incoming frame parser   (receive_packet)
 *   - Debug printf via TYPE_DEBUG packet (debug_printf)
 *
 * Depends on: protocol.h, XUartPs (Xilinx driver), stdio.h / stdarg.h
 *
 * The caller must call framing_init() once after XUartPs_CfgInitialize()
 * to give this module access to the UART instance.
 */
#pragma once
#include <stdint.h>
#include <stddef.h>
#include "xuartps.h"

/* ---- Initialisation ------------------------------------------------- */

/** Supply the initialised UART instance pointer before using any other call. */
void framing_init(XUartPs *uart);

/* ---- CRC ------------------------------------------------------------ */

uint8_t crc8_xor(const uint8_t *data, size_t len);

/* ---- Little-endian deserialisation ---------------------------------- */

int32_t unpack_i32_le(const uint8_t b[4]);
float   unpack_f32_le(const uint8_t b[4]);

/* ---- Framed packet TX ----------------------------------------------- */

/*
 * send_frame - build and transmit one framed response packet.
 *
 * Frame: SOF(2) | type(1) | len(1) | payload(len) | CRC8(1)
 * CRC covers [type, len, payload...].
 * Pass payload=NULL or len=0 for empty-payload frames.
 */
void send_frame(uint8_t type, const uint8_t *payload, uint8_t len);

/* ---- Framed packet RX ----------------------------------------------- */

/*
 * receive_packet - block until one valid framed packet arrives.
 *
 * Fills out_payload (caller must provide 255 bytes) with the
 * packet payload, writes the payload length to *out_len, and returns
 * the TYPE byte.  Frames with CRC mismatches emit a debug_printf and
 * re-synchronise on the next SOF.
 *
 * Returns TYPE_TIMEOUT (0xFF) if no byte arrives within the inter-byte
 * deadline (~0.4 s per byte at 125 MHz).  The caller should treat this
 * as a fatal communication loss and enter a safe shutdown state.
 */
uint8_t receive_packet(uint8_t *out_payload, uint8_t *out_len);

/* Sentinel returned by receive_packet on inter-byte timeout (not a wire type). */
#define TYPE_TIMEOUT 0xFFu

/* ---- Debug output --------------------------------------------------- */

/*
 * debug_printf - send a formatted string to the PC as a TYPE_DEBUG (0xF0)
 * framed packet.  The PC reader thread prints these as "[FPGA] <msg>".
 * Use this in place of xil_printf while the UART is in framed-protocol mode.
 */
void debug_printf(const char *fmt, ...);
