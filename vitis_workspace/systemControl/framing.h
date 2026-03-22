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
#include "xuartps.h"

/* ---- Initialisation ------------------------------------------------- */

/** Supply the initialised UART instance pointer before using any other call. */
void framing_init(XUartPs *uart);

/* ---- CRC ------------------------------------------------------------ */

uint8_t crc8_xor(const uint8_t *data, unsigned len);

/* ---- Little-endian deserialisation ---------------------------------- */

int32_t unpack_i32_le(const uint8_t b[4]);
float   unpack_f32_le(const uint8_t b[4]);

/* ---- Low-level UART I/O --------------------------------------------- */

void    uart_send(const uint8_t *buf, unsigned len);
void    uart_flush_tx(void);
uint8_t uart_recv_byte(void);

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
 * Fills out_payload (caller must provide MAX_PAYLOAD bytes) with the
 * packet payload and returns the TYPE byte.  Frames with CRC mismatches
 * are silently discarded and the function re-synchronises on the next SOF.
 */
uint8_t receive_packet(uint8_t *out_payload);

/* ---- Debug output --------------------------------------------------- */

/*
 * debug_printf - send a formatted string to the PC as a TYPE_DEBUG (0xF0)
 * framed packet.  The PC reader thread prints these as "[FPGA] <msg>".
 * Use this in place of xil_printf while the UART is in framed-protocol mode.
 */
void debug_printf(const char *fmt, ...);
