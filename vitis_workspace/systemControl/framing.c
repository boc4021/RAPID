/*
 * framing.c - RAPID packet framing for the bare-metal FPGA side.
 *
 * See framing.h for the public API.
 * Protocol constants (SOF bytes, TYPE_*, POINT_LEN, …) come from protocol.h.
 */

#include "framing.h"
#include "protocol.h"

#include <stddef.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "xuartps_hw.h"

/* ------------------------------------------------------------------ */
/*  Module state                                                       */
/* ------------------------------------------------------------------ */

static XUartPs *s_uart = NULL;

void framing_init(XUartPs *uart)
{
    if (!uart) { for (;;) ; }   /* hard fault — caller passed NULL */
    s_uart = uart;
}

/* ------------------------------------------------------------------ */
/*  CRC                                                               */
/* ------------------------------------------------------------------ */

uint8_t crc8_xor(const uint8_t *data, size_t len)
{
    uint8_t c = 0;
    for (size_t i = 0; i < len; i++)
        c ^= data[i];
    return c;
}

/* ------------------------------------------------------------------ */
/*  Little-endian deserialisation                                     */
/* ------------------------------------------------------------------ */

int32_t unpack_i32_le(const uint8_t b[4])
{
    return (int32_t)(
        ((uint32_t)b[0])        |
        ((uint32_t)b[1] <<  8)  |
        ((uint32_t)b[2] << 16)  |
        ((uint32_t)b[3] << 24)
    );
}

float unpack_f32_le(const uint8_t b[4])
{
    uint32_t bits = ((uint32_t)b[0])        |
                    ((uint32_t)b[1] <<  8)  |
                    ((uint32_t)b[2] << 16)  |
                    ((uint32_t)b[3] << 24);
    float v;
    memcpy(&v, &bits, 4);
    return v;
}

/* ------------------------------------------------------------------ */
/*  Low-level UART I/O                                                */
/* ------------------------------------------------------------------ */

static void uart_send(const uint8_t *buf, unsigned len)
{
    if (!s_uart) { for (;;) ; }   /* guard: framing_init not called */
    UINTPTR base = s_uart->Config.BaseAddress;
    for (unsigned i = 0; i < len; i++)
        XUartPs_SendByte(base, buf[i]);
}

static void uart_flush_tx(void)
{
    if (!s_uart) { for (;;) ; }   /* guard: framing_init not called */
    UINTPTR base = s_uart->Config.BaseAddress;
    while (!(XUartPs_ReadReg(base, XUARTPS_SR_OFFSET) & XUARTPS_SR_TXEMPTY))
        ;
}

/*
 * RECV_TIMEOUT_CYCLES: iterations before declaring the bus dead.
 * At 125 MHz, 50 000 000 iterations ≈ 0.4 s — generous for 115 200 baud
 * where a byte arrives every ~87 µs.
 */
#define RECV_TIMEOUT_CYCLES 50000000u

/*
 * recv_byte_timed - receive one byte with a bounded timeout.
 * Used internally by receive_packet so individual byte waits are bounded.
 * Returns the byte value (0–255) on success, or -1 on timeout.
 */
static int recv_byte_timed(void)
{
    uint8_t b;
    uint32_t retries = RECV_TIMEOUT_CYCLES;
    while (XUartPs_Recv(s_uart, &b, 1) != 1) {
        if (--retries == 0)
            return -1;
    }
    return (int)(uint8_t)b;
}

/* ------------------------------------------------------------------ */
/*  Framed packet TX                                                  */
/* ------------------------------------------------------------------ */

void send_frame(uint8_t type, const uint8_t *payload, uint8_t len)
{
    uint8_t hdr[4] = { SOF_BYTE_1, SOF_BYTE_2, type, len };

    uint8_t crc = type ^ len;
    if (len && payload) {
        for (uint8_t i = 0; i < len; i++)
            crc ^= payload[i];
    }

    uart_send(hdr, sizeof(hdr));
    if (len && payload)
        uart_send(payload, len);
    uart_send(&crc, 1);
    uart_flush_tx();
}

/* ------------------------------------------------------------------ */
/*  Framed packet RX                                                  */
/* ------------------------------------------------------------------ */

uint8_t receive_packet(uint8_t *out_payload, uint8_t *out_len)
{
    for (;;) {
        int b;

        /* synchronise on SOF — timeout on either byte returns TYPE_TIMEOUT */
        b = recv_byte_timed();
        if (b < 0) return TYPE_TIMEOUT;
        if ((uint8_t)b != SOF_BYTE_1) continue;

        b = recv_byte_timed();
        if (b < 0) return TYPE_TIMEOUT;
        if ((uint8_t)b != SOF_BYTE_2) continue;

        b = recv_byte_timed();
        if (b < 0) return TYPE_TIMEOUT;
        uint8_t type = (uint8_t)b;

        b = recv_byte_timed();
        if (b < 0) return TYPE_TIMEOUT;
        uint8_t len = (uint8_t)b;

        /* read payload directly into caller's buffer */
        for (uint8_t i = 0; i < len; i++) {
            b = recv_byte_timed();
            if (b < 0) return TYPE_TIMEOUT;
            out_payload[i] = (uint8_t)b;
        }

        b = recv_byte_timed();
        if (b < 0) return TYPE_TIMEOUT;
        uint8_t rx_crc = (uint8_t)b;

        /* verify CRC incrementally over [TYPE, LEN, PAYLOAD...] */
        uint8_t crc = type ^ len;
        for (uint8_t i = 0; i < len; i++)
            crc ^= out_payload[i];

        if (crc != rx_crc) {
            debug_printf("[FRAMING] CRC mismatch: expected 0x%02X got 0x%02X (type=0x%02X len=%u)",
                         crc, rx_crc, type, len);
            continue;   /* drop frame and resync */
        }

        *out_len = len;
        return type;
    }
}

/* ------------------------------------------------------------------ */
/*  Debug output                                                      */
/* ------------------------------------------------------------------ */

void debug_printf(const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    uint8_t len = 0;
    while (buf[len] && len < 255) len++;
    send_frame(TYPE_DEBUG, (const uint8_t *)buf, len);
}
