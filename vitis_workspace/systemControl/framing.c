/*
 * framing.c - RAPID packet framing for the bare-metal FPGA side.
 *
 * See framing.h for the public API.
 * Protocol constants (SOF bytes, TYPE_*, POINT_LEN, …) come from protocol.h.
 */

#include "framing.h"
#include "protocol.h"

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
    s_uart = uart;
}

/* ------------------------------------------------------------------ */
/*  CRC                                                               */
/* ------------------------------------------------------------------ */

uint8_t crc8_xor(const uint8_t *data, unsigned len)
{
    uint8_t c = 0;
    for (unsigned i = 0; i < len; i++)
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

void uart_send(const uint8_t *buf, unsigned len)
{
    UINTPTR base = s_uart->Config.BaseAddress;
    for (unsigned i = 0; i < len; i++)
        XUartPs_SendByte(base, buf[i]);
}

void uart_flush_tx(void)
{
    UINTPTR base = s_uart->Config.BaseAddress;
    while (!(XUartPs_ReadReg(base, XUARTPS_SR_OFFSET) & XUARTPS_SR_TXEMPTY))
        ;
}

uint8_t uart_recv_byte(void)
{
    uint8_t b;
    while (XUartPs_Recv(s_uart, &b, 1) != 1)
        ;
    return b;
}

/* ------------------------------------------------------------------ */
/*  Framed packet TX                                                  */
/* ------------------------------------------------------------------ */

void send_frame(uint8_t type, const uint8_t *payload, uint8_t len)
{
    uint8_t hdr[4] = { SOF_BYTE_1, SOF_BYTE_2, type, len };

    uint8_t crc = type ^ len;
    for (uint8_t i = 0; i < len; i++)
        crc ^= payload[i];

    uart_send(hdr, sizeof(hdr));
    if (len && payload)
        uart_send(payload, len);
    uart_send(&crc, 1);
    uart_flush_tx();
}

/* ------------------------------------------------------------------ */
/*  Framed packet RX                                                  */
/* ------------------------------------------------------------------ */

#define MAX_PAYLOAD 255

uint8_t receive_packet(uint8_t *out_payload)
{
    for (;;) {
        /* synchronise on SOF */
        if (uart_recv_byte() != SOF_BYTE_1) continue;
        if (uart_recv_byte() != SOF_BYTE_2) continue;

        uint8_t type = uart_recv_byte();
        uint8_t len  = uart_recv_byte();

        uint8_t payload[MAX_PAYLOAD];
        for (uint8_t i = 0; i < len; i++)
            payload[i] = uart_recv_byte();

        uint8_t rx_crc = uart_recv_byte();

        /* verify CRC over [TYPE, LEN, PAYLOAD...] */
        uint8_t chk[2 + MAX_PAYLOAD];
        chk[0] = type;
        chk[1] = len;
        for (uint8_t i = 0; i < len; i++)
            chk[2 + i] = payload[i];

        if (crc8_xor(chk, 2u + len) != rx_crc)
            continue;   /* CRC mismatch — drop frame and resync */

        for (uint8_t i = 0; i < len; i++)
            out_payload[i] = payload[i];

        return type;
    }
}

/* ------------------------------------------------------------------ */
/*  Debug output                                                      */
/* ------------------------------------------------------------------ */

void debug_printf(const char *fmt, ...)
{
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    uint8_t len = 0;
    while (buf[len] && len < 255) len++;
    send_frame(TYPE_DEBUG, (const uint8_t *)buf, len);
}
