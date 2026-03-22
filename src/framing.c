/*
 * framing.c - RAPID packet encoding, decoding, and serial transport (PC side).
 */
#include "framing.h"
#include "protocol.h"

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ---- CRC ------------------------------------------------------------ */

uint8_t crc8_xor(const uint8_t *data, size_t len) {
    uint8_t c = 0;
    for (size_t i = 0; i < len; i++)
        c ^= data[i];
    return c;
}

/* ---- Little-endian pack / unpack ------------------------------------ */

void pack_i32_le(uint8_t out[4], int32_t v) {
    out[0] = (uint8_t)(v);
    out[1] = (uint8_t)(v >> 8);
    out[2] = (uint8_t)(v >> 16);
    out[3] = (uint8_t)(v >> 24);
}

void pack_f32_le(uint8_t out[4], float v) {
    uint32_t bits;
    memcpy(&bits, &v, 4);
    out[0] = (uint8_t)(bits);
    out[1] = (uint8_t)(bits >> 8);
    out[2] = (uint8_t)(bits >> 16);
    out[3] = (uint8_t)(bits >> 24);
}

int32_t unpack_i32_le(const uint8_t b[4]) {
    return (int32_t)(
        ((uint32_t)b[0])       |
        ((uint32_t)b[1] << 8)  |
        ((uint32_t)b[2] << 16) |
        ((uint32_t)b[3] << 24)
    );
}

float unpack_f32_le(const uint8_t b[4]) {
    uint32_t bits = ((uint32_t)b[0])       |
                    ((uint32_t)b[1] << 8)  |
                    ((uint32_t)b[2] << 16) |
                    ((uint32_t)b[3] << 24);
    float v;
    memcpy(&v, &bits, 4);
    return v;
}

/* ---- Serial transport ----------------------------------------------- */

HANDLE open_serial(const char *com_name, int baud) {
    char path[64];
    snprintf(path, sizeof(path), "\\\\.\\%s", com_name);

    HANDLE h = CreateFileA(
        path, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (h == INVALID_HANDLE_VALUE)
        return INVALID_HANDLE_VALUE;

    DCB dcb = {0};
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(h, &dcb))          { CloseHandle(h); return INVALID_HANDLE_VALUE; }
    dcb.BaudRate = baud;
    dcb.ByteSize = 8;
    dcb.Parity   = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    if (!SetCommState(h, &dcb))          { CloseHandle(h); return INVALID_HANDLE_VALUE; }

    /*
     * ReadIntervalTimeout = MAXDWORD with the other two read values = 0
     * gives "return immediately with whatever is in the driver buffer"
     * behaviour.  The reader thread yields with Sleep(1) when nothing was
     * available, keeping CPU usage near zero while still giving
     * sub-millisecond response when data arrives.
     */
    COMMTIMEOUTS t = {0};
    t.ReadIntervalTimeout         = MAXDWORD;
    t.ReadTotalTimeoutConstant    = 0;
    t.ReadTotalTimeoutMultiplier  = 0;
    t.WriteTotalTimeoutConstant   = 2000;
    t.WriteTotalTimeoutMultiplier = 10;
    SetCommTimeouts(h, &t);

    PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);
    return h;
}

int write_all(HANDLE h, const uint8_t *buf, size_t len) {
    size_t sent = 0;
    while (sent < len) {
        DWORD wrote = 0;
        if (!WriteFile(h, buf + sent, (DWORD)(len - sent), &wrote, NULL))
            return 0;
        sent += (size_t)wrote;
    }
    return 1;
}

/* ---- Outgoing packet builders --------------------------------------- */

/* SOF(2) + TYPE(1) + LEN(1) + PAYLOAD(8) + CRC(1) = 13 bytes */
#define FRAME_SIZE 13

int send_polar_point(HANDLE h, double r_um, double theta_deg) {
    int32_t r_i32 = (int32_t)llround(r_um);
    float   t_f32 = (float)theta_deg;

    uint8_t frame[FRAME_SIZE];
    size_t  idx = 0;

    frame[idx++] = SOF_BYTE_1;
    frame[idx++] = SOF_BYTE_2;
    frame[idx++] = TYPE_POINT;
    frame[idx++] = POINT_LEN;
    pack_i32_le(&frame[idx], r_i32);  idx += 4;
    pack_f32_le(&frame[idx], t_f32);  idx += 4;
    frame[idx]   = crc8_xor(&frame[2], 2 + POINT_LEN);

    return write_all(h, frame, FRAME_SIZE);
}

int send_end_packet(HANDLE h) {
    uint8_t frame[5];  /* SOF(2) + TYPE + LEN=0 + CRC */
    frame[0] = SOF_BYTE_1;
    frame[1] = SOF_BYTE_2;
    frame[2] = TYPE_END;
    frame[3] = 0x00;
    frame[4] = TYPE_END ^ 0x00;   /* CRC over TYPE + LEN */

    return write_all(h, frame, sizeof(frame));
}
