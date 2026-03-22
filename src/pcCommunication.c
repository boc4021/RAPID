/*
 * pcCommunication.c - PC-side UART sender for the RAPID system.
 *
 * Reads polar coordinates from a GDS input file (via inputParser), frames
 * each point into a binary packet, and streams them over a serial COM port
 * to the FPGA.  A background reader thread receives framed responses (ACKs,
 * debug strings, etc.) and implements stop-and-wait flow control so the
 * next point is only sent after the FPGA acknowledges the previous one.
 *
 * Packet wire format (both directions):
 *   SOF (0xAA 0x55) | TYPE (1 B) | LEN (1 B) | PAYLOAD (LEN B) | CRC8
 *
 * Outgoing TYPE 0x01, LEN 0x08:
 *   r_um      (int32,   little-endian, micrometres)
 *   theta_deg (float32, little-endian, degrees)
 *
 * Outgoing TYPE 0x03, LEN 0x00:  end of sequence
 *
 * Incoming responses from FPGA:
 *   TYPE 0x81 LEN 0x08  - ACK echo of the point payload
 *   TYPE 0x81 LEN 0x00  - ACK for end packet
 *   TYPE 0xF0 LEN N     - debug / status string
 *
 * Compile (MinGW / MSYS2 UCRT64):
 *   gcc -O2 -Wall -Wextra -o RAPID.exe pcCommunication.c inputParser.c -lm
 */

#include <windows.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "inputParser.h"

/* ------------------------------------------------------------------ */
/*  Constants                                                         */
/* ------------------------------------------------------------------ */

#define SOF_BYTE_1           0xAA
#define SOF_BYTE_2           0x55
#define TYPE_POINT           0x01   /* outgoing polar-point payload   */
#define TYPE_END             0x03   /* outgoing end-of-sequence       */
#define TYPE_ACK             0x81   /* FPGA acknowledgement           */
#define TYPE_DEBUG           0xF0   /* FPGA debug string              */
#define POINT_LEN            0x08   /* payload length for a point     */
#define FRAME_SIZE           13     /* SOF(2) + TYPE + LEN + PAYLOAD(8) + CRC */
#define RX_BUF_SIZE          256    /* serial read buffer (bulk reads)   */
#define ACK_TIMEOUT          2000   /* ms to wait for each point ACK     */
#define FPGA_INIT_WAIT_MS    32000  /* ms to wait at startup for FPGA stepper zeroing
                                     * (matches ZERO_WAIT_US=30 s in systemControl.c
                                     *  plus a small margin)             */
#define BAUD_RATE            115200
#define DISC_RADIUS_UM       33000  /* physical disc radius in µm (33 mm standard CD) */

/* ------------------------------------------------------------------ */
/*  CRC / serialisation helpers                                       */
/* ------------------------------------------------------------------ */

/** XOR-based CRC-8 over a byte buffer. */
static uint8_t crc8_xor(const uint8_t *data, size_t len) {
    uint8_t c = 0;
    for (size_t i = 0; i < len; i++)
        c ^= data[i];
    return c;
}

/** Pack a 32-bit signed integer into 4 bytes, little-endian. */
static void pack_i32_le(uint8_t out[4], int32_t v) {
    out[0] = (uint8_t)(v);
    out[1] = (uint8_t)(v >> 8);
    out[2] = (uint8_t)(v >> 16);
    out[3] = (uint8_t)(v >> 24);
}

/** Unpack 4 little-endian bytes into a 32-bit signed integer. */
static int32_t unpack_i32_le(const uint8_t b[4]) {
    return (int32_t)(
        ((uint32_t)b[0])       |
        ((uint32_t)b[1] << 8)  |
        ((uint32_t)b[2] << 16) |
        ((uint32_t)b[3] << 24)
    );
}

/** Pack a 32-bit float into 4 bytes, little-endian (IEEE 754). */
static void pack_f32_le(uint8_t out[4], float v) {
    uint32_t bits;
    memcpy(&bits, &v, 4);
    out[0] = (uint8_t)(bits);
    out[1] = (uint8_t)(bits >> 8);
    out[2] = (uint8_t)(bits >> 16);
    out[3] = (uint8_t)(bits >> 24);
}

/** Unpack 4 little-endian bytes into a 32-bit float (IEEE 754). */
static float unpack_f32_le(const uint8_t b[4]) {
    uint32_t bits = ((uint32_t)b[0])       |
                    ((uint32_t)b[1] << 8)  |
                    ((uint32_t)b[2] << 16) |
                    ((uint32_t)b[3] << 24);
    float v;
    memcpy(&v, &bits, 4);
    return v;
}

/* ------------------------------------------------------------------ */
/*  Low-level serial I/O                                              */
/* ------------------------------------------------------------------ */

/** Write the full contents of buf to the serial handle. */
static int write_all(HANDLE h, const uint8_t *buf, size_t len) {
    size_t sent = 0;
    while (sent < len) {
        DWORD wrote = 0;
        if (!WriteFile(h, buf + sent, (DWORD)(len - sent), &wrote, NULL))
            return 0;
        sent += (size_t)wrote;
    }
    return 1;
}

/**
 * Open a COM port for bidirectional 8N1 communication.
 *
 * Read timeouts are kept short so the reader thread can respond quickly
 * without blocking forever when no data is available.
 */
static HANDLE open_serial(const char *com_name, int baud) {
    char path[64];
    snprintf(path, sizeof(path), "\\\\.\\%s", com_name);

    HANDLE h = CreateFileA(
        path, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (h == INVALID_HANDLE_VALUE)
        return INVALID_HANDLE_VALUE;

    /* configure 8N1 at the requested baud rate */
    DCB dcb = {0};
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(h, &dcb))          { CloseHandle(h); return INVALID_HANDLE_VALUE; }
    dcb.BaudRate = baud;
    dcb.ByteSize = 8;
    dcb.Parity   = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    if (!SetCommState(h, &dcb))          { CloseHandle(h); return INVALID_HANDLE_VALUE; }

    /*
     * Timeouts: ReadIntervalTimeout = MAXDWORD with the other two read
     * values set to 0 gives "return immediately with whatever is in the
     * driver buffer" behaviour — the fastest non-blocking read pattern
     * on Windows.  The reader thread yields with Sleep(1) when nothing
     * was available, which keeps CPU usage near zero while still
     * providing sub-millisecond response when data arrives.
     */
    COMMTIMEOUTS t  = {0};
    t.ReadIntervalTimeout         = MAXDWORD;
    t.ReadTotalTimeoutConstant    = 0;
    t.ReadTotalTimeoutMultiplier  = 0;
    t.WriteTotalTimeoutConstant   = 2000;
    t.WriteTotalTimeoutMultiplier = 10;
    SetCommTimeouts(h, &t);

    PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);
    return h;
}

/* ------------------------------------------------------------------ */
/*  Packet framing - transmit                                         */
/* ------------------------------------------------------------------ */

/** Build and send a single polar-point frame over the serial link. */
static int send_polar_point(HANDLE h, double r_um, double theta_deg) {
    int32_t r_i32 = (int32_t)llround(r_um);
    float   t_f32 = (float)theta_deg;

    uint8_t frame[FRAME_SIZE];
    size_t idx = 0;

    frame[idx++] = SOF_BYTE_1;
    frame[idx++] = SOF_BYTE_2;
    frame[idx++] = TYPE_POINT;
    frame[idx++] = POINT_LEN;
    pack_i32_le(&frame[idx], r_i32);  idx += 4;
    pack_f32_le(&frame[idx], t_f32);  idx += 4;
    frame[idx++] = crc8_xor(&frame[2], 2 + POINT_LEN); /* CRC over TYPE+LEN+PAYLOAD */

    return write_all(h, frame, FRAME_SIZE);
}

/**
 * Build and send a TYPE_END frame: signals the FPGA that all points
 * have been sent and it should shut down laser and motors gracefully.
 */
static int send_end_packet(HANDLE h) {
    uint8_t frame[5];  /* SOF(2) + TYPE + LEN=0 + CRC */
    frame[0] = SOF_BYTE_1;
    frame[1] = SOF_BYTE_2;
    frame[2] = TYPE_END;
    frame[3] = 0x00;
    frame[4] = TYPE_END ^ 0x00;   /* CRC over TYPE + LEN + (no payload) */

    return write_all(h, frame, sizeof(frame));
}

/* ------------------------------------------------------------------ */
/*  Reader thread context & ACK signalling                            */
/* ------------------------------------------------------------------ */

typedef struct {
    HANDLE          h;             /* serial port handle              */
    volatile LONG   running;       /* 1 while thread should run       */
    volatile LONG   ack_count;     /* total ACKs received so far      */
    HANDLE          ack_event;     /* signalled on every new ACK      */
} ReaderCtx;

/**
 * Block until ack_count >= target, or timeout_ms elapses.
 * Uses an auto-reset event so we wake instantly when an ACK arrives
 * instead of polling with Sleep(1).
 */
static int wait_for_ack(ReaderCtx *ctx, LONG target, DWORD timeout_ms) {
    DWORD deadline = GetTickCount() + timeout_ms;

    for (;;) {
        if (InterlockedCompareExchange(&ctx->ack_count, 0, 0) >= target)
            return 1;

        DWORD remaining = deadline - GetTickCount();
        if ((int)remaining <= 0)
            return 0;

        /* sleep until the reader thread signals an ACK (or timeout) */
        WaitForSingleObject(ctx->ack_event, remaining);
    }
}

/* ------------------------------------------------------------------ */
/*  Reader thread - packet framing state machine                      */
/* ------------------------------------------------------------------ */

/** States for the byte-level receive state machine. */
enum rx_state { S_SOF1, S_SOF2, S_TYPE, S_LEN, S_PAYLOAD, S_CRC };

/**
 * Background thread: reads serial data in bulk, runs it through a
 * framing state machine, and dispatches complete packets.
 */
static DWORD WINAPI reader_thread(LPVOID param) {
    ReaderCtx *ctx = (ReaderCtx *)param;

    enum rx_state st = S_SOF1;
    uint8_t type = 0, len = 0;
    uint8_t payload[255];
    uint8_t pay_i = 0;

    uint8_t rxbuf[RX_BUF_SIZE];

    while (InterlockedCompareExchange(&ctx->running, 1, 1)) {
        /* bulk read — returns immediately with 0..RX_BUF_SIZE bytes */
        DWORD got = 0;
        if (!ReadFile(ctx->h, rxbuf, sizeof(rxbuf), &got, NULL) || got == 0) {
            Sleep(1);   /* nothing available; yield briefly */
            continue;
        }

        /* feed every received byte through the state machine */
        for (DWORD bi = 0; bi < got; bi++) {
            uint8_t b = rxbuf[bi];

            switch (st) {
            case S_SOF1:
                if (b == SOF_BYTE_1) st = S_SOF2;
                break;

            case S_SOF2:
                st = (b == SOF_BYTE_2) ? S_TYPE : S_SOF1;
                break;

            case S_TYPE:
                type = b;
                st = S_LEN;
                break;

            case S_LEN:
                len   = b;
                pay_i = 0;
                st = (len == 0) ? S_CRC : S_PAYLOAD;
                break;

            case S_PAYLOAD:
                payload[pay_i++] = b;
                if (pay_i >= len) st = S_CRC;
                break;

            case S_CRC: {
                /* verify CRC over [TYPE, LEN, PAYLOAD...] */
                uint8_t chk[2 + 255];
                chk[0] = type;
                chk[1] = len;
                for (uint8_t i = 0; i < len; i++) chk[2 + i] = payload[i];

                if (crc8_xor(chk, 2 + len) != b) {
                    fprintf(stderr,
                        "[RX] CRC mismatch (type=0x%02X, len=%u)\n", type, len);
                    st = S_SOF1;
                    break;
                }

                /* ---- dispatch valid packet ---- */
                if (type == TYPE_DEBUG) {
                    /* FPGA debug / status string */
                    char msg[256];
                    uint8_t n = (len < 255) ? len : 255;
                    for (uint8_t i = 0; i < n; i++) msg[i] = (char)payload[i];
                    msg[n] = '\0';
                    printf("[FPGA] %s\n", msg);
                }
                else if (type == TYPE_ACK) {
                    /* ACK — signal the sender unconditionally */
                    if (len == POINT_LEN) {
                        int32_t r_um    = unpack_i32_le(&payload[0]);
                        float theta_deg = unpack_f32_le(&payload[4]);
                        printf("[ACK] r=%ld um, theta=%.2f deg\n",
                               (long)r_um, (double)theta_deg);
                    }
                    InterlockedIncrement(&ctx->ack_count);
                    SetEvent(ctx->ack_event);  /* wake wait_for_ack() */
                }
                else {
                    printf("[RX] type=0x%02X len=%u\n", type, len);
                }

                st = S_SOF1;
            } break;

            default:
                st = S_SOF1;
                break;
            }
        }
    }

    return 0;
}

/* ------------------------------------------------------------------ */
/*  Cleanup helper                                                    */
/* ------------------------------------------------------------------ */

/** Stop reader thread and release all handles / memory. */
static void cleanup(ReaderCtx *ctx, HANDLE thread, HANDLE port, PolarPoint *polar) {
    InterlockedExchange(&ctx->running, 0);
    SetEvent(ctx->ack_event);              /* unblock wait_for_ack if stuck */
    WaitForSingleObject(thread, INFINITE);
    CloseHandle(thread);
    CloseHandle(ctx->ack_event);
    CloseHandle(port);
    free(polar);
}

/* ------------------------------------------------------------------ */
/*  Entry point                                                       */
/* ------------------------------------------------------------------ */

int main(int argc, char **argv) {
    /* disable C-runtime buffering so the dashboard gets lines instantly */
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);

    const char *port_name = (argc >= 2) ? argv[1] : "COM25";
    const char *file      = (argc >= 3) ? argv[2] : "input.gds";

    /* ---- parse input file ---- */
    size_t count = 0;
    Coordinate *coords = getCoordinates(file, &count);
    if (!coords || count == 0) {
        fprintf(stderr, "Failed to parse coordinates from %s\n", file);
        return 1;
    }

    PolarPoint *polar = convertToPolar(coords, count);
    free(coords);
    if (!polar) {
        fprintf(stderr, "Failed to convert to polar\n");
        return 1;
    }

    /* ---- open serial port ---- */
    HANDLE h = open_serial(port_name, BAUD_RATE);
    if (h == INVALID_HANDLE_VALUE) {
        fprintf(stderr, "Failed to open %s\n", port_name);
        free(polar);
        return 1;
    }

    /* ---- start reader thread ---- */
    ReaderCtx ctx;
    ctx.h         = h;
    ctx.running   = 1;
    ctx.ack_count = 0;
    ctx.ack_event = CreateEvent(NULL, FALSE, FALSE, NULL); /* auto-reset */

    HANDLE th = CreateThread(NULL, 0, reader_thread, &ctx, 0, NULL);
    if (!th) {
        fprintf(stderr, "Failed to create reader thread\n");
        CloseHandle(ctx.ack_event);
        CloseHandle(h);
        free(polar);
        return 1;
    }

    /* ---- wait for FPGA stepper initialization (zeroing to proximity switch) ---- */
    printf("Disc radius: %d um, max steps: 8500\n", DISC_RADIUS_UM);
    {
        /* RAPID_INIT_WAIT_MS env var overrides the default wait; set to 0 for testing. */
        const char *env = getenv("RAPID_INIT_WAIT_MS");
        DWORD wait_ms = (env != NULL) ? (DWORD)atoi(env) : FPGA_INIT_WAIT_MS;
        printf("Waiting %lu ms for FPGA stepper initialization (zeroing to proximity switch)...\n",
               (unsigned long)wait_ms);
        Sleep(wait_ms);
    }

    /* ---- send points with stop-and-wait flow control ---- */
    printf("Sending %zu polar points over %s...\n", count, port_name);

    for (size_t i = 0; i < count; i++) {
        LONG target_ack = (LONG)(i + 1);

        if (!send_polar_point(h, polar[i].r, polar[i].theta)) {
            fprintf(stderr, "UART send failed at i=%zu\n", i);
            cleanup(&ctx, th, h, polar);
            return 1;
        }

        if (!wait_for_ack(&ctx, target_ack, ACK_TIMEOUT)) {
            fprintf(stderr, "Timeout waiting for ACK %ld (i=%zu)\n",
                    (long)target_ack, i);
            cleanup(&ctx, th, h, polar);
            return 1;
        }
    }

    /* ---- send end packet — FPGA shuts down laser and motors ---- */
    if (!send_end_packet(h)) {
        fprintf(stderr, "UART send failed (end packet)\n");
        cleanup(&ctx, th, h, polar);
        return 1;
    }
    if (!wait_for_ack(&ctx, (LONG)(count + 1), ACK_TIMEOUT)) {
        fprintf(stderr, "Timeout waiting for end ACK\n");
        cleanup(&ctx, th, h, polar);
        return 1;
    }

    printf("Done - %zu points sent, %ld ACKs received.\n",
           count, (long)ctx.ack_count);

    cleanup(&ctx, th, h, polar);
    return 0;
}