/*
 * pcCommunication.c - PC-side UART sender for the RAPID system.
 *
 * Reads polar coordinates from a GDS input file (via inputParser), frames
 * each point into a binary packet, and streams them over a serial COM port
 * to the FPGA.  A background reader thread receives framed responses (ACKs,
 * debug strings, etc.) and implements stop-and-wait flow control so the
 * next point is only sent after the FPGA acknowledges the previous one.
 *
 * Structured output (for gui.py):
 *   Lines beginning with ">> " carry a JSON event object.  Human-readable
 *   lines are always printed alongside them for terminal use.
 *
 *   >> {"type":"ack",      "r_um":<int>, "theta_deg":<float>}
 *   >> {"type":"crc_error"}
 *   >> {"type":"done",     "points_sent":<int>, "acks_received":<int>}
 *
 * Protocol constants, disc geometry, and frame layout: see src/protocol.h
 * Low-level framing and serial I/O:               see src/framing.h / framing.c
 *
 * Compile (MinGW / MSYS2 UCRT64):
 *   gcc -O2 -Wall -Wextra -o RAPID.exe pcCommunication.c framing.c inputParser.c -lm
 */

#include <windows.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "protocol.h"
#include "framing.h"
#include "inputParser.h"

/* ---- PC-side timing constants --------------------------------------- */
#define RX_BUF_SIZE      256    /* serial read buffer (bulk reads)          */
#define ACK_TIMEOUT      2000   /* ms to wait for each point ACK            */
#define FPGA_INIT_WAIT_MS 32000 /* ms to wait at startup for FPGA zeroing
                                 * (matches ZERO_WAIT_US in systemControl.c
                                 *  plus a small margin)                    */

/* ------------------------------------------------------------------ */
/*  Reader thread context & ACK signalling                            */
/* ------------------------------------------------------------------ */

typedef struct {
    HANDLE        h;            /* serial port handle              */
    volatile LONG running;      /* 1 while thread should run       */
    volatile LONG ack_count;    /* total ACKs received so far      */
    HANDLE        ack_event;    /* signalled on every new ACK      */
} ReaderCtx;

/**
 * Block until ack_count >= target, or timeout_ms elapses.
 * Uses an auto-reset event so we wake instantly on each ACK.
 */
static int wait_for_ack(ReaderCtx *ctx, LONG target, DWORD timeout_ms) {
    DWORD deadline = GetTickCount() + timeout_ms;
    for (;;) {
        if (InterlockedCompareExchange(&ctx->ack_count, 0, 0) >= target)
            return 1;
        DWORD remaining = deadline - GetTickCount();
        if ((int)remaining <= 0)
            return 0;
        WaitForSingleObject(ctx->ack_event, remaining);
    }
}

/* ------------------------------------------------------------------ */
/*  Reader thread — framing state machine                             */
/* ------------------------------------------------------------------ */

enum rx_state { S_SOF1, S_SOF2, S_TYPE, S_LEN, S_PAYLOAD, S_CRC };

static DWORD WINAPI reader_thread(LPVOID param) {
    ReaderCtx *ctx = (ReaderCtx *)param;

    enum rx_state st = S_SOF1;
    uint8_t type = 0, len = 0;
    uint8_t payload[255];
    uint8_t pay_i = 0;
    uint8_t rxbuf[RX_BUF_SIZE];

    while (InterlockedCompareExchange(&ctx->running, 1, 1)) {
        DWORD got = 0;
        if (!ReadFile(ctx->h, rxbuf, sizeof(rxbuf), &got, NULL) || got == 0) {
            Sleep(1);
            continue;
        }

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
                uint8_t chk[2 + 255];
                chk[0] = type;
                chk[1] = len;
                for (uint8_t i = 0; i < len; i++) chk[2 + i] = payload[i];

                if (crc8_xor(chk, 2 + len) != b) {
                    printf("[RX] CRC mismatch (type=0x%02X, len=%u)\n", type, len);
                    printf(">> {\"type\":\"crc_error\"}\n");
                    st = S_SOF1;
                    break;
                }

                if (type == TYPE_DEBUG) {
                    char msg[256];
                    uint8_t n = (len < 255) ? len : 255;
                    for (uint8_t i = 0; i < n; i++) msg[i] = (char)payload[i];
                    msg[n] = '\0';
                    printf("[FPGA] %s\n", msg);
                }
                else if (type == TYPE_ACK) {
                    if (len == POINT_LEN) {
                        int32_t r_um    = unpack_i32_le(&payload[0]);
                        float theta_deg = unpack_f32_le(&payload[4]);
                        printf("[ACK] r=%ld um, theta=%.2f deg\n",
                               (long)r_um, (double)theta_deg);
                        printf(">> {\"type\":\"ack\",\"r_um\":%ld,\"theta_deg\":%.4f}\n",
                               (long)r_um, (double)theta_deg);
                    }
                    InterlockedIncrement(&ctx->ack_count);
                    SetEvent(ctx->ack_event);
                }
                else {
                    printf("[RX] unknown type=0x%02X len=%u\n", type, len);
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
/*  Cleanup                                                           */
/* ------------------------------------------------------------------ */

static void cleanup(ReaderCtx *ctx, HANDLE thread, HANDLE port, PolarPoint *polar) {
    InterlockedExchange(&ctx->running, 0);
    SetEvent(ctx->ack_event);
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
    ctx.ack_event = CreateEvent(NULL, FALSE, FALSE, NULL);

    HANDLE th = CreateThread(NULL, 0, reader_thread, &ctx, 0, NULL);
    if (!th) {
        fprintf(stderr, "Failed to create reader thread\n");
        CloseHandle(ctx.ack_event);
        CloseHandle(h);
        free(polar);
        return 1;
    }

    /* ---- wait for FPGA stepper zeroing ---- */
    printf("Disc radius: %u um, max steps: %u\n", DISC_RADIUS_UM, MAX_STEPS);
    {
        /* RAPID_INIT_WAIT_MS env var overrides the default; set to 0 for testing. */
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

    /* ---- end packet ---- */
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
    printf(">> {\"type\":\"done\",\"points_sent\":%zu,\"acks_received\":%ld}\n",
           count, (long)ctx.ack_count);

    cleanup(&ctx, th, h, polar);
    return 0;
}
