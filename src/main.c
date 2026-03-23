/*
 * main.c - PC-side UART sender for the RAPID system.
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
 *   gcc -O2 -Wall -Wextra -o RAPID.exe main.c framing.c inputParser.c -lm
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
#define RX_BUF_SIZE       256   /* serial read buffer (bulk reads)          */
#define ACK_TIMEOUT       2000  /* ms to wait for each point ACK            */
#define FPGA_INIT_WAIT_MS 32000 /* ms to wait at startup for FPGA zeroing
                                 * (matches ZERO_WAIT_US in vitis_workspace/systemControl/main.c
                                 *  plus a small margin)                    */

/* ------------------------------------------------------------------ */
/*  Dependency types  (S-DIP-01)                                      */
/* ------------------------------------------------------------------ */

/** Function pointer type for opening a serial port.  Production code
 *  passes open_serial(); tests can pass a stub that returns a pipe. */
typedef HANDLE (*OpenSerialFn)(const char *port, int baud);

/** Function pointer type for loading polar points from a file.
 *  Production code passes load_polar_points(); tests can pass a stub. */
typedef PolarPoint *(*LoadPointsFn)(const char *file, size_t *count);

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
/*  Packet handlers — one function per packet type  (S-OCP-01)       */
/* ------------------------------------------------------------------ */

static void handle_debug(uint8_t len, const uint8_t *payload, ReaderCtx *ctx)
{
    (void)ctx;
    char msg[256];
    uint8_t n = (len < 255) ? len : 255;
    for (uint8_t i = 0; i < n; i++) msg[i] = (char)payload[i];
    msg[n] = '\0';
    printf("[FPGA] %s\n", msg);
}

static void handle_ack(uint8_t len, const uint8_t *payload, ReaderCtx *ctx)
{
    if (len == POINT_LEN) {
        int32_t r_um    = unpack_i32_le(&payload[0]);
        float theta_deg = unpack_f32_le(&payload[4]);
        printf("[ACK] r=%ld um, theta=%.2f deg\n",
               (long)r_um, (double)theta_deg);
        printf(">> {\"type\":\"ack\",\"r_um\":%ld,\"theta_deg\":%.4f}\n",
               (long)r_um, (double)theta_deg);
    } else if (len != 0) {
        /* len=0 is valid for the end-packet ACK; any other length is wrong */
        fprintf(stderr, "[RX] ACK len mismatch: expected %u or 0, got %u — ignoring\n",
                POINT_LEN, len);
        printf(">> {\"type\":\"ack_len_error\",\"len\":%u}\n", (unsigned)len);
        return;   /* do NOT advance stop-and-wait state machine */
    }
    InterlockedIncrement(&ctx->ack_count);
    SetEvent(ctx->ack_event);
}

/* Dispatch table — add a row here to support a new packet type.
 * dispatch_packet itself never needs to change.  (S-OCP-01)         */
typedef void (*PktHandlerFn)(uint8_t len, const uint8_t *payload, ReaderCtx *ctx);

static const struct { uint8_t type; PktHandlerFn fn; } dispatch_table[] = {
    { TYPE_DEBUG, handle_debug },
    { TYPE_ACK,   handle_ack   },
};
#define DISPATCH_COUNT (sizeof(dispatch_table) / sizeof(dispatch_table[0]))

static void dispatch_packet(uint8_t type, uint8_t len,
                             const uint8_t *payload, void *ctx_v)
{
    ReaderCtx *ctx = (ReaderCtx *)ctx_v;
    for (size_t i = 0; i < DISPATCH_COUNT; i++) {
        if (dispatch_table[i].type == type) {
            dispatch_table[i].fn(len, payload, ctx);
            return;
        }
    }
    printf("[RX] unknown type=0x%02X len=%u\n", type, len);
}

/* ------------------------------------------------------------------ */
/*  Reader thread — I/O loop only; FSM lives in framing.c  (S-SRP-02)*/
/* ------------------------------------------------------------------ */

static DWORD WINAPI reader_thread(LPVOID param) {
    ReaderCtx *ctx = (ReaderCtx *)param;
    FrameState fs;
    framing_state_init(&fs);

    uint8_t rxbuf[RX_BUF_SIZE];

    while (InterlockedCompareExchange(&ctx->running, 1, 1)) {
        DWORD got = 0;
        if (!ReadFile(ctx->h, rxbuf, sizeof(rxbuf), &got, NULL)) {
            DWORD err = GetLastError();
            if (err != ERROR_SUCCESS) {
                fprintf(stderr, "[RX] ReadFile error 0x%08lX — stopping reader\n",
                        (unsigned long)err);
                printf(">> {\"type\":\"serial_error\",\"code\":%lu}\n",
                       (unsigned long)err);
                InterlockedExchange(&ctx->running, 0);
            }
            Sleep(1);
            continue;
        }
        if (got == 0) { Sleep(1); continue; }

        for (DWORD bi = 0; bi < got; bi++)
            framing_feed(&fs, rxbuf[bi], dispatch_packet, ctx);
    }

    return 0;
}

/* ------------------------------------------------------------------ */
/*  Session — groups all runtime state for one engraving run         */
/* ------------------------------------------------------------------ */

typedef struct {
    PolarPoint *polar;      /* converted point array           */
    size_t      count;      /* number of points                */
    HANDLE      port;       /* open serial port handle         */
    ReaderCtx   reader;     /* background reader thread state  */
    HANDLE      thread;     /* reader thread handle            */
} Session;

/* ------------------------------------------------------------------ */
/*  Production point loader — wraps the two-step inputParser API     */
/* ------------------------------------------------------------------ */

static PolarPoint *load_polar_points(const char *file, size_t *count)
{
    Coordinate *coords = getCoordinates(file, count);
    if (!coords || *count == 0) {
        free(coords);
        return NULL;
    }
    PolarPoint *polar = convertToPolar(coords, *count);
    free(coords);
    return polar;   /* NULL propagates naturally on allocation failure */
}

static void cleanup_session(Session *s);   /* forward declaration */

/* ------------------------------------------------------------------ */
/*  init_session sub-functions  (S-SRP-01)                           */
/* ------------------------------------------------------------------ */

/** Load points from file into s->polar / s->count. */
static int load_points(Session *s, const char *file, LoadPointsFn load_fn)
{
    s->polar = load_fn(file, &s->count);
    if (!s->polar || s->count == 0) {
        fprintf(stderr, "Failed to parse coordinates from %s\n", file);
        printf(">> {\"type\":\"error\",\"msg\":\"coordinate parse failed\","
               "\"file\":\"%s\"}\n", file);
        return 1;
    }
    return 0;
}

/** Open the serial port and store the handle in s->port. */
static int open_port(Session *s, const char *port_name, OpenSerialFn open_fn)
{
    s->port = open_fn(port_name, BAUD_RATE);
    if (s->port == INVALID_HANDLE_VALUE) {
        fprintf(stderr, "Failed to open %s\n", port_name);
        printf(">> {\"type\":\"error\",\"msg\":\"serial open failed\","
               "\"port\":\"%s\"}\n", port_name);
        return 1;
    }
    return 0;
}

/** Create the ACK event and start the reader thread. */
static int start_reader(Session *s)
{
    s->reader.h         = s->port;
    s->reader.running   = 1;
    s->reader.ack_count = 0;
    s->reader.ack_event = CreateEvent(NULL, FALSE, FALSE, NULL);
    if (!s->reader.ack_event) {
        fprintf(stderr, "CreateEvent failed: 0x%08lX\n",
                (unsigned long)GetLastError());
        printf(">> {\"type\":\"error\",\"msg\":\"CreateEvent failed\"}\n");
        return 1;
    }

    s->thread = CreateThread(NULL, 0, reader_thread, &s->reader, 0, NULL);
    if (!s->thread) {
        fprintf(stderr, "Failed to create reader thread\n");
        printf(">> {\"type\":\"error\",\"msg\":\"reader thread failed\"}\n");
        CloseHandle(s->reader.ack_event);
        return 1;
    }
    return 0;
}

/**
 * init_session - sequencer: load points, open port, start reader.  (S-SRP-01)
 * open_fn and load_fn are injected so callers can substitute stubs.  (S-DIP-01)
 * Returns 0 on success.
 */
static int init_session(Session *s,
                         const char *port_name, const char *file,
                         OpenSerialFn open_fn, LoadPointsFn load_fn)
{
    memset(s, 0, sizeof(*s));
    s->port   = INVALID_HANDLE_VALUE;
    s->thread = NULL;

    if (load_points(s, file, load_fn) != 0)    return 1;
    if (open_port(s, port_name, open_fn) != 0) { cleanup_session(s); return 1; }
    if (start_reader(s) != 0)                  { cleanup_session(s); return 1; }
    return 0;
}

/* ------------------------------------------------------------------ */
/*  Cleanup                                                           */
/* ------------------------------------------------------------------ */

static void cleanup_session(Session *s) {
    if (s->thread) {
        InterlockedExchange(&s->reader.running, 0);
        SetEvent(s->reader.ack_event);
        if (WaitForSingleObject(s->thread, 3000) == WAIT_TIMEOUT) {
            fprintf(stderr, "[cleanup] reader thread did not exit — forcing termination\n");
            TerminateThread(s->thread, 1);
        }
        CloseHandle(s->thread);
        CloseHandle(s->reader.ack_event);
    }
    if (s->port != INVALID_HANDLE_VALUE)
        CloseHandle(s->port);
    free(s->polar);
}

/* ------------------------------------------------------------------ */
/*  Entry point                                                       */
/* ------------------------------------------------------------------ */

int main(int argc, char **argv) {
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);

    const char *port_name = (argc >= 2) ? argv[1] : "COM25";
    const char *file      = (argc >= 3) ? argv[2] : "input.gds";

    Session s;
    if (init_session(&s, port_name, file, open_serial, load_polar_points) != 0)
        return 1;

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
    printf("Sending %zu polar points over %s...\n", s.count, port_name);

    for (size_t i = 0; i < s.count; i++) {
        LONG target_ack = (LONG)(i + 1);

        if (!send_polar_point(s.port, s.polar[i].r, s.polar[i].theta)) {
            fprintf(stderr, "UART send failed at i=%zu\n", i);
            cleanup_session(&s);
            return 1;
        }

        if (!wait_for_ack(&s.reader, target_ack, ACK_TIMEOUT)) {
            fprintf(stderr, "Timeout waiting for ACK %ld (i=%zu)%s\n",
                    (long)target_ack, i,
                    (i == 0) ? " — verify FPGA booted and UART is connected"
                             : "");
            cleanup_session(&s);
            return 1;
        }
    }

    /* ---- end packet ---- */
    if (!send_end_packet(s.port)) {
        fprintf(stderr, "UART send failed (end packet)\n");
        cleanup_session(&s);
        return 1;
    }
    if (!wait_for_ack(&s.reader, (LONG)(s.count + 1), ACK_TIMEOUT)) {
        fprintf(stderr, "Timeout waiting for end ACK\n");
        cleanup_session(&s);
        return 1;
    }

    LONG final_acks = InterlockedCompareExchange(&s.reader.ack_count, 0, 0);
    printf("Done - %zu points sent, %ld ACKs received.\n",
           s.count, (long)final_acks);
    printf(">> {\"type\":\"done\",\"points_sent\":%zu,\"acks_received\":%ld}\n",
           s.count, (long)final_acks);

    cleanup_session(&s);
    return 0;
}
