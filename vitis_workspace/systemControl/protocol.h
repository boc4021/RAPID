/*
 * protocol.h - RAPID wire protocol constants (FPGA / Vitis side).
 *
 * This file is an exact copy of src/protocol.h in the PC-side project.
 * The Vitis build environment uses isolated include paths, so it cannot
 * reference src/ directly.  Keep both files in sync whenever constants change.
 *
 * Canonical location: src/protocol.h
 */
#pragma once
#include <stdint.h>

/* ---- Frame structure -------------------------------------------------
 *   SOF (2 B) | TYPE (1 B) | LEN (1 B) | PAYLOAD (LEN B) | CRC8 (1 B)
 *   CRC8 is XOR of all bytes from TYPE through the last PAYLOAD byte.
 * -------------------------------------------------------------------- */
#define SOF_BYTE_1      0xAAu
#define SOF_BYTE_2      0x55u

/* ---- Packet type codes ----------------------------------------------- */
#define TYPE_POINT      0x01u   /* PC → FPGA: polar point     (LEN = POINT_LEN) */
#define TYPE_END        0x03u   /* PC → FPGA: end of sequence (LEN = 0)         */
#define TYPE_ACK        0x81u   /* FPGA → PC: ACK, echoes incoming payload       */
#define TYPE_DEBUG      0xF0u   /* FPGA → PC: UTF-8 debug / status string        */

/* ---- Point payload (TYPE_POINT and TYPE_ACK for points) -------------
 *   Bytes 0–3: r_um      int32   little-endian  radius in micrometres
 *   Bytes 4–7: theta_deg float32 little-endian  angle  in degrees [0, 360)
 * -------------------------------------------------------------------- */
#define POINT_LEN       0x08u

/* ---- Transport ------------------------------------------------------ */
#define BAUD_RATE       115200u

/* ---- Physical disc parameters ---------------------------------------
 *   Stepper-to-radius mapping:
 *     steps = round( r_um * MAX_STEPS / DISC_RADIUS_UM )
 *   0 µm     → step 0        (inner edge / home position)
 *   33000 µm → step 8500     (outer edge, full disc travel)
 * -------------------------------------------------------------------- */
#define DISC_RADIUS_UM  33000u  /* outer radius in µm  (33 mm standard CD) */
#define MAX_STEPS       8500u   /* stepper steps from home to outer edge    */
