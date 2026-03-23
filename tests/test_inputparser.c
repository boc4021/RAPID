/*
 * test_inputparser.c - Unit tests for src/inputParser.c
 *
 * Covers: getCoordinates (normal parse, no-XY-block, multi-coordinate,
 *         realloc growth path, inline-XY format), convertToPolar (basic,
 *         origin, negative-Y theta normalisation, all-quadrant angles).
 *
 * Compile (MinGW / MSYS2 UCRT64 from project root):
 *   gcc -O0 -Wall -Wextra -I src -o build/test_inputparser \
 *       tests/test_inputparser.c src/inputParser.c -lm
 *   build/test_inputparser && echo PASS
 *
 * Or via Make:
 *   make test-inputparser
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "inputParser.h"

/* ---- Soft assertion — logs failure and increments counter, does not abort - */

static int _check_failures = 0;

#define CHECK(expr)                                                          \
    do {                                                                     \
        if (!(expr)) {                                                       \
            fprintf(stderr, "  FAIL: %s  (%s:%d)\n", #expr, __FILE__, __LINE__); \
            _check_failures++;                                               \
            return;                                                          \
        }                                                                    \
    } while (0)

/* ---- Helpers ------------------------------------------------------------- */

/** Write content to a temporary file; return the path. */
static const char *write_tmp(const char *path, const char *content)
{
    FILE *f = fopen(path, "w");
    if (!f) { perror("write_tmp"); return NULL; }
    fputs(content, f);
    fclose(f);
    return path;
}

#define TMP(name, body) write_tmp("_t_" name ".gds", body)

/* ---- Test runner bookkeeping --------------------------------------------- */

static int tests_run    = 0;
static int tests_failed = 0;

#define RUN(fn)                                                     \
    do {                                                            \
        int _before = _check_failures;                              \
        printf("  %-55s", #fn "...");                              \
        fn();                                                       \
        tests_run++;                                                \
        if (_check_failures > _before) {                            \
            tests_failed++;                                         \
            printf("FAIL\n");                                       \
        } else {                                                    \
            printf("PASS\n");                                       \
        }                                                           \
    } while (0)

/* ============================================================= */
/*  getCoordinates tests                                         */
/* ============================================================= */

static void test_gc_basic_two_points(void)
{
    const char *path = TMP("basic", "XY 33000 : 0\n16500 : 0\nENDEL\n");
    size_t n = 0;
    Coordinate *c = getCoordinates(path, &n);
    CHECK(c && n == 2);
    CHECK(c[0].x == 33000 && c[0].y == 0);
    CHECK(c[1].x == 16500 && c[1].y == 0);
    free(c);
}

static void test_gc_xy_inline_first_coord(void)
{
    /* The first coordinate sits on the same line as "XY" — parser strips "XY". */
    const char *path = TMP("inline", "XY 10000 : 20000\nENDEL\n");
    size_t n = 0;
    Coordinate *c = getCoordinates(path, &n);
    CHECK(c && n == 1);
    CHECK(c[0].x == 10000 && c[0].y == 20000);
    free(c);
}

static void test_gc_no_xy_block_returns_zero(void)
{
    const char *path = TMP("noxy", "BOUNDARY\n1000 : 2000\nENDEL\n");
    size_t n = 0;
    Coordinate *c = getCoordinates(path, &n);
    /* No XY header → zero coordinates; array may be non-NULL but n must be 0 */
    CHECK(n == 0);
    free(c);
}

static void test_gc_negative_coordinates(void)
{
    const char *path = TMP("neg", "XY -16500 : -16500\nENDEL\n");
    size_t n = 0;
    Coordinate *c = getCoordinates(path, &n);
    CHECK(c && n == 1);
    CHECK(c[0].x == -16500 && c[0].y == -16500);
    free(c);
}

static void test_gc_realloc_growth(void)
{
    /* More than 8 coordinates triggers the dynamic realloc path
     * (initial capacity = 8 in inputParser.c). */
    const char *path = TMP("big",
        "XY 1 : 1\n2 : 2\n3 : 3\n4 : 4\n"
        "5 : 5\n6 : 6\n7 : 7\n8 : 8\n"
        "9 : 9\n10 : 10\n11 : 11\nENDEL\n");
    size_t n = 0;
    Coordinate *c = getCoordinates(path, &n);
    CHECK(c && n == 11);
    CHECK(c[10].x == 11 && c[10].y == 11);
    free(c);
}

static void test_gc_stops_at_endel(void)
{
    /* Coordinates after ENDEL must not be read. */
    const char *path = TMP("endel",
        "XY 100 : 0\n200 : 0\nENDEL\n300 : 0\n");
    size_t n = 0;
    Coordinate *c = getCoordinates(path, &n);
    CHECK(n == 2);
    free(c);
}

static void test_gc_empty_file_returns_zero(void)
{
    const char *path = TMP("empty", "");
    size_t n = 0;
    Coordinate *c = getCoordinates(path, &n);
    CHECK(n == 0);
    free(c);
}

static void test_gc_file_not_found_returns_null(void)
{
    size_t n = 0;
    Coordinate *c = getCoordinates("_t_nonexistent_xyz.gds", &n);
    CHECK(c == NULL);
}

/* ============================================================= */
/*  convertToPolar tests                                         */
/* ============================================================= */

static void test_cp_basic_x_axis(void)
{
    /* (33000, 0) → r = 33000 µm, theta = 0° */
    Coordinate c = {33000, 0};
    PolarPoint *p = convertToPolar(&c, 1);
    CHECK(p);
    CHECK(fabs(p[0].r - 33000.0) < 0.5);
    CHECK(fabs(p[0].theta) < 0.001);
    free(p);
}

static void test_cp_origin(void)
{
    /* (0, 0) → r = 0, theta = 0 (atan2(0,0) is implementation-defined but 0) */
    Coordinate c = {0, 0};
    PolarPoint *p = convertToPolar(&c, 1);
    CHECK(p);
    CHECK(p[0].r == 0.0);
    free(p);
}

static void test_cp_positive_y_axis(void)
{
    /* (0, 33000) → r = 33000 µm, theta = 90° */
    Coordinate c = {0, 33000};
    PolarPoint *p = convertToPolar(&c, 1);
    CHECK(p);
    CHECK(fabs(p[0].r - 33000.0) < 0.5);
    CHECK(fabs(p[0].theta - 90.0) < 0.001);
    free(p);
}

static void test_cp_negative_y_normalised(void)
{
    /* (0, -33000) → atan2 gives -90°; after normalisation → 270° */
    Coordinate c = {0, -33000};
    PolarPoint *p = convertToPolar(&c, 1);
    CHECK(p);
    CHECK(p[0].theta >= 0.0 && p[0].theta < 360.0);
    CHECK(fabs(p[0].theta - 270.0) < 0.001);
    free(p);
}

static void test_cp_third_quadrant_normalised(void)
{
    /* (-33000, -33000) → atan2 gives -135°; after normalisation → 225° */
    Coordinate c = {-33000, -33000};
    PolarPoint *p = convertToPolar(&c, 1);
    CHECK(p);
    CHECK(p[0].theta >= 0.0 && p[0].theta < 360.0);
    CHECK(fabs(p[0].theta - 225.0) < 0.001);
    free(p);
}

static void test_cp_45_degrees(void)
{
    /* (16500, 16500) → r ≈ 23334 µm, theta ≈ 45° */
    Coordinate c = {16500, 16500};
    PolarPoint *p = convertToPolar(&c, 1);
    CHECK(p);
    CHECK(fabs(p[0].r - 16500.0 * sqrt(2.0)) < 1.0);
    CHECK(fabs(p[0].theta - 45.0) < 0.001);
    free(p);
}

static void test_cp_all_theta_in_range(void)
{
    /* All four quadrants must produce theta in [0, 360). */
    Coordinate cs[4] = {
        { 1,  1},   /* Q1: ~45° */
        {-1,  1},   /* Q2: ~135° */
        {-1, -1},   /* Q3: ~225° */
        { 1, -1},   /* Q4: ~315° */
    };
    PolarPoint *p = convertToPolar(cs, 4);
    CHECK(p);
    for (int i = 0; i < 4; i++) {
        CHECK(p[i].theta >= 0.0 && p[i].theta < 360.0);
    }
    free(p);
}

static void test_cp_multiple_points_count(void)
{
    Coordinate cs[3] = {{1000, 0}, {0, 1000}, {-1000, 0}};
    PolarPoint *p = convertToPolar(cs, 3);
    CHECK(p);
    /* Just verify all three radii are ~1000 µm */
    for (int i = 0; i < 3; i++)
        CHECK(fabs(p[i].r - 1000.0) < 0.5);
    free(p);
}

/* ============================================================= */
/*  main                                                         */
/* ============================================================= */

int main(void)
{
    printf("=== test_inputparser ===\n\n");

    /* getCoordinates */
    RUN(test_gc_basic_two_points);
    RUN(test_gc_xy_inline_first_coord);
    RUN(test_gc_no_xy_block_returns_zero);
    RUN(test_gc_negative_coordinates);
    RUN(test_gc_realloc_growth);
    RUN(test_gc_stops_at_endel);
    RUN(test_gc_empty_file_returns_zero);
    RUN(test_gc_file_not_found_returns_null);

    /* convertToPolar */
    RUN(test_cp_basic_x_axis);
    RUN(test_cp_origin);
    RUN(test_cp_positive_y_axis);
    RUN(test_cp_negative_y_normalised);
    RUN(test_cp_third_quadrant_normalised);
    RUN(test_cp_45_degrees);
    RUN(test_cp_all_theta_in_range);
    RUN(test_cp_multiple_points_count);

    printf("\n%d / %d tests passed.\n", tests_run - tests_failed, tests_run);
    return tests_failed ? 1 : 0;
}
