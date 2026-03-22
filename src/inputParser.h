#pragma once
#include <stddef.h>
#include <stdint.h>

typedef struct {
    int x;
    int y;
} Coordinate;

typedef struct {
    double r;
    double theta;
} PolarPoint;

Coordinate *getCoordinates(const char *filename, size_t *count);
PolarPoint  *convertToPolar(const Coordinate *coords, size_t count);