#include "inputParser.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define PI 3.14159265358979323846

/*
UNITS ARE IN UM (micrometres)
COMPILE WITH:
gcc -Wall -Wextra -O2 -o inputParser inputParser.c -lm
*/

// read input file
// grab the points starting from "XY"
Coordinate *getCoordinates(const char *filename, size_t *count) {
    FILE *fp = fopen(filename, "r");
    if (!fp) { perror("fopen"); return NULL; }

    char line[256]; // buffer for reading file
    int reading = 0;
    int x, y;

    size_t capacity = 8; // array capacity
    *count = 0; // current number of coordinate pairs

    Coordinate *coordinates = malloc(capacity * sizeof(Coordinate));
    if (!coordinates) { fclose(fp); return NULL; }

    while (fgets(line, sizeof(line), fp)) {
        if (!reading && strncmp(line, "XY", 2) == 0) {
            reading = 1;
            memmove(line, line + 2, strlen(line + 2) + 1); // strips leading "XY"
        }

        if (reading) {
            if (strstr(line, "ENDEL")) break; // end of coordinates

            if (sscanf(line, "%d : %d", &x, &y) == 2) {
                if (*count == capacity) { // array dynamic resize
                    capacity *= 2;
                    Coordinate *temp = realloc(coordinates, capacity * sizeof(Coordinate));
                    if (!temp) { free(coordinates); fclose(fp); return NULL; }
                    coordinates = temp;
                }

                coordinates[*count].x = x;
                coordinates[*count].y = y;
                (*count)++;
            } else if (line[0] != '\n' && line[0] != '\r' && line[0] != '\0') {
                fprintf(stderr, "[WARN] unparseable line: %s", line);
            }
        }
    }

    fclose(fp);
    return coordinates;
}

PolarPoint *convertToPolar(const Coordinate *coords, size_t count) {
    PolarPoint *polar = malloc(count * sizeof(PolarPoint));
    if (!polar) return NULL;

    for (size_t i = 0; i < count; i++) {
        double x = (double)coords[i].x;
        double y = (double)coords[i].y;

        polar[i].r = sqrt(x * x + y * y);

        double theta_rad = atan2(y, x);
        double theta_deg = theta_rad * (180.0 / PI);

        if (theta_deg < 0) theta_deg += 360.0; // normalize to [0,360)
        polar[i].theta = theta_deg;
    }

    return polar;
}

