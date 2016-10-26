/**
arm-016-util.h
*/


#ifndef RPI_UTIL_H
#define RPI_UTIL_H

#define SCREEN_WIDTH    640
#define SCREEN_HEIGHT   480
#define SCREEN_DEPTH    24      /* 16 or 32-bit */

#define COLOUR_DELTA    0.05    /* Float from 0 to 1 incremented by this amount */

typedef struct {
    float r;
    float g;
    float b;
    float a;
    } colour_t;


extern void setColor(volatile unsigned char* fb, colour_t pixel, long x, long y, long pitch);

#endif