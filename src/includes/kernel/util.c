/**
util.c
*/
#include "util.h"

void setColor(volatile unsigned char* fb, colour_t pixel, long x, long y, long pitch)
{
	int pixel_offset = ( x * ( SCREEN_DEPTH >> 3 ) ) + ( y * pitch );

    int r = (int)( pixel.r * 0xFF ) & 0xFF;
    int g = (int)( pixel.g * 0xFF ) & 0xFF;
    int b = (int)( pixel.b * 0xFF ) & 0xFF;
    int a = (int)( pixel.a * 0xFF ) & 0xFF;

    if( SCREEN_DEPTH == 32 )
    {
        /* Four bytes to write */
        fb[ pixel_offset++ ] = r;
        fb[ pixel_offset++ ] = g;
        fb[ pixel_offset++ ] = b;
        fb[ pixel_offset++ ] = a;
    }
    else if( SCREEN_DEPTH == 24 )
    {
        /* Three bytes to write */
        fb[ pixel_offset++ ] = r;
        fb[ pixel_offset++ ] = g;
        fb[ pixel_offset++ ] = b;
    }
    else if( SCREEN_DEPTH == 16 )
    {
        /* Two bytes to write */
        /* Bit pack RGB565 into the 16-bit pixel offset */
        *(unsigned short*)&fb[pixel_offset] = ( (r >> 3) << 11 ) | ( ( g >> 2 ) << 5 ) | ( b >> 3 );
    }
    else
    {
        /* Palette mode. TODO: Work out a colour scheme for
           packing rgb into an 8-bit palette! */
    }
}



// Takes a computed sin value and finds what pixel it cooresponds to
int scale_to_y_pixel(double y) 
{
    return (int) ((SCREEN_HEIGHT / 2) * (1 - y));
}

// Takes a horizontal pixel location and finds where it would be in a normal graph 0 < t < 2pi
double scale_to_x_origin(int x)
{
    return x * 2 * M_PI / SCREEN_WIDTH;
}


// Returns the coordinates of the sin function. sin[x] = y coordinate on the horizontal pixel x.
int *getCoordinates()
{
    int x = 0;
    int *yCords = (int*) malloc(sizeof(int) * SCREEN_WIDTH);
    memset(yCords, 0, SCREEN_WIDTH);
    for (x = 0; x < SCREEN_WIDTH; ++x)
    {
        double float_x = scale_to_x_origin(x);
        yCords[x] = scale_to_y_pixel(sin(float_x));
    }

    return yCords;
}


