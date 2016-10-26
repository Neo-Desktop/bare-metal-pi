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




