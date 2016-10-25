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

double sin(double x)
{
    double x2 = x * x;
    if (x < PI)
        return (((((-0.0000000205342856289746600727*x2 + 0.00000270405218307799040084)*x2 - 0.000198125763417806681909)*x2 + 0.00833255814755188010464)*x2 - 0.166665772196961623983)*x2 + 0.999999707044156546685)*x;

    else if (x <= PI * 2) return (-sin(x - PI));

    while (x > PI * 2)
        x -= PI * 2;

    return sin(x);
}




