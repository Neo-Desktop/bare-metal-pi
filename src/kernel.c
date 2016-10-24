/*

    Part of the Raspberry-Pi Bare Metal Tutorials
    Copyright (c) 2015, Brian Sidebotham
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice,
        this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
        this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
    ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
    CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
    SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
    INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
    CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
    ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

*/

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "rpi/aux.h"
#include "rpi/armtimer.h"
#include "rpi/gpio.h"
#include "rpi/interrupts.h"
#include "rpi/mailbox-interface.h"
#include "rpi/systimer.h"
#include "kernel/util.h"


double pow(double, int);
double sin(double);

/** Main function - we'll never return from here */
void kernel_main( unsigned int r0, unsigned int r1, unsigned int atags )
{
    int width = SCREEN_WIDTH, height = SCREEN_HEIGHT, bpp = SCREEN_DEPTH;
    int x, y, pitch = 0;
    colour_t current_colour;
    volatile unsigned char* fb = NULL;
    int pixel_offset;
    int r, g, b, a;
    float cd = COLOUR_DELTA, x_float = 0, y_float = 0;
    double sinTest, y_test = 0;
    unsigned int frame_count = 0;

    /* Write 1 to the LED init nibble in the Function Select GPIO
       peripheral register to enable LED pin as an output */
    RPI_GetGpio()->LED_GPFSEL |= LED_GPFBIT;

    /* Enable the timer interrupt IRQ */
    RPI_GetIrqController()->Enable_Basic_IRQs = RPI_BASIC_ARM_TIMER_IRQ;

    /* Setup the system timer interrupt */
    /* Timer frequency = Clk/256 * 0x400 */
    RPI_GetArmTimer()->Load = 0x400;

    /* Setup the ARM Timer */
    RPI_GetArmTimer()->Control =
            RPI_ARMTIMER_CTRL_23BIT |
            RPI_ARMTIMER_CTRL_ENABLE |
            RPI_ARMTIMER_CTRL_INT_ENABLE |
            RPI_ARMTIMER_CTRL_PRESCALE_256;

    /* Enable interrupts! */
    _enable_interrupts();

    /* Initialise the UART */
    RPI_AuxMiniUartInit( 115200, 8 );

    /* Print to the UART using the standard libc functions */
    printf( "Bare Metal RPI\r\n" );
    printf( "Initialise UART console with standard libc\r\n" );
    printf( "BY ME AND MY BAE XOXOXOX OXOX :D\r\n\rn" );

    RPI_PropertyInit();
    RPI_PropertyAddTag( TAG_GET_BOARD_MODEL );
    RPI_PropertyAddTag( TAG_GET_BOARD_REVISION );
    RPI_PropertyAddTag( TAG_GET_FIRMWARE_VERSION );
    RPI_PropertyAddTag( TAG_GET_BOARD_MAC_ADDRESS );
    RPI_PropertyAddTag( TAG_GET_BOARD_SERIAL );
    RPI_PropertyAddTag( TAG_GET_MAX_CLOCK_RATE, TAG_CLOCK_ARM );
    RPI_PropertyProcess();

    rpi_mailbox_property_t* mp;
    mp = RPI_PropertyGet( TAG_GET_BOARD_MODEL );

    if( mp )
        printf( "Board Model: %d\r\n", mp->data.value_32 );
    else
        printf( "Board Model: NULL\r\n" );

    mp = RPI_PropertyGet( TAG_GET_BOARD_REVISION );

    if( mp )
        printf( "Board Revision: %d\r\n", mp->data.value_32 );
    else
        printf( "Board Revision: NULL\r\n" );

    mp = RPI_PropertyGet( TAG_GET_FIRMWARE_VERSION );

    if( mp )
        printf( "Firmware Version: %d\r\n", mp->data.value_32 );
    else
        printf( "Firmware Version: NULL\r\n" );

    mp = RPI_PropertyGet( TAG_GET_BOARD_MAC_ADDRESS );

    if( mp )
        printf( "MAC Address: %2.2X:%2.2X:%2.2X:%2.2X:%2.2X:%2.2X\r\n",
               mp->data.buffer_8[0], mp->data.buffer_8[1], mp->data.buffer_8[2],
               mp->data.buffer_8[3], mp->data.buffer_8[4], mp->data.buffer_8[5] );
    else
        printf( "MAC Address: NULL\r\n" );

    mp = RPI_PropertyGet( TAG_GET_BOARD_SERIAL );

    if( mp )
        printf( "Serial Number: %8.8X%8.8X\r\n",
                mp->data.buffer_32[0], mp->data.buffer_32[1] );
    else
        printf( "Serial Number: NULL\r\n" );

    mp = RPI_PropertyGet( TAG_GET_MAX_CLOCK_RATE );

    if( mp )
        printf( "Maximum ARM Clock Rate: %dHz\r\n", mp->data.buffer_32[1] );
    else
        printf( "Maximum ARM Clock Rate: NULL\r\n" );

    /* Ensure the ARM is running at it's maximum rate */
    RPI_PropertyInit();
    RPI_PropertyAddTag( TAG_SET_CLOCK_RATE, TAG_CLOCK_ARM, mp->data.buffer_32[1] );
    RPI_PropertyProcess();

    RPI_PropertyInit();
    RPI_PropertyAddTag( TAG_GET_CLOCK_RATE, TAG_CLOCK_ARM );
    RPI_PropertyProcess();

    mp = RPI_PropertyGet( TAG_GET_CLOCK_RATE );

    if( mp )
        printf( "Set ARM Clock Rate: %dHz\r\n", mp->data.buffer_32[1] );
    else
        printf( "Set ARM Clock Rate: NULL\r\n" );

    /* Initialise a framebuffer... */
    RPI_PropertyInit();
    RPI_PropertyAddTag( TAG_ALLOCATE_BUFFER );
    RPI_PropertyAddTag( TAG_SET_PHYSICAL_SIZE, SCREEN_WIDTH, SCREEN_HEIGHT );
    RPI_PropertyAddTag( TAG_SET_VIRTUAL_SIZE, SCREEN_WIDTH, SCREEN_HEIGHT * 2 );
    RPI_PropertyAddTag( TAG_SET_DEPTH, SCREEN_DEPTH );
    RPI_PropertyAddTag( TAG_GET_PITCH );
    RPI_PropertyAddTag( TAG_GET_PHYSICAL_SIZE );
    RPI_PropertyAddTag( TAG_GET_DEPTH );
    RPI_PropertyProcess();

    if( ( mp = RPI_PropertyGet( TAG_GET_PHYSICAL_SIZE ) ) )
    {
        width = mp->data.buffer_32[0];
        height = mp->data.buffer_32[1];

        printf( "Initialised Framebuffer: %dx%d ", width, height );
    }

    if( ( mp = RPI_PropertyGet( TAG_GET_DEPTH ) ) )
    {
        bpp = mp->data.buffer_32[0];
        printf( "%dbpp\r\n", bpp );
    }

    if( ( mp = RPI_PropertyGet( TAG_GET_PITCH ) ) )
    {
        pitch = mp->data.buffer_32[0];
        printf( "Pitch: %d bytes\r\n", pitch );
    }

    if( ( mp = RPI_PropertyGet( TAG_ALLOCATE_BUFFER ) ) )
    {
        fb = (unsigned char*)mp->data.buffer_32[0];
        printf( "Framebuffer address: %8.8X\r\n", (unsigned int)fb );
    }

    /* Never exit as there is no OS to exit to! */
    current_colour.r = 0x4B; 
    current_colour.g = 0x00; 
    current_colour.b = 0x82;
    current_colour.a = 0;

    double t = 0;

    while( 1 )
    {
        t += 0.014;
        /* Produce a colour spread across the screen */
        for( y = 0; y < height; y++ )
        {
            // current_colour.r += ( 1.0 / height );

            for( x = 0; x < width; x++ )
            {
                pixel_offset = ( x * ( bpp >> 3 ) ) + ( y * pitch );

                
                r = (int)( current_colour.r * 0xFF ) & 0xFF;
                g = (int)( current_colour.g * 0xFF ) & 0xFF;
                b = (int)( current_colour.b * 0xFF ) & 0xFF;
                a = (int)( current_colour.a * 0xFF ) & 0xFF;

                /*
                // Three bytes to write
                fb[ pixel_offset++ ] = r;
                fb[ pixel_offset++ ] = g;
                fb[ pixel_offset++ ] = b;
                */

                /*
                if (x < 480 && y < 480) {
                    pixel_offset -= 3;
                    fb[ pixel_offset ] = image_data[x*y];
                    pixel_offset += 3;
                }
                */

                x_float = ((float)x)/(float)SCREEN_WIDTH;
                y_float = 1 - ((2 * (float)y) / (float)SCREEN_HEIGHT);
                sinTest = sin(x_float*2*3.14159265 + t);
                y_test = y_float;
                if ( sinTest > y_test && (sinTest - 0.014 < y_test)) {
                    fb[ pixel_offset++ ] = r;
                    fb[ pixel_offset++ ] = g;
                    fb[ pixel_offset++ ] = b;
                    //printf("at (%d, %d) - Sine = %f - Y = %f \r\n", x, y, sinTest, y_test);
                }

                /*printf("Sine test pi / 2 [1]- %f\r\n", sin(3.1415926 / 2));
                printf("Sine test pi / 4 [0.707]- %f\r\n", sin(3.1415926 / 4));
                printf("Sine test pi [0]- %f\r\n", sin(3.1415926));
                printf("Sine test pi / 3 [0.866]- %f\r\n", sin(3.1415926 / 3));
                printf("Sine test pi / 6 [0.5]- %f\r\n ", sin(3.1415926 / 6));*/

                /*

                if ( sin )

                */

                /*
                if( bpp == 32 )
                {
                    //Four bytes to write
                    fb[ pixel_offset++ ] = r;
                    fb[ pixel_offset++ ] = g;
                    fb[ pixel_offset++ ] = b;
                    fb[ pixel_offset++ ] = a;
                }
                else if( bpp == 24 )
                {
                    // Three bytes to write
                    fb[ pixel_offset++ ] = r;
                    fb[ pixel_offset++ ] = g;
                    fb[ pixel_offset++ ] = b;
                }
                else if( bpp == 16 )
                {
                    // Two bytes to write 
                    // Bit pack RGB565 into the 16-bit pixel offset 
                    *(unsigned short*)&fb[pixel_offset] = ( (r >> 3) << 11 ) | ( ( g >> 2 ) << 5 ) | ( b >> 3 );
                }
                else
                {
                    //Palette mode. TODO: Work out a colour scheme for
                    //   packing rgb into an 8-bit palette! 
                }
                */

                // current_colour.b += ( 1.0 / width );
            }
        }

        /* Scroll through the green colour */
        /*
        current_colour.g += cd;
        if( current_colour.g > 1.0 )
        {
            current_colour.g = 1.0;
            cd = -COLOUR_DELTA;
        }
        else if( current_colour.g < 0.0 )
        {
            current_colour.g = 0.0;
            cd = COLOUR_DELTA;
        }
        */

        frame_count++;
        if( calculate_frame_count )
        {
            calculate_frame_count = 0;

            /* Number of frames in a minute, divided by seconds per minute */
            float fps = (float)frame_count / 60;
            printf( "FPS: %.2f\r\n", fps );

            frame_count = 0;
        }
    }
}

double sin(double x)
{
    double x2 = x * x;
    if (x < 3.14159265359)
        return (((((-0.0000000205342856289746600727*x2 + 0.00000270405218307799040084)*x2 - 0.000198125763417806681909)*x2 + 0.00833255814755188010464)*x2 - 0.166665772196961623983)*x2 + 0.999999707044156546685)*x;

    else if (x <= 3.14159265359 * 2) return (-sin(x - 3.14159265359));

    else
    {
        while (x > 3.14159265359 * 2)
            x -= 3.14159265359 * 2;

        return sin(x);
    }
}

