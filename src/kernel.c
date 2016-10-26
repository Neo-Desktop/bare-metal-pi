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
#include <math.h>

#include "rpi/aux.h"
#include "rpi/armtimer.h"
#include "rpi/gpio.h"
#include "rpi/interrupts.h"
#include "rpi/mailbox-interface.h"
#include "rpi/systimer.h"
#include "kernel/util.h"
#include "kernel/image_data.h"


/** Main function - we'll never return from here */
void kernel_main( unsigned int r0, unsigned int r1, unsigned int atags )
{
    int width = SCREEN_WIDTH, height = SCREEN_HEIGHT, bpp = SCREEN_DEPTH;
    int x, y, pitch = 0;
    colour_t current_colour;
    volatile unsigned char* fb = NULL;
    int pixel_offset;
    int r, g, b, a;
    float cd = COLOUR_DELTA, x_float = 0, y_float = 0, t = 0, fps = 0, sinTest = 0;
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
    printf( "\033c\r\n" );
    printf( "Bare Metal RPI\r\n" );
    printf( "------------------------------------------------\r\n" );

    RPI_PropertyInit();
    RPI_PropertyAddTag( TAG_GET_BOARD_MODEL );
    RPI_PropertyAddTag( TAG_GET_BOARD_REVISION );
    RPI_PropertyAddTag( TAG_GET_FIRMWARE_VERSION );
    RPI_PropertyAddTag( TAG_GET_BOARD_MAC_ADDRESS );
    RPI_PropertyAddTag( TAG_GET_BOARD_SERIAL );
    RPI_PropertyAddTag( TAG_GET_MAX_CLOCK_RATE, TAG_CLOCK_ARM );
    RPI_PropertyProcess();

    rpi_mailbox_property_t* mp;

    if( ( mp = RPI_PropertyGet( TAG_GET_BOARD_MODEL ) ) )
        printf( "Board Model: %d\r\n", mp->data.value_32 );
    else
        printf( "Board Model: NULL\r\n" );

    if( ( mp = RPI_PropertyGet( TAG_GET_BOARD_REVISION ) ) )
        printf( "Board Revision: %d\r\n", mp->data.value_32 );
    else
        printf( "Board Revision: NULL\r\n" );

    if( ( mp = RPI_PropertyGet( TAG_GET_FIRMWARE_VERSION ) ) )
        printf( "Firmware Version: %d\r\n", mp->data.value_32 );
    else
        printf( "Firmware Version: NULL\r\n" );

    if( ( mp = RPI_PropertyGet( TAG_GET_BOARD_MAC_ADDRESS ) ) )
        printf( "MAC Address: %2.2X:%2.2X:%2.2X:%2.2X:%2.2X:%2.2X\r\n",
               mp->data.buffer_8[0], mp->data.buffer_8[1], mp->data.buffer_8[2],
               mp->data.buffer_8[3], mp->data.buffer_8[4], mp->data.buffer_8[5] );
    else
        printf( "MAC Address: NULL\r\n" );

    if( ( mp = RPI_PropertyGet( TAG_GET_BOARD_SERIAL ) ) )
        printf( "Serial Number: %8.8X%8.8X\r\n",
                mp->data.buffer_32[0], mp->data.buffer_32[1] );
    else
        printf( "Serial Number: NULL\r\n" );

    if( ( mp = RPI_PropertyGet( TAG_GET_MAX_CLOCK_RATE ) ) )
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

    if( ( mp = RPI_PropertyGet( TAG_GET_CLOCK_RATE ) ) )
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
    current_colour.g = 0x100; 
    current_colour.b = 0x82;
    current_colour.a = 0;

    while( 1 )
    {
        t += 0.02;

/*
        y = 50;
        x = 0;
        
        y = 5 * (cos(x) + sin(x))
*/

        /* Produce a colour spread across the screen */
        for( y = 0; y < height; y++ )
        {
            // current_colour.r += ( 1.0 / height );

            if (y > 120 && y < 365)
            for( x = 0; x < width; x++ )
            {
                pixel_offset = ( x * ( bpp >> 3 ) ) + ( y * pitch );
                
                r = (int)( current_colour.r * 0xFF ) & 0xFF;
                g = (int)( current_colour.g * 0xFF ) & 0xFF;
                b = (int)( current_colour.b * 0xFF ) & 0xFF;
                a = (int)( current_colour.a * 0xFF ) & 0xFF;

                x_float = ((float)x)/(float)SCREEN_WIDTH;
                y_float = 1 - ((2 * (float)y) / (float)SCREEN_HEIGHT);
                sinTest = sin(x_float * 4 * M_PI + t);

                if ( sinTest / 2 > y_float && ((sinTest - 0.05) / 2 < y_float)) {
                    fb[ pixel_offset++ ] = r;
                    fb[ pixel_offset++ ] = g;
                    fb[ pixel_offset++ ] = b;
                    //printf("at (%d, %d) - Sine = %f - Y = %f \r\n", x, y, sinTest, y_float);
                } else {
                    fb[ pixel_offset++ ] = 0;
                    fb[ pixel_offset++ ] = 0;
                    fb[ pixel_offset++ ] = 0;
                }

            }
        }

        frame_count++;
        if( calculate_frame_count )
        {
            calculate_frame_count = 0;

            /* Number of frames in a minute, divided by seconds per minute */
            fps = (float)frame_count / 60;
            printf( "FPS: %.2f\r\n", fps );

            frame_count = 0;
        }
    }
}
