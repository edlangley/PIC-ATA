/*-------------------------------------------------------------------
Name......... platform.h
Author....... Ed Langley
Date......... 23/12/2014
Description.. Header file with platform related definitions
............. for the ATA interface on a PIC MCU
-------------------------------------------------------------------*/
#ifndef _PLATFORM_H
#define _PLATFORM_H

#if USE_PICC18 == 1
 #include <pic18.h>

 #define SPINS_FOR_1MS                   308 /* Based on clk freq = 4x xtal freq */
 #define DELAY_MS(x)                     { volatile uint32_t i; i=x*SPINS_FOR_1MS; while(i) i--; }
#elif USE_PICC == 1
 #include <pic.h>

 #define SPINS_FOR_1MS                   308 /* Based on clk freq = 4x xtal freq */
 #define DELAY_MS(x)                     { volatile uint32_t i; i=x*SPINS_FOR_1MS; while(i) i--; }
#else
 #include <xc.h>

 #define DELAY_MS(x)                     __delay_ms(x)
#endif

#define _XTAL_FREQ                       3686000L

/* Type definitions */
typedef signed char int8_t;
typedef unsigned char uint8_t;
typedef signed short int16_t;
typedef unsigned short uint16_t;
typedef signed long int32_t;
typedef unsigned long uint32_t;

/* Defines */
#define TRUE                             1
#define FALSE                            0

#define PORT_INPUT                       0xFF
#define PORT_OUTPUT                      0x00


/* Macros */
#define BIT_SET(var, bitnum)             var |= (0x01 << bitnum)
#define BIT_CLR(var, bitnum)             var &= ~(0x01 << bitnum)

#define PIN_INPUT                        BIT_SET
#define PIN_OUTPUT                       BIT_CLR

#endif
