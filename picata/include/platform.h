/*-------------------------------------------------------------------
Name......... platform.h
Author....... Ed Langley
Date......... 23/12/2014
Description.. Header file with platform related definitions
............. for the ATA interface on a PIC MCU
-------------------------------------------------------------------*/

#if USE_PICC18 == 1
 #include <pic18.h>
#elif USE_PICC == 1
 #include <pic.h>
#else
#include <xc.h>
#endif

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
