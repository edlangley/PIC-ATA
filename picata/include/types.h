/*-------------------------------------------------------------------
Name......... types.h
Author....... Ed Langley
Date......... 14/11/2006
Description.. Definitions and macros for an MP3 player on a PIC MCU
-------------------------------------------------------------------*/

/* Type definitions */
typedef near bit BIT;
typedef signed char int8_t;
typedef unsigned char uint8_t;
typedef signed short int16_t;
typedef unsigned short uint16_t;
typedef signed long int32_t;
typedef unsigned long uint32_t;

/* Defines */
#define BIT_INPUT                        0x01
#define BIT_OUTPUT                       0x00

#define PORT_INPUT                       0xFF
#define PORT_OUTPUT                      0x00

#define ENABLE                           1
#define DISABLE                          0

#define N_ENABLE                         0
#define N_DISABLE                        1

#define TRUE                             1
#define FALSE                            0

/* Macros */
#define PORTBIT( adr, bit ) ( ( unsigned ) ( &adr ) * 8 + ( bit ) )

