/*-------------------------------------------------------------------
Name......... serial.h
Author....... Ed Langley
Date......... 19/10/2006
Description.. Low level stdin/stdout code for the printf() and other
............. string handling functions in the Hitech library.
-------------------------------------------------------------------*/

/* Comms setup */
#define BAUD                             9600
#define NINE                             0          /* Use 9bit communication? FALSE=8bit */
#define FOSC                             3686000L
#define HIGH_SPEED                       1

#ifdef USE_HSPLL  /* could change this in CFLAGS */
 #define CLK_FREQ                        (FOSC * 4)
#else
 #define CLK_FREQ                        FOSC
#endif



#if HIGH_SPEED == 1
 #define DIVIDER ((int)(CLK_FREQ/(16UL * BAUD) -1))
 #define SPEED 0x4
#else
 #define DIVIDER ((int)(CLK_FREQ/(64UL * BAUD) -1))
 #define SPEED 0
#endif

#if NINE == 1
 #define NINE_BITS 0x40
#else
 #define NINE_BITS 0
#endif

#define RX_PIN TRISC7
#define TX_PIN TRISC6

void InitComms();
void putch(unsigned char byte);
