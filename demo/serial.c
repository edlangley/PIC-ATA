/*-------------------------------------------------------------------
Name......... testata.c
Author....... Ed Langley
Date......... 19/10/2006
Description.. Low level stdin/stdout code for the printf() and other
............. string handling functions in the Hitech library.
-------------------------------------------------------------------*/

#include "platform.h"
#include <conio.h>
#include "serial.h"

/* Serial initialization */
void InitComms()
{
  RX_PIN = 1;
  TX_PIN = 1;
  SPBRG = DIVIDER;
  RCSTA = (NINE_BITS|0x90);
  TXSTA = (SPEED|NINE_BITS|0x20);
}

/* PUTCH() - outputs a byte to the serial port */
//void putch(unsigned char byte)
void putch(char byte)
{
	while(!TXIF);	/* set when register is empty */
		TXREG = byte;	/* output one byte */
}

/* Output a string via the serial port */
/*
void puts(const char *s)
{
	while(s && *s)
		putch(*s++);
}
*/

/* filler for now */
char getch(void)
{
  return 0xff;
}
