/*-------------------------------------------------------------------
Name......... ata.c
Author....... Ed Langley
Date......... 19/10/2006
Description.. Driver to operate an ATA interface on a PIC MCU
-------------------------------------------------------------------*/

#include "platform.h"
#include "types.h"
#include "ata.h"

#if defined(ATA_USE_ID)
HDINFO	hd0info;
#endif

uint8_t CurrentDrv = 0;
uint32_t CurrentLBAAddr = 0;

/* API functions */

int8_t ATA_Init(uint8_t drivenumber)
{
	uint32_t i;

	/* disable A/Ds to allow pins to be used as digital I/O */
	ADCON1 = ALL_DIGITAL;

	/***** Set relevant PIC port bits to be outputs (clear TRIS bits) */
	/* 8255 address bits as outputs */
	TRIS_8255_ADDRESS &= ADDR_8255_CLEAR;

	/* 8255 CS bit as output */
	//Tris8255nCS = BIT_OUTPUT;

	/* RD/WR bits to 8255 as outputs */
	Tris8255nRD = BIT_OUTPUT;
	Tris8255nWR = BIT_OUTPUT;

	/* CS for 8255 as output*/
	Tris8255nCS = BIT_OUTPUT;

	/***** now talk to the ATA device */
	/* perform a reset */
	resetata();

	/* clear select card/head register prior to drive selection to
	   ensure known state */
	setataregbyte(ATA_SELECT_REG, 0);
	ATA_DriveSelect(drivenumber);
//	CurrentDrv = drivenumber;

	/* wait for RDY bit set */
	i = ATA_TIMEOUT_VAL;
	while(!(getataregbyte(ATA_STATUS_REG) & ATA_DRDY))
	{
		if(--i==0)
		{
			return(ATA_RDY_TIMEOUT);
		}
	}

	/* wait for BSY bit clear */
	i = ATA_TIMEOUT_VAL;
	while(getataregbyte(ATA_STATUS_REG) & ATA_BSY)
	{
		if(--i==0)
		{
			return(ATA_BSY_TIMEOUT);
		}
	}

	/* disable interrupts in the ATA device */
	setataregbyte(ATA_CONTROL_REG, ATA_INT_DISABLE);

	ATA_DriveSelect(drivenumber);

	setataregbyte(ATA_NSECTORS_REG, ATA_SECTORS_PER_TRACK);

	setataregbyte(ATA_COMMAND_REG, ATA_CMD_INIT);

	/* wait for BSY bit clear */
	i = ATA_TIMEOUT_VAL;
	while(getataregbyte(ATA_STATUS_REG) & ATA_BSY)
	{
		if(--i==0)
		{
			return(ATA_BSY_TIMEOUT);
		}
	}

	/* recalibrate - deprecated command, perhaps necessary
	   for older drives */
	//	setataregbyte(ATA_COMMAND_REG, ATA_CMD_RECAL);

	/* wait for BSY bit clear */
	/*
	i = ATA_TIMEOUT_VAL;
	while(getataregbyte(ATA_STATUS_REG) & ATA_BSY)
	{
		if(--i==0)
		{
			return(ATA_BSY_TIMEOUT);
		}
	}
	*/

	return(ATA_OK);
}

void ATA_DriveSelect(uint8_t drivenumber)
{
	uint8_t ataselectsetting;

	ataselectsetting = getataregbyte(ATA_SELECT_REG);
	if(drivenumber == ATA_MASTER)
	{
		ataselectsetting &= ~ATA_SLAVE;		/* Clear DRV bit: drive 0 */
	}
	else
	{
		ataselectsetting |= ATA_SLAVE;		/* Set DRV bit: drive 1 */
	}
	setataregbyte(ATA_SELECT_REG, ataselectsetting);

	CurrentDrv = drivenumber;
}

int8_t ATA_ReadDriveInfo()
{
	uint32_t	i;


//	drive now selected as part of API
//	driveselect(atadevice);

	/* wait for BSY bit clear */
	i = ATA_TIMEOUT_VAL;
	while(ATA_BSY & getataregbyte(ATA_STATUS_REG))
	{
		if(--i==0)
		{
			return(ATA_BSY_TIMEOUT);
		}
	}

	setataregbyte(ATA_COMMAND_REG, ATA_CMD_ID);

	/* wait for BSY bit clear */
	i = ATA_TIMEOUT_VAL;
	while(ATA_BSY & getataregbyte(ATA_STATUS_REG))
	{
		if(--i==0)
		{
			return(ATA_BSY_TIMEOUT);
		}
	}
	/* wait for DRQ bit set */
	i = ATA_TIMEOUT_VAL;
	while((ATA_DRQ & getataregbyte(ATA_STATUS_REG)) == 0)
	{
		if(--i==0)
		{
			return(ATA_DRQ_TIMEOUT);
		}
	}

	/* read the useful drive data to the union */
	for(i=0; i < HDINFO_SIZE_WORDS; i++)
	{
		//n = getataregword(ATA_DATA_REG);
		//vars->hd->bw[i] = BYTESWAP(n);
		//hd0info.bw[i] = getataregword(ATA_DATA_REG);
#if defined(ATA_USE_ID)
	  switch(i)
	    {
	    case ATA_DRVINFOWORD_CYLS:
	      hd0info.cyls = getataregword(ATA_DATA_REG);
	      break;
	    case ATA_DRVINFOWORD_HEADS:
	      hd0info.heads = getataregword(ATA_DATA_REG);
	      break;
	    case ATA_DRVINFOWORD_SECTORS:
	      hd0info.sectors = getataregword(ATA_DATA_REG);
	      break;
	    default:
	      if( (i>= ATA_DRVINFOWORD_MODELSTART) &&
		  (i < (ATA_DRVINFOWORD_MODELSTART + ATA_DRVINFOWORD_MODELLENGTH)) )
		{
		  hd0info.model[i-ATA_DRVINFOWORD_MODELSTART] = getataregword(ATA_DATA_REG);
		  //ATAModel[i-ATA_DRVINFOWORD_MODELSTART] = getataregword(ATA_DATA_REG);
		}
	      else if( (i>= ATA_DRVINFOWORD_REVSTART) &&
		  (i < (ATA_DRVINFOWORD_REVSTART + ATA_DRVINFOWORD_REVLENGTH)) )
		{
		  hd0info.fw_rev[i-ATA_DRVINFOWORD_REVSTART] = getataregword(ATA_DATA_REG);
		}
	      else if( (i>= ATA_DRVINFOWORD_SERNUMSTART) &&
		  (i < (ATA_DRVINFOWORD_SERNUMSTART + ATA_DRVINFOWORD_SERNUMLENGTH)) )
		{
		  hd0info.fw_rev[i-ATA_DRVINFOWORD_SERNUMSTART] = getataregword(ATA_DATA_REG);
		}
	      else
		{
		  getataregword(ATA_DATA_REG);

		}
	    }
#else
	  getataregword(ATA_DATA_REG);
#endif
	}

	/* finish up the command by reading the rest of the
	   drive data to nothing */
	/*for(i = REDUCED_HDINFO_SIZE_WORDS; i < HDINFO_SIZE_WORDS; i++)
	{
		getataregword(ATA_DATA_REG);
	}
	*/

	return(ATA_OK);
}

#if defined(ATA_USE_ID)
uint8_t *ATA_GetInfoModel()
{
  //hd0info.info.model[39] = '\0';
  //return(hd0info.info.model);

  hd0info.model[ATA_DRVINFOWORD_MODELLENGTH-1] &= '\0';
  return((uint8_t*)(hd0info.model));

  //ATAModel[ATA_DRVINFOWORD_MODELLENGTH-1] &= '\0';
  //return((uint8_t*)(ATAModel));
}

uint8_t *ATA_GetInfoRev()
{
  //hd0info.info.fw_rev[7] = '\0';
  //return(hd0info.info.fw_rev);
  hd0info.fw_rev[ATA_DRVINFOWORD_REVLENGTH-1] &= '\0';
  return((uint8_t*)(hd0info.fw_rev));
}

uint8_t *ATA_GetInfoSerialNum()
{
  //hd0info.info.serial_no[19] = '\0';
  //return(hd0info.info.serial_no);
  hd0info.serial_no[ATA_DRVINFOWORD_SERNUMLENGTH-1] = '\0';
  return((uint8_t*)(hd0info.serial_no));
}

uint16_t ATA_GetInfoNumCyls()
{
  //return(hd0info.info.cyls);
  return(hd0info.cyls);
}

uint16_t ATA_GetInfoNumHeads()
{
  //return(hd0info.info.heads);
  return(hd0info.heads);
}

uint16_t ATA_GetInfoNumSectors()
{
  //return(hd0info.info.sectors);
  return(hd0info.sectors);
}

#endif

int8_t ATA_SetLBAForRead(uint32_t lba)
{
  uint32_t	timeout;

  writelba(lba);

  setataregbyte(ATA_NSECTORS_REG, 1);
  setataregbyte(ATA_COMMAND_REG, ATA_CMD_READ);

  timeout = ATA_TIMEOUT_VAL;

  while( (ATA_BSY & getataregbyte(ATA_STATUS_REG))
    || (!(ATA_DRQ & getataregbyte(ATA_STATUS_REG))) )
    {
      timeout--;
      if(!timeout)
	return(ATA_BSY_TIMEOUT);
    }

  return(ATA_OK);
}

uint16_t ATA_ReadWord()
{
  uint16_t data;

  while(getataregbyte(ATA_STATUS_REG) & ATA_BSY)
    ;

  data = getataregword(ATA_DATA_REG);

  /* swap data because it is returned as little endian */
  //return( (data<<8) | (data>>8) );
  return(data);
}

void ATA_SkipWords(uint16_t numwords)
{
  while(numwords)
    {
      getataregword(ATA_DATA_REG);
      numwords--;
    }
}

uint32_t ATA_CurrentLBAAddr()
{
  return CurrentLBAAddr;
}

int8_t ATA_ReadSectors(uint32_t lba, uint16_t *buffer, uint8_t count)
{
	register uint32_t	nsectors;
	register uint32_t	timeout;
	register uint32_t	transferwords;

//	drive now selected as part of API
//	driveselect(atadevice);

	writelba(lba);

	setataregbyte(ATA_NSECTORS_REG, count);
	setataregbyte(ATA_COMMAND_REG, ATA_CMD_READ);

	nsectors = count;
	if ( nsectors == 0 ) nsectors=256;

	timeout = ATA_TIMEOUT_VAL;

	while( nsectors )
	{
		/* wait for BSY bit clear or DRQ for extra compatibility*/
		if ( (!(ATA_BSY & getataregbyte(ATA_STATUS_REG)))
			 || (ATA_DRQ & getataregbyte(ATA_STATUS_REG)) )
		{
			timeout = ATA_TIMEOUT_VAL;

			transferwords = SECTORTRANSFER;

			while(transferwords)
			{
				while(( (getataregbyte(ATA_STATUS_REG)) & ATA_BSY)) {};
				*buffer++ = getataregword(ATA_DATA_REG);
				transferwords--;
			}

			nsectors--;
		}
		else
		{
			timeout--;
			if(!timeout) return(ATA_BSY_TIMEOUT);
		}
	}

	return(ATA_OK);
}


/* Internal functions */

void resetata()
{
	volatile uint32_t i;

	TRIS_8255_DATA = PORT_OUTPUT;

	Pin8255nRD = N_DISABLE;
	Pin8255nWR = N_DISABLE;
	Pin8255nCS = N_ENABLE;

	/***** Configure 8255 for writing: */
	/* Address 8255 control word */
	PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
	PORT_8255_ADDRESS |= ADDR_8255_CTRLWD;

	Pin8255nWR = N_ENABLE;
	/* Configure 8255 to write to ATA regs */
	PORT_8255_DATA = SET_8255_WRITE_ATA_REG;
	Pin8255nWR = N_DISABLE;

	/***** Reset ATA device */
	/* Address 8255 ATA control signals */
	PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
	PORT_8255_ADDRESS |= ADDR_8255_ATA_CTRL;

	Pin8255nWR = N_ENABLE;
	PORT_8255_DATA = ATA_RESET_ENABLE;
	Pin8255nWR = N_DISABLE;

	i = ATA_TIMEOUT_VAL;
	while(i)
		i--;

	Pin8255nWR = N_ENABLE;
	PORT_8255_DATA = 0;
	Pin8255nWR = N_DISABLE;

	Pin8255nCS = N_DISABLE;
}

void writelba(uint32_t lba)
{
	register uint32_t byte1, byte2, byte3, byte4;

	// little endian?
	byte4 = (0xff & (lba>>24));
	byte3 = (0xff & (lba>>16));
	byte2 = (0xff & (lba>>8));
	byte1 = (0xff & (lba));

	/* LBA addressing mode, don't alter DRV bit */
	//if(atadevice->vars->config->driveselect == ATA_MASTER)
	if(CurrentDrv == ATA_MASTER)
	{
		/* Clear DRV bit: drive 0 */
		byte4 &= ~ATA_SLAVE;
		/* Set LBA mode */
		byte4 |= ATA_LBA;
	}
	else
	{
		/* Set DRV bit: drive 1, set LBA mode */
		byte4 |= (ATA_SLAVE | ATA_LBA);
	}

	setataregbyte(ATA_SELECT_REG, byte4);
	setataregbyte(ATA_HCYL_REG, byte3);
	setataregbyte(ATA_LCYL_REG, byte2);
	setataregbyte(ATA_SECTOR_REG, byte1);

	/* update active LBA address */
	CurrentLBAAddr = lba;
}

void setataregbyte(uint8_t address, uint8_t data)
{
	/* On entry to this function the PIC and 8255 should be set up to
	   write to the ATA device (all ports output) */

	volatile uint32_t i;

	Pin8255nCS = N_ENABLE;

	/***** Write ATA reg address: */
	/* Address 8255->ATA control signals */
	PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
	PORT_8255_ADDRESS |= ADDR_8255_ATA_CTRL;

	Pin8255nWR = N_ENABLE;
	/* set ATA register address (A0, A1, A2, CS0, CS1) */
	PORT_8255_DATA = (address);
	Pin8255nWR = N_DISABLE;
	/* possibly wait here? */

	/***** Write ATA reg data: */
	/* Address 8255->ATA data low */
	PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
	PORT_8255_ADDRESS |= ADDR_8255_ATA_DATALO;

	Pin8255nWR = N_ENABLE;
	/* write ATA register data */
	PORT_8255_DATA = data;
	Pin8255nWR = N_DISABLE;
	/* possibly wait here? */

	/***** Finish write to ATA reg: */
	/* Address 8255->ATA control signals */
	PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
	PORT_8255_ADDRESS |= ADDR_8255_ATA_CTRL;

	Pin8255nWR = N_ENABLE;
	/* Assert ATA write signal */
	PORT_8255_DATA = (address | ATA_WRITE_ENABLE);
	Pin8255nWR = N_DISABLE;

	i = ATA_TIMEOUT_VAL;
	while(i)
		i--;

	Pin8255nWR = N_ENABLE;
	/* Clear ATA write line but keep address */
	PORT_8255_DATA = address;
	Pin8255nWR = N_DISABLE;

	PORT_8255_DATA = 0;

	Pin8255nCS = N_DISABLE;
}

void setataregword(uint8_t address, uint16_t data)
{
	/* On entry to this function the PIC and 8255 should be set up to
	   write to the ATA device (all ports output) */

	volatile uint32_t i;

	Pin8255nCS = N_ENABLE;

	/***** Write ATA reg address: */
	/* Address 8255->ATA control signals */
	PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
	PORT_8255_ADDRESS |= ADDR_8255_ATA_CTRL;

	Pin8255nWR = N_ENABLE;
	/* set ATA register address (A0, A1, A2, CS0, CS1) */
	PORT_8255_DATA = (address);
	Pin8255nWR = N_DISABLE;
	/* possibly wait here? */

	/***** Write ATA reg data: */
	/* Address 8255->ATA data low */
	PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
	PORT_8255_ADDRESS |= ADDR_8255_ATA_DATALO;

	Pin8255nWR = N_ENABLE;
	/* write ATA register data */
	// edit this for little endian
	PORT_8255_DATA = (uint8_t)(data &0x00FF);
	Pin8255nWR = N_DISABLE;
	/* possibly wait here? */

	/* Address 8255->ATA data high */
	PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
	PORT_8255_ADDRESS |= ADDR_8255_ATA_DATAHI;

	Pin8255nWR = N_ENABLE;
	/* write ATA register data */
	// edit this for little endian
	PORT_8255_DATA = (uint8_t)(data >> 8);
	Pin8255nWR = N_DISABLE;
	/* possibly wait here? */

	/***** Finish write to ATA reg: */
	/* Address 8255->ATA control signals */
	PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
	PORT_8255_ADDRESS |= ADDR_8255_ATA_CTRL;

	Pin8255nWR = N_ENABLE;
	/* Assert ATA write signal */
	PORT_8255_DATA = (address | ATA_WRITE_ENABLE);
	Pin8255nWR = N_DISABLE;

	i = ATA_TIMEOUT_VAL;
	while(i)
		i--;

	Pin8255nWR = N_ENABLE;
	/* Clear ATA write line but keep address */
	PORT_8255_DATA = address;
	Pin8255nWR = N_DISABLE;

	PORT_8255_DATA = 0;

	Pin8255nCS = N_DISABLE;
}

uint8_t getataregbyte(uint8_t address)
{
	/* On entry to this function the PIC and 8255 should be set up to
	   write to the ATA device (all ports output) */

	uint8_t dataread;

	Pin8255nCS = N_ENABLE;

	/***** Configure 8255 for reading: */
	/* Address 8255 control word */
	PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
	PORT_8255_ADDRESS |= ADDR_8255_CTRLWD;

	Pin8255nWR = N_ENABLE;
	/* Configure 8255 to write to ATA regs */
	PORT_8255_DATA = SET_8255_READ_ATA_REG;
	Pin8255nWR = N_DISABLE;


	/***** Write ATA reg address: */
	/* Address 8255 ATA->control signals */
	PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
	PORT_8255_ADDRESS |= ADDR_8255_ATA_CTRL;

	Pin8255nWR = N_ENABLE;
	/* set ATA register address (A0, A1, A2, CS0, CS1) */
	PORT_8255_DATA = address;
	Pin8255nWR = N_DISABLE;
	/* possibly wait here? */

	Pin8255nWR = N_ENABLE;
	/* assert ATA read line*/
	PORT_8255_DATA = (address | ATA_READ_ENABLE);
	Pin8255nWR = N_DISABLE;


	/***** Read ATA reg data: */
	/* Address 8255->ATA data low */
	PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
	PORT_8255_ADDRESS |= ADDR_8255_ATA_DATALO;

	/* Switch 8255 data port to an input */
	TRIS_8255_DATA = PORT_INPUT;

	Pin8255nRD = N_ENABLE;
	/* read ATA register data */
	dataread = PORT_8255_DATA;
	Pin8255nRD = N_DISABLE;

	/* Switch 8255 data port back to output */
	TRIS_8255_DATA = PORT_OUTPUT;


	/***** Clear ATA read line: */
	/* Address 8255 ATA->control signals */
	PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
	PORT_8255_ADDRESS |= ADDR_8255_ATA_CTRL;

	Pin8255nWR = N_ENABLE;
	/* Clear ATA read line but keep address */
	PORT_8255_DATA = address;
	Pin8255nWR = N_DISABLE;
	/* possibly wait here? */


	/***** Configure 8255 for writing: */
	/* Address 8255 control word */
	PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
	PORT_8255_ADDRESS |= ADDR_8255_CTRLWD;

	Pin8255nWR = N_ENABLE;
	/* Configure 8255 to write to ATA regs */
	PORT_8255_DATA = SET_8255_WRITE_ATA_REG;
	Pin8255nWR = N_DISABLE;

	//	PORT_8255_DATA = 0;

	/* assert PIC PORT E bit 2 to disable 8255 CS */
	Pin8255nCS = N_DISABLE;

	return dataread;
}

uint16_t getataregword(uint8_t address)
{
	/* On entry to this function the PIC and 8255 should be set up to
	   write to the ATA device (all ports output) */

	uint16_t datareadlow, datareadhigh;

	Pin8255nCS = N_ENABLE;

	/***** Configure 8255 for reading: */
	/* Address 8255 control word */
	PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
	PORT_8255_ADDRESS |= ADDR_8255_CTRLWD;

	Pin8255nWR = N_ENABLE;
	/* Configure 8255 to write to ATA regs */
	PORT_8255_DATA = SET_8255_READ_ATA_REG;
	Pin8255nWR = N_DISABLE;


	/***** Write ATA reg address: */
	/* Address 8255 ATA->control signals */
	PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
	PORT_8255_ADDRESS |= ADDR_8255_ATA_CTRL;

	Pin8255nWR = N_ENABLE;
	/* set ATA register address (A0, A1, A2, CS0, CS1) */
	PORT_8255_DATA = address;
	Pin8255nWR = N_DISABLE;
	/* possibly wait here? */

	Pin8255nWR = N_ENABLE;
	/* assert ATA read line*/
	PORT_8255_DATA = (address | ATA_READ_ENABLE);
	Pin8255nWR = N_DISABLE;


	/***** Read ATA reg data low byte: */
	/* Address 8255->ATA data low */
	PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
	PORT_8255_ADDRESS |= ADDR_8255_ATA_DATALO;

	/* Switch 8255 data port to an input */
	TRIS_8255_DATA = PORT_INPUT;

	Pin8255nRD = N_ENABLE;
	/* read ATA register data */
	datareadlow = PORT_8255_DATA;
	Pin8255nRD = N_DISABLE;

	/***** Read ATA reg data high byte: */
	/* Address 8255->ATA data high */
	PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
	PORT_8255_ADDRESS |= ADDR_8255_ATA_DATAHI;

	Pin8255nRD = N_ENABLE;
	/* read ATA register data */
	datareadhigh = PORT_8255_DATA;
	Pin8255nRD = N_DISABLE;

	/* Switch 8255 data port back to output */
	TRIS_8255_DATA = PORT_OUTPUT;


	/***** Clear ATA read line: */
	/* Address 8255 ATA->control signals */
	PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
	PORT_8255_ADDRESS |= ADDR_8255_ATA_CTRL;

	Pin8255nWR = N_ENABLE;
	/* Clear ATA read line but keep address */
	PORT_8255_DATA = address;
	Pin8255nWR = N_DISABLE;
	/* possibly wait here? */


	/***** Configure 8255 for writing: */
	/* Address 8255 control word */
	PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
	PORT_8255_ADDRESS |= ADDR_8255_CTRLWD;

	Pin8255nWR = N_ENABLE;
	/* Configure 8255 to write to ATA regs */
	PORT_8255_DATA = SET_8255_WRITE_ATA_REG;
	Pin8255nWR = N_DISABLE;

	//	PORT_8255_DATA = 0;

	Pin8255nCS = N_DISABLE;

	// edit this for little endian
	//return ((datareadhigh << 8) | (datareadlow & 0x00FF));
	return ((datareadlow << 8) | (datareadhigh & 0x00FF));
}














#if 0 /* old versions */

void setataregbyte(uint8_t address, uint8_t data)
{
	/* set PIC PORT D direction to output */
	TRIS_8255_DATA = PORT_OUTPUT;

	/***** Configure 8255 PORTS A, B & C to be outputs, mode 0: */
	/* set PIC PORT A bits 1 & 2 to address 8255 control word */
	PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
	PORT_8255_ADDRESS |= ADDR_8255_CTRLWD;

	/* set nRD, set nWR prior to enabling 8255 */
	Pin8255nRD = N_DISABLE;
	Pin8255nWR = N_DISABLE;

	/* negate PIC PORT E bit 2 to enable 8255 CS */
	Pin8255nCS = N_ENABLE;

	/* negate nWR ready for write to 8255 */
	Pin8255nWR = N_ENABLE;

	/* set PIC PORT D data to control word for 8255 */
	PORT_8255_DATA = SET_8255_WRITE_ATA_REG;

	/* set nWR to perform the write to 8255 control word */
	Pin8255nWR = N_DISABLE;

	/* possibly wait here? */


	/***** Write ATA reg address: */
	/* set PIC PORT A bits 1 & 2 to address 8255 PORT C */
	PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
	PORT_8255_ADDRESS |= ADDR_8255_PORTC;

	/* negate nWR ready for write to 8255 */
	Pin8255nWR = N_ENABLE;

	/* set PIC PORT D data to ATA register address (A0, A1, A2, CS0, CS1) */
	PORT_8255_DATA = address;

	/* set nWR to perform the write to 8255 PORT C */
	Pin8255nWR = N_DISABLE;

	/* possibly wait here? */


	/***** Write ATA reg data: */
	/* set PIC PORT A bits 1 & 2 to address 8255 PORT A */
	PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
	PORT_8255_ADDRESS |= ADDR_8255_PORTA;

	/* negate nWR ready for write to 8255 */
	Pin8255nWR = N_ENABLE;

	/* set PIC PORT D data to ATA register data */
	PORT_8255_DATA = data;

	/* set nWR to perform the write to 8255 control word */
	Pin8255nWR = N_DISABLE;

	/* possibly wait here? */

	/***** Perform Write to ATA reg: */
	/* set PIC PORT A bits 1 & 2 to address 8255 PORT C */
	PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
	PORT_8255_ADDRESS |= ADDR_8255_PORTC;

	/* set bit on PIC PORT D to activate the ATA WR signal (which is inverted) */
	PinATAWR = ENABLE;

	/* Toggle nWR to perform the write to 8255 PORT C */
	Pin8255nWR = N_ENABLE;
	/* possibly wait here? */
	Pin8255nWR = N_DISABLE;

	/* negate bit on PIC PORT D to disactivate the ATA WR signal (which is inverted) */
	PinATAWR = DISABLE;

	/* Toggle nWR to perform the write to 8255 PORT C */
	Pin8255nWR = N_ENABLE;
	/* possibly wait here? */
	Pin8255nWR = N_DISABLE;

	/* assert PIC PORT E bit 2 to disable 8255 CS */
	Pin8255nCS = N_DISABLE;
}

uint8_t getataregbyte(uint8_t address)
{
	uint8_t dataread;

	/* set PIC PORT D direction to output initially */
	TRIS_8255_DATA = PORT_OUTPUT;

	/***** Configure 8255 PORTS A, B as inputs & C to be outputs, mode 0: */
	/* set PIC PORT A bits 1 & 2 to address 8255 control word */
	PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
	PORT_8255_ADDRESS |= ADDR_8255_CTRLWD;

	/* set nRD, set nWR prior to enabling 8255 */
	Pin8255nRD = N_DISABLE;
	Pin8255nWR = N_DISABLE;

	/* negate PIC PORT E bit 2 to enable 8255 CS */
	Pin8255nCS = N_ENABLE;

	/* negate nWR ready for write to 8255 */
	Pin8255nWR = N_ENABLE;

	/* set PIC PORT D data to control word for 8255 */
	PORT_8255_DATA = SET_8255_READ_ATA_REG;

	/* set nWR to perform the write to 8255 control word */
	Pin8255nWR = N_DISABLE;


	/***** Write ATA reg address: */
	/* set PIC PORT A bits 1 & 2 to address 8255 PORT C */
	PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
	PORT_8255_ADDRESS |= ADDR_8255_PORTC;

	/* negate nWR ready for write to 8255 */
	Pin8255nWR = N_ENABLE;

	/* set PIC PORT D data to ATA register address (A0, A1, A2, CS0, CS1) */
	PORT_8255_DATA = address;

	/* set nWR to perform the write to 8255 PORT C */
	Pin8255nWR = N_DISABLE;

	/* possibly wait here? */


	/***** Read ATA reg data: */
	/* set PIC PORT A bits 1 & 2 to address 8255 PORT C */
	PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
	PORT_8255_ADDRESS |= ADDR_8255_PORTC;

	/* set bit on PIC PORT D to activate the ATA RD signal (which is inverted) */
	PinATARD = ENABLE;

	/* Toggle nWR to perform the write to 8255 PORT C */
	Pin8255nWR = N_ENABLE;
	/* possibly wait here? */
	Pin8255nWR = N_DISABLE;


	/* set PIC PORT A bits 1 & 2 to address 8255 PORT A */
	PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
	PORT_8255_ADDRESS |= ADDR_8255_PORTA;

	/* negate nRD ready for Read from 8255 */
	Pin8255nRD = N_ENABLE;

	/* Switch PIC PORT D to an input in order to read 8255 PORT A */
	TRIS_8255_DATA = PORT_INPUT;

	/* set nRD to perform the read from 8255 PORT A */
	Pin8255nWR = N_DISABLE;

	/* read PIC PORT D data */
	dataread = PORT_8255_DATA;

	/* set PIC PORT D direction back to output */
	TRIS_8255_DATA = PORT_OUTPUT;

	/* set PIC PORT A bits 1 & 2 to address 8255 PORT C */
	PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
	PORT_8255_ADDRESS |= ADDR_8255_PORTC;

	/* set bit on PIC PORT D to de-activate the ATA RD signal (which is inverted) */
	PinATARD = DISABLE;

	/* Toggle nWR to perform the write to 8255 PORT C */
	Pin8255nWR = N_ENABLE;
	/* possibly wait here? */
	Pin8255nWR = N_DISABLE;

	/* assert PIC PORT E bit 2 to disable 8255 CS */
	Pin8255nCS = N_DISABLE;

	return dataread;
}

uint16_t getataregword(uint8_t address)
{
	uint8_t datareadlow, datareadhigh;


	/* set PIC PORT D direction to output initially */
	TRIS_8255_DATA = PORT_OUTPUT;

	/***** Configure 8255 PORTS A, B as inputs & C to be outputs, mode 0: */
	/* set PIC PORT A bits 1 & 2 to address 8255 control word */
	PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
	PORT_8255_ADDRESS |= ADDR_8255_CTRLWD;

	/* set nRD, set nWR prior to enabling 8255 */
	Pin8255nRD = N_DISABLE;
	Pin8255nWR = N_DISABLE;

	/* negate PIC PORT E bit 2 to enable 8255 CS */
	Pin8255nCS = N_ENABLE;

	/* negate nWR ready for write to 8255 */
	Pin8255nWR = N_ENABLE;

	/* set PIC PORT D data to control word for 8255 */
	PORT_8255_DATA = SET_8255_READ_ATA_REG;

	/* set nWR to perform the write to 8255 control word */
	Pin8255nWR = N_DISABLE;


	/***** Write ATA reg address: */
	/* set PIC PORT A bits 1 & 2 to address 8255 PORT C */
	PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
	PORT_8255_ADDRESS |= ADDR_8255_PORTC;

	/* negate nWR ready for write to 8255 */
	Pin8255nWR = N_ENABLE;

	/* set PIC PORT D data to ATA register address (A0, A1, A2, CS0, CS1) */
	PORT_8255_DATA = address;

	/* set nWR to perform the write to 8255 PORT C */
	Pin8255nWR = N_DISABLE;

	/* possibly wait here? */


	/***** Read ATA reg data: */
	/* set PIC PORT A bits 1 & 2 to address 8255 PORT C */
	PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
	PORT_8255_ADDRESS |= ADDR_8255_PORTC;

	/* set bit on PIC PORT D to activate the ATA RD signal (which is inverted) */
	PinATARD = ENABLE;

	/* Toggle nWR to perform the write to 8255 PORT C */
	Pin8255nWR = N_ENABLE;
	/* possibly wait here? */
	Pin8255nWR = N_DISABLE;


	/** read low byte */
	/* set PIC PORT A bits 1 & 2 to address 8255 PORT A */
	PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
	PORT_8255_ADDRESS |= ADDR_8255_PORTA;

	// /* negate nRD ready for Read from 8255 */
	//Pin8255nRD = N_ENABLE;

	/* Switch PIC PORT D to an input in order to read 8255 PORT A */
	TRIS_8255_DATA = PORT_INPUT;

	// /* set nRD to perform the read from 8255 PORT A */
	//Pin8255nWR = N_ENABLE;

	/* Toggle nRD to perform the Read from 8255 */
	Pin8255nRD = N_ENABLE;
	Pin8255nRD = N_DISABLE;

	/* read PIC PORT D data */
	datareadlow = PORT_8255_DATA;

	/** read high byte */
	/* set PIC PORT A bits 1 & 2 to address 8255 PORT B */
	PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
	PORT_8255_ADDRESS |= ADDR_8255_PORTB;

	/* Toggle nRD to perform the Read from 8255 */
	Pin8255nRD = N_ENABLE;
	Pin8255nRD = N_DISABLE;

	/* read PIC PORT D data */
	datareadhigh = PORT_8255_DATA;

	/* set PIC PORT D direction back to output */
	TRIS_8255_DATA = PORT_OUTPUT;

	/* set PIC PORT A bits 1 & 2 to address 8255 PORT C */
	PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
	PORT_8255_ADDRESS |= ADDR_8255_PORTC;

	/* set bit on PIC PORT D to de-activate the ATA RD signal (which is inverted) */
	PinATARD = DISABLE;

	/* Toggle nWR to perform the write to 8255 PORT C */
	Pin8255nWR = N_ENABLE;
	/* possibly wait here? */
	Pin8255nWR = N_DISABLE;

	/* assert PIC PORT E bit 2 to disable 8255 CS */
	Pin8255nCS = N_DISABLE;

	return ((datareadhigh << 8) | datareadlow);
}
#endif
