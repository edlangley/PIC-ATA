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

/* Internal function prototypes */
void resetata();
void writelba(uint32_t);
void setataregbyte(uint8_t, uint8_t);
uint8_t getataregbyte(uint8_t);
uint16_t getataregword(uint8_t);


/* API functions */

int8_t ATA_Init(uint8_t drivenumber)
{
    uint32_t i;

    /* disable A/Ds to allow pins to be used as digital I/O */
    ADCON1 = ALL_DIGITAL;

    /* set relevant PIC port bits to be outputs (clear TRIS bits) */
    /* address bits to 8255 as outputs */
    TRIS_8255_ADDRESS &= ADDR_8255_CLEAR;

    /* RD/WR bits to 8255 as outputs */
    Tris8255nRD = BIT_OUTPUT;
    Tris8255nWR = BIT_OUTPUT;

    /* CS for 8255 as output*/
    Tris8255nCS = BIT_OUTPUT;

    /* now talk to the ATA device */
    resetata();

    /* clear select card/head register prior to drive selection to
       ensure known state */
    setataregbyte(ATA_SELECT_REG, 0);
    ATA_DriveSelect(drivenumber);

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

    return(ATA_OK);
}

void ATA_DriveSelect(uint8_t drivenumber)
{
    uint8_t ataselectsetting;

    ataselectsetting = getataregbyte(ATA_SELECT_REG);
    if(drivenumber == ATA_MASTER)
    {
        ataselectsetting &= ~ATA_SLAVE; /* Clear DRV bit: drive 0 */
    }
    else
    {
        ataselectsetting |= ATA_SLAVE;  /* Set DRV bit: drive 1 */
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

    return(ATA_OK);
}

#if defined(ATA_USE_ID)
uint8_t *ATA_GetInfoModel()
{
    hd0info.model[ATA_DRVINFOWORD_MODELLENGTH-1] &= '\0';
    return((uint8_t*)(hd0info.model));
}

uint8_t *ATA_GetInfoRev()
{
    hd0info.fw_rev[ATA_DRVINFOWORD_REVLENGTH-1] &= '\0';
    return((uint8_t*)(hd0info.fw_rev));
}

uint8_t *ATA_GetInfoSerialNum()
{
    hd0info.serial_no[ATA_DRVINFOWORD_SERNUMLENGTH-1] = '\0';
    return((uint8_t*)(hd0info.serial_no));
}

uint16_t ATA_GetInfoNumCyls()
{
    return(hd0info.cyls);
}

uint16_t ATA_GetInfoNumHeads()
{
    return(hd0info.heads);
}

uint16_t ATA_GetInfoNumSectors()
{
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

    while( (ATA_BSY & getataregbyte(ATA_STATUS_REG)) ||
           (!(ATA_DRQ & getataregbyte(ATA_STATUS_REG))) )
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

    writelba(lba);

    setataregbyte(ATA_NSECTORS_REG, count);
    setataregbyte(ATA_COMMAND_REG, ATA_CMD_READ);

    nsectors = count;
    if ( nsectors == 0 )
        nsectors=256;

    timeout = ATA_TIMEOUT_VAL;

    while( nsectors )
    {
        /* wait for BSY bit clear or DRQ for extra compatibility*/
        if ( (!(ATA_BSY & getataregbyte(ATA_STATUS_REG))) ||
             (ATA_DRQ & getataregbyte(ATA_STATUS_REG)) )
        {
            timeout = ATA_TIMEOUT_VAL;

            transferwords = SECTORTRANSFER;

            while(transferwords)
            {
                while(( (getataregbyte(ATA_STATUS_REG)) & ATA_BSY))
                    ;
                *buffer++ = getataregword(ATA_DATA_REG);
                transferwords--;
            }

            nsectors--;
        }
        else
        {
            timeout--;
            if(!timeout)
                return(ATA_BSY_TIMEOUT);
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

    /* configure 8255 for writing: */
    /* address 8255 control word */
    PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
    PORT_8255_ADDRESS |= ADDR_8255_CTRLWD;

    Pin8255nWR = N_ENABLE;
    /* configure 8255 to write to ATA regs */
    PORT_8255_DATA = SET_8255_WRITE_ATA_REG;
    Pin8255nWR = N_DISABLE;

    /* reset ATA device */
    /* address 8255 ATA control signals */
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

    /* don't alter DRV bit */
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

    /* write ATA reg address: */
    /* address 8255->ATA control signals */
    PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
    PORT_8255_ADDRESS |= ADDR_8255_ATA_CTRL;

    Pin8255nWR = N_ENABLE;
    /* set ATA register address (A0, A1, A2, CS0, CS1) */
    PORT_8255_DATA = (address);
    Pin8255nWR = N_DISABLE;
    /* possibly wait here? */

    /* write ATA reg data: */
    /* address 8255->ATA data low */
    PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
    PORT_8255_ADDRESS |= ADDR_8255_ATA_DATALO;

    Pin8255nWR = N_ENABLE;
    /* write ATA register data */
    PORT_8255_DATA = data;
    Pin8255nWR = N_DISABLE;
    /* possibly wait here? */

    /* finish write to ATA reg: */
    /* address 8255->ATA control signals */
    PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
    PORT_8255_ADDRESS |= ADDR_8255_ATA_CTRL;

    Pin8255nWR = N_ENABLE;
    /* assert ATA write signal */
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

    /* write ATA reg address: */
    /* address 8255->ATA control signals */
    PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
    PORT_8255_ADDRESS |= ADDR_8255_ATA_CTRL;

    Pin8255nWR = N_ENABLE;
    /* set ATA register address (A0, A1, A2, CS0, CS1) */
    PORT_8255_DATA = (address);
    Pin8255nWR = N_DISABLE;
    /* possibly wait here? */

    /* write ATA reg data: */
    /* address 8255->ATA data low */
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

    /* finish write to ATA reg: */
    /* address 8255->ATA control signals */
    PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
    PORT_8255_ADDRESS |= ADDR_8255_ATA_CTRL;

    Pin8255nWR = N_ENABLE;
    /* assert ATA write signal */
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

    /* configure 8255 for reading: */
    /* address 8255 control word */
    PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
    PORT_8255_ADDRESS |= ADDR_8255_CTRLWD;

    Pin8255nWR = N_ENABLE;
    /* configure 8255 to write to ATA regs */
    PORT_8255_DATA = SET_8255_READ_ATA_REG;
    Pin8255nWR = N_DISABLE;


    /* write ATA reg address: */
    /* address 8255 ATA->control signals */
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


    /* read ATA reg data: */
    /* Address 8255->ATA data low */
    PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
    PORT_8255_ADDRESS |= ADDR_8255_ATA_DATALO;

    /* switch 8255 data port to an input */
    TRIS_8255_DATA = PORT_INPUT;

    Pin8255nRD = N_ENABLE;
    /* read ATA register data */
    dataread = PORT_8255_DATA;
    Pin8255nRD = N_DISABLE;

    /* switch 8255 data port back to output */
    TRIS_8255_DATA = PORT_OUTPUT;


    /* clear ATA read line: */
    /* address 8255 ATA->control signals */
    PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
    PORT_8255_ADDRESS |= ADDR_8255_ATA_CTRL;

    Pin8255nWR = N_ENABLE;
    /* Clear ATA read line but keep address */
    PORT_8255_DATA = address;
    Pin8255nWR = N_DISABLE;
    /* possibly wait here? */


    /* configure 8255 for writing: */
    /* address 8255 control word */
    PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
    PORT_8255_ADDRESS |= ADDR_8255_CTRLWD;

    Pin8255nWR = N_ENABLE;
    /* configure 8255 to write to ATA regs */
    PORT_8255_DATA = SET_8255_WRITE_ATA_REG;
    Pin8255nWR = N_DISABLE;

    Pin8255nCS = N_DISABLE;

    return dataread;
}

uint16_t getataregword(uint8_t address)
{
    /* On entry to this function the PIC and 8255 should be set up to
       write to the ATA device (all ports output) */

    uint16_t datareadlow, datareadhigh;

    Pin8255nCS = N_ENABLE;

    /* configure 8255 for reading: */
    /* address 8255 control word */
    PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
    PORT_8255_ADDRESS |= ADDR_8255_CTRLWD;

    Pin8255nWR = N_ENABLE;
    /* configure 8255 to write to ATA regs */
    PORT_8255_DATA = SET_8255_READ_ATA_REG;
    Pin8255nWR = N_DISABLE;


    /* write ATA reg address: */
    /* address 8255 ATA->control signals */
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


    /* read ATA reg data low byte: */
    /* address 8255->ATA data low */
    PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
    PORT_8255_ADDRESS |= ADDR_8255_ATA_DATALO;

    /* switch 8255 data port to an input */
    TRIS_8255_DATA = PORT_INPUT;

    Pin8255nRD = N_ENABLE;
    /* read ATA register data */
    datareadlow = PORT_8255_DATA;
    Pin8255nRD = N_DISABLE;

    /* read ATA reg data high byte: */
    /* address 8255->ATA data high */
    PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
    PORT_8255_ADDRESS |= ADDR_8255_ATA_DATAHI;

    Pin8255nRD = N_ENABLE;
    /* read ATA register data */
    datareadhigh = PORT_8255_DATA;
    Pin8255nRD = N_DISABLE;

    /* Switch 8255 data port back to output */
    TRIS_8255_DATA = PORT_OUTPUT;


    /* clear ATA read line: */
    /* address 8255 ATA->control signals */
    PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
    PORT_8255_ADDRESS |= ADDR_8255_ATA_CTRL;

    Pin8255nWR = N_ENABLE;
    /* clear ATA read line but keep address */
    PORT_8255_DATA = address;
    Pin8255nWR = N_DISABLE;
    /* possibly wait here? */


    /* configure 8255 for writing: */
    /* address 8255 control word */
    PORT_8255_ADDRESS &= ADDR_8255_CLEAR;
    PORT_8255_ADDRESS |= ADDR_8255_CTRLWD;

    Pin8255nWR = N_ENABLE;
    /* configure 8255 to write to ATA regs */
    PORT_8255_DATA = SET_8255_WRITE_ATA_REG;
    Pin8255nWR = N_DISABLE;

    Pin8255nCS = N_DISABLE;

    return ((datareadlow << 8) | (datareadhigh & 0x00FF));
}
