/*-------------------------------------------------------------------
Name......... ata.c
Author....... Ed Langley
Date......... 19/10/2006
Description.. Driver to operate an ATA interface on a PIC MCU
-------------------------------------------------------------------*/

#include "platform.h"
#include "types.h"
#include "ata.h"


/* My circuit:
PIC PORT A -> 8255 address signals:
  bit 0: A0
  bit 1: A1
PIC PORT E -> 8255 control signals:
  bit 0: /RD
  bit 1: /WR
  bit 2: CS

PIC PORT D -> 8255 PORT A -> ATA data 0:7
PIC PORT D -> 8255 PORT B -> ATA data 8:15
PIC PORT D -> 8255 PORT C -> ATA control signals:
  bit 0: A0
  bit 1: A1
  bit 2: A2
  bit 3: CS0
  bit 4: CS1
  bit 5: RD
  bit 6: WR
  bit 7: RES

The ATA control signals are active high due to invertors, so no '/'s.
Set bits to activate signals.
*/

/* Change the following settings depending on how:
     the PIC is connected to the 8255
*/
#define PIC_PORT_8255_DATA               PORTD
#define PIC_TRIS_8255_DATA               TRISD
#define PIC_PORT_8255_ADDRESS            PORTA
#define PIC_TRIS_8255_ADDRESS            TRISA
#define PIC_PORT_8255_CS                 PORTE
#define PIC_TRIS_8255_CS                 TRISE
#define PIC_PORT_8255_RDWR               PORTE
#define PIC_TRIS_8255_RDWR               TRISE

#define PIC_BITOFFSET_8255_ADDRESS       0

#define PIC_BITNUM_8255_RD               0
#define PIC_BITNUM_8255_WR               1
#define PIC_BITNUM_8255_CS               2
/* end of changeable settings */


static BIT Pin8255nCS                    @ PORTBIT(PIC_PORT_8255_CS, PIC_BITNUM_8255_CS);
static BIT Pin8255nRD                    @ PORTBIT(PIC_PORT_8255_RDWR, PIC_BITNUM_8255_RD);
static BIT Pin8255nWR                    @ PORTBIT(PIC_PORT_8255_RDWR, PIC_BITNUM_8255_WR);
static BIT Tris8255nCS                   @ PORTBIT(PIC_TRIS_8255_CS, PIC_BITNUM_8255_CS);
static BIT Tris8255nRD                   @ PORTBIT(PIC_TRIS_8255_RDWR, PIC_BITNUM_8255_RD);
static BIT Tris8255nWR                   @ PORTBIT(PIC_TRIS_8255_RDWR, PIC_BITNUM_8255_WR);

/* PIC ADCON settings */
#define ALL_DIGITAL                      0x06

/* 8255 addresses and settings: */
#define _8255_ADDR_CLEAR                ~(0x03 << PIC_BITOFFSET_8255_ADDRESS)
#define _8255_ADDR_PORTA                 (0x00 << PIC_BITOFFSET_8255_ADDRESS)
#define _8255_ADDR_PORTB                 (0x01 << PIC_BITOFFSET_8255_ADDRESS)
#define _8255_ADDR_PORTC                 (0x02 << PIC_BITOFFSET_8255_ADDRESS)
#define _8255_ADDR_CTRLWD                (0x03 << PIC_BITOFFSET_8255_ADDRESS)

#define _8255_CTRLWD_PORTC_LO_INPUT      (1 << 0)
#define _8255_CTRLWD_PORTB_INPUT         (1 << 1)
#define _8255_CTRLWD_GROUPB_MODE(x)      ((x & 0x01) << 2)
#define _8255_CTRLWD_PORTC_HI_INPUT      (1 << 3)
#define _8255_CTRLWD_PORTA_INPUT         (1 << 4)
#define _8255_CTRLWD_GROUPA_MODE(x)      ((x & 0x03) << 5)
#define _8255_CTRLWD_MODESET             (1 << 7)

#define _8255_GROUPB_MODE0               0x00
#define _8255_GROUPB_MODE1               0x01
#define _8255_GROUPB_MODE2               0x02


/* Change the following settings depending on how:
     the 8255 is connected to the ATA signals
*/
#define _8255_ADDR_ATA_DATALO            _8255_ADDR_PORTA
#define _8255_ADDR_ATA_DATAHI            _8255_ADDR_PORTB
#define _8255_ADDR_ATA_CTRL              _8255_ADDR_PORTC

#define _8255_CTRLWD_ATA_DATALO_RD       _8255_CTRLWD_PORTA_INPUT
#define _8255_CTRLWD_ATA_DATAHI_RD       _8255_CTRLWD_PORTB_INPUT
#define _8255_CTRLWD_ATA_CTRL_RD         (_8255_CTRLWD_PORTC_LO_INPUT | _8255_CTRLWD_PORTC_HI_INPUT)

#define ATA_CTRL_BITOFFSET_ADDRESS       0

#define ATA_CTRL_BIT_CS0                 0x08
#define ATA_CTRL_BIT_CS1                 0x10
#define ATA_CTRL_BIT_RD                  0x20
#define ATA_CTRL_BIT_WR                  0x40
#define ATA_CTRL_BIT_RESET               0x80
/* end of changeable settings */

#define ATA_CTRL_BIT_ADDR(a)             ((a & 0x07) << ATA_CTRL_BITOFFSET_ADDRESS)

#if 0
/* ATA register addresses - read access */
/* CFv3 spec uses actual logic levels in address table,
   regardless of whether signal is active low or not
*/
#define ATA_ADDR_RD_DATA                 0x08
#define ATA_ADDR_RD_ERROR                0x09
#define ATA_ADDR_RD_NSECTORS             0x0A
#define ATA_ADDR_RD_SECTOR               0x0B
#define ATA_ADDR_RD_LCYL                 0x0C
#define ATA_ADDR_RD_HCYL                 0x0D
#define ATA_ADDR_RD_SELECT               0x0E
#define ATA_ADDR_RD_STATUS               0x0F
#define ATA_ADDR_RD_ALTSTATUS            0x16

/* ATA register addresses - write access */
#define ATA_ADDR_WR_FEATURE              ATA_ADDR_RD_ERROR_REG
#define ATA_ADDR_WR_COMMAND              ATA_ADDR_RD_STATUS_REG
#define ATA_ADDR_WR_CONTROL              ATA_ADDR_RD_ALTSTATUS_REG
#endif

/* ATA read and write addresses */
#define ATA_ADDR_DATA                    (ATA_CTRL_BIT_CS0 | ATA_CTRL_BIT_ADDR(0x00))
#define ATA_ADDR_NSECTORS                (ATA_CTRL_BIT_CS0 | ATA_CTRL_BIT_ADDR(0x02))
#define ATA_ADDR_LBALO                   (ATA_CTRL_BIT_CS0 | ATA_CTRL_BIT_ADDR(0x03))
#define ATA_ADDR_LBAMID                  (ATA_CTRL_BIT_CS0 | ATA_CTRL_BIT_ADDR(0x04))
#define ATA_ADDR_LBAHI                   (ATA_CTRL_BIT_CS0 | ATA_CTRL_BIT_ADDR(0x05))
#define ATA_ADDR_SELECT                  (ATA_CTRL_BIT_CS0 | ATA_CTRL_BIT_ADDR(0x06))
/* ATA read addresses */
#define ATA_ADDR_ERROR                   (ATA_CTRL_BIT_CS0 | ATA_CTRL_BIT_ADDR(0x01))
#define ATA_ADDR_STATUS                  (ATA_CTRL_BIT_CS0 | ATA_CTRL_BIT_ADDR(0x07))
#define ATA_ADDR_ALTSTATUS               (ATA_CTRL_BIT_CS1 | ATA_CTRL_BIT_ADDR(0x06))
/* ATA write addresses */
#define ATA_ADDR_FEATURE                 (ATA_CTRL_BIT_CS0 | ATA_CTRL_BIT_ADDR(0x01))
#define ATA_ADDR_COMMAND                 (ATA_CTRL_BIT_CS0 | ATA_CTRL_BIT_ADDR(0x07))
#define ATA_ADDR_CONTROL                 (ATA_CTRL_BIT_CS1 | ATA_CTRL_BIT_ADDR(0x06))


/** ATA register bit masks **/
/* drive info fields */
//#define EIDE_VALID                       0x02
//#define PIO_MODE3_MASK                   0x01
//#define PIO_MODE4_MASK                   0x02
//#define ATA_PIO_MODE3                    3
//#define ATA_PIO_MODE4                    4

/* Select Reg */
#define ATA_SELECT_MASTER                0x00
#define ATA_SELECT_SLAVE                 0x10
#define ATA_SELECT_LBA                   0xe0 /* The actual LBA bit is 0x40 but
                                                 bits 7 and 5 should remain set */

/* Status Reg */
#define ATA_STATUS_BSY                   0x80
#define ATA_STATUS_DRDY                  0x40
#define ATA_STATUS_DF                    0x20
#define ATA_STATUS_DSC                   0x10
#define ATA_STATUS_DRQ                   0x08
#define ATA_STATUS_CORR                  0x04
#define ATA_STATUS_CHECK                 0x01

/* Control Reg */
#define ATA_CONTROL_INT_DISABLE          0x02

/* Feature Reg */
#define ATA_SETPIO                       0x03
#define ATA_PIO_MODE                     0x08

/* Sector Count Reg */
#define ATA_NSECTORS_SECS_PER_TRACK      64 /* not hex */

/* Command Reg */
#define ATA_CMD_RECAL                    0x10
#define ATA_CMD_READ                     0x20
#define ATA_CMD_WRITE                    0x30
#define ATA_CMD_INIT                     0x91
#define ATA_CMD_ID                       0xEC
#define ATA_CMD_SPINDOWN                 0xE0
#define ATA_CMD_SPINUP                   0xE1
#define ATA_CMD_SETFEATURES              0xEF

#define SECTORTRANSFER                   256

#define ATA_TIMEOUT_VAL                  8000

/* Actual drive information sizes: */
#define HDINFO_SIZE_BYTES                512
#define HDINFO_SIZE_WORDS                256
#define HDINFO_SIZE_LONGS                128




#if defined(ATA_USE_ID)
HDINFO	hd0info;
#endif

uint8_t Current8255CtrlWd = 0x00;
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

    /* set control signals to 8255 to be outputs */
    Pin8255nCS = N_DISABLE;
    Tris8255nCS = BIT_OUTPUT;
    Tris8255nRD = BIT_OUTPUT;
    Pin8255nRD = N_DISABLE;
    Tris8255nWR = BIT_OUTPUT;
    Pin8255nWR = N_DISABLE;

    // DEBUG:
    PIC_TRIS_8255_CS &= ~(0x01 << PIC_BITNUM_8255_CS);
    PIC_PORT_8255_CS |= (0x01 << PIC_BITNUM_8255_CS);

    /* address bits to 8255 as outputs */
    PIC_PORT_8255_ADDRESS &= _8255_ADDR_CLEAR;
    PIC_TRIS_8255_ADDRESS &= _8255_ADDR_CLEAR;

    /* set other signals to 8255 to a sensible initial direction and state */
    PIC_TRIS_8255_DATA = PORT_OUTPUT;
    PIC_PORT_8255_DATA = 0x00;

    /* the 8255 control register will always have MODE SET active */
    Current8255CtrlWd = _8255_CTRLWD_MODESET;

    /* now talk to the ATA device */
    resetata();

    /* clear select card/head register prior to drive selection to
       ensure known state */
    setataregbyte(ATA_ADDR_SELECT, 0);
    ATA_DriveSelect(drivenumber);

    /* wait for RDY bit set */
    i = ATA_TIMEOUT_VAL;
    while(!(getataregbyte(ATA_ADDR_STATUS) & ATA_STATUS_DRDY))
    {
        if(--i==0)
        {
            return(ATA_RDY_TIMEOUT);
        }
    }

    /* wait for BSY bit clear */
    i = ATA_TIMEOUT_VAL;
    while(getataregbyte(ATA_ADDR_STATUS) & ATA_STATUS_BSY)
    {
        if(--i==0)
        {
            return(ATA_BSY_TIMEOUT);
        }
    }

    /* disable interrupts in the ATA device */
    setataregbyte(ATA_ADDR_CONTROL, ATA_CONTROL_INT_DISABLE);

    ATA_DriveSelect(drivenumber);

    setataregbyte(ATA_ADDR_NSECTORS, ATA_NSECTORS_SECS_PER_TRACK);

    setataregbyte(ATA_ADDR_COMMAND, ATA_CMD_INIT);

    /* wait for BSY bit clear */
    i = ATA_TIMEOUT_VAL;
    while(getataregbyte(ATA_ADDR_STATUS) & ATA_STATUS_BSY)
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

    ataselectsetting = getataregbyte(ATA_ADDR_SELECT);
    if(drivenumber == ATA_MASTER)
    {
        ataselectsetting &= ~ATA_SELECT_SLAVE; /* Clear DRV bit: drive 0 */
    }
    else
    {
        ataselectsetting |= ATA_SELECT_SLAVE;  /* Set DRV bit: drive 1 */
    }
    setataregbyte(ATA_ADDR_SELECT, ataselectsetting);

    CurrentDrv = drivenumber;
}

int8_t ATA_ReadDriveInfo()
{
    uint32_t	i;

    /* wait for BSY bit clear */
    i = ATA_TIMEOUT_VAL;
    while(getataregbyte(ATA_ADDR_STATUS) & ATA_STATUS_BSY)
    {
        if(--i==0)
        {
            return(ATA_BSY_TIMEOUT);
        }
    }

    setataregbyte(ATA_ADDR_COMMAND, ATA_CMD_ID);

    /* wait for BSY bit clear */
    i = ATA_TIMEOUT_VAL;
    while(getataregbyte(ATA_ADDR_STATUS) & ATA_STATUS_BSY)
    {
        if(--i==0)
        {
            return(ATA_BSY_TIMEOUT);
        }
    }
    /* wait for DRQ bit set */
    i = ATA_TIMEOUT_VAL;
    while((getataregbyte(ATA_ADDR_STATUS) & ATA_STATUS_DRQ) == 0)
    {
        if(--i==0)
        {
            return(ATA_DRQ_TIMEOUT);
        }
    }

    /* read the useful drive data to the union */
    for(i=0; i < HDINFO_SIZE_WORDS; i++)
    {
#if defined(ATA_USE_ID)
        switch(i)
        {
        case ATA_DRVINFOWORD_CYLS:
            hd0info.cyls = getataregword(ATA_ADDR_DATA);
            break;
        case ATA_DRVINFOWORD_HEADS:
            hd0info.heads = getataregword(ATA_ADDR_DATA);
            break;
        case ATA_DRVINFOWORD_SECTORS:
            hd0info.sectors = getataregword(ATA_ADDR_DATA);
            break;
        default:
            if( (i>= ATA_DRVINFOWORD_MODELSTART) &&
                (i < (ATA_DRVINFOWORD_MODELSTART + ATA_DRVINFOWORD_MODELLENGTH)) )
            {
                hd0info.model[i-ATA_DRVINFOWORD_MODELSTART] = getataregword(ATA_ADDR_DATA);
            }
            else if( (i>= ATA_DRVINFOWORD_REVSTART) &&
                     (i < (ATA_DRVINFOWORD_REVSTART + ATA_DRVINFOWORD_REVLENGTH)) )
            {
                hd0info.fw_rev[i-ATA_DRVINFOWORD_REVSTART] = getataregword(ATA_ADDR_DATA);
            }
            else if( (i>= ATA_DRVINFOWORD_SERNUMSTART) &&
                     (i < (ATA_DRVINFOWORD_SERNUMSTART + ATA_DRVINFOWORD_SERNUMLENGTH)) )
            {
                hd0info.fw_rev[i-ATA_DRVINFOWORD_SERNUMSTART] = getataregword(ATA_ADDR_DATA);
            }
            else
            {
                getataregword(ATA_ADDR_DATA);
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

    /* set up to read 1 sector: */
    setataregbyte(ATA_ADDR_NSECTORS, 1);
    setataregbyte(ATA_ADDR_COMMAND, ATA_CMD_READ);

    timeout = ATA_TIMEOUT_VAL;

    while( (getataregbyte(ATA_ADDR_STATUS) & ATA_STATUS_BSY) ||
           (!(getataregbyte(ATA_ADDR_STATUS) & ATA_STATUS_DRQ)) )
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

    while(getataregbyte(ATA_ADDR_STATUS) & ATA_STATUS_BSY)
        ;

    data = getataregword(ATA_ADDR_DATA);

    return(data);
}

void ATA_SkipWords(uint16_t numwords)
{
    while(numwords)
    {
        getataregword(ATA_ADDR_DATA);
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

    setataregbyte(ATA_ADDR_NSECTORS, count);
    setataregbyte(ATA_ADDR_COMMAND, ATA_CMD_READ);

    nsectors = count;
    if ( nsectors == 0 )
        nsectors=256;

    timeout = ATA_TIMEOUT_VAL;

    while( nsectors )
    {
        /* wait for BSY bit clear or DRQ for extra compatibility*/
        if ( (!(getataregbyte(ATA_ADDR_STATUS) & ATA_STATUS_BSY)) ||
             (getataregbyte(ATA_ADDR_STATUS) & ATA_STATUS_DRQ) )
        {
            timeout = ATA_TIMEOUT_VAL;

            transferwords = SECTORTRANSFER;

            while(transferwords)
            {
                while(( (getataregbyte(ATA_ADDR_STATUS)) & ATA_STATUS_BSY))
                    ;
                *buffer++ = getataregword(ATA_ADDR_DATA);
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
uint8_t read8255port(uint8_t portAddr)
{
    uint8_t readValue;

    Pin8255nRD = N_DISABLE;
    Pin8255nWR = N_DISABLE;
    Pin8255nCS = N_ENABLE;

    /* write to control word to choose
     * direction for specified 8255 port
     */
    PIC_TRIS_8255_DATA = PORT_OUTPUT;
    PIC_PORT_8255_ADDRESS &= _8255_ADDR_CLEAR;
    PIC_PORT_8255_ADDRESS |= _8255_ADDR_CTRLWD;

    switch(portAddr)
    {
    case _8255_ADDR_ATA_DATALO:
        Current8255CtrlWd |= _8255_CTRLWD_ATA_DATALO_RD;
                break;
    case _8255_ADDR_ATA_DATAHI:
        Current8255CtrlWd |= _8255_CTRLWD_ATA_DATAHI_RD;
                break;
    case _8255_ADDR_ATA_CTRL:
        Current8255CtrlWd |= _8255_CTRLWD_ATA_CTRL_RD;
        break;
    default:
        /* Hmmm. */
        break;
    }
    Pin8255nWR = N_ENABLE;
    PIC_PORT_8255_DATA = Current8255CtrlWd;
    Pin8255nWR = N_DISABLE;

    /* now read from the specified 8255 port */
    PIC_TRIS_8255_DATA = PORT_INPUT;
    PIC_PORT_8255_ADDRESS &= _8255_ADDR_CLEAR;
    PIC_PORT_8255_ADDRESS |= portAddr;

    Pin8255nRD = N_ENABLE;
    readValue = PIC_PORT_8255_DATA;
    Pin8255nRD = N_DISABLE;
    Pin8255nCS = N_DISABLE;

    return readValue;
}

void write8255port(uint8_t portAddr, uint8_t value)
{
    Pin8255nRD = N_DISABLE;
    Pin8255nWR = N_DISABLE;
    Pin8255nCS = N_ENABLE;

    /* write to control word to choose
     * direction for specified 8255 port
     */
    PIC_TRIS_8255_DATA = PORT_OUTPUT;
    PIC_PORT_8255_ADDRESS &= _8255_ADDR_CLEAR;
    PIC_PORT_8255_ADDRESS |= _8255_ADDR_CTRLWD;

    switch(portAddr)
    {
    case _8255_ADDR_ATA_DATALO:
        Current8255CtrlWd &= ~_8255_CTRLWD_ATA_DATALO_RD;
                break;
    case _8255_ADDR_ATA_DATAHI:
        Current8255CtrlWd &= ~_8255_CTRLWD_ATA_DATAHI_RD;
                break;
    case _8255_ADDR_ATA_CTRL:
        Current8255CtrlWd &= ~_8255_CTRLWD_ATA_CTRL_RD;
        break;
    default:
        /* Hmmm. */
        break;
    }
    Pin8255nWR = N_ENABLE;
    PIC_PORT_8255_DATA = Current8255CtrlWd;
    Pin8255nWR = N_DISABLE;

    /* now write to the specified 8255 port */
    PIC_TRIS_8255_DATA = PORT_OUTPUT;
    PIC_PORT_8255_ADDRESS &= _8255_ADDR_CLEAR;
    PIC_PORT_8255_ADDRESS |= portAddr;

    Pin8255nWR = N_ENABLE;
    PIC_PORT_8255_DATA = value;
    Pin8255nWR = N_DISABLE;
    Pin8255nCS = N_DISABLE;
}

void resetata()
{
    volatile uint32_t i;

    write8255port(_8255_ADDR_ATA_CTRL, ATA_CTRL_BIT_RESET);

    i = ATA_TIMEOUT_VAL;
    while(i)
        i--;

    write8255port(_8255_ADDR_ATA_CTRL, 0);
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
        byte4 &= ~ATA_SELECT_SLAVE;
        /* Set LBA mode */
        byte4 |= ATA_SELECT_LBA;
    }
    else
    {
        /* Set DRV bit: drive 1, set LBA mode */
        byte4 |= (ATA_SELECT_SLAVE | ATA_SELECT_LBA);
    }

    setataregbyte(ATA_ADDR_SELECT, byte4);
    setataregbyte(ATA_ADDR_LBAHI, byte3);
    setataregbyte(ATA_ADDR_LBAMID, byte2);
    setataregbyte(ATA_ADDR_LBALO, byte1);

    /* update active LBA address */
    CurrentLBAAddr = lba;
}

void setataregbyte(uint8_t address, uint8_t data)
{
    volatile uint32_t i;

    write8255port(_8255_ADDR_ATA_CTRL, address);
    write8255port(_8255_ADDR_ATA_DATALO, data);
    write8255port(_8255_ADDR_ATA_CTRL, address | ATA_CTRL_BIT_WR);

    i = ATA_TIMEOUT_VAL;
    while(i)
        i--;

    write8255port(_8255_ADDR_ATA_CTRL, address);
}

void setataregword(uint8_t address, uint16_t data)
{
    volatile uint32_t i;

    write8255port(_8255_ADDR_ATA_CTRL, address);
    write8255port(_8255_ADDR_ATA_DATALO, (uint8_t)(data & 0x00FF));
    write8255port(_8255_ADDR_ATA_DATAHI, (uint8_t)(data >> 8));
    write8255port(_8255_ADDR_ATA_CTRL, address | ATA_CTRL_BIT_WR);

    i = ATA_TIMEOUT_VAL;
    while(i)
        i--;

    write8255port(_8255_ADDR_ATA_CTRL, address);
}

uint8_t getataregbyte(uint8_t address)
{
    uint8_t dataread;

    write8255port(_8255_ADDR_ATA_CTRL, address);
    write8255port(_8255_ADDR_ATA_CTRL, address | ATA_CTRL_BIT_RD);

    dataread = read8255port(_8255_ADDR_ATA_DATALO);

    write8255port(_8255_ADDR_ATA_CTRL, address);

    return dataread;
}

uint16_t getataregword(uint8_t address)
{
    uint16_t datareadlow, datareadhigh;

    write8255port(_8255_ADDR_ATA_CTRL, address);
    write8255port(_8255_ADDR_ATA_CTRL, address | ATA_CTRL_BIT_RD);

    datareadlow = read8255port(_8255_ADDR_ATA_DATALO);
    datareadhigh = read8255port(_8255_ADDR_ATA_DATAHI);

    write8255port(_8255_ADDR_ATA_CTRL, address);

    return ((datareadlow << 8) | (datareadhigh & 0x00FF));
}
