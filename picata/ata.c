/*-------------------------------------------------------------------
Name......... ata.c
Author....... Ed Langley
Date......... 19/10/2006
Description.. Driver to operate an ATA interface on a PIC MCU
-------------------------------------------------------------------*/

#include "platform.h"
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

#define PIN_nENABLE                      BIT_CLR
#define PIN_nDISABLE                     BIT_SET
/* end of changeable settings */


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

#define BYTE_SWAP_16(w)                  ((w << 8) | (w >> 8))


/* Word numbers of useful parts in drive info block */
#define ATA_DRVINFOWORD_CYLS             1
#define ATA_DRVINFOWORD_HEADS            3
#define ATA_DRVINFOWORD_SECTORS          6
#define ATA_DRVINFOWORD_MODEL_START      27
#define ATA_DRVINFOWORD_REV_START        23
#define ATA_DRVINFOWORD_SERNUM_START     10
#define ATA_DRVINFOWORD_MODEL_LEN        ((ATA_DRVINFO_MODEL_LEN-1)/2)
#define ATA_DRVINFOWORD_REV_LEN          ((ATA_DRVINFO_REV_LEN-1)/2)
#define ATA_DRVINFOWORD_SERNUM_LEN       ((ATA_DRVINFO_SERNUM_LEN-1)/2)

static uint8_t Current8255CtrlWd = 0x00;
static uint8_t CurrentDrv = 0;
static uint32_t CurrentLBAAddr = 0;
static uint8_t CurrentReadByteOdd = 0;
static uint16_t CurrentReadWord = 0;

/* Internal function prototypes */
static void set8255DataPortsDir(uint8_t dir);
static uint8_t read8255Port(uint8_t portAddr);
static void write8255Port(uint8_t portAddr, uint8_t value);
static void setAtaRegByte(uint8_t ataRegAddr, uint8_t data);
static void setAtaRegWord(uint8_t ataRegAddr, uint16_t data);
static uint8_t getAtaRegByte(uint8_t ataRegAddr);
static uint16_t getAtaRegWord(uint8_t ataRegAddr);
static void resetAta();
static void writeLba(uint32_t lba);
#ifdef ATA_USE_WIRINGTEST
static void ataWiringTest();
#endif

/* API functions */

int8_t ATA_Init(uint8_t drivenumber)
{
    uint32_t i;

    /* disable A/Ds to allow pins to be used as digital I/O */
    ADCON1 = ALL_DIGITAL;

    /* set control signals to 8255 to be outputs */
    PIN_OUTPUT(PIC_TRIS_8255_CS, PIC_BITNUM_8255_CS);
    PIN_nDISABLE(PIC_PORT_8255_CS, PIC_BITNUM_8255_CS);
    PIN_OUTPUT(PIC_TRIS_8255_RDWR, PIC_BITNUM_8255_RD);
    PIN_nDISABLE(PIC_PORT_8255_RDWR, PIC_BITNUM_8255_RD);
    PIN_OUTPUT(PIC_TRIS_8255_RDWR, PIC_BITNUM_8255_WR);
    PIN_nDISABLE(PIC_PORT_8255_RDWR, PIC_BITNUM_8255_WR);

    /* address bits to 8255 as outputs */
    PIC_PORT_8255_ADDRESS &= _8255_ADDR_CLEAR;
    PIC_TRIS_8255_ADDRESS &= _8255_ADDR_CLEAR;

    /* set other signals to 8255 to a sensible initial direction and state */
    PIC_TRIS_8255_DATA = PORT_OUTPUT;
    PIC_PORT_8255_DATA = 0x00;

    /* the 8255 control register will always have MODE SET active,
     * the 8255 port handling the ATA control signals will always
     * be an output port, so leave those bits cleared for output */
    Current8255CtrlWd = _8255_CTRLWD_MODESET;

#ifdef ATA_USE_WIRINGTEST
    ataWiringTest();
#endif

    write8255Port(_8255_ADDR_ATA_DATALO, 0);
    write8255Port(_8255_ADDR_ATA_DATAHI, 0);
    write8255Port(_8255_ADDR_ATA_CTRL, 0);
    DELAY_MS(1);

    /* now talk to the ATA device */
    resetAta();

    /* clear select card/head register prior to drive selection to
       ensure known state */
    setAtaRegByte(ATA_ADDR_SELECT, 0);
    ATA_DriveSelect(drivenumber);

    /* wait for RDY bit set */
    i = ATA_TIMEOUT_VAL;
    while(!(getAtaRegByte(ATA_ADDR_STATUS) & ATA_STATUS_DRDY))
    {
        DELAY_MS(1);
        if(--i==0)
        {
            return(ATA_RDY_TIMEOUT);
        }
    }

    /* wait for BSY bit clear */
    i = ATA_TIMEOUT_VAL;
    while(getAtaRegByte(ATA_ADDR_STATUS) & ATA_STATUS_BSY)
    {
        DELAY_MS(1);
        if(--i==0)
        {
            return(ATA_BSY_TIMEOUT);
        }
    }

    /* disable interrupts in the ATA device */
    setAtaRegByte(ATA_ADDR_CONTROL, ATA_CONTROL_INT_DISABLE);

    ATA_DriveSelect(drivenumber);
    setAtaRegByte(ATA_ADDR_NSECTORS, ATA_NSECTORS_SECS_PER_TRACK);
    setAtaRegByte(ATA_ADDR_COMMAND, ATA_CMD_INIT);
    /* wait for BSY bit clear */
    i = ATA_TIMEOUT_VAL;
    while(getAtaRegByte(ATA_ADDR_STATUS) & ATA_STATUS_BSY)
    {
        DELAY_MS(1);
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

    ataselectsetting = getAtaRegByte(ATA_ADDR_SELECT);
    if(drivenumber == ATA_MASTER)
    {
        ataselectsetting &= ~ATA_SELECT_SLAVE; /* Clear DRV bit: drive 0 */
    }
    else
    {
        ataselectsetting |= ATA_SELECT_SLAVE;  /* Set DRV bit: drive 1 */
    }
    setAtaRegByte(ATA_ADDR_SELECT, ataselectsetting);

    CurrentDrv = drivenumber;
}

#if defined(ATA_USE_ID)
#define HD_INFO_DBG
#ifdef HD_INFO_DBG
 #include <stdio.h>
#endif

int8_t ATA_ReadDriveInfo(HDINFO *hdInfo)
{
    uint32_t i;

    /* wait for BSY bit clear */
    i = ATA_TIMEOUT_VAL;
    while(getAtaRegByte(ATA_ADDR_STATUS) & ATA_STATUS_BSY)
    {
        DELAY_MS(1);
        if(--i==0)
        {
            return(ATA_BSY_TIMEOUT);
        }
    }

    ATA_DriveSelect(CurrentDrv);
    /* wait for RDY bit set */
    i = ATA_TIMEOUT_VAL;
    while(!(getAtaRegByte(ATA_ADDR_STATUS) & ATA_STATUS_DRDY))
    {
        DELAY_MS(1);
        if(--i==0)
        {
            return(ATA_RDY_TIMEOUT);
        }
    }

    setAtaRegByte(ATA_ADDR_COMMAND, ATA_CMD_ID);
    /* wait for BSY bit clear */
    i = ATA_TIMEOUT_VAL;
    while(getAtaRegByte(ATA_ADDR_STATUS) & ATA_STATUS_BSY)
    {
        DELAY_MS(1);
        if(--i==0)
        {
            return(ATA_BSY_TIMEOUT);
        }
    }

    /* wait for DRQ bit set */
    i = ATA_TIMEOUT_VAL;
    while((getAtaRegByte(ATA_ADDR_STATUS) & ATA_STATUS_DRQ) == 0)
    {
        DELAY_MS(1);
        if(--i==0)
        {
            return(ATA_DRQ_TIMEOUT);
        }
    }

    /* read the useful drive data to the union */
    for(i=0; i < HDINFO_SIZE_WORDS; i++)
    {
        uint16_t idword;
        uint16_t *infostrptr;
        idword = getAtaRegWord(ATA_ADDR_DATA);

 #ifdef HD_INFO_DBG
        if(!(i % 8))
            printf("\n\r%03X: ", i);
        printf(" %02X %02X", (idword >> 8), (idword & 0x00FF));
 #endif

        switch(i)
        {
        case ATA_DRVINFOWORD_CYLS:
            hdInfo->cyls = idword;
            break;
        case ATA_DRVINFOWORD_HEADS:
            hdInfo->heads = idword;
            break;
        case ATA_DRVINFOWORD_SECTORS:
            hdInfo->sectors = idword;
            break;
        default:
            if( (i>= ATA_DRVINFOWORD_MODEL_START) &&
                (i < (ATA_DRVINFOWORD_MODEL_START + ATA_DRVINFOWORD_MODEL_LEN)) )
            {
                infostrptr = (uint16_t*)hdInfo->model;
                infostrptr[i-ATA_DRVINFOWORD_MODEL_START] = BYTE_SWAP_16(idword);
            }
            else if( (i>= ATA_DRVINFOWORD_REV_START) &&
                     (i < (ATA_DRVINFOWORD_REV_START + ATA_DRVINFOWORD_REV_LEN)) )
            {
                infostrptr = (uint16_t*)hdInfo->fwRev;
                infostrptr[i-ATA_DRVINFOWORD_REV_START] = BYTE_SWAP_16(idword);
            }
            else if( (i>= ATA_DRVINFOWORD_SERNUM_START) &&
                     (i < (ATA_DRVINFOWORD_SERNUM_START + ATA_DRVINFOWORD_SERNUM_LEN)) )
            {
                infostrptr = (uint16_t*)hdInfo->serialNum;
                infostrptr[i-ATA_DRVINFOWORD_SERNUM_START] = BYTE_SWAP_16(idword);
            }
        }
    }

    /* Terminate those info strings */
    hdInfo->model[ATA_DRVINFO_MODEL_LEN-1] = 0;
    hdInfo->fwRev[ATA_DRVINFO_REV_LEN-1] = 0;
    hdInfo->serialNum[ATA_DRVINFO_SERNUM_LEN-1] = 0;

    return(ATA_OK);
}
#endif

int8_t ATA_SetSectorLBAForRead(uint32_t lba)
{
    uint32_t	timeout;

    writeLba(lba);

    /* set up to read 1 sector: */
    setAtaRegByte(ATA_ADDR_NSECTORS, 1);
    setAtaRegByte(ATA_ADDR_COMMAND, ATA_CMD_READ);

    timeout = ATA_TIMEOUT_VAL;

    while( (getAtaRegByte(ATA_ADDR_STATUS) & ATA_STATUS_BSY) ||
           (!(getAtaRegByte(ATA_ADDR_STATUS) & ATA_STATUS_DRQ)) )
    {
        timeout--;
        if(!timeout)
            return(ATA_BSY_TIMEOUT);
    }

    CurrentReadByteOdd = 0;

    return(ATA_OK);
}

uint8_t ATA_ReadSectorByte()
{
    if(!CurrentReadByteOdd)
    {
        while(getAtaRegByte(ATA_ADDR_STATUS) & ATA_STATUS_BSY)
            ;
        CurrentReadWord = getAtaRegWord(ATA_ADDR_DATA);
        CurrentReadByteOdd = 1;

        return(CurrentReadWord & 0x00FF);
    }
    else
    {
        CurrentReadByteOdd = 0;

        return(CurrentReadWord >> 8);
    }
}

void ATA_SkipSectorBytes(uint16_t numbytes)
{
    while(numbytes)
    {
        if(!CurrentReadByteOdd)
        {
            while(getAtaRegByte(ATA_ADDR_STATUS) & ATA_STATUS_BSY)
                ;
            CurrentReadWord = getAtaRegWord(ATA_ADDR_DATA);
            CurrentReadByteOdd = 1;
        }
        else
        {
            CurrentReadByteOdd = 0;
        }

        numbytes--;
    }
}

uint32_t ATA_CurrentSectorLBAAddr()
{
    return CurrentLBAAddr;
}

int8_t ATA_ReadSectors(uint32_t lba, uint16_t *buffer, uint8_t count)
{
    register uint32_t	nsectors;
    register uint32_t	timeout;
    register uint32_t	transferwords;

    writeLba(lba);

    setAtaRegByte(ATA_ADDR_NSECTORS, count);
    setAtaRegByte(ATA_ADDR_COMMAND, ATA_CMD_READ);

    nsectors = count;
    if ( nsectors == 0 )
        nsectors=256;

    timeout = ATA_TIMEOUT_VAL;

    while( nsectors )
    {
        /* wait for BSY bit clear or DRQ for extra compatibility*/
        if ( (!(getAtaRegByte(ATA_ADDR_STATUS) & ATA_STATUS_BSY)) ||
             (getAtaRegByte(ATA_ADDR_STATUS) & ATA_STATUS_DRQ) )
        {
            timeout = ATA_TIMEOUT_VAL;

            transferwords = SECTORTRANSFER;

            while(transferwords)
            {
                while(( (getAtaRegByte(ATA_ADDR_STATUS)) & ATA_STATUS_BSY))
                    ;
                *buffer++ = getAtaRegWord(ATA_ADDR_DATA);
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
static void set8255DataPortsDir(uint8_t dir)
{
    PIN_nENABLE(PIC_PORT_8255_CS, PIC_BITNUM_8255_CS);

    /* address the 8255 control word */
    PIC_TRIS_8255_DATA = PORT_OUTPUT;
    PIC_PORT_8255_ADDRESS &= _8255_ADDR_CLEAR;
    PIC_PORT_8255_ADDRESS |= _8255_ADDR_CTRLWD;

    if(dir == PORT_OUTPUT)
    {
        Current8255CtrlWd &= ~(_8255_CTRLWD_ATA_DATALO_RD | _8255_CTRLWD_ATA_DATAHI_RD);
    }
    else
    {
        Current8255CtrlWd |= (_8255_CTRLWD_ATA_DATALO_RD | _8255_CTRLWD_ATA_DATAHI_RD);
    }

    /* do the write */
    PIN_nENABLE(PIC_PORT_8255_RDWR, PIC_BITNUM_8255_WR);
    PIC_PORT_8255_DATA = Current8255CtrlWd;
    PIN_nDISABLE(PIC_PORT_8255_RDWR, PIC_BITNUM_8255_WR);

    PIN_nDISABLE(PIC_PORT_8255_CS, PIC_BITNUM_8255_CS);
}

static uint8_t read8255Port(uint8_t portAddr)
{
    uint8_t readValue;

    PIN_nENABLE(PIC_PORT_8255_CS, PIC_BITNUM_8255_CS);

    /* address the correct 8255 port */
    PIC_TRIS_8255_DATA = PORT_INPUT;
    PIC_PORT_8255_ADDRESS &= _8255_ADDR_CLEAR;
    PIC_PORT_8255_ADDRESS |= portAddr;

    /* do the read */
    PIN_nENABLE(PIC_PORT_8255_RDWR, PIC_BITNUM_8255_RD);
    readValue = PIC_PORT_8255_DATA;
    PIN_nDISABLE(PIC_PORT_8255_RDWR, PIC_BITNUM_8255_RD);

    PIN_nDISABLE(PIC_PORT_8255_CS, PIC_BITNUM_8255_CS);

    return readValue;
}

static void write8255Port(uint8_t portAddr, uint8_t value)
{
    PIN_nENABLE(PIC_PORT_8255_CS, PIC_BITNUM_8255_CS);

    /* address the correct 8255 port */
    PIC_TRIS_8255_DATA = PORT_OUTPUT;
    PIC_PORT_8255_ADDRESS &= _8255_ADDR_CLEAR;
    PIC_PORT_8255_ADDRESS |= portAddr;

    /* do the write */
    PIN_nENABLE(PIC_PORT_8255_RDWR, PIC_BITNUM_8255_WR);
    PIC_PORT_8255_DATA = value;
    PIN_nDISABLE(PIC_PORT_8255_RDWR, PIC_BITNUM_8255_WR);

    PIN_nDISABLE(PIC_PORT_8255_CS, PIC_BITNUM_8255_CS);
}

static void setAtaRegByte(uint8_t ataRegAddr, uint8_t data)
{
    set8255DataPortsDir(PORT_OUTPUT);

    write8255Port(_8255_ADDR_ATA_CTRL, ataRegAddr);
    write8255Port(_8255_ADDR_ATA_DATALO, data);
    write8255Port(_8255_ADDR_ATA_CTRL, ataRegAddr | ATA_CTRL_BIT_WR);

    /* data latched into drive when DIOW- de-asserted */
    write8255Port(_8255_ADDR_ATA_CTRL, ataRegAddr);
    write8255Port(_8255_ADDR_ATA_CTRL, 0);
}

static void setAtaRegWord(uint8_t ataRegAddr, uint16_t data)
{
    set8255DataPortsDir(PORT_OUTPUT);

    write8255Port(_8255_ADDR_ATA_CTRL, ataRegAddr);
    write8255Port(_8255_ADDR_ATA_DATALO, (uint8_t)(data & 0x00FF));
    write8255Port(_8255_ADDR_ATA_DATAHI, (uint8_t)(data >> 8));
    write8255Port(_8255_ADDR_ATA_CTRL, ataRegAddr | ATA_CTRL_BIT_WR);

    /* data latched into drive when DIOW- de-asserted */
    write8255Port(_8255_ADDR_ATA_CTRL, ataRegAddr);
    write8255Port(_8255_ADDR_ATA_CTRL, 0);
}

static uint8_t getAtaRegByte(uint8_t ataRegAddr)
{
    uint8_t dataread;

    set8255DataPortsDir(PORT_INPUT);

    write8255Port(_8255_ADDR_ATA_CTRL, ataRegAddr);
    write8255Port(_8255_ADDR_ATA_CTRL, ataRegAddr | ATA_CTRL_BIT_RD);

    dataread = read8255Port(_8255_ADDR_ATA_DATALO);

    write8255Port(_8255_ADDR_ATA_CTRL, ataRegAddr);
    write8255Port(_8255_ADDR_ATA_CTRL, 0);

    return dataread;
}

static uint16_t getAtaRegWord(uint8_t ataRegAddr)
{
    uint16_t datareadlow, datareadhigh;

    set8255DataPortsDir(PORT_INPUT);

    write8255Port(_8255_ADDR_ATA_CTRL, ataRegAddr);
    write8255Port(_8255_ADDR_ATA_CTRL, ataRegAddr | ATA_CTRL_BIT_RD);

    datareadlow = read8255Port(_8255_ADDR_ATA_DATALO);
    datareadhigh = read8255Port(_8255_ADDR_ATA_DATAHI);

    write8255Port(_8255_ADDR_ATA_CTRL, ataRegAddr);
    write8255Port(_8255_ADDR_ATA_CTRL, 0);

    /* Store the 16bit value in big endian internally,
     * makes no difference outside of the API
     */
    return ((datareadhigh << 8) | (datareadlow & 0x00FF));
}

static void resetAta()
{
    write8255Port(_8255_ADDR_ATA_CTRL, ATA_CTRL_BIT_RESET);
    DELAY_MS(20);

    write8255Port(_8255_ADDR_ATA_CTRL, 0);
    DELAY_MS(5);
}

static void writeLba(uint32_t lba)
{
    register uint32_t byte1, byte2, byte3, byte4;

    /* little endian */
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

    setAtaRegByte(ATA_ADDR_SELECT, byte4);
    setAtaRegByte(ATA_ADDR_LBAHI, byte3);
    setAtaRegByte(ATA_ADDR_LBAMID, byte2);
    setAtaRegByte(ATA_ADDR_LBALO, byte1);

    /* update active LBA address */
    CurrentLBAAddr = lba;
}

#ifdef ATA_USE_WIRINGTEST
static void ataWiringTest()
{
    uint8_t portAddr, bitIx, dataOut;

    set8255DataPortsDir(PORT_OUTPUT);

    for(portAddr = _8255_ADDR_ATA_DATALO;
        portAddr <= _8255_ADDR_ATA_CTRL;
        portAddr++)
    {
        for(bitIx = 0; bitIx < 8; bitIx++)
        {
            dataOut = (0x01 << bitIx);

            write8255Port(portAddr, dataOut);
        }
    }
}
#endif
