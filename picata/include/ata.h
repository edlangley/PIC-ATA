/*-------------------------------------------------------------------
Name......... ata.h
Author....... Ed Langley
Date......... 14/11/2006
Description.. Header file with port, address and bitmask definitions
............. for the ATA interface on a PIC MCU
-------------------------------------------------------------------*/


#define ATA_USE_ID 1

/* API: */
int8_t ATA_Init(uint8_t);
void ATA_DriveSelect(uint8_t);
int8_t ATA_ReadDriveInfo(void);
#if defined(ATA_USE_ID)
uint8_t *ATA_GetInfoModel(void);
uint8_t *ATA_GetInfoRev(void);
uint8_t *ATA_GetInfoSerialNum(void);
uint16_t ATA_GetInfoNumCyls(void);
uint16_t ATA_GetInfoNumHeads(void);
uint16_t ATA_GetInfoNumSectors(void);
#endif
int8_t ATA_SetLBAForRead(uint32_t lba);
uint16_t ATA_ReadWord();
void ATA_SkipWords(uint16_t numwords);
uint32_t ATA_CurrentLBAAddr();
int8_t ATA_ReadSectors(uint32_t, uint16_t *, uint8_t);
/* Internal */
void resetata();
void writelba(uint32_t);
void setataregbyte(uint8_t, uint8_t);
uint8_t getataregbyte(uint8_t);
uint16_t getataregword(uint8_t);

/* Defines and absolute bit variables */

/* PIC ports and bits: */
/* Change these settings depending on how the PIC is connected to the 8255: */
#define PORT_8255_ADDRESS			PORTA
//static volatile       unsigned char	PORT_8255_ADDRESS	@ &PORTA;
#define TRIS_8255_ADDRESS			TRISA
//static volatile bank1 unsigned char	TRIS_8255_ADDRESS	@ &TRISA;
#define PORT_8255_DATA				PORTD
#define TRIS_8255_DATA				TRISD
#define PORT_8255_CS				PORTE
#define TRIS_8255_CS				TRISE
#define PORT_8255_RDWR                          PORTE
#define TRIS_8255_RDWR                          TRISE

#define BITOFFSET_8255_ADDRESS                  0

#define BITNUM_8255_CS				2
#define BITNUM_8255_RD 0
#define BITNUM_8255_WR 1
//#define BITNUM_8255_RESET 2

/* end of changeable settings */

static BIT Pin8255nCS				@ PORTBIT(PORT_8255_CS, BITNUM_8255_CS);
static BIT Pin8255nRD				@ PORTBIT(PORT_8255_RDWR, BITNUM_8255_RD);
static BIT Pin8255nWR				@ PORTBIT(PORT_8255_RDWR, BITNUM_8255_WR);
static BIT Tris8255nCS				@ PORTBIT(TRIS_8255_CS, BITNUM_8255_CS);
static BIT Tris8255nRD				@ PORTBIT(TRIS_8255_RDWR, BITNUM_8255_RD);
static BIT Tris8255nWR				@ PORTBIT(TRIS_8255_RDWR, BITNUM_8255_WR);

/* PIC ADCON settings */
#define ALL_DIGITAL                             0x06

/* 8255 adresses and settings: */
#define ADDR_8255_CLEAR			~(0x03 << BITOFFSET_8255_ADDRESS)
#define ADDR_8255_PORTA			(0x00 << BITOFFSET_8255_ADDRESS)
#define ADDR_8255_PORTB			(0x01 << BITOFFSET_8255_ADDRESS)
#define ADDR_8255_PORTC			(0x02 << BITOFFSET_8255_ADDRESS)
#define ADDR_8255_CTRLWD		(0x03 << BITOFFSET_8255_ADDRESS)

/* Change these settings depending on how the 8255 is connected to the ATA device: */
#define	ADDR_8255_ATA_DATALO		ADDR_8255_PORTA
#define ADDR_8255_ATA_DATAHI		ADDR_8255_PORTB
#define ADDR_8255_ATA_CTRL			ADDR_8255_PORTC

// check the following setting against 8255 datasheet:
#define SET_8255_WRITE_ATA_REG	0x80
#define SET_8255_READ_ATA_REG	0x92

/* end of changeable settings */


/* ATA register addresses and bitmasks */
/* Signals are active high due to invertors, so no '/'s, set bits to activate signals. */

/* JMB circuit:
PIC PORT D -> 8255 PORT C
bit 0: RES
bit 1: WR
bit 2: CS0
bit 3: CS1
bit 4: RD
bit 5: A0
bit 6: A1
bit 7: A2
*/

/* My circuit:
PIC PORT D -> 8255 PORT C
bit 0: A0
bit 1: A1
bit 2: A2
bit 3: CS0
bit 4: CS1
bit 5: RD
bit 6: WR
bit 7: RES
*/

/* these are set seperately to the main ATA reg address so use bit variables */
/*
#define BITNUM_ATA_RD			5
#define BITNUM_ATA_WR			6
#define BITNUM_ATA_RES			7

static BIT PinATARD				@ PORTBIT(PORT_8255_DATA, BITNUM_ATA_RD);
static BIT PinATAWR				@ PORTBIT(PORT_8255_DATA, BITNUM_ATA_WR);
static BIT PinATARES			@ PORTBIT(PORT_8255_DATA, BITNUM_ATA_RES);
*/

#define ATA_READ_ENABLE			0x20
#define ATA_WRITE_ENABLE		0x40
#define ATA_RESET_ENABLE		0x80

//#define ATA_REG_READ			0x20
//#define ATA_REG_WRITE			0x40

#define ATA_TIMEOUT_VAL 8000

/* ATA register addresses - read access */
/* CFv3 spec uses actual logic levels in address table,
   regardless of whether signal is active low or not
*/
#define ATA_DATA_REG			0x08
#define ATA_ERROR_REG			0x09
#define ATA_NSECTORS_REG		0x0A
#define ATA_SECTOR_REG			0x0B
#define ATA_LCYL_REG			0x0C
#define ATA_HCYL_REG			0x0D
#define ATA_SELECT_REG			0x0E
#define ATA_STATUS_REG			0x0F
#define ATA_ALTSTATUS_REG		0x16


/* ATA register addresses - write access */
#define ATA_FEATURE_REG			ATA_ERROR_REG
#define ATA_COMMAND_REG			ATA_STATUS_REG
#define ATA_CONTROL_REG			ATA_ALTSTATUS_REG


/** ATA register bit masks **/
/* drive info fields */
#define EIDE_VALID		0x02
#define PIO_MODE3_MASK	0x01
#define PIO_MODE4_MASK	0x02
#define ATA_PIO_MODE3	3
#define ATA_PIO_MODE4	4

/* Select Reg */
#define ATA_MASTER		0x00
#define ATA_SLAVE		0x10
#define ATA_LBA			0xe0 /* The actual LBA bit is 0x40 but
								bits 7 and 5 should remain set */

/* Status Reg */
#define ATA_BSY			0x80
#define ATA_DRDY		0x40
#define ATA_DF			0x20
#define ATA_DSC			0x10
#define ATA_DRQ			0x08
#define ATA_CORR		0x04
#define ATA_CHECK		0x01

/* Control Reg */
#define ATA_INT_DISABLE	0x02

/* Feature Reg */
#define ATA_SETPIO		0x03
#define ATA_PIO_MODE	0x08

/* Sector Count Reg */
#define ATA_SECTORS_PER_TRACK	64 /* not hex */

/* Command Reg */
#define ATA_CMD_RECAL		0x10
#define ATA_CMD_READ		0x20
#define ATA_CMD_WRITE		0x30
#define ATA_CMD_INIT		0x91
#define ATA_CMD_ID			0xEC
#define ATA_CMD_SPINDOWN	0xE0
#define ATA_CMD_SPINUP		0xE1
#define	ATA_CMD_SETFEATURES 0xEF

#define	SECTORTRANSFER		256


/* Return codes */
#define ATA_OK				0
#define ATA_DRQ_TIMEOUT		-1
#define ATA_BSY_TIMEOUT		-2
#define	ATA_PIO_INVALID		-3
#define ATA_RDY_TIMEOUT		-4


/* Actual drive information sizes: */
#define HDINFO_SIZE_BYTES 512
#define HDINFO_SIZE_WORDS 256
#define HDINFO_SIZE_LONGS 128

/* Just use a small global structure with the most useful
   drive info variables: */
/* word numbers of usefule info: */
#define ATA_DRVINFOWORD_CYLS           1
#define ATA_DRVINFOWORD_HEADS          3
#define ATA_DRVINFOWORD_SECTORS        6
#define ATA_DRVINFOWORD_MODELSTART     27
//#define ATA_DRVINFOWORD_MODELLENGTH    20
#define ATA_DRVINFOWORD_MODELLENGTH    12
#define ATA_DRVINFOWORD_REVSTART       23
#define ATA_DRVINFOWORD_REVLENGTH      4
#define ATA_DRVINFOWORD_SERNUMSTART    10
//#define ATA_DRVINFOWORD_SERNUMLENGTH   10
#define ATA_DRVINFOWORD_SERNUMLENGTH   2

#if defined(ATA_USE_ID)
typedef struct
{
  uint16_t cyls;           /* "physical" cyls */
  uint16_t heads;          /* "physical" heads */
  uint16_t sectors;        /* "physical" sectors per track */
  uint16_t model[ATA_DRVINFOWORD_MODELLENGTH];       /* 0 = not_specified */
  uint16_t fw_rev[ATA_DRVINFOWORD_REVLENGTH];        /* 0 = not_specified */
  uint16_t serial_no[ATA_DRVINFOWORD_SERNUMLENGTH];  /* 0 = not_specified */
} HDINFO;
#endif
