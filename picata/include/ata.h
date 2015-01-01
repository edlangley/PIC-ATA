/*-------------------------------------------------------------------
Name......... ata.h
Author....... Ed Langley
Date......... 14/11/2006
Description.. Header file with port, address and bitmask definitions
............. for the ATA interface on a PIC MCU
-------------------------------------------------------------------*/

#define ATA_USE_WIRINGTEST               1
#define ATA_USE_ID 1


/* Return codes */
#define ATA_OK                           0
#define ATA_DRQ_TIMEOUT                 -1
#define ATA_BSY_TIMEOUT                 -2
#define	ATA_PIO_INVALID                 -3
#define ATA_RDY_TIMEOUT                 -4

/* Drive select */
#define ATA_MASTER                       0
#define ATA_SLAVE                        1

/* leaving the HDINFO here in the header as it
 * will be part of API and remove getter functions
 * shortly:
 */

/* Just use a small global structure with the most useful
   drive info variables: */
/* word numbers of useful info: */
#define ATA_DRVINFOWORD_CYLS             1
#define ATA_DRVINFOWORD_HEADS            3
#define ATA_DRVINFOWORD_SECTORS          6
#define ATA_DRVINFOWORD_MODELSTART       27
#define ATA_DRVINFOWORD_MODELLENGTH      12
#define ATA_DRVINFOWORD_REVSTART         23
#define ATA_DRVINFOWORD_REVLENGTH        4
#define ATA_DRVINFOWORD_SERNUMSTART      10
#define ATA_DRVINFOWORD_SERNUMLENGTH     2

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

/* API functions */
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
