/*-------------------------------------------------------------------
Name......... ata.h
Author....... Ed Langley
Date......... 14/11/2006
Description.. Header file with port, address and bitmask definitions
............. for the ATA interface on a PIC MCU
-------------------------------------------------------------------*/

#define ATA_USE_WIRINGTEST               0
#define ATA_USE_ID                       1


/* Return codes */
#define ATA_OK                           0
#define ATA_DRQ_TIMEOUT                 -1
#define ATA_BSY_TIMEOUT                 -2
#define	ATA_PIO_INVALID                 -3
#define ATA_RDY_TIMEOUT                 -4

/* Drive select */
#define ATA_MASTER                       0
#define ATA_SLAVE                        1

/* Reading multi-byte values macros for easier porting,
 * stored on disk as little endian so OR bytes in reverse */
#define ATA_READ_16BIT_VAL(x) \
    x = (uint16_t)ATA_ReadSectorByte(); \
    x |= ((uint16_t)ATA_ReadSectorByte() << 8)
#define ATA_READ_32BIT_VAL(x) \
    x = (uint32_t)ATA_ReadSectorByte(); \
    x |= ((uint32_t)ATA_ReadSectorByte() << 8); \
    x |= ((uint32_t)ATA_ReadSectorByte() << 16); \
    x |= ((uint32_t)ATA_ReadSectorByte() << 24)


#define ATA_DRVINFO_MODEL_LEN            25
#define ATA_DRVINFO_REV_LEN              9
#define ATA_DRVINFO_SERNUM_LEN           5

#if defined(ATA_USE_ID)
typedef struct
{
  uint16_t cyls;           /* "physical" cyls */
  uint16_t heads;          /* "physical" heads */
  uint16_t sectors;        /* "physical" sectors per track */
  uint8_t model[ATA_DRVINFO_MODEL_LEN];       /* 0 = not_specified */
  uint8_t fwRev[ATA_DRVINFO_REV_LEN];        /* 0 = not_specified */
  uint8_t serialNum[ATA_DRVINFO_SERNUM_LEN];  /* 0 = not_specified */
} HDINFO;
#endif

/* API functions */
int8_t ATA_Init(uint8_t);
void ATA_DriveSelect(uint8_t);
#if defined(ATA_USE_ID)
int8_t ATA_ReadDriveInfo(HDINFO *hdInfo);
#endif
int8_t ATA_SetSectorLBAForRead(uint32_t lba);
uint8_t ATA_ReadSectorByte();
void ATA_SkipSectorBytes(uint16_t numbytes);
uint32_t ATA_CurrentSectorLBAAddr();
int8_t ATA_ReadSectors(uint32_t, uint16_t *, uint8_t);
