/*-------------------------------------------------------------------
Name......... fat32.h
Author....... Ed Langley
Date......... 29/11/2006
Description.. Header file for a driver to read a FAT32 FS over an ATA
............. interface on a PIC MCU
-------------------------------------------------------------------*/

/* return codes */
#define FAT32_OK                         0
#define FAT32_INVALID_MBR               -10
#define FAT32_PART_NOT_FOUND            -11
#define FAT32_EODIRENTRYS               -12
#define FAT32_DIRENTRY_IS_DIR           -13
#define FAT32_EOF                       -14
#define FAT32_FILE_NOT_FOUND            -15


#define SHORT_FILENAME_LENGTH_BYTES      13 /* 8 + '.' + 3 + terminating zero */

typedef struct
{
    /* This name string is the only item which should be looked at
     * in this structure outside of the API
     */
    uint8_t currentDirEntryName[SHORT_FILENAME_LENGTH_BYTES];

    uint8_t currentDirEntryAttrib;
    uint32_t currentDirEntryFirstCluster;
    uint32_t currentDirEntrySizeBytes;

    uint32_t currentDirFirstCluster;
    uint32_t currentDirClusterNum;
    uint8_t currentDirSectorNum;     /* within the current cluster */
    uint16_t currentDirSectorOffset; /* within the current sector */
} DirDesc;

typedef struct
{
    uint32_t firstCluster;
    uint32_t currentClusterNum;
    uint32_t currentSectorNum;
    uint32_t currentSectorPos;
    uint32_t sizeBytes;
    uint32_t position;
} FD;


int8_t FAT32_Mount(uint8_t drvnum);
int8_t FAT32_DirOpen(DirDesc *dirdesc, const uint8_t *dirname);
int8_t FAT32_DirLoadNextEntry(DirDesc *dirdesc);

int8_t FAT32_FileOpen(FD *fd, DirDesc *dirdesc, const uint8_t *filename);
int8_t FAT32_FileRead(FD *fd, uint16_t numBytes, uint8_t *dataBuf);

