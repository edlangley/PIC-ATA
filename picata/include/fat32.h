/*-------------------------------------------------------------------
Name......... fat32.h
Author....... Ed Langley
Date......... 29/11/2006
Description.. Header file for a driver to read a FAT32 FS over an ATA
............. interface on a PIC MCU
-------------------------------------------------------------------*/

/* MBR & BPB/Volume ID stuff */
#define MBR_SIG_OFFSET_WORDS             205
#define MBR_SIG                          0x55AA
#define MBR_PART_OFFSET_WORDS            446
#define PART_TYPE_OFFSET_WORDS           2
#define MBR_FAT32_PART_SIG1              0x0B
#define MBR_FAT32_PART_SIG2              0x0C

#define BPB_SEC_PER_CLUS_OFFSET_WORDS    6
#define BPB_SEC_PER_FAT_OFFSET_WORDS     9
#define BPB_ROOT_DIR_CLUSTER_OFFSET_WORDS 4//2

/* This is always the same */
#define BPB_BYTES_PER_SECTOR             512

/* dir entry stuff */
#define DIR_REC_LENGTH_WORDS             16
#define DIR_REC_LENGTH_BYTES             32
#define DIR_RECS_PER_SECTOR              16
#define SHORT_FILENAME_LENGTH_BYTES      11
#define SHORT_FILENAME_LENGTH_WORDS      5
#define DIR_REC_CLUSTER_HI_OFFSET_WORDS  4
#define DIR_REC_CLUSTER_LO_OFFSET_WORDS  2

#define DIRENTRY_NAMECHAR_UNUSED         0xE5
#define DIRENTRY_ATTRIB_DIR              0x10
#define DIRENTRY_ATTRIB_LFN              0x0F

/* FAT/Cluster chain stuff */
#define EOF_CLUSTER_MARKER               0xFFFFFFFF


/* file reading */
/* read a word from disk, and a byte from API, so need
   to do disk read every other byte
*/
#define EVEN_BYTE                        0
#define ODD_BYTE                         1

/* return codes */
#define FAT32_OK                         0
#define FAT32_INVALID_MBR               -10
#define FAT32_PART_NOT_FOUND            -11
#define FAT32_EODIRENTRYS               -12
#define FAT32_DIRENTRY_IS_DIR           -13
#define FAT32_EOF                       -14
#define FAT32_FILE_NOT_FOUND            -15

#define CLUSTERNUMTOLBA(clusternumber)(ClusterBeginLBA + (clusternumber - 2) * SectorsPerCluster)

typedef struct
{
  uint32_t firstcluster;
  uint32_t currentclusternum;
  //  uint32_t currentclusterbytesleft;
  uint32_t currentsectornum;
  uint32_t currentsectorpos;
  uint32_t sizebytes;
  uint32_t position;
  uint8_t evenbyte;
  uint16_t currentword;
} FD;


int8_t FAT32_Mount(uint8_t drvnum);
int8_t FAT32_DirOpen(uint8_t *dirname);
int8_t FAT32_DirLoadNextEntry();
uint8_t *FAT32_DirEntryName();

int8_t FAT32_FileOpen(FD *fd, uint8_t *filename);
int8_t FAT32_FileRead(FD *fd, uint16_t numBytes, uint8_t *dataBuf);

