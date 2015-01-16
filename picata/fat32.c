/*-------------------------------------------------------------------
Name......... fat32.c
Author....... Ed Langley
Date......... 29/11/2006
Description.. Driver to read a FAT32 FS over an ATA interface on a
............. PIC MCU with minimal RAM usage
-------------------------------------------------------------------*/

#include "platform.h"
#include <string.h>
#include "ata.h"
#include "fat32.h"

/* MBR & BPB/Volume ID stuff */
#define MBR_SIG_OFFSET_BYTES             510
#define MBR_SIG1                         0x55
#define MBR_SIG2                         0xAA
#define MBR_PART_OFFSET_BYTES            446
#define PART_TYPE_OFFSET_BYTES           4
#define PART_LBABEGIN_OFFSET_BYTES       3
#define MBR_FAT32_PART_SIG1              0x0B
#define MBR_FAT32_PART_SIG2              0x0C

#define BPB_SEC_PER_CLUS_OFFSET_BYTES    13
#define BPB_SEC_PER_FAT_OFFSET_BYTES     19
#define BPB_ROOT_DIR_CLUSTER_OFFSET_BYTES 4

/* This is always the same */
#define BPB_BYTES_PER_SECTOR             512

/* dir entry stuff */
#define DIRENT_LENGTH_BYTES              32
#define DIRENTS_PER_SECTOR               16

#define DIRENT_CLUSTER_HI_OFFSET_BYTES   8
#define DIRENT_CLUSTER_LO_OFFSET_BYTES   4
#define DIRENT_FILENAME_LEN_BYTES        11
#define DIRENT_NAMECHAR_UNUSED           0xE5
#define DIRENT_ATTRIB_DIR                0x10
#define DIRENT_ATTRIB_LFN                0x0F

/* FAT/Cluster chain stuff */
#define EOF_CLUSTER_MARKER               0xFFFFFFFF

#define CLUSTERNUMTOLBA(clusternumber)(ClusterBeginLBA + (clusternumber - 2) * SectorsPerCluster)


uint32_t FatBeginLBA;
uint32_t ClusterBeginLBA;
uint8_t SectorsPerCluster;
uint32_t RootDirFirstCluster;


/* API functions */
int8_t FAT32_Mount(uint8_t drvnum)
{
    int8_t retval, filenameix;
    uint32_t templong;

    uint16_t signature;
    uint32_t partitionlbabegin;

    uint16_t bpbrsvdseccnt;
    uint8_t bpbnumfats;
    uint32_t bpbsecsperfat;


    /* validate MBR */
    if((retval = ATA_SetSectorLBAForRead(0)) != ATA_OK)
    {
        return retval;
    }

    ATA_SkipSectorBytes(MBR_SIG_OFFSET_BYTES);

    signature = ATA_ReadSectorByte();
    if(signature != MBR_SIG1)
    {
        return FAT32_INVALID_MBR;
    }
    signature = ATA_ReadSectorByte();
    if(signature != MBR_SIG2)
    {
        return FAT32_INVALID_MBR;
    }

    /* MBR OK */
    if((retval = ATA_SetSectorLBAForRead(0)) != ATA_OK)
    {
        return retval;
    }

    ATA_SkipSectorBytes(MBR_PART_OFFSET_BYTES + PART_TYPE_OFFSET_BYTES);

    signature = ATA_ReadSectorByte();
    if( (signature != MBR_FAT32_PART_SIG1) && (signature != MBR_FAT32_PART_SIG2) )
    {
        /* for now assume first partition is FAT32 */
        return FAT32_PART_NOT_FOUND;
    }

    ATA_SkipSectorBytes(PART_LBABEGIN_OFFSET_BYTES);

    ATA_READ_32BIT_VAL(partitionlbabegin);

    /* now move on to the volume ID sector or BPB of the FAT32 partition */
    if((retval = ATA_SetSectorLBAForRead(partitionlbabegin)) != ATA_OK)
    {
        return retval;
    }

    ATA_SkipSectorBytes(BPB_SEC_PER_CLUS_OFFSET_BYTES);

    SectorsPerCluster = ATA_ReadSectorByte();

    ATA_READ_16BIT_VAL(bpbrsvdseccnt);

    bpbnumfats = ATA_ReadSectorByte();

    ATA_SkipSectorBytes(BPB_SEC_PER_FAT_OFFSET_BYTES);

    ATA_READ_32BIT_VAL(bpbsecsperfat);

    ATA_SkipSectorBytes(BPB_ROOT_DIR_CLUSTER_OFFSET_BYTES);

    ATA_READ_32BIT_VAL(RootDirFirstCluster);

    FatBeginLBA = partitionlbabegin + bpbrsvdseccnt;
    ClusterBeginLBA = partitionlbabegin + bpbrsvdseccnt + (bpbnumfats * bpbsecsperfat);

    return(FAT32_OK);
}

int8_t FAT32_DirOpen(DirDesc *dirdesc, const uint8_t *dirname)
{
    int8_t retval = 0, filenameix;

    /* Ensure the filename string is terminated */
    for(filenameix = 0; filenameix < (SHORT_FILENAME_LENGTH_BYTES); filenameix++)
    {
        dirdesc->currentDirEntryName[filenameix] = 0;
    }

    /* check for root dir name (\\) to reset to start of root dir records */
    if(!strncmp(dirname, "\\", 1))
    {
        dirdesc->currentDirClusterNum = RootDirFirstCluster;
        dirdesc->currentDirSectorNum = 0;
        dirdesc->currentDirSectorOffset = 0;
        dirdesc->currentDirEntryName[0] = '\\';
        return FAT32_OK;
    }

    /* check for current dir name (.) to reset to start of dir records */
    if(!strncmp(dirname, ".", 1))
    {
        dirdesc->currentDirClusterNum = dirdesc->currentDirFirstCluster;
        dirdesc->currentDirSectorNum = 0;
        dirdesc->currentDirSectorOffset = 0;
        dirdesc->currentDirEntryName[0] = '.';
        return FAT32_OK;
    }

    while( (strncmp(dirname, dirdesc->currentDirEntryName, SHORT_FILENAME_LENGTH_BYTES)) &&
           (retval != FAT32_EODIRENTRYS) )
    {
        retval = FAT32_DirLoadNextEntry(dirdesc);
    }

    switch(retval)
    {
    case FAT32_DIRENTRY_IS_DIR:
        /* set to start of dir records for the dir found */
        dirdesc->currentDirClusterNum = dirdesc->currentDirEntryFirstCluster;
        dirdesc->currentDirSectorNum = 0;
        dirdesc->currentDirSectorOffset = 0;
        return FAT32_OK;
    case FAT32_EODIRENTRYS:
        return FAT32_FILE_NOT_FOUND;
    case FAT32_OK:
        /* next dir record is ok, keep looping until the dirname
           matches */
        break;
    default:
        break;
    }

    /* if no cases matched return mystery value and check it */
    return retval;
}

int8_t FAT32_DirLoadNextEntry(DirDesc *dirdesc)
{
    uint8_t filenameix, filenamelen;
    int8_t retval;
    uint32_t templong;

    dirdesc->currentDirEntryName[0] = DIRENT_NAMECHAR_UNUSED;
    while(((dirdesc->currentDirEntryName[0] == DIRENT_NAMECHAR_UNUSED) ||
           (dirdesc->currentDirEntryAttrib & DIRENT_ATTRIB_LFN)) &&
          (dirdesc->currentDirEntryName[0] != 0))
    {
        /* check if a new sector must be read */
        if(dirdesc->currentDirSectorOffset >= BPB_BYTES_PER_SECTOR)
        {
            /* check if a new cluster must be read */
            if(dirdesc->currentDirSectorNum >= SectorsPerCluster)
            {
                /* look up next cluster in FAT:
                 * - shift down 7 because cluster number size is 4 bytes, so 512/4=128 = 0x80,
                 * so >>7 is quick way of converting cluster number to sector containing it */

                if((retval = ATA_SetSectorLBAForRead(FatBeginLBA + (dirdesc->currentDirClusterNum >> 7))) != ATA_OK)
                {
                    return retval;
                }
                /* again quickly get offset in current sector to desired cluster number */
                ATA_SkipSectorBytes(dirdesc->currentDirClusterNum & 0x0000007F);

                ATA_READ_32BIT_VAL(dirdesc->currentDirClusterNum);
                dirdesc->currentDirSectorNum = 0;
                dirdesc->currentDirSectorOffset = 0;
            }
            else
            {
                /* read the next sector */
                dirdesc->currentDirSectorNum++;
                dirdesc->currentDirSectorOffset = 0;
            }
        }

        /* set disk reading position back to dir position after possibly
           being changed elsewhere */
        if((retval = ATA_SetSectorLBAForRead(CLUSTERNUMTOLBA(dirdesc->currentDirClusterNum) + dirdesc->currentDirSectorNum)) != ATA_OK)
        {
            return retval;
        }
        ATA_SkipSectorBytes(dirdesc->currentDirSectorOffset);

        /* 8.3 filename */
        for(filenameix = 0, filenamelen = 0; filenameix < DIRENT_FILENAME_LEN_BYTES; filenameix++)
        {
            if(filenameix == 8)
            {
                dirdesc->currentDirEntryName[filenamelen] = '.';
                filenamelen++;
            }

            dirdesc->currentDirEntryName[filenamelen] = ATA_ReadSectorByte();

            if(dirdesc->currentDirEntryName[filenamelen] != ' ')
            {
                filenamelen++;
            }
            else
            {
                if(filenameix == 8)
                {
                    /* first character of extension is a space, assume there is no extension */
                    dirdesc->currentDirEntryName[filenamelen-1] = 0;
                }
            }
        }
        dirdesc->currentDirEntryName[filenamelen] = 0;

        /* attrib */
        dirdesc->currentDirEntryAttrib = ATA_ReadSectorByte();

        /* cluster numbers */
        ATA_SkipSectorBytes(DIRENT_CLUSTER_HI_OFFSET_BYTES);
        ATA_READ_16BIT_VAL(dirdesc->currentDirEntryFirstCluster);
        dirdesc->currentDirEntryFirstCluster <<= 16;
        ATA_SkipSectorBytes(DIRENT_CLUSTER_LO_OFFSET_BYTES);
        ATA_READ_16BIT_VAL(templong);
        dirdesc->currentDirEntryFirstCluster |= (templong & 0x0000FFFF);

        ATA_READ_32BIT_VAL(dirdesc->currentDirEntrySizeBytes);

        dirdesc->currentDirSectorOffset += DIRENT_LENGTH_BYTES;
    }

    /* decide return code */
    if(dirdesc->currentDirEntryAttrib & DIRENT_ATTRIB_DIR)
    {
        return FAT32_DIRENTRY_IS_DIR;
    }
    else if(dirdesc->currentDirEntryName[0] == 0)
    {
        return FAT32_EODIRENTRYS;
    }
    else
    {
        return FAT32_OK;
    }
}

int8_t FAT32_FileOpen(FD *fd, DirDesc *dirdesc, const uint8_t *filename)
{
    int8_t retval = 0;

    while( (strncmp(filename, dirdesc->currentDirEntryName, SHORT_FILENAME_LENGTH_BYTES)) &&
           (retval != FAT32_EODIRENTRYS) )
    {
        retval = FAT32_DirLoadNextEntry(dirdesc);
    }

    switch(retval)
    {
    case FAT32_DIRENTRY_IS_DIR:
        return FAT32_DIRENTRY_IS_DIR;
    case FAT32_EODIRENTRYS:
        return FAT32_FILE_NOT_FOUND;
    case FAT32_OK:
        /* next dir record is ok, keep looping until the filename
           matches */
        break;
    default:
        break;
    }


    fd->firstCluster = dirdesc->currentDirEntryFirstCluster;
    fd->currentClusterNum = dirdesc->currentDirEntryFirstCluster;
    fd->currentSectorNum = 0;
    fd->currentSectorPos = 0;
    fd->sizeBytes = dirdesc->currentDirEntrySizeBytes;
    fd->position = 0;

    return FAT32_OK;
}

int8_t FAT32_FileRead(FD *fd, uint16_t numBytes, uint8_t *dataBuf)
{
    uint16_t i;
    uint32_t templong;
    int8_t retval;

    for(i = 0; i < numBytes; i++)
    {
        /* if LBA has been changed elsewhere set disk reading position
           back to current file position */
        if(ATA_CurrentSectorLBAAddr() != (CLUSTERNUMTOLBA(fd->currentClusterNum) + fd->currentSectorNum))
        {
            if((retval = ATA_SetSectorLBAForRead(CLUSTERNUMTOLBA(fd->currentClusterNum) + fd->currentSectorNum)) != ATA_OK)
            {
                return retval;
            }
            ATA_SkipSectorBytes(fd->currentSectorPos);
        }

        /* check if a new sector must be read */
        if(fd->currentSectorPos >= BPB_BYTES_PER_SECTOR)
        {
            /* then check if a new cluster must be read */
            if(fd->currentSectorNum >= SectorsPerCluster)
            {
                /* look up next cluster in FAT */
                if((retval = ATA_SetSectorLBAForRead(FatBeginLBA + (fd->currentClusterNum >> 7))) != ATA_OK)
                {
                    return retval;
                }
                ATA_SkipSectorBytes(fd->currentClusterNum & 0x0000007F);

                ATA_READ_32BIT_VAL(fd->currentClusterNum);

                if(fd->currentClusterNum == EOF_CLUSTER_MARKER)
                {
                    return FAT32_EOF;
                }

                fd->currentSectorNum = 0;
                fd->currentSectorPos = 0;
                if((retval = ATA_SetSectorLBAForRead(CLUSTERNUMTOLBA(fd->currentClusterNum))) != ATA_OK)
                {
                    return retval;
                }
            }
            else
            {
                /* read the next sector */
                fd->currentSectorNum++;
                fd->currentSectorPos = 0;
                if((retval = ATA_SetSectorLBAForRead(CLUSTERNUMTOLBA(fd->currentClusterNum) + fd->currentSectorNum)) != ATA_OK)
                {
                    return retval;
                }
            }

        }

        /* check for end of file according to size from dir entry */
        if(fd->position >= fd->sizeBytes)
        {
            return FAT32_EOF;
        }

        fd->currentSectorPos++;
        fd->position++;
        *dataBuf = ATA_ReadSectorByte();

        dataBuf++;
    }

    /* must have worked if we got this far */
    return FAT32_OK;
}

