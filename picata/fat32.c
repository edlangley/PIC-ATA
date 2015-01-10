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

uint32_t FatBeginLBA;
uint32_t ClusterBeginLBA;
uint8_t SectorsPerCluster;
uint32_t RootDirFirstCluster;

uint8_t CurrentDirEntryName[SHORT_FILENAME_LENGTH_BYTES+1];

uint8_t CurrentDirEntryAttrib;
uint32_t CurrentDirEntryFirstCluster;
uint32_t CurrentDirEntrySizeBytes;
uint32_t CurrentDirEntryNum;    /* In the current directory */

uint32_t CurrentDirClusterNum;
uint32_t CurrentDirSectorNum;   /* within the current cluster */
uint32_t CurrentDirFirstCluster;


/* API functions */
int8_t FAT32_Mount(uint8_t drvnum)
{
    int8_t retval;
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

    /* set up the root dir .... */
    /*
    if((retval = ATA_SetSectorLBAForRead(CLUSTERNUMTOLBA(RootDirFirstCluster))) != ATA_OK)
    {
        return retval;
    }
    */
    CurrentDirFirstCluster = RootDirFirstCluster;
    CurrentDirEntryNum = 0;
    CurrentDirSectorNum = 0;
    CurrentDirClusterNum = CurrentDirFirstCluster;

    /* terminate the filename string */
    CurrentDirEntryName[0] = '\\';
    CurrentDirEntryName[1] = 0;

    return(FAT32_OK);
}

int8_t FAT32_DirOpen(uint8_t *dirname)
{
    int8_t retval = 0;

    /* check for current dir name (.) to reset to start of dir records */
    if(!strncmp(dirname, ".", 1))
    {
        CurrentDirEntryNum = 0;
        CurrentDirSectorNum = 0;
        CurrentDirClusterNum = CurrentDirFirstCluster;
        return FAT32_OK;
    }

    /* check for root dir name (\\) to reset to start of root dir records */
    if(!strncmp(dirname, "\\", 1))
    {
        CurrentDirEntryNum = 0;
        CurrentDirSectorNum = 0;
        CurrentDirClusterNum = RootDirFirstCluster;
        return FAT32_OK;
    }

    while( (strncmp(dirname, CurrentDirEntryName, SHORT_FILENAME_LENGTH_BYTES)) &&
           (retval != FAT32_EODIRENTRYS) )
    {
        retval = FAT32_DirLoadNextEntry();
    }

    switch(retval)
    {
    case FAT32_DIRENTRY_IS_DIR:
        /* set to start of dir records for the dir found */
        CurrentDirEntryNum = 0;
        CurrentDirSectorNum = 0;
        CurrentDirClusterNum = CurrentDirEntryFirstCluster;
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

int8_t FAT32_DirLoadNextEntry()
{
    uint8_t i, j;
    int8_t retval;
    uint32_t templong;

    /* set disk reading position back to dir position after being changed
       elsewhere */
    if((retval = ATA_SetSectorLBAForRead(CLUSTERNUMTOLBA(CurrentDirClusterNum) + CurrentDirSectorNum)) != ATA_OK)
    {
        return retval;
    }

    ATA_SkipSectorBytes(DIR_REC_LENGTH_BYTES * CurrentDirEntryNum);

    CurrentDirEntryName[0] = DIRENTRY_NAMECHAR_UNUSED;
    while( (CurrentDirEntryName[0] == DIRENTRY_NAMECHAR_UNUSED) ||
           (CurrentDirEntryAttrib & DIRENTRY_ATTRIB_LFN) ||
           (CurrentDirEntryName[0] != 0) )
    {
        /* check if a new sector must be read */
        if( (CurrentDirEntryNum * DIR_REC_LENGTH_BYTES) >= (CurrentDirSectorNum * BPB_BYTES_PER_SECTOR))
        {
            /* check if a new cluster must be read */
            if(CurrentDirSectorNum >= SectorsPerCluster)
            {
                /* look up next cluster in FAT:
                 * - shift down 7 because cluster record size is 4 bytes, so 512/4=128 = 0x80,
                 * so >>7 is quick way of converting cluster record number to sector containing it */

                if((retval = ATA_SetSectorLBAForRead(FatBeginLBA + (CurrentDirClusterNum >> 7))) != ATA_OK)
                {
                    return retval;
                }
                /* again quickly get offset in current sector to desired cluster record */
                ATA_SkipSectorBytes(CurrentDirClusterNum & 0x0000007F);

                ATA_READ_32BIT_VAL(CurrentDirClusterNum);

                CurrentDirSectorNum = 0;
                if((retval = ATA_SetSectorLBAForRead(CLUSTERNUMTOLBA(CurrentDirClusterNum))) != ATA_OK)
                {
                    return retval;
                }
            }
            else
            {
                /* read the next sector */
                CurrentDirSectorNum++;
                if((retval = ATA_SetSectorLBAForRead(CLUSTERNUMTOLBA(CurrentDirClusterNum) + CurrentDirSectorNum)) != ATA_OK)
                {
                    return retval;
                }
            }
        }

        for(i = 0; i < SHORT_FILENAME_LENGTH_BYTES; i++)
        {
            CurrentDirEntryName[i] = ATA_ReadSectorByte();
        }

        /* attrib */
        CurrentDirEntryAttrib = ATA_ReadSectorByte();

        /* read cluster numbers */
        /*
        ATA_SkipWords(DIR_REC_CLUSTER_HI_OFFSET_WORDS);
        CurrentDirEntryFirstCluster = 0;
        CurrentDirEntryFirstCluster = ATA_ReadWord();
        ATA_SkipWords(DIR_REC_CLUSTER_LO_OFFSET_WORDS);

        templong = (uint32_t)ATA_ReadWord();
        CurrentDirEntryFirstCluster |= (templong << 16);
        */
        ATA_SkipSectorBytes(DIR_REC_CLUSTER_HI_OFFSET_BYTES);
        ATA_READ_32BIT_VAL(CurrentDirEntryFirstCluster);

        ATA_READ_32BIT_VAL(CurrentDirEntrySizeBytes);

        CurrentDirEntryNum++;
    }

    /* decide return code */
    if(CurrentDirEntryAttrib & DIRENTRY_ATTRIB_DIR)
    {
        return FAT32_DIRENTRY_IS_DIR;
    }
    else if(CurrentDirEntryName[0] == 0)
    {
        return FAT32_EODIRENTRYS;
    }
    else
    {
        return FAT32_OK;
    }
}

uint8_t *FAT32_DirEntryName()
{
    return CurrentDirEntryName;
}

int8_t FAT32_FileOpen(FD *fd, uint8_t *filename)
{
    int8_t retval = 0;

    while( (strncmp(filename, CurrentDirEntryName, SHORT_FILENAME_LENGTH_BYTES)) &&
           (retval != FAT32_EODIRENTRYS) )
    {
        retval = FAT32_DirLoadNextEntry();
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


    fd->firstcluster = CurrentDirEntryFirstCluster;
    fd->currentclusternum = CurrentDirEntryFirstCluster;
    fd->currentsectornum = 0;
    fd->currentsectorpos = 0;
    fd->sizebytes = CurrentDirEntrySizeBytes;
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
        if(ATA_CurrentSectorLBAAddr() != (CLUSTERNUMTOLBA(fd->currentclusternum) + fd->currentsectornum))
        {
            if((retval = ATA_SetSectorLBAForRead(CLUSTERNUMTOLBA(fd->currentclusternum) + fd->currentsectornum)) != ATA_OK)
            {
                return retval;
            }
            ATA_SkipSectorBytes(fd->currentsectorpos);
        }

        /* check if a new sector must be read */
        if(fd->currentsectorpos >= BPB_BYTES_PER_SECTOR)
        {
            /* then check if a new cluster must be read */
            if(fd->currentsectornum >= SectorsPerCluster)
            {
                /* look up next cluster in FAT */
                if((retval = ATA_SetSectorLBAForRead(FatBeginLBA + (fd->currentclusternum >> 7))) != ATA_OK)
                {
                    return retval;
                }
                ATA_SkipSectorBytes(fd->currentclusternum & 0x0000007F);

                ATA_READ_32BIT_VAL(fd->currentclusternum);

                if(fd->currentclusternum == EOF_CLUSTER_MARKER)
                {
                    return FAT32_EOF;
                }

                fd->currentsectornum = 0;
                fd->currentsectorpos = 0;
                if((retval = ATA_SetSectorLBAForRead(CLUSTERNUMTOLBA(fd->currentclusternum))) != ATA_OK)
                {
                    return retval;
                }
            }
            else
            {
                /* read the next sector */
                fd->currentsectornum++;
                fd->currentsectorpos = 0;
                if((retval = ATA_SetSectorLBAForRead(CLUSTERNUMTOLBA(fd->currentclusternum) + fd->currentsectornum)) != ATA_OK)
                {
                    return retval;
                }
            }

        }

        /* check for end of file according to size from dir entry */
        if(fd->position >= fd->sizebytes)
        {
            return FAT32_EOF;
        }

        fd->currentsectorpos++;
        fd->position++;
        *dataBuf = ATA_ReadSectorByte();

        dataBuf++;
    }

    /* must have worked if we got this far */
    return FAT32_OK;
}

