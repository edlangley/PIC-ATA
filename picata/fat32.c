/*-------------------------------------------------------------------
Name......... fat32.c
Author....... Ed Langley
Date......... 29/11/2006
Description.. Driver to read a FAT32 FS over an ATA interface on a
............. PIC MCU with minimal RAM usage
-------------------------------------------------------------------*/

#include "platform.h"
//#include <stdlib.h>
#include <string.h>
#include "types.h"
#include "ata.h"
#include "fat32.h"

uint32_t FatBeginLBA; //= Partition_LBA_Begin + Number_of_Reserved_Sectors;
uint32_t ClusterBeginLBA; //= Partition_LBA_Begin + Number_of_Reserved_Sectors + (Number_of_FATs * Sectors_Per_FAT);
uint8_t SectorsPerCluster; //= BPB_SecPerClus;
uint32_t RootDirFirstCluster; //= BPB_RootClus;

uint8_t CurrentDirEntryName[SHORT_FILENAME_LENGTH_BYTES+1];
//uint8_t CurrentDirEntryName[1];

uint8_t CurrentDirEntryAttrib;
uint32_t CurrentDirEntryFirstCluster;
uint32_t CurrentDirEntrySizeBytes;
uint32_t CurrentDirEntryNum;       /* In the current directory */

uint32_t CurrentDirClusterNum;
uint32_t CurrentDirSectorNum; /* within the current cluster */
uint32_t CurrentDirFirstCluster;


/* API functions */
int8_t FAT32_Mount(uint8_t drvnum)
{
	int8_t retval;
	uint32_t templong;//=0;

	uint16_t partitionsig;
	uint32_t partitionlbabegin;

//	uint8_t bpbsecsperclus;
	uint16_t bpbrsvdseccnt;
	uint8_t bpbnumfats;
	uint32_t bpbsecsperfat;


	/* validate MBR */
	if((retval = ATA_SetLBAForRead(0)) != ATA_OK)
	{
		return retval;
	}

	ATA_SkipWords(MBR_SIG_OFFSET_WORDS);

	if(ATA_ReadWord() != MBR_SIG)
	{
		return FAT32_INVALID_MBR;
	}

	/* MBR OK */
	if((retval = ATA_SetLBAForRead(0)) != ATA_OK)
	{
		return retval;
	}

	ATA_SkipWords(MBR_PART_OFFSET_WORDS + PART_TYPE_OFFSET_WORDS);

	partitionsig = ATA_ReadWord();
	//partitionsig >>= 8;
	if( (partitionsig != MBR_FAT32_PART_SIG1) && (partitionsig != MBR_FAT32_PART_SIG2) )
	{
		/* for now assume first partition is FAT32 */
		return FAT32_PART_NOT_FOUND;
	}

	/* move forward one word to the LBA begin field */
	ATA_SkipWords(1);

	//partitionlbabegin = (uint32_t)((ATA_ReadWord() << 16) | ATA_ReadWord());
	partitionlbabegin = (uint32_t)ATA_ReadWord();
	templong = (uint32_t)ATA_ReadWord();
	partitionlbabegin |= (templong << 16);

	/* now move on to the volume ID sector or BPB of the FAT32 partition */
	if((retval = ATA_SetLBAForRead(partitionlbabegin)) != ATA_OK)
	{
		return retval;
	}

	ATA_SkipWords(BPB_SEC_PER_CLUS_OFFSET_WORDS);

	SectorsPerCluster = (uint8_t)ATA_ReadWord();

	bpbrsvdseccnt = ATA_ReadWord();

	bpbnumfats = (uint8_t)ATA_ReadWord();

	ATA_SkipWords(BPB_SEC_PER_FAT_OFFSET_WORDS);

	//bpbsecsperfat = (uint32_t)((ATA_ReadWord() << 16) | ATA_ReadWord());
	bpbsecsperfat = (uint32_t)(ATA_ReadWord());
	templong = (uint32_t)ATA_ReadWord();
	bpbsecsperfat |= (templong << 16);

	ATA_SkipWords(BPB_ROOT_DIR_CLUSTER_OFFSET_WORDS);

	//RootDirFirstCluster = (uint32_t)((ATA_ReadWord() << 16) | ATA_ReadWord());
	RootDirFirstCluster = (uint32_t)ATA_ReadWord();
	templong = (uint32_t)ATA_ReadWord();
	RootDirFirstCluster |= (templong << 16);

	FatBeginLBA = partitionlbabegin + bpbrsvdseccnt;
	ClusterBeginLBA = partitionlbabegin + bpbrsvdseccnt + (bpbnumfats * bpbsecsperfat);

	/* set up the root dir .... */
	ATA_SetLBAForRead(CLUSTERNUMTOLBA(RootDirFirstCluster));
	CurrentDirFirstCluster = RootDirFirstCluster;
	CurrentDirEntryNum = 0;
	CurrentDirSectorNum = 0;
	CurrentDirClusterNum = CurrentDirFirstCluster;

	/* terminate the filename string */
	CurrentDirEntryName[SHORT_FILENAME_LENGTH_BYTES] = 0;

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
  uint16_t wordread;
  uint8_t i, j;
  uint32_t templong;

  /* set disk reading position back to file position after being changed
     elsewhere */
  ATA_SetLBAForRead(CLUSTERNUMTOLBA(CurrentDirClusterNum) + CurrentDirSectorNum);
  ATA_SkipWords(DIR_REC_LENGTH_WORDS * CurrentDirEntryNum);

  CurrentDirEntryName[0] = DIRENTRY_NAMECHAR_UNUSED;
  while( (CurrentDirEntryName[0] == DIRENTRY_NAMECHAR_UNUSED) ||
	 (CurrentDirEntryAttrib & DIRENTRY_ATTRIB_LFN) ||
	 (CurrentDirEntryName[0] != 0) )
    {
      /* check if a new sector must be read */
      if( (CurrentDirEntryNum * DIR_REC_LENGTH_BYTES) >= BPB_BYTES_PER_SECTOR)
	{
	  /* check if a new cluster must be read */
	  if(CurrentDirSectorNum >= SectorsPerCluster)
	    {
	      /* look up next cluster in FAT */
	      ATA_SetLBAForRead(FatBeginLBA + (CurrentDirClusterNum >> 7) );
	      ATA_SkipWords(CurrentDirClusterNum & 0x0000007F);


	      CurrentDirClusterNum = (uint32_t)ATA_ReadWord();
	      templong = (uint32_t)ATA_ReadWord();
	      CurrentDirClusterNum |= (templong << 16);

	      CurrentDirSectorNum = 0;
	      ATA_SetLBAForRead(CLUSTERNUMTOLBA(CurrentDirClusterNum));
	    }
	  else
	    {
	      /* read the next sector */

	      CurrentDirSectorNum++;
	      ATA_SetLBAForRead(CLUSTERNUMTOLBA(CurrentDirClusterNum) + CurrentDirSectorNum);

	    }


	}

      for(i = 0, j = 0; i < SHORT_FILENAME_LENGTH_WORDS; i++)
	{
	  wordread = ATA_ReadWord();

	  //CurrentDirEntryName[j++] = (uint8_t)((wordread >> 8) & 0x00FF);
	  //CurrentDirEntryName[j++] = (uint8_t)(wordread & 0x00FF);
	  CurrentDirEntryName[j++] = (uint8_t)(wordread & 0x00FF);
	  CurrentDirEntryName[j++] = (uint8_t)((wordread >> 8) & 0x00FF);
	}

      /* last byte of name and attrib */
      wordread = ATA_ReadWord();
      //CurrentDirEntryName[j] = (uint8_t)((wordread >> 8) & 0x00FF);
      //CurrentDirEntryAttrib = (uint8_t)(wordread & 0x00FF);
      CurrentDirEntryName[j] = (uint8_t)(wordread & 0x00FF);
      CurrentDirEntryAttrib = (uint8_t)((wordread >> 8) & 0x00FF);

      /* read cluster numbers */
      ATA_SkipWords(DIR_REC_CLUSTER_HI_OFFSET_WORDS);
      CurrentDirEntryFirstCluster = 0;
      CurrentDirEntryFirstCluster = ATA_ReadWord();
      ATA_SkipWords(DIR_REC_CLUSTER_LO_OFFSET_WORDS);

      templong = (uint32_t)ATA_ReadWord();
      CurrentDirEntryFirstCluster |= (templong << 16);

      CurrentDirEntrySizeBytes = ATA_ReadWord();
      templong = (uint32_t)ATA_ReadWord();
      CurrentDirEntrySizeBytes = (templong << 16);

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
  //int8_t direntryretval;
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
  //fd->currentclusterbytesleft = (SectorsPerCluster * BPB_BYTES_PER_SECTOR);
  fd->currentsectornum = 0;
  fd->currentsectorpos = 0;
  fd->sizebytes = CurrentDirEntrySizeBytes;
  fd->position = 0;
  fd->evenbyte = TRUE;

  return FAT32_OK;
}

int8_t FAT32_FileRead(FD *fd, uint16_t numBytes, uint8_t *dataBuf)
{
  int i;
  uint32_t templong;

  for(i = 0; i < numBytes; i++)
  {
      /* If LBA has been changed elsewhere set disk reading position
         back to current file position */
      if(ATA_CurrentLBAAddr() != (CLUSTERNUMTOLBA(fd->currentclusternum) + fd->currentsectornum))
        {
          ATA_SetLBAForRead(CLUSTERNUMTOLBA(fd->currentclusternum) + fd->currentsectornum);
          ATA_SkipWords(fd->currentsectorpos);
        }

      if(fd->evenbyte)
        {
          /* need to read next word from disk */
          /* check if a new sector must be read */
          if(fd->currentsectorpos >= (BPB_BYTES_PER_SECTOR/2))
        {
          /* then check if a new cluster must be read */
          if(fd->currentsectornum >= SectorsPerCluster)
            {
              /* look up next cluster in FAT */
              ATA_SetLBAForRead(FatBeginLBA + (fd->currentclusternum >> 7) );
              ATA_SkipWords(fd->currentclusternum & 0x0000007F);

              fd->currentclusternum = (uint32_t)ATA_ReadWord();
              templong = (uint32_t)ATA_ReadWord();
              fd->currentclusternum |= (templong << 16);

              if(fd->currentclusternum == EOF_CLUSTER_MARKER)
            {
              return FAT32_EOF;
            }

              fd->currentsectornum = 0;
              fd->currentsectorpos = 0;
              ATA_SetLBAForRead(CLUSTERNUMTOLBA(fd->currentclusternum));
            }
          else
            {


              /* read the next sector */
              fd->currentsectornum++;
              fd->currentsectorpos = 0;
              ATA_SetLBAForRead(CLUSTERNUMTOLBA(fd->currentclusternum) + fd->currentsectornum);
            }

        }


          /* check for end of file according to size from dir entry */
          if(fd->position >= fd->sizebytes)
        {
          return FAT32_EOF;
        }

          fd->currentword = ATA_ReadWord();
          fd->currentsectorpos++;
          fd->position++;
          *dataBuf = (int8_t)(fd->currentword >> 8);
          fd->evenbyte = FALSE;
        }
      else
        {
          /* byte is top half of word read from disk last time, so no access required */

          /* check for end of file according to size from dir entry */
          if(fd->position >= fd->sizebytes)
        {
          return FAT32_EOF;
        }

          fd->position++;
          *dataBuf = (int8_t)(fd->currentword & 0x00FF);
          fd->evenbyte = TRUE;
        }

      dataBuf++;
  }

  /* must have worked if we got this far */
  return FAT32_OK;
}

