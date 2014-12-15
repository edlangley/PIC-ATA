/*-------------------------------------------------------------------
Name......... main.c
Author....... Ed Langley
Date......... 19/10/2006
Description.. Main code to operate an ATA interface and FAT32 FS on
............. a PIC MCU
-------------------------------------------------------------------*/

#include "platform.h"
#include <stdio.h>
#include "types.h"
#include "ata.h"
#include "serial.h"
#include "fat32.h"


void main(void)
{
  int8_t retval;

  InitComms();
  printf("PIC ATA interface test\n\r");

  ATA_Init(ATA_MASTER);
  retval = ATA_ReadDriveInfo();


#if defined(ATA_USE_ID)
  switch(retval)
    {
    case ATA_OK:
      printf("Drive model: %s\n\rRevision: %s\n\rSerial #:%s\n\rCylinders: %ud\n\rHeads: %ud\n\rSectors: %ud\n\r",
              ATA_GetInfoModel(),
              ATA_GetInfoRev(),
              ATA_GetInfoSerialNum(),
              ATA_GetInfoNumCyls(),
              ATA_GetInfoNumHeads(),
              ATA_GetInfoNumSectors());
      break;
    case ATA_BSY_TIMEOUT:
      printf("ATA timed out on BSY\n\r");
      break;
    case ATA_DRQ_TIMEOUT:
      printf("ATA timed out on DRQ\n\r");
      break;
    }
#endif



#if 0
  if(FAT32_Mount(0) == FAT32_OK)
    {
      printf("FAT32 filesystem mounted succesfully\n\r");
      while((retval = FAT32_DirLoadNextEntry()) != FAT32_EODIRENTRYS)
	{
	  printf("%s", FAT32_DirEntryName());
	  if(retval == FAT32_DIRENTRY_IS_DIR)
	    {
	      printf("\t<dir>\n\r");
	    }
	  else
	    {
	      printf("\n\r");
	    }
	}
      printf("End of directory records\n\r");
    }
    else
    {
      printf("Error mounting FAT32 filesystem\n\r");
    }
#endif



}
