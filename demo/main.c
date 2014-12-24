/*-------------------------------------------------------------------
Name......... main.c
Author....... Ed Langley
Date......... 19/10/2006
Description.. Main code to operate an ATA interface and FAT32 FS on
............. a PIC MCU
-------------------------------------------------------------------*/

#include "platform.h"
#include <stdio.h>
#include "ata.h"
#include "serial.h"
#include "fat32.h"

void main(void)
{
    int8_t retval;

    InitComms();

    printf("PIC ATA interface test\n\r");

    printf("Init ATA: ");
    retval = ATA_Init(ATA_MASTER);
    switch(retval)
    {
    case ATA_OK:
        printf("ATA master initialised\n\r");
        break;
    case ATA_RDY_TIMEOUT:
        printf("ATA RDY timeout\n\r");
        break;
    case ATA_BSY_TIMEOUT:
        printf("ATA BSY timeout\n\r");
        break;
    case ATA_DRQ_TIMEOUT:
        printf("ATA DRQ timeout\n\r");
        break;
    }

#if defined(ATA_USE_ID)
    printf("Read drive info: ");
    retval = ATA_ReadDriveInfo();

    switch(retval)
    {
    case ATA_OK:
        printf("\n\r\tDrive model: %s\n\r\tRevision: %s\n\r\tSerial #:%s\n\r\tCylinders: %ud\n\r\tHeads: %ud\n\r\tSectors: %ud\n\r",
        ATA_GetInfoModel(),
        ATA_GetInfoRev(),
        ATA_GetInfoSerialNum(),
        ATA_GetInfoNumCyls(),
        ATA_GetInfoNumHeads(),
        ATA_GetInfoNumSectors());
        break;
    case ATA_BSY_TIMEOUT:
        printf("ATA BSY timeout\n\r");
        break;
    case ATA_DRQ_TIMEOUT:
        printf("ATA DRQ timeout\n\r");
        break;
    }
#endif

#ifdef FAT32_ENABLED
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
