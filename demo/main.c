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
    uint8_t sectorbyte;
    uint16_t byteix;

    InitComms();

    printf("\n\rPIC ATA interface test\n\r");

    printf("Init ATA: ");
    if((retval = ATA_Init(ATA_MASTER)) != ATA_OK)
    {
        printf("ATA error %d\n\r", retval);
    }
    else
    {
        printf("OK\n\r");
    }

    // DEBUG: dump the first sector
    printf("Reading sector 0: ");
    if((retval = ATA_SetSectorLBAForRead(0)) != ATA_OK)
    {
        printf("ATA error %d\n\r", retval);
    }
    else
    {
        printf("LBA set\n\r");
    }
    for(byteix = 0; byteix < 512; byteix++)
    {
        sectorbyte = ATA_ReadSectorByte();
        if(!(byteix % 16))
            printf("\n\r%03X: ", byteix);
        printf(" %02X", sectorbyte);
    }

#if defined(ATA_USE_ID)
    printf("\n\r\n\rRead drive info: ");
    if((retval = ATA_ReadDriveInfo()) != ATA_OK)
    {
        printf("ATA error %d\n\r", retval);
    }
    else
    {
        printf("\n\r\tDrive model: %s\n\r\tRevision: %s\n\r\tSerial #:%s\n\r\tCylinders: %u\n\r\tHeads: %u\n\r\tSectors: %u\n\r",
                ATA_GetInfoModel(),
                ATA_GetInfoRev(),
                ATA_GetInfoSerialNum(),
                ATA_GetInfoNumCyls(),
                ATA_GetInfoNumHeads(),
                ATA_GetInfoNumSectors());
    }
#endif

#ifdef FAT32_ENABLED
    printf("Mount FAT32: ");
    if((retval = FAT32_Mount(0)) != FAT32_OK)
    {
        printf("Error %d\n\r", retval);
    }
    else
    {
        printf("OK\n\r");

        printf("Root dir contents:\n\r");
        while((retval = FAT32_DirLoadNextEntry()) == FAT32_OK)
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
        if(retval != FAT32_EODIRENTRYS)
        {
            printf("Error %d loading next dir record\n\r", retval);
        }
    }
#endif
}
