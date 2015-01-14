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
    printf("\n\rReading sector 0: ");
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

    // DEBUG: dump the BPB of the FAT32 FS
    printf("\n\r\n\rReading sector 0x00004ABC: ");
    if((retval = ATA_SetSectorLBAForRead(0x00004ABC)) != ATA_OK)
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
    printf("\n\rMount FAT32: ");
    if((retval = FAT32_Mount(0)) != FAT32_OK)
    {
        printf("Error %d\n\r", retval);
    }
    else
    {
        FD testfd;
        uint8_t databyte;

        printf("OK\n\r");

        printf("Root dir contents:\n\r");
        retval = FAT32_DirLoadNextEntry();
        while((retval == FAT32_OK) || (retval == FAT32_DIRENTRY_IS_DIR))
        {
            printf("%-12s", FAT32_DirEntryName());
            if(retval == FAT32_DIRENTRY_IS_DIR)
            {
                printf("\t<dir>\n\r");
            }
            else
            {
                printf("\n\r");
            }
            retval = FAT32_DirLoadNextEntry();
        }
        if(retval != FAT32_EODIRENTRYS)
        {
            printf("Error %d loading next dir record\n\r", retval);
        }

        printf("\n\rRe-opening root dir: ");
        if((retval = FAT32_DirOpen("\\")) != FAT32_OK)
        {
            printf("Error %d\n\r", retval);
        }
        else
        {
            printf("OK\n\r");
        }

        printf("\n\rOpening file HELLO.TXT: ");
        if((retval = FAT32_FileOpen(&testfd, "HELLO.TXT")) != FAT32_OK)
        {
            printf("Error %d\n\r", retval);
        }
        else
        {
            printf("OK\n\r");
        }

        printf("\n\rPrinting file HELLO.TXT:\n\r");
        retval = FAT32_FileRead(&testfd, 1, &databyte);
        while(retval == FAT32_OK)
        {
            printf("%c", databyte);
            retval = FAT32_FileRead(&testfd, 1, &databyte);
        }
        printf("\n\r");
    }
#endif
}
