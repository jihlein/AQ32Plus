/**
 ********************************************************************************
 **
 **  File        : FileUtils.c
 **
 **  Abstract    : Fonction permettant de manipuler des fichiers (Read/Write) avec la
 **  			   bibliothèque FatFs
 **
 **  Environment : Atollic TrueSTUDIO(R)
 **                STMicroelectronics STM32F4xx Standard Peripherals Library
 **
 **  Author 	 : Ronan Douguet
 **
 **  Date	     : 08-03-2012
 **
 ********************************************************************************
 */

/* Includes -------------------------------------------------------------------*/
#include "board.h"
// HJI #include <stdio.h>
// HJI #include "FileUtils.h"


/*-----------------------------------------------------------------------*/
/* @descr  die          						 						 */
/* @param  none															 */
/* @retval none					 										 */
/*-----------------------------------------------------------------------*/
void die(   /* Stop with dying message */
    FRESULT rc	/* FatFs return value */
)
{
    // HJI USART_SendString(USART3, "Failed with rc=%u.\n");
	  cliPrintF("Failed withrc = %u.\n");

    for (;;) ;
}


/*-----------------------------------------------------------------------*/
/* @descr  CREATE FILE   						 						 */
/* @param  none															 */
/* @retval none					 										 */
/*-----------------------------------------------------------------------*/
void CreateFile(void)
{
    /* varibles pour tester l'ecriture en FAT32 */
    FRESULT rc;				/* Result code */
    FIL Fil;			/* File object */
    FATFS Fatfs;		/* File system object */

    /* Register volume work area (never fails) */
    f_mount(0, &Fatfs);

    //DSTATUS driveStatus = disk_initialize(0);

    //if(driveStatus & STA_NOINIT || driveStatus & STA_NODISK || driveStatus & STA_PROTECT)
    //{
    //flag error.
    //}

    /*  Sometimes you may want to format the disk */
    //if(f_mkfs(0,0,0)!=FR_OK)
    //{
    //error
    //}

    /* Creation du fichier Datalog.txt */
    rc = f_open(&Fil, "Datalog.TXT", FA_WRITE | FA_CREATE_ALWAYS);

    if (rc) die(rc);

    /* Ecriture de la Date,Heure,AN0 et AN1 dans le fichier */
    f_printf(&Fil, "Date\tTime\tSecondFraction\tMillisecondes\tDixSecondes\r\n");

    /* Fermeture du fichier Fil */
    rc = f_close(&Fil);

    if (rc) die(rc);

    /* Unregister work area prior to discard it */
    f_mount(0, NULL);
}

/*-----------------------------------------------------------------------*/
/* @descr  WRITE FILE   						 						 */
/* @param  none															 */
/* @retval none					 										 */
/*-----------------------------------------------------------------------*/
void WriteFile(void)
{
    RTC_TimeTypeDef RTC_TimeStructure;
    RTC_DateTypeDef RTC_DateStructure;
    uint32_t sec_frac;

    float time_ms;
    char time_ms_ascii[8];
    char time_dms_ascii[8];


    /* varibles pour tester l'ecriture en FAT32 */
    FRESULT rc;				/* Result code */
    FIL Fil;			/* File object */
    FATFS Fatfs;		/* File system object */

    /* Register volume work area (never fails) */
    f_mount(0, &Fatfs);

    /* Creation du fichier Datalog.txt */
    rc = f_open(&Fil, "Datalog.TXT", FA_WRITE);

    if (rc) die(rc);

    /* Move to end of the file to append data */
    rc = f_lseek(&Fil, f_size(&Fil));

    /* Ecriture de la Date,Heure,AN0 et AN1 dans le fichier */
    sec_frac = RTC_GetSubSecond();
    time_ms = (249.00 - sec_frac) / 250;
    sprintf(time_ms_ascii,"%f", time_ms);
    sprintf(time_dms_ascii,"%.1f", time_ms);
    // HJI sprintf(time_dms_ascii, "%d", (float)0.2);
    // HJI sprintf(time_dms_ascii, "%d", (float)12.33);
    RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);
    RTC_GetDate(RTC_Format_BIN, &RTC_DateStructure);

    f_printf(&Fil, "%02d\\%02d\\%02d\t%02d:%02d:%02d\t%Lu\t%s\t%s\r\n", RTC_DateStructure.RTC_Date, RTC_DateStructure.RTC_Month, RTC_DateStructure.RTC_Year, RTC_TimeStructure.RTC_Hours, RTC_TimeStructure.RTC_Minutes, RTC_TimeStructure.RTC_Seconds, sec_frac, time_ms_ascii, time_dms_ascii);

    /* Fermeture du fichier Fil */
    rc = f_close(&Fil);

    if (rc) die(rc);

    /* Unregister work area prior to discard it */
    f_mount(0, NULL);
}

/*-----------------------------------------------------------------------*/
/* @descr  READ FILE   						 						 */
/* @param  none															 */
/* @retval none					 										 */
/*-----------------------------------------------------------------------*/
void ReadFile(void)
{
    /* varibles pour tester l'ecriture en FAT32 */
    FRESULT rc;				/* Result code */
    FIL Fil;			/* File object */
    FATFS Fatfs;		/* File system object */
    char ligne[8];

    /* Register volume work area (never fails) */
    f_mount(0, &Fatfs);

    /* Creation du fichier Datalog.txt */
    rc = f_open(&Fil, "HEADING.cal", FA_OPEN_EXISTING | FA_READ);

    if (rc) die(rc);

    /* Ecriture de la Date,Heure,AN0 et AN1 dans le fichier */
    while (f_gets(ligne, 8, &Fil) != NULL)
    {
        //USART_SendString(USART3,ligne);
    }

    /* Fermeture du fichier Fil */
    rc = f_close(&Fil);

    /* Unregister work area prior to discard it */
    f_mount(0, NULL);
}
