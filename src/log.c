///////////////////////////////////////////////////////////////////////////////

#include "board.h"

///////////////////////////////////////////////////////////////////////////////

static uint16_t sd_card_available = 0;

static FATFS FATFS_Obj;

static FIL file;

///////////////////////////////////////////////////////////////////////////////

void logInit(void)
{
    int result = disk_initialize(0);

    if (result == 0)
    {
        f_mount(0, &FATFS_Obj);

        char filename[200];
        {
            int filecounter = 0;
            FILINFO filetest;
            sprintf(filename, "0:log%05u.csv", filecounter);

            while (f_stat(filename, &filetest) == FR_OK)
            {
                filecounter++;
                sprintf(filename, "0:log%05u.csv", filecounter);
            }
        }

        int result = f_open(&file, filename, FA_CREATE_NEW | FA_WRITE);

        if (result != 0)
        {
            cliPrintF("SD failed at f_open\n");
            sd_card_available = 0;
            return;
        }
        else
        {
        	cliPrintF("SD succeed - open file\n");
        }

        result = f_sync(&file);

        if (result != 0)
        {
            cliPrintF("SD failed at f_sync\n");
            sd_card_available = 0;
            return;
        }
        else
        {
        	cliPrintF("SD succeed - sync file\n");
        }

        result = f_lseek(&file, file.fsize);

        if (result != 0)
        {
            cliPrintF("SD failed at f_lseek");
            sd_card_available = 0;
            return;
        }
        else
        {
        	cliPrintF("SD succeed - lseek file\n");
        }

        cliPrintF("SD success filename: %s\n", filename);
        sd_card_available = 1;
    }
}

///////////////////////////////////////////////////////////////////////////////

void writeToFile(const char *fname, uint8_t *buffer, uint32_t length)
{
    if (sd_card_available == 0)
    {
        return;
    }

    static FIL file2;

    char filename[200];
    sprintf(filename, "0:%s", fname);
    int result = f_open(&file2, filename, FA_CREATE_ALWAYS | FA_WRITE);

    if (result != 0)
    {
        cliPrintF("SD failed at f_open:write\n");
        sd_card_available = 0;
        return;
    }

    unsigned int bw = 0;

    result = f_write(&file2, buffer, length, &bw);

    if (result != 0)
    {
        cliPrintF("SD failed at f_write:write\n");
        sd_card_available = 0;
        return;
    }

    if (bw != length)
    {
        cliPrintF("SD failed due to length mismatch:write\n");
        sd_card_available = 0;
        return;
    }

    result = f_close(&file2);

    if (result != 0)
    {
        cliPrintF("SD failed at f_close:write\n");
        sd_card_available = 0;
        return;
    }
}

///////////////////////////////////////////////////////////////////////////////

void logPrintF(const char *text, ...)
{
    char tmp[500];
    va_list args;
    va_start(args, text);
    vsnprintf(tmp, 500, text, args);
    va_end(args);

    if (sd_card_available == 0)
    {
        return;
    }

    char line[500];

    uint32_t mmillis = millis();

    uint32_t seconds = mmillis / 1000;

    uint32_t fract   = mmillis - (seconds * 1000);

    snprintf(line, 500, "%5lu.%03lu, %s", seconds, fract, tmp);

    unsigned int len = strlen(line);

    unsigned int bw = 0;

    unsigned int result = f_write(&file, line, len, &bw);

    if (result != 0)
    {
        cliPrintF("SD failed at f_write:logprintf\n");
        sd_card_available = 0;
        return;
    }

    if (bw != len)
    {
        cliPrintF("SD failed due to length mismatch:logprintf\n");
        sd_card_available = 0;
        return;
    }
}

///////////////////////////////////////////////////////////////////////////////

void logSync(void)
{
    if (sd_card_available == 0)
    {
        return;
    }

    unsigned int result = f_sync(&file);


    if (result != 0)
    {
        cliPrintF("SD failed at f_sync:log_sync\n");
        sd_card_available = 0;
        return;
    }
}

///////////////////////////////////////////////////////////////////////////////


