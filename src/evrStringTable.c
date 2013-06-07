/**
  \brief      EVR (EVent Report) strings tables.
  \remark     Ported for AQ32Plus.
*/

#include "board.h"

constStrArr_t evrInfo = {
    "None",
    "Normal Reset",
    "Starting Main Loop"
};

constStrArr_t evrWarn = {
    "Abnormal Reset",
    "Battery Low",
    "Battery Very Low",
    "Config has CRC Bad History flag set! Use CLI to clear",
    "SD Card failed or not installed",
};

constStrArr_t evrError = {
    "Out of EVR Listener slots",
    "Failed to register Frame Reset Watchdog",
    "Failed to register Frame Lost Watchdog",
    "Receive pilot command frame timeout!",
    "Battery dangerously Low!",
    "Flash CRC failed! Bad History Set",
    "Flash erase failed",
    "Flash programming failed"
};

constStrArr_t evrFatal = {
    "",
};

const evrStringTable_t evrStringTable[evrTypesNUM] = {
    { "Information:", evrInfo,  sizeof(evrInfo)/sizeof(char*)  },
    { "Warning:    ", evrWarn,  sizeof(evrWarn)/sizeof(char*)  },
    { "Error:      ", evrError, sizeof(evrError)/sizeof(char*) },
    { "Fatal:      ", evrFatal, sizeof(evrFatal)/sizeof(char*) }
};
