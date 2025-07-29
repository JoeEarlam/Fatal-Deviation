#ifndef BOT_SOFTWARE_H
#define BOT_SOFTWARE_H

#include "defines.h"
#include "types.h"
#include "ibus.h"

//scheduler
void mainTaskCB();
void rgbCB();
void writePWMCB();
void readAnalogueCB();
void readRadioCB();
void debugSerialCB();
void readImuCB();
void weaponCB();
void inputHandlerCB();

//non-scheduler
Radio_t parseChannels(uint16_t (&chanData)[IBUS_CHANNELS]);

#endif
