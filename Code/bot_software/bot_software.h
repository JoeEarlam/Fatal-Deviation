#ifndef BOT_SOFTWARE_H
#define BOT_SOFTWARE_H

#include "defines.h"
#include "types.h"
#include "src/ibus.h"

//scheduler
void rgbCB();
void writePWMCB();
void readAnalogueCB();
void readRadioCB();
void driveMathsCB();
void debugSerialCB();
void readImuCB();

//non-scheduler
Radio_t parseChannels(uint16_t (&chanData)[IBUS_CHANNELS]);

#endif
