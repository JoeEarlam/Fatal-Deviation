#include "ibus.h"

FlySkyIBus::FlySkyIBus() {
}

void FlySkyIBus::begin(Stream& stream) {
  this->stream = &stream;
}

void FlySkyIBus::setData(uint8_t *buffer){
    uint8_t i;

    for (i=1; i<IBUS_CHANNELS+1; i++) {
       channelData [i-1] = buffer[i * 2] + (buffer[i * 2 + 1] << 8);
    }
    newData = true;
}

bool FlySkyIBus::parseMsg(uint8_t *buffer){
    uint8_t i;
    uint16_t checksum = 0xFFFF;
    uint16_t rxChecksum = 0x0000;

    for (i=0; i<30; i++) {
        checksum -= buffer[i];
    }

    rxChecksum = buffer[30] + (buffer[31] << 8);

    if (rxChecksum == checksum) {
        return true;
    } else {
        return false;
    }
}


void FlySkyIBus::loop(void) {
    if (stream->available() >= 32) {
        if (stream->read() == 0x20) { // Start bit of frame
            uint8_t buffer[32];
            buffer[0] = 0x20;

            uint8_t i;
            for (i=1; i<32; i++) {
                buffer[i] = stream->read();
            }


            if (parseMsg(buffer) == true) {
                setData(buffer);
            }
        }
    }
}

uint16_t FlySkyIBus::readChannel(uint8_t channelNr) {
  if (channelNr < IBUS_CHANNELS) {
    return channelData[channelNr];
  } else {
    return 0;
  }
}

bool FlySkyIBus::readChannels(uint16_t (&channelOut)[IBUS_CHANNELS]) {
  if (newData) {
    newData = false;
    memcpy(channelOut, channelData, IBUS_CHANNELS * 2);
    return true;
  } else return false;
}
