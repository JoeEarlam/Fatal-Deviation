#ifndef IBUS_H
#define IBUS_H

/*
 * Ibus decoder state machine borrowed from https://gitlab.com/timwilkinson/FlySkyIBus
 * Adapted for better asychronous use
 */

#include <Arduino.h>

#define IBUS_CHANNELS 14   //supports 14 channels regardless of TX and RX
#define IBUS_LENGTH 0x20   //packet is <len><cmd><data....><chkl><chkh>
#define IBUS_OVERHEAD 3    //overhead = cmd + chk bytes
#define IBUS_TIMEGAP 7     //time between packets in milliseconds
#define IBUS_COMMAND 0x40  //Command is always 0x40

class FlySkyIBus {
private:
  enum State_t {
    GET_LENGTH,
    GET_DATA,
    GET_CHKSUML,
    GET_CHKSUMH,
    DISCARD,
  };

  Stream* stream;
  uint16_t channelData[IBUS_CHANNELS];
  bool newData;

  void setData(uint8_t*);
  bool parseMsg(uint8_t*);



public:
  FlySkyIBus();
  void begin(Stream& stream);
  void loop(void);
  uint16_t readChannel(uint8_t channelNr);
  bool readChannels(uint16_t (&channelOut)[IBUS_CHANNELS]);
};

#endif
