#ifndef SERIALBUF_H
#define SERIALBUF_H

#include <CircularBuffer.hpp>

#define SERIAL_TX_BUF_LEN 16

struct Msg_t {
  uint32_t time;
  char txt[128];
};

class SerialBuf {
public:
  SerialBuf(Stream& _serial);

  bool pushMsg(Msg_t& Msg);
  void sendBuffer();

  uint8_t getBuffSize();

private:
  Stream* _serial;

  CircularBuffer<Msg_t, SERIAL_TX_BUF_LEN> _srlOutBuf;
};

SerialBuf::SerialBuf(Stream& _serial) {
  this->_serial = &_serial;
}

bool SerialBuf::pushMsg(Msg_t& msg) {

 // msg.time = millis();
  bool overflow = _srlOutBuf.push(msg);

  return overflow;  //returns false if buffer overflows
}

void SerialBuf::sendBuffer() {
  while (!_srlOutBuf.isEmpty()) {
    Msg_t msg = _srlOutBuf.shift();

   // _serial->print(msg.time);
   // _serial->print(": ");
    _serial->print(msg.txt);
  }
}

#endif