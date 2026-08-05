#ifndef LORA_H
#define LORA_H

#include <Arduino.h>
#include <RadioLib.h>

extern SPIClass spi;
extern SX1262 radio;
extern volatile bool rxFlag;

void IRAM_ATTR onReceive();
int loraBegin();

// Legacy plain-text send/receive path. Still used for the raw String API,
// but mesh traffic should go through the raw-byte functions below since
// mesh packets contain binary header bytes that String can mangle.
int loraSend(const String& message);
void onSerialLine(const String& line);

// Raw byte send/receive — used by the mesh layer for packet framing.
// Returns RADIOLIB_ERR_NONE (0) on success, or a RadioLib error code.
int loraSendRaw(const uint8_t* data, size_t len);

// Reads the most recently received raw packet into outBuf (caller-sized,
// pass outBuf size in outLen as input, receives actual length as output).
// Returns RADIOLIB_ERR_NONE on success.
int loraReceiveRaw(uint8_t* outBuf, size_t& outLen);

#endif // LORA_H