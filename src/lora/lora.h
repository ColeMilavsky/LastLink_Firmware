#ifndef LORA_H
#define LORA_H

#include <Arduino.h>
#include <RadioLib.h>

extern SPIClass spi;
extern SX1262 radio;
extern volatile bool rxFlag;

void IRAM_ATTR onReceive();
int loraBegin();
int loraSend(const String& message);
void onSerialLine(const String& line);

#endif // LORA_H
