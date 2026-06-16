#include <RadioLib.h>
#include "lora.h"
#include "config.h"


// ─── RadioLib setup ───────────────────────────────────────────────────────────
SPIClass spi(HSPI);
SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY, spi);

volatile bool rxFlag = false;

void IRAM_ATTR onReceive() {
    rxFlag = true;
}

int loraBegin() {
    spi.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);

    int state = radio.begin();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[LoRa] INIT FAILED: %d\n", state);
        while (true) { delay(1000); }
    }

    radio.setFrequency(915.0);
    radio.setBandwidth(125.0);
    radio.setSpreadingFactor(7);
    radio.setCodingRate(5);
    radio.setSyncWord(0x34);
    radio.setOutputPower(20);
    radio.setPreambleLength(8);
    radio.setCRC(true);

    if (state != RADIOLIB_ERR_NONE) {
        return state;
    }

    radio.setCRC(true);
    radio.setDio1Action(onReceive);
    radio.startReceive();

    return RADIOLIB_ERR_NONE;
}

// ─── Send over LoRa ───────────────────────────────────────────────────────────
void loraSend(const String&message) {
    radio.clearDio1Action();
    String msg = message;
    int state = radio.transmit(msg);
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println("[LoRa] TX: " + message);
    } else {
        Serial.printf("[LoRa] TX failed: %d\n", state);
    }
    radio.setDio1Action(onReceive);
    radio.startReceive();
}

void onSerialLine(const String& line) {
    loraSend(line);
}