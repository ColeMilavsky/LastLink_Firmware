#include <RadioLib.h>
#include "lora.h"
#include "../../include/config.h"


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

// ─── Send over LoRa (plain text — legacy path, kept for compatibility) ────────
int loraSend(const String& message) {
    radio.clearDio1Action();
    String msg = message;
    int state = radio.transmit(msg);
    radio.setDio1Action(onReceive);
    radio.startReceive();
    return state;
}

// ─── Send over LoRa (raw bytes — used by the mesh packet framing) ────────────
int loraSendRaw(const uint8_t* data, size_t len) {
    radio.clearDio1Action();
    int state = radio.transmit((uint8_t*)data, len);
    radio.setDio1Action(onReceive);
    radio.startReceive();
    return state;
}

// ─── Receive raw bytes off the radio ──────────────────────────────────────────
// outLen (input) is the capacity of outBuf; on return it holds the actual
// number of bytes received. Per RadioLib's API, getPacketLength() must be
// called BEFORE readData() to know how much was actually received.
int loraReceiveRaw(uint8_t* outBuf, size_t& outLen) {
    size_t capacity = outLen;
    size_t received = radio.getPacketLength();

    if (received == 0) {
        outLen = 0;
        return RADIOLIB_ERR_RX_TIMEOUT;
    }
    if (received > capacity) {
        received = capacity; // truncate rather than overflow the caller's buffer
    }

    int state = radio.readData(outBuf, received);
    outLen = (state == RADIOLIB_ERR_NONE) ? received : 0;
    return state;
}

void onSerialLine(const String& line) {
    loraSend(line);
}