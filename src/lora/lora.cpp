#include <RadioLib.h>
#include "lora.h"
#include "../../include/config.h"


// ─── RadioLib setup ───────────────────────────────────────────────────────────
SPIClass spi(HSPI);
SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY, spi);

volatile bool rxFlag = false;

// In: none (radio interrupt handler). Out: none.
// Sets rxFlag so the main loop knows to drain a received packet; kept
// minimal since this runs in interrupt context.
void IRAM_ATTR onReceive() {
    rxFlag = true;
}

// In: none. Out: RADIOLIB_ERR_NONE on success, or a RadioLib error code.
// Initializes SPI and the SX1262 radio with this project's fixed link
// parameters, wires up the receive interrupt, and starts listening.
int loraBegin() {
    spi.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);

    int state = radio.begin();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[LoRa] INIT FAILED: %d\n", state);
        while (true) { delay(1000); }
    }

    radio.setFrequency(915.0);
    radio.setBandwidth(125.0);
    radio.setSpreadingFactor(9);
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
// In: message - plain text to transmit. Out: RadioLib transmit status code.
// Legacy plain-text send path; briefly disables the RX interrupt during
// transmit, then re-arms it and resumes listening.
int loraSend(const String& message) {
    radio.clearDio1Action();
    String msg = message;
    int state = radio.transmit(msg);
    radio.setDio1Action(onReceive);
    radio.startReceive();
    return state;
}

// ─── Send over LoRa (raw bytes — used by the mesh packet framing) ────────────
// In: data/len - raw bytes to transmit (a mesh packet). Out: RadioLib
//     transmit status code.
// Disables the RX interrupt during transmit, sends the raw bytes, then
// re-arms the interrupt and resumes listening. Used for all mesh traffic.
int loraSendRaw(const uint8_t* data, size_t len) {
    radio.clearDio1Action();
    int state = radio.transmit((uint8_t*)data, len);
    radio.setDio1Action(onReceive);
    radio.startReceive();
    return state;
}

// In: outBuf - caller-owned buffer; outLen (as input) - its capacity.
// Out: outBuf filled with the received packet; outLen (as output) - actual
//      bytes received; returns RADIOLIB_ERR_NONE on success or an error code.
// Per RadioLib's API, getPacketLength() must be called BEFORE readData() to
// know how much was actually received; truncates rather than overflowing outBuf.
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

// In: line - a line of text read from Serial. Out: none.
// Sends the line over LoRa via the legacy plain-text path.
void onSerialLine(const String& line) {
    loraSend(line);
}