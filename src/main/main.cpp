#include <Arduino.h>
#include "../serial/serial_handler.h"
#include "../mesh/mesh.h"
#include "../lora/lora.h"
#include "config.h"

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    //unsigned long start = millis();
    //while (!Serial && millis() - start < 5000) { delay(10); }
    delay(2000);

    Serial.println("[SYS] Booting...");

    int state = loraBegin();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[LoRa] INIT FAILED: %d\n", state);
        Serial.printf("[LoRa] %s\n", String(stateDecode(state)).c_str());
        while (true) { delay(1000); }
    }

    Serial.printf("[LoRa] Ready on %.1f MHz\n", LORA_FREQUENCY);

    SerialInput.begin();
    SerialInput.onLine(onSerialLine);
    Mesh.begin();

    Serial.println("─────────────────────────────────");
    Serial.println("  LastLink Firmware - Ready");
    Serial.println("  Type a message + Enter to TX");
    Serial.println("─────────────────────────────────");
}

// ─── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
    SerialInput.update();

    if (rxFlag) {
        rxFlag = false;
        String received;
        int state = radio.readData(received);
        if (state == RADIOLIB_ERR_NONE) {
            int   rssi = (int)radio.getRSSI();
            float snr  = radio.getSNR();
            Serial.printf("\n>>> [RX | RSSI: %d | SNR: %.1f] %s\n\n",
                          rssi, snr, received.c_str());
        }
        radio.startReceive();
    }
}