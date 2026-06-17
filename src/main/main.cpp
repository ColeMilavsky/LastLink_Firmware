#include <Arduino.h>
#include "../serial/serial_handler.h"
#include "../mesh/mesh.h"
#include "../lora/lora.h"
#include "../../include/config.h"

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("[SYS] Booting...");

    int state = loraBegin();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[LoRa] INIT FAILED: %s\n", String(stateDecode(state).c_str()));
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

void loop() {
    SerialInput.update();

    if (rxFlag) {
        rxFlag = false;
        String received;
        int state = radio.readData(received);
        if (state == RADIOLIB_ERR_NONE) {
            int   rssi = (int)radio.getRSSI();
            float snr  = radio.getSNR();
            Serial.println();
            Serial.println("┌─────────────────────────────────┐");
            Serial.printf( "│ RX  RSSI: %4d dBm  SNR: %5.1f  │\n", rssi, snr);
            Serial.println("│                                 │");
            Serial.printf( "│  %s\n", received.c_str());
            Serial.println("└─────────────────────────────────┘");
            Serial.println();
        } else {
            Serial.printf("[LoRa] RX error: %s\n", stateDecode(state).c_str());
        }
        radio.startReceive();
    }
}