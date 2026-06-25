#include <Arduino.h>
#include "../serial/serial_handler.h"
#include "../mesh/mesh.h"
#include "../lora/lora.h"
#include "../ble/ble_handler.h"
#include "config.h"

enum MsgSource { SRC_SERIAL, SRC_BLE };

void sendOverLoRa(const String& message, MsgSource source) {
    String tag = (source == SRC_BLE) ? "[BLE]" : "[SERIAL]";

    int state = loraSend(message);  // see note below — make loraSend return state
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println();
        Serial.println("┌─────────────────────────────────┐");
        Serial.printf( "│ TX via %-7s                 │\n", tag.c_str());
        Serial.printf( "│  %s\n", message.c_str());
        Serial.println("└─────────────────────────────────┘");
        Serial.println();
    } else {
        Serial.printf("[LoRa] TX failed: %s\n", stateDecode(state).c_str());
    }
}

void onSerialMessage(const String& line) {
    sendOverLoRa(line, SRC_SERIAL);
}

void onBleMessage(const String& message) {
    sendOverLoRa(message, SRC_BLE);
}

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("[SYS] Booting...");

    int state = loraBegin();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[LoRa] INIT FAILED: %s\n", stateDecode(state).c_str());
        while (true) { delay(1000); }
    }
    Serial.printf("[LoRa] Ready on %.1f MHz\n", LORA_FREQUENCY);

    SerialInput.begin();
    SerialInput.onLine(onSerialMessage);

    Ble.begin("LastLink-B");
    Ble.onMessage(onBleMessage);

    Mesh.begin();

    Serial.println("─────────────────────────────────");
    Serial.println("  LastLink Firmware - Ready");
    Serial.println("  Type a message + Enter to TX");
    Serial.println("─────────────────────────────────");
}

void loop() {
    SerialInput.update();
    Ble.update();

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
            Serial.printf( "│  %s\n", received.c_str());
            Serial.println("└─────────────────────────────────┘");
            Serial.println();

            // Forward to phone over BLE, tag where it's going
            if (Ble.isConnected()) {
                Serial.println("[BLE] Forwarding RX to connected phone");
                Ble.send(received);
            }
        } else {
            Serial.printf("[LoRa] RX error: %s\n", stateDecode(state).c_str());
        }
        radio.startReceive();
    }
}