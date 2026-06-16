#include "serial_handler.h"
#include "../include/config.h"

SerialHandler SerialInput;

SerialHandler::SerialHandler() : _lineCallback(nullptr) {}

void SerialHandler::begin() {
    Serial.begin(SERIAL_BAUD);
    // Wait for USB CDC to connect
    unsigned long start = millis();
    while (!Serial && millis() - start < 3000) {
        delay(10);
    }
    Serial.println("[Serial] Ready. Type a message and press Enter to send over LoRa.");
}

void SerialHandler::onLine(SerialLineCallback callback) {
    _lineCallback = callback;
}

void SerialHandler::update() {
    while (Serial.available()) {
        char c = Serial.read();

        if (c == MSG_DELIMITER || c == '\r') {
            // Carriage return — ignore (handle \r\n pairs)
            if (c == '\r') continue;

            // Newline — flush buffer
            _buffer.trim();
            if (_buffer.length() > 0 && _lineCallback) {
                _lineCallback(_buffer);
            }
            _buffer = "";
        } else if (c == 0x08 || c == 0x7F) {
            // Backspace / DEL — remove last char
            if (_buffer.length() > 0) {
                _buffer.remove(_buffer.length() - 1);
            }
        } else {
            if (_buffer.length() < MSG_MAX_LEN) {
                _buffer += c;
            }
        }
    }
}
