#include "serial_handler.h"
#include "../../include/config.h"

SerialHandler SerialInput;

// In: none. Out: none. Constructs the handler with no line callback registered.
SerialHandler::SerialHandler() : _lineCallback(nullptr) {}

// In: none. Out: none.
// Starts the Serial port at SERIAL_BAUD and waits briefly for USB CDC to enumerate.
void SerialHandler::begin() {
    Serial.begin(SERIAL_BAUD);
    // Wait for USB CDC to connect
    unsigned long start = millis();
    while (!Serial && millis() - start < 3000) {
        delay(10);
    }
    Serial.println("[Serial] Ready. Type a message and press Enter to send over LoRa.");
}

// In: callback - function to invoke with each complete line typed over Serial.
// Out: none.
void SerialHandler::onLine(SerialLineCallback callback) {
    _lineCallback = callback;
}

// In: none. Out: none.
// Drains available Serial bytes into a line buffer, handling backspace/DEL
// and \r\n, and fires the line callback once a newline-terminated,
// non-empty line has accumulated.
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