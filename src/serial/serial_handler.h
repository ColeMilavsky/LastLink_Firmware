#pragma once

#include <Arduino.h>

// Callback type: called when a complete line is ready to send
typedef void (*SerialLineCallback)(const String& line);

class SerialHandler {
public:
    SerialHandler();

    void begin();

    // Register callback for when a full line is entered
    void onLine(SerialLineCallback callback);

    // Call in loop() — reads available serial bytes
    void update();

private:
    String            _buffer;
    SerialLineCallback _lineCallback;
};

extern SerialHandler SerialInput;
