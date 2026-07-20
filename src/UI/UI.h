#pragma once

#include <Arduino.h>

class UiHandler {
public:
    UiHandler();

    void begin();
    void update();

    void showBleConnected(const String& deviceName);
    void showBleDisconnected();

    void showSending(const String& message, const String& source);
    void showSendComplete(const String& message, const String& source);
    void showSendFailed(const String& message);

    void showReceiving();
    void showReceiveComplete(const String& message, int rssi, float snr);

    void showIdle();

    // Shows the known mesh directory (node id -> nickname). Call after the
    // mesh directory changes, or periodically/on a button press.
    void showMeshDirectory();

private:
    bool   _bleConnected;
    String _lastBleDevice;

    void _drawHeader(const char* label, bool sending);
    void _wrapAndPrint(const String& text, int startY);
};

extern UiHandler Ui;