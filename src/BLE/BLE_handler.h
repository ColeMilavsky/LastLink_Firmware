#pragma once

#include <Arduino.h>

typedef void (*BleLineCallback)(const String& message);
typedef void (*BleConnectionCallback)(bool connected, const String& deviceName);

class BleHandler {
public:
    BleHandler();

    void begin(const String& deviceName);
    void onMessage(BleLineCallback callback);
    void onConnectionChange(BleConnectionCallback callback);  // NEW
    void send(const String& message);
    void update();
    bool isConnected() const { return _connected; }

    void _setConnected(bool connected);
    void _handleIncoming(const String& message);

private:
    bool                   _connected;
    String                 _deviceName;          // NEW — store our own name to report back
    BleLineCallback        _messageCallback;
    BleConnectionCallback  _connectionCallback;  // NEW
};

extern BleHandler Ble;