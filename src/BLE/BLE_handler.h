#pragma once

#include <Arduino.h>

typedef void (*BleLineCallback)(const String& message);
typedef void (*BleConnectionCallback)(bool connected, const String& deviceName);

class BleHandler {
public:
    BleHandler();

    void begin(const String& deviceName);
    void onMessage(BleLineCallback callback);
    void onConnectionChange(BleConnectionCallback callback);
    void send(const String& message);
    void update();  // call every loop() — promotes a pending subscribe into the connected state
    bool isConnected() const { return _connected; }

    void _setConnected(bool connected);
    void _handleIncoming(const String& message);
    void _onNotifyEnabled(); // called from the RX characteristic's CCCD write callback once the phone subscribes

private:
    bool                   _connected;
    volatile bool          _pendingSubscribe;
    String                 _deviceName;
    BleLineCallback        _messageCallback;
    BleConnectionCallback  _connectionCallback;
};

extern BleHandler Ble;