#include <Arduino.h>

// Callback type: called when a complete message is received over BLE
typedef void (*BleLineCallback)(const String& message);

class BleHandler {
public:
    BleHandler();

    // Start advertising and set up the GATT server. deviceName shows up
    // in the phone's BLE scanner (e.g. "LastLink-A").
    void begin(const String& deviceName);

    // Register callback for when a phone writes a message to us
    void onMessage(BleLineCallback callback);

    // Send a message out to the connected phone (e.g. an incoming LoRa packet)
    void send(const String& message);

    // Call in loop() — currently a light heartbeat / reconnect check
    void update();

    bool isConnected() const { return _connected; }

    // Internal — used by the BLE server/characteristic callback classes.
    // Public so the .cpp's callback subclasses can reach them.
    void _setConnected(bool connected);
    void _handleIncoming(const String& message);

private:
    bool            _connected;
    BleLineCallback _messageCallback;
};

extern BleHandler Ble;