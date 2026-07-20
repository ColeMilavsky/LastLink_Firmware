#include "BLE_handler.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLESecurity.h>

#define LASTLINK_SERVICE_UUID      "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define LASTLINK_TX_CHAR_UUID      "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define LASTLINK_RX_CHAR_UUID      "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

BleHandler Ble;

static BLEServer*         pServer           = nullptr;
static BLECharacteristic* pTxCharacteristic = nullptr;
static BLECharacteristic* pRxCharacteristic = nullptr;

class LastLinkServerCallbacks : public BLEServerCallbacks {
    // In: server - the BLE server that gained a connection. Out: none.
    // Marks Ble as connected.
    void onConnect(BLEServer* server) override {
        Ble._setConnected(true);
        Serial.println("[BLE] Phone connected");
    }

    // In: server - the BLE server that lost its connection. Out: none.
    // Marks Ble as disconnected and restarts advertising so another phone can connect.
    void onDisconnect(BLEServer* server) override {
        Ble._setConnected(false);
        Serial.println("[BLE] Phone disconnected");
        server->getAdvertising()->start();
    }
};

class LastLinkTxCallbacks : public BLECharacteristicCallbacks {
    // In: characteristic - the TX characteristic the phone just wrote to.
    // Out: none. Forwards a non-empty written value into Ble's incoming-message path.
    void onWrite(BLECharacteristic* characteristic) override {
        std::string value = characteristic->getValue();
        if (value.length() > 0) {
            Ble._handleIncoming(String(value.c_str()));
        }
    }
};

// In: none. Out: none.
// Constructs the handler in a disconnected state with no callbacks registered.
BleHandler::BleHandler() : _connected(false), _messageCallback(nullptr), _connectionCallback(nullptr) {}

// In: deviceName - the BLE advertised name (e.g. "LastLink-A"). Out: none.
// Initializes the BLE stack, sets up the Nordic UART-style TX/RX
// characteristics, and starts advertising under deviceName.
void BleHandler::begin(const String& deviceName) {
    _deviceName = deviceName;

    BLEDevice::init(deviceName.c_str());

    // Default BLE MTU is 23 bytes (~20 usable), which truncates anything
    // beyond a short message. Mesh-routed messages get a "Nickname: " prefix
    // added, so bump this up. Actual negotiated MTU still depends on what
    // the phone's BLE stack supports/requests.
    BLEDevice::setMTU(247);

    BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT_NO_MITM);
    BLESecurity* pSecurity = new BLESecurity();
    pSecurity->setAuthenticationMode(ESP_LE_AUTH_NO_BOND);

    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new LastLinkServerCallbacks());

    BLEService* pService = pServer->createService(LASTLINK_SERVICE_UUID);

    pTxCharacteristic = pService->createCharacteristic(
        LASTLINK_TX_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    pTxCharacteristic->setCallbacks(new LastLinkTxCallbacks());

    pRxCharacteristic = pService->createCharacteristic(
        LASTLINK_RX_CHAR_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pRxCharacteristic->addDescriptor(new BLE2902());

    pService->start();

    // ── Explicit advertising data setup ──────────────────────────────────────
    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();

    BLEAdvertisementData advData;
    advData.setFlags(0x06);  // general discoverable, BR/EDR not supported
    advData.setCompleteServices(BLEUUID(LASTLINK_SERVICE_UUID));
    advData.setName(deviceName.c_str());
    pAdvertising->setAdvertisementData(advData);

    BLEAdvertisementData scanResponseData;
    scanResponseData.setName(deviceName.c_str());
    pAdvertising->setScanResponseData(scanResponseData);

    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMaxPreferred(0x12);
    pAdvertising->start();

    Serial.println("[BLE] Advertising as \"" + deviceName + "\"");
}

// In: callback - function to invoke when the phone writes a message. Out: none.
void BleHandler::onMessage(BleLineCallback callback) {
    _messageCallback = callback;
}

// In: callback - function to invoke when the connection state changes. Out: none.
void BleHandler::onConnectionChange(BleConnectionCallback callback) {
    _connectionCallback = callback;
}

// In: message - text to push to the connected phone. Out: none.
// No-ops if no phone is connected; otherwise notifies via the RX characteristic.
void BleHandler::send(const String& message) {
    if (!_connected || pRxCharacteristic == nullptr) return;
    pRxCharacteristic->setValue(message.c_str());
    pRxCharacteristic->notify();
}

// In: none. Out: none. Reserved for future use (nothing to poll today).
void BleHandler::update() {
    // Reserved for future use
}

// In: connected - the new connection state. Out: none.
// Updates internal state and notifies the registered connection callback, if any.
void BleHandler::_setConnected(bool connected) {
    _connected = connected;
    if (_connectionCallback) {
        _connectionCallback(connected, _deviceName);
    }
}

// In: message - raw text written by the phone. Out: none.
// Logs it and forwards it to the registered message callback, if any.
void BleHandler::_handleIncoming(const String& message) {
    Serial.println("[BLE] RX from phone: " + message);
    if (_messageCallback) {
        _messageCallback(message);
    }
}