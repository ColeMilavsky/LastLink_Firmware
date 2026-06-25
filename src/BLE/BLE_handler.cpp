#include "BLE_handler.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ─── UUIDs ─────────────────────────────────────────────────────────────────────
#define LASTLINK_SERVICE_UUID      "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define LASTLINK_TX_CHAR_UUID      "6e400002-b5a3-f393-e0a9-e50e24dcca9e" // phone -> Heltec (write)
#define LASTLINK_RX_CHAR_UUID      "6e400003-b5a3-f393-e0a9-e50e24dcca9e" // Heltec -> phone (notify)

BleHandler Ble;

static BLEServer*         pServer           = nullptr;
static BLECharacteristic* pTxCharacteristic = nullptr;
static BLECharacteristic* pRxCharacteristic = nullptr;

// ─── Server connect/disconnect callbacks ──────────────────────────────────────
class LastLinkServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* server) override {
        Ble._setConnected(true);
        Serial.println("[BLE] Phone connected");
    }

    void onDisconnect(BLEServer* server) override {
        Ble._setConnected(false);
        Serial.println("[BLE] Phone disconnected");
        server->getAdvertising()->start();
    }
};

// ─── Characteristic write callback (phone -> Heltec) ──────────────────────────
// NOTE: the BLE library hands us a std::string here — that's the ONLY place
// std::string should appear. Convert to Arduino String immediately so the
// rest of the codebase (Serial, lora.cpp, serial_handler.cpp) stays consistent.
class LastLinkTxCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* characteristic) override {
        std::string value = characteristic->getValue();
        if (value.length() > 0) {
            Ble._handleIncoming(String(value.c_str()));  // convert here
        }
    }
};

// ─── BleHandler ────────────────────────────────────────────────────────────────

BleHandler::BleHandler() : _connected(false), _messageCallback(nullptr) {}

void BleHandler::begin(const String& deviceName) {
    BLEDevice::init(deviceName.c_str());

    // Explicitly disable security/pairing requirements
    BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT_NO_MITM);
    BLESecurity* pSecurity = new BLESecurity();
    pSecurity->setAuthenticationMode(ESP_LE_AUTH_NO_BOND);

    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new LastLinkServerCallbacks());

    BLEService* pService = pServer->createService(LASTLINK_SERVICE_UUID);

    // Phone -> Heltec (write)
    pTxCharacteristic = pService->createCharacteristic(
        LASTLINK_TX_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    pTxCharacteristic->setCallbacks(new LastLinkTxCallbacks());

    // Heltec -> Phone (notify)
    pRxCharacteristic = pService->createCharacteristic(
        LASTLINK_RX_CHAR_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pRxCharacteristic->addDescriptor(new BLE2902());

    pService->start();

    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(LASTLINK_SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->start();

    Serial.println("[BLE] Advertising as \"" + deviceName + "\"");
}

void BleHandler::onMessage(BleLineCallback callback) {
    _messageCallback = callback;
}

void BleHandler::send(const String& message) {
    if (!_connected || pRxCharacteristic == nullptr) return;

    pRxCharacteristic->setValue(message.c_str());
    pRxCharacteristic->notify();
}

void BleHandler::update() {
    // Nothing needed every loop right now — connection state is event driven
    // via the server callbacks above. Reserved for future use (e.g. timeouts).
}

void BleHandler::_setConnected(bool connected) {
    _connected = connected;
}

void BleHandler::_handleIncoming(const String& message) {
    Serial.println("[BLE] RX from phone: " + message);
    if (_messageCallback) {
        _messageCallback(message);
    }
}