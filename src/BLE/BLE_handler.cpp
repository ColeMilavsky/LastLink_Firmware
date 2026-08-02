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
    // Physical link only — not yet "connected" from the protocol's point of view until the phone subscribes.
    void onConnect(BLEServer* server) override {
        Serial.println("[BLE] Phone link up, awaiting subscribe");
    }

    // Marks Ble as disconnected and restarts advertising so another phone can connect.
    void onDisconnect(BLEServer* server) override {
        Ble._setConnected(false);
        Serial.println("[BLE] Phone disconnected");
        server->getAdvertising()->start();
    }
};

class LastLinkTxCallbacks : public BLECharacteristicCallbacks {
    // Forwards a non-empty written value into Ble's incoming-message path.
    void onWrite(BLECharacteristic* characteristic) override {
        std::string value = characteristic->getValue();
        if (value.length() > 0) {
            Ble._handleIncoming(String(value.c_str()));
        }
    }
};

class LastLinkRxCccdCallbacks : public BLEDescriptorCallbacks {
    // Fires when the phone writes the RX characteristic's CCCD; enabling notifications here is the
    // real "ready to receive" moment (the raw link-connect event fires well before this).
    void onWrite(BLEDescriptor* descriptor) override {
        uint8_t* value = descriptor->getValue();
        if (descriptor->getLength() >= 1 && (value[0] & 0x01)) {
            Ble._onNotifyEnabled();
        }
    }
};

// Constructs the handler in a disconnected state with no callbacks registered.
BleHandler::BleHandler() : _connected(false), _pendingSubscribe(false), _messageCallback(nullptr), _connectionCallback(nullptr) {}

// Initializes the BLE stack, sets up the Nordic UART-style TX/RX characteristics, and starts advertising.
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
    BLE2902* pRxCccd = new BLE2902();
    pRxCccd->setCallbacks(new LastLinkRxCccdCallbacks());
    pRxCharacteristic->addDescriptor(pRxCccd);

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

void BleHandler::onMessage(BleLineCallback callback) {
    _messageCallback = callback;
}

void BleHandler::onConnectionChange(BleConnectionCallback callback) {
    _connectionCallback = callback;
}

// No-ops if no phone is subscribed; otherwise notifies via the RX characteristic.
void BleHandler::send(const String& message) {
    if (!_connected || pRxCharacteristic == nullptr) return;
    pRxCharacteristic->setValue(message.c_str());
    pRxCharacteristic->notify();
}

// Promotes a pending subscribe (flagged by the CCCD write callback) into the connected state.
// Done here rather than directly in that callback so the resulting full BLE sync runs on the main
// loop task, strictly before this tick's Mesh.update() — never racing a concurrent table mutation.
void BleHandler::update() {
    if (_pendingSubscribe) {
        _pendingSubscribe = false;
        _setConnected(true);
    }
}

// Updates internal state and notifies the registered connection callback, if any.
void BleHandler::_setConnected(bool connected) {
    _connected = connected;
    if (!connected) _pendingSubscribe = false;
    if (_connectionCallback) {
        _connectionCallback(connected, _deviceName);
    }
}

// Logs and forwards a message written by the phone to the registered message callback, if any.
void BleHandler::_handleIncoming(const String& message) {
    Serial.println("[BLE] RX from phone: " + message);
    if (_messageCallback) {
        _messageCallback(message);
    }
}

// Called from the RX characteristic's CCCD write callback once the phone enables notifications.
void BleHandler::_onNotifyEnabled() {
    _pendingSubscribe = true;
}