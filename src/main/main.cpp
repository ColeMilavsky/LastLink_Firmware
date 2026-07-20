#include <Arduino.h>
#include "../serial/serial_handler.h"
#include "../mesh/mesh.h"
#include "../lora/lora.h"
#include "../ble/ble_handler.h"
#include "../ui/UI.h"
#include "config.h"

// ─── Node identity ─────────────────────────────────────────────────────────
// Node ID is derived from the BLE device name suffix, e.g. "LastLink-A" -> 'A'.
// To stand up another node, just change DEVICE_NAME below (or move it to
// config.h if you want it per-build via build_flags).
#define DEVICE_NAME "LastLink-A"

static char deriveNodeId(const String& deviceName) {
    int dash = deviceName.lastIndexOf('-');
    if (dash >= 0 && dash + 1 < (int)deviceName.length()) {
        return deviceName.charAt(dash + 1);
    }
    // Fallback: last character of the name, so this never silently breaks
    // if someone names a device without a dash.
    return deviceName.length() > 0 ? deviceName.charAt(deviceName.length() - 1) : '?';
}

enum MsgSource { SRC_SERIAL, SRC_BLE };

// ─── Outgoing chat (typed locally over Serial, or relayed from this node's
//     own phone) — addressed to a nickname, or broadcast if none given ──────
void sendChat(const String& destNickname, const String& message, MsgSource source) {
    String tag = (source == SRC_BLE) ? "BLE" : "SERIAL";
    String label = destNickname.length() ? (tag + "->" + destNickname) : tag;

    Ui.showSending(message, label);

    bool ok = Mesh.sendChat(destNickname, message);
    if (ok) {
        Serial.println();
        Serial.println("┌─────────────────────────────────┐");
        Serial.printf( "│ TX via %-7s                 │\n", label.c_str());
        Serial.printf( "│  %s\n", message.c_str());
        Serial.println("└─────────────────────────────────┘");
        Serial.println();

        Ui.showSendComplete(message, label);
    } else {
        Serial.println("[LoRa] TX failed");
        Ui.showSendFailed(message);
    }
}

// Parses "@nickname message text" -> destNickname="nickname", message="message text".
// If there's no leading '@', destNickname is left empty (broadcast).
static void parseAddressedMessage(const String& line, String& destNickname, String& message) {
    destNickname = "";
    message = line;

    if (line.length() > 1 && line.charAt(0) == '@') {
        int spaceIdx = line.indexOf(' ');
        if (spaceIdx > 1) {
            destNickname = line.substring(1, spaceIdx);
            message = line.substring(spaceIdx + 1);
        }
    }
}

void onSerialMessage(const String& line) {
    // Local debug convenience: typing "/nick Foo" on Serial sets this node's
    // own phone-facing nickname too, same as the phone would over BLE.
    if (line.startsWith("/nick ")) {
        Mesh.setLocalNickname(line.substring(6));
        Serial.println("[Mesh] Nickname set via Serial: " + line.substring(6));
        return;
    }

    String destNickname, message;
    parseAddressedMessage(line, destNickname, message);
    sendChat(destNickname, message, SRC_SERIAL);
}

void onBleMessage(const String& message) {
    // Phone registers/changes its nickname with "/nick Foo".
    if (message.startsWith("/nick ")) {
        String nickname = message.substring(6);
        nickname.trim();
        Mesh.setLocalNickname(nickname);
        Ble.send("[OK] nickname set to " + nickname);
        return;
    }

    // Phone addresses a specific other phone with "@nickname message".
    String destNickname, chatMessage;
    parseAddressedMessage(message, destNickname, chatMessage);
    sendChat(destNickname, chatMessage, SRC_BLE);
}

void onBleConnectionChange(bool connected, const String& deviceName) {
    if (connected) {
        Ui.showBleConnected(deviceName);
    } else {
        Ui.showBleDisconnected();
    }
}

// ─── Incoming chat from the mesh, addressed to us or broadcast ─────────────
void onMeshChatReceived(const String& senderNickname, char srcNode, const String& message) {
    String from = senderNickname.length() ? senderNickname : String(srcNode);

    Serial.println();
    Serial.println("┌─────────────────────────────────┐");
    Serial.printf( "│ RX from %-7s                │\n", from.c_str());
    Serial.printf( "│  %s\n", message.c_str());
    Serial.println("└─────────────────────────────────┘");
    Serial.println();

    Ui.showReceiveComplete(from + ": " + message, (int)radio.getRSSI(), radio.getSNR());

    if (Ble.isConnected()) {
        Ble.send(from + ": " + message);
    }
}

void onMeshDirectoryChanged() {
    // Only refresh the idle screen opportunistically — avoid interrupting an
    // in-progress send/receive animation on the OLED.
    if (!rxFlag) {
        Ui.showMeshDirectory();
    }
}

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("[SYS] Booting...");

    Ui.begin();  // init OLED before anything else tries to draw to it

    int state = loraBegin();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[LoRa] INIT FAILED: %d\n", state);
        while (true) { delay(1000); }
    }
    Serial.printf("[LoRa] Ready on %.1f MHz\n", LORA_FREQUENCY);

    SerialInput.begin();
    SerialInput.onLine(onSerialMessage);

    Ble.begin(DEVICE_NAME);
    Ble.onMessage(onBleMessage);
    Ble.onConnectionChange(onBleConnectionChange);

    char nodeId = deriveNodeId(DEVICE_NAME);
    Mesh.begin(nodeId);
    Mesh.onChatReceived(onMeshChatReceived);
    Mesh.onDirectoryChanged(onMeshDirectoryChanged);

    Serial.println("─────────────────────────────────");
    Serial.println("  LastLink Firmware - Ready");
    Serial.printf( "  Node ID: %c\n", nodeId);
    Serial.println("  Type a message + Enter to TX");
    Serial.println("  '/nick Name'        sets your nickname");
    Serial.println("  '@Name message'     sends to a specific nickname");
    Serial.println("  'message'           broadcasts to the mesh");
    Serial.println("─────────────────────────────────");
}

void loop() {
    SerialInput.update();
    Ble.update();
    Mesh.update();

    if (rxFlag) {
        rxFlag = false;

        Ui.showReceiving();

        uint8_t buf[MESH_HEADER_LEN + MESH_MAX_PAYLOAD];
        size_t  len = sizeof(buf);
        int state = loraReceiveRaw(buf, len);

        if (state == RADIOLIB_ERR_NONE) {
            int   rssi = (int)radio.getRSSI();
            float snr  = radio.getSNR();

            MeshRxAction action = Mesh.handleIncomingPacket(buf, len, rssi, snr);

            switch (action) {
                case MESH_RX_FOR_ME:
                    // onMeshChatReceived already handled UI/BLE forwarding.
                    break;
                case MESH_RX_RELAYED:
                    Serial.println("[Mesh] Relayed packet not addressed to us");
                    break;
                case MESH_RX_PRESENCE:
                    Serial.println("[Mesh] Presence beacon received/relayed");
                    break;
                case MESH_RX_DUPLICATE:
                    Serial.println("[Mesh] Duplicate packet dropped");
                    break;
                case MESH_RX_EXPIRED:
                    Serial.println("[Mesh] Packet TTL expired, dropped");
                    break;
            }
        } else {
            Serial.printf("[LoRa] RX error: %d\n", state);
        }

        radio.startReceive();
    }
}