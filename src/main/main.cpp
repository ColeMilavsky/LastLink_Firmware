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
#define DEVICE_NAME "LastLink-B"

// In: deviceName - the BLE advertised name (e.g. "LastLink-A"). Out: the
//     single-character node id parsed after the last '-', or the name's
//     last character if there's no dash.
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

// Tracks the most recent unicast send so onMeshDeliveryStatus() can tell
// whether an ack/timeout belongs to it (and is therefore worth a brief OLED
// overlay) versus an older, already-superseded in-flight message.
static char    g_lastSentDest    = 0;
static uint8_t g_lastSentMsgId   = 0;
static bool    g_lastSentPending = false;

// In: none. Out: none.
// Pushes a "[STATUS] node=<id> nick=<nickname>" line to the connected phone
// so it always knows which node/nickname it's on. No-op if nothing's connected.
void pushMeshStatus() {
    if (!Ble.isConnected()) return;
    Ble.send("[STATUS] node=" + String(Mesh.nodeId()) + " nick=" + Mesh.localNickname());
}

// In: none. Out: none.
// Pushes a full "[ROUTES] dest:nextHop:cost:ageSeconds,..." dump of the
// current routing table to the connected phone. Used once on BLE connect so
// the app starts with complete state; subsequent changes travel as smaller
// [ROUTE+]/[ROUTE-] deltas from onMeshRouteChanged(). No-op if not connected.
void pushRouteDump() {
    if (!Ble.isConnected()) return;

    String msg = "[ROUTES] ";
    int count = Mesh.routeCount();
    for (int i = 0; i < count; i++) {
        char          dest, nextHop;
        uint8_t       cost;
        unsigned long ageMs;
        if (Mesh.routeEntryAt(i, dest, nextHop, cost, ageMs)) {
            if (i > 0) msg += ",";
            msg += String(dest) + ":" + String(nextHop) + ":" + String(cost) + ":" + String(ageMs / 1000);
        }
    }
    Ble.send(msg);
}

// ─── Outgoing chat (typed locally over Serial, or relayed from this node's
//     own phone) — addressed to a nickname, or broadcast if none given ──────
// In: destNickname - target nickname (empty = broadcast); message - text to
//     send; source - where the send originated (Serial or BLE), used for logging/UI.
// Out: none. Drives the OLED send animation, calls Mesh.sendChat(), logs/
// displays success or failure, and for unicast sends, tracks the assigned
// msgId and pushes "[SENT]" so the phone can correlate a later ack.
void sendChat(const String& destNickname, const String& message, MsgSource source) {
    String tag = (source == SRC_BLE) ? "BLE" : "SERIAL";
    String label = destNickname.length() ? (tag + "->" + destNickname) : tag;

    Ui.notifySending(message, label);

    char    destNode = NODE_BROADCAST;
    uint8_t msgId    = 0;
    bool ok = Mesh.sendChat(destNickname, message, &destNode, &msgId);

    if (ok) {
        Serial.println();
        Serial.println("┌─────────────────────────────────┐");
        Serial.printf( "│ TX via %-7s                 │\n", label.c_str());
        Serial.printf( "│  %s\n", message.c_str());
        Serial.println("└─────────────────────────────────┘");
        Serial.println();

        Ui.notifySendComplete(message, label);

        if (destNode != NODE_BROADCAST) {
            g_lastSentDest    = destNode;
            g_lastSentMsgId   = msgId;
            g_lastSentPending = true;
            if (Ble.isConnected()) {
                Ble.send("[SENT] id=" + String(msgId) + " dest=" + String(destNode));
            }
        }
    } else {
        Serial.println("[LoRa] TX failed");
        Ui.notifySendFailed(message);
    }
}

// In: line - raw input, optionally in "@nickname message text" form.
// Out: destNickname/message split from line; destNickname is left empty
//      (meaning broadcast) if there's no leading '@'.
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

// In: line - a complete line typed over Serial. Out: none.
// Handles the local "/nick" convenience command, otherwise parses it as an
// addressed or broadcast chat message and sends it.
void onSerialMessage(const String& line) {
    // Local debug convenience: typing "/nick Foo" on Serial sets this node's
    // own phone-facing nickname too, same as the phone would over BLE.
    if (line.startsWith("/nick ")) {
        Mesh.setLocalNickname(line.substring(6));
        Ui.setIdentity(Mesh.nodeId(), Mesh.localNickname());
        pushMeshStatus();
        Serial.println("[Mesh] Nickname set via Serial: " + line.substring(6));
        return;
    }

    String destNickname, message;
    parseAddressedMessage(line, destNickname, message);
    sendChat(destNickname, message, SRC_SERIAL);
}

// In: message - raw text written by the connected phone. Out: none.
// Handles the "/nick" registration command (acking over BLE), otherwise
// parses it as an addressed or broadcast chat message and sends it.
void onBleMessage(const String& message) {
    // Phone registers/changes its nickname with "/nick Foo".
    if (message.startsWith("/nick ")) {
        String nickname = message.substring(6);
        nickname.trim();
        Mesh.setLocalNickname(nickname);
        Ui.setIdentity(Mesh.nodeId(), nickname);
        Ble.send("[OK] nickname set to " + nickname);
        pushMeshStatus();
        return;
    }

    // Phone addresses a specific other phone with "@nickname message".
    String destNickname, chatMessage;
    parseAddressedMessage(message, destNickname, chatMessage);
    sendChat(destNickname, chatMessage, SRC_BLE);
}

// In: connected - new BLE connection state; deviceName - the connected
//     device's name (unused when disconnected). Out: none.
// Fires the connected/disconnected OLED event, and on a fresh connection
// pushes the phone its initial state (node identity + full routing table)
// since it starts with no prior knowledge.
void onBleConnectionChange(bool connected, const String& deviceName) {
    if (connected) {
        Ui.notifyBleConnected(deviceName);
        pushMeshStatus();
        pushRouteDump();
    } else {
        Ui.notifyBleDisconnected();
    }
}

// ─── Incoming chat from the mesh, addressed to us or broadcast ─────────────
// In: senderNickname - sender's nickname if known (empty otherwise); srcNode
//     - sender's node id; message - the chat text. Out: none.
// Logs the message, fires the OLED "message received" event (sender +
// content), and forwards it to the connected phone over BLE if one is present.
void onMeshChatReceived(const String& senderNickname, char srcNode, const String& message) {
    String from = senderNickname.length() ? senderNickname : String(srcNode);

    Serial.println();
    Serial.println("┌─────────────────────────────────┐");
    Serial.printf( "│ RX from %-7s                │\n", from.c_str());
    Serial.printf( "│  %s\n", message.c_str());
    Serial.println("└─────────────────────────────────┘");
    Serial.println();

    Ui.notifyMessageReceived(from, message);

    if (Ble.isConnected()) {
        Ble.send(from + ": " + message);
    }
}

// In: destNode - node the original chat was sent to; msgId - its sequence
//     number; status - MESH_DELIVERY_ACKED or MESH_DELIVERY_FAILED. Out: none.
// Logs the outcome, pushes a structured "[ACK]" line (tagged with msgId so
// the phone can match it to the specific message it sent) over BLE, and — if
// this is the most recently sent message — fires the OLED delivery-status event.
void onMeshDeliveryStatus(char destNode, uint8_t msgId, MeshDeliveryStatus status) {
    bool delivered = (status == MESH_DELIVERY_ACKED);
    String statusStr = delivered ? "delivered" : "failed";
    Serial.printf("[Mesh] Msg %u to %c: %s\n", msgId, destNode, statusStr.c_str());

    if (Ble.isConnected()) {
        Ble.send("[ACK] id=" + String(msgId) + " dest=" + String(destNode) + " status=" + statusStr);
    }

    if (g_lastSentPending && destNode == g_lastSentDest && msgId == g_lastSentMsgId) {
        g_lastSentPending = false;
        Ui.notifyAckReceived(destNode, delivered);
    }
}

// In: none. Out: none.
// Redraws the OLED Mesh screen if it's currently showing (no-op otherwise —
// UiHandler decides that itself).
void onMeshDirectoryChanged() {
    Ui.refreshMesh();
}

// In: destNode/nextHop/cost - the changed routing entry; removed - true if
//     destNode's route was just dropped (nextHop/cost are meaningless then).
// Out: none. Redraws the OLED Mesh screen if it's currently showing, and
// pushes a compact delta ("[ROUTE+]"/"[ROUTE-]") to the phone — this only
// fires on substantive table changes (see MeshLayer::_learnRoute), so it
// doesn't spam either surface.
void onMeshRouteChanged(char destNode, char nextHop, uint8_t cost, bool removed) {
    Ui.refreshMesh();

    if (Ble.isConnected()) {
        if (removed) {
            Ble.send("[ROUTE-] dest=" + String(destNode));
        } else {
            Ble.send("[ROUTE+] dest=" + String(destNode) + " via=" + String(nextHop) + " cost=" + String(cost));
        }
    }
}

// In: none (Arduino entry point, called once at boot). Out: none.
// Brings up Serial, the OLED, the LoRa radio, the Serial-line handler, BLE,
// and the mesh layer, wiring each one's callbacks before printing the ready banner.
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
    Mesh.onDeliveryStatus(onMeshDeliveryStatus);
    Mesh.onRouteChanged(onMeshRouteChanged);
    Ui.setIdentity(nodeId, "");

    Serial.println("─────────────────────────────────");
    Serial.println("  LastLink Firmware - Ready");
    Serial.printf( "  Node ID: %c\n", nodeId);
    Serial.println("  Type a message + Enter to TX");
    Serial.println("  '/nick Name'        sets your nickname");
    Serial.println("  '@Name message'     sends to a specific nickname");
    Serial.println("  'message'           broadcasts to the mesh");
    Serial.println("─────────────────────────────────");
}

// In: none (Arduino main loop, called repeatedly). Out: none.
// Polls the Serial/BLE/mesh/UI handlers each tick, and when a radio packet
// has arrived, reads it, hands it to Mesh.handleIncomingPacket(), logs the
// resulting action, and re-arms the receiver. The OLED only reacts to
// specific named events (chat-for-me, ack-for-me, etc. via their onMesh*
// callbacks) — arbitrary radio activity (relays, presence, duplicates) is
// intentionally not shown on-screen.
void loop() {
    SerialInput.update();
    Ble.update();
    Mesh.update();
    Ui.update();

    if (rxFlag) {
        rxFlag = false;

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
                case MESH_RX_ACK:
                    Serial.println("[Mesh] Ack packet received/relayed");
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