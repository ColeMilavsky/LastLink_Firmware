#include <Arduino.h>
#include "../serial/serial_handler.h"
#include "../mesh/mesh.h"
#include "../lora/lora.h"
#include "../ble/ble_handler.h"
#include "../ui/UI.h"
#include "config.h"

// Node ID is derived from the BLE device name suffix, e.g. "LastLink-A" -> 'A'.
#define DEVICE_NAME "LastLink-A"

// Parses the single-char node id after the last '-' in deviceName (or its last char if no dash).
static char deriveNodeId(const String& deviceName) {
    int dash = deviceName.lastIndexOf('-');
    if (dash >= 0 && dash + 1 < (int)deviceName.length()) {
        return deviceName.charAt(dash + 1);
    }
    return deviceName.length() > 0 ? deviceName.charAt(deviceName.length() - 1) : '?';
}

// Default boot nickname: the node id as a one-character string (e.g. 'A' -> "A").
static String deriveDefaultNickname(char nodeId) {
    return String(nodeId);
}

enum MsgSource { SRC_SERIAL, SRC_BLE };

// Tracks the most recent unicast send so onMeshDeliveryStatus() knows whether an ack/timeout is for it.
static char    g_lastSentDest    = 0;
static uint8_t g_lastSentMsgId   = 0;
static bool    g_lastSentPending = false;

// ─── BLE node-sync (combined directory + route view for the phone) ────────
// Nodes marked dirty this tick get one coalesced [NODE+]/[NODE-] push via flushNodeSync().
#define BLE_NODE_SYNC_SIZE 16
struct PendingNodeSync { bool used = false; char nodeId = 0; };
static PendingNodeSync g_pendingNodeSync[BLE_NODE_SYNC_SIZE];

// Safe per-notify payload cap (well under a possibly-lower-than-247 negotiated MTU) and the largest
// entry count either full-sync dump can produce, for sizing the chunk-boundary scratch array below.
#define BLE_SYNC_CHUNK_MAX_LEN 150
#define BLE_SYNC_MAX_ENTRIES ((MESH_DIRECTORY_SIZE > MESH_ROUTE_TABLE_SIZE) ? MESH_DIRECTORY_SIZE : MESH_ROUTE_TABLE_SIZE)

// Sends entries[0..count) as one or more "tag[ i/N] e1,e2,..." notifications, split on entry
// boundaries (never mid-entry) so a long dump can't get truncated at the BLE layer. The "i/N" suffix
// is only added when more than one chunk is actually needed, so a small mesh keeps the plain
// "tag e1,e2,..." format unchanged.
static void sendEntriesChunked(const String& tag, String* entries, int count) {
    if (!Ble.isConnected()) return;

    int chunkStart[BLE_SYNC_MAX_ENTRIES + 1];
    int chunkCount = 0;
    int i = 0;
    do {
        chunkStart[chunkCount++] = i;
        int len = 0;
        while (i < count) {
            int add = entries[i].length() + (len > 0 ? 1 : 0);
            if (len > 0 && len + add > BLE_SYNC_CHUNK_MAX_LEN) break;
            len += add;
            i++;
        }
    } while (i < count);
    chunkStart[chunkCount] = count;

    for (int c = 0; c < chunkCount; c++) {
        String msg = tag;
        if (chunkCount > 1) msg += " " + String(c + 1) + "/" + String(chunkCount);
        msg += " ";
        for (int k = chunkStart[c]; k < chunkStart[c + 1]; k++) {
            if (k > chunkStart[c]) msg += ",";
            msg += entries[k];
        }
        Ble.send(msg);
    }
}

// Marks nodeId dirty for the next flushNodeSync(); idempotent.
static void markNodeDirty(char nodeId) {
    for (int i = 0; i < BLE_NODE_SYNC_SIZE; i++) {
        if (g_pendingNodeSync[i].used && g_pendingNodeSync[i].nodeId == nodeId) return;
    }
    for (int i = 0; i < BLE_NODE_SYNC_SIZE; i++) {
        if (!g_pendingNodeSync[i].used) {
            g_pendingNodeSync[i].used   = true;
            g_pendingNodeSync[i].nodeId = nodeId;
            return;
        }
    }
}

// Looks up nodeId's nickname (directory) and hop count (routing table); returns false if known in neither.
static bool lookupNodeSyncInfo(char nodeId, String& outNickname, uint8_t& outHops, bool& outHopsKnown) {
    bool foundNickname = false;
    for (int i = 0; i < Mesh.directoryCount(); i++) {
        char          dirNodeId;
        String        dirNickname;
        unsigned long ageMs;
        if (Mesh.directoryEntryAt(i, dirNodeId, dirNickname, ageMs) && dirNodeId == nodeId) {
            outNickname   = dirNickname;
            foundNickname = true;
            break;
        }
    }

    outHopsKnown = (nodeId == Mesh.nodeId());
    outHops      = 0;
    if (!outHopsKnown) {
        for (int i = 0; i < Mesh.routeCount(); i++) {
            char          dest, nextHop;
            uint8_t       cost;
            unsigned long ageMs;
            if (Mesh.routeEntryAt(i, dest, nextHop, cost, ageMs) && dest == nodeId) {
                outHops      = cost;
                outHopsKnown = true;
                break;
            }
        }
    }

    return foundNickname || outHopsKnown;
}

// Sends "[NODE+]" or "[NODE-]" for nodeId's current combined state.
static void pushNodeSync(char nodeId) {
    if (!Ble.isConnected()) return;

    String  nickname;
    uint8_t hops;
    bool    hopsKnown;
    if (!lookupNodeSyncInfo(nodeId, nickname, hops, hopsKnown)) {
        Ble.send("[NODE-] id=" + String(nodeId));
        return;
    }

    Ble.send("[NODE+] id=" + String(nodeId) + " nick=" + nickname + " hops=" + (hopsKnown ? String(hops) : "?"));
}

// Pushes a sync line for every node marked dirty since the last flush, then clears the pending set.
static void flushNodeSync() {
    for (int i = 0; i < BLE_NODE_SYNC_SIZE; i++) {
        if (g_pendingNodeSync[i].used) {
            pushNodeSync(g_pendingNodeSync[i].nodeId);
            g_pendingNodeSync[i].used = false;
        }
    }
}

// Pushes a full "[NODES] id:nick:hops,..." dump (chunked if long); used once on BLE connect.
void pushFullNodeSync() {
    if (!Ble.isConnected()) return;

    String entries[BLE_SYNC_MAX_ENTRIES];
    int    count = 0;
    int    total = Mesh.directoryCount();
    for (int i = 0; i < total && count < BLE_SYNC_MAX_ENTRIES; i++) {
        char          nodeId;
        String        dirNickname;
        unsigned long ageMs;
        if (!Mesh.directoryEntryAt(i, nodeId, dirNickname, ageMs)) continue;

        String  nickname;
        uint8_t hops;
        bool    hopsKnown;
        lookupNodeSyncInfo(nodeId, nickname, hops, hopsKnown); // always true here — nodeId came from the directory itself

        entries[count++] = String(nodeId) + ":" + nickname + ":" + (hopsKnown ? String(hops) : "?");
    }
    sendEntriesChunked("[NODES]", entries, count);
}

// Pushes "[STATUS] node=<id> nick=<nickname>" to the connected phone.
void pushMeshStatus() {
    if (!Ble.isConnected()) return;
    Ble.send("[STATUS] node=" + String(Mesh.nodeId()) + " nick=" + Mesh.localNickname());
}

// Pushes a full "[ROUTES] dest:nextHop:cost:ageSeconds:path,..." dump (chunked if long); used once on BLE connect.
void pushRouteDump() {
    if (!Ble.isConnected()) return;

    String entries[BLE_SYNC_MAX_ENTRIES];
    int    count = 0;
    int    total = Mesh.routeCount();
    for (int i = 0; i < total && count < BLE_SYNC_MAX_ENTRIES; i++) {
        char          dest, nextHop;
        uint8_t       cost;
        unsigned long ageMs;
        if (Mesh.routeEntryAt(i, dest, nextHop, cost, ageMs)) {
            String path;
            Mesh.routePathFor(dest, path);
            entries[count++] = String(dest) + ":" + String(nextHop) + ":" + String(cost) + ":" + String(ageMs / 1000) + ":" + path;
        }
    }
    sendEntriesChunked("[ROUTES]", entries, count);
}

// Sends a chat (typed over Serial, or from this node's own phone) to a nickname, or broadcasts if none given.
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

// Splits "@nickname message text" into destNickname/message; destNickname empty means broadcast.
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

// Handles "/nick" over Serial, otherwise parses and sends the line as a chat.
void onSerialMessage(const String& line) {
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

// Handles "/nick" from the phone (acked over BLE), otherwise parses and sends the message as a chat.
void onBleMessage(const String& message) {
    if (message.startsWith("/nick ")) {
        String nickname = message.substring(6);
        nickname.trim();
        Mesh.setLocalNickname(nickname);
        Ui.setIdentity(Mesh.nodeId(), nickname);
        Ble.send("[OK] nickname set to " + nickname);
        pushMeshStatus();
        return;
    }

    String destNickname, chatMessage;
    parseAddressedMessage(message, destNickname, chatMessage);
    sendChat(destNickname, chatMessage, SRC_BLE);
}

// Fires the OLED connect/disconnect event, and on connect pushes the phone its initial state.
void onBleConnectionChange(bool connected, const String& deviceName) {
    if (connected) {
        Ui.notifyBleConnected(deviceName);
        pushMeshStatus();
        pushRouteDump();
        pushFullNodeSync();
    } else {
        Ui.notifyBleDisconnected();
    }
}

// Logs and displays an incoming chat, and forwards it to the connected phone over BLE.
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

// Logs the delivery outcome, pushes "[ACK]" over BLE, and updates the OLED if this was the last-sent message.
void onMeshDeliveryStatus(char destNode, uint8_t msgId, MeshDeliveryStatus status, unsigned long elapsedMs) {
    bool delivered = (status == MESH_DELIVERY_ACKED);
    String statusStr = delivered ? "delivered" : "failed";
    Serial.printf("[Mesh] Msg %u to %c: %s\n", msgId, destNode, statusStr.c_str());

    if (Ble.isConnected()) {
        Ble.send("[ACK] id=" + String(msgId) + " dest=" + String(destNode) + " status=" + statusStr);
    }

    if (g_lastSentPending && destNode == g_lastSentDest && msgId == g_lastSentMsgId) {
        g_lastSentPending = false;
        Ui.notifyAckReceived(destNode, delivered, elapsedMs);
    }
}

// Redraws the OLED Mesh screen and marks nodeId dirty for the next coalesced BLE push.
void onMeshDirectoryChanged(char nodeId, const String& nickname, bool removed) {
    Ui.refreshMesh();
    markNodeDirty(nodeId);
}

// Redraws the OLED Mesh screen, pushes a "[ROUTE+]"/"[ROUTE-]" delta, and marks destNode dirty.
void onMeshRouteChanged(char destNode, char nextHop, uint8_t cost, bool removed) {
    Ui.refreshMesh();

    if (Ble.isConnected()) {
        if (removed) {
            Ble.send("[ROUTE-] dest=" + String(destNode));
        } else {
            Ble.send("[ROUTE+] dest=" + String(destNode) + " via=" + String(nextHop) + " cost=" + String(cost));
        }
    }

    markNodeDirty(destNode);
}

// Arduino entry point: brings up Serial, OLED, LoRa, BLE, and the mesh layer, wiring their callbacks.
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

    // Auto-configure the nickname from the node id; a later "/nick <name>" still overrides it.
    String autoNickname = deriveDefaultNickname(nodeId);
    Mesh.setLocalNickname(autoNickname);
    Ui.setIdentity(nodeId, autoNickname);

    Serial.println("─────────────────────────────────");
    Serial.println("  LastLink Firmware - Ready");
    Serial.printf( "  Node ID: %c\n", nodeId);
    Serial.printf( "  Nickname: %s (auto)\n", autoNickname.c_str());
    Serial.println("  Type a message + Enter to TX");
    Serial.println("  '/nick Name'        sets your nickname");
    Serial.println("  '@Name message'     sends to a specific nickname");
    Serial.println("  'message'           broadcasts to the mesh");
    Serial.println("─────────────────────────────────");
}

// Arduino main loop: polls Serial/BLE/mesh/UI, handles any received radio packet, then flushes BLE node-sync.
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
                case MESH_RX_ROUTE_ADV:
                    Serial.println("[Mesh] Route advertisement processed");
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

    flushNodeSync();
}
