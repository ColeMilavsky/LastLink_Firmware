#pragma once

// ─── Mesh Layer ────────────────────────────────────────────────────────────
//
// Flood-routing mesh over LoRa. Each node has a single-character Node ID
// (derived from its BLE device name, e.g. "LastLink-A" -> 'A').
//
// A phone connects to exactly one node over BLE and registers a nickname.
// Each node periodically broadcasts a presence beacon (nodeId, nickname) so
// every node in range builds up a directory mapping nickname -> nodeId.
// When a phone wants to message another phone by nickname, the local node
// looks up the nickname's owning nodeId and addresses the packet to it.
//
// Routing strategy: simple flood. Every node that receives a packet not
// addressed to itself, and not seen before (srcNode+msgId dedup cache),
// decrements TTL and rebroadcasts it. No routing tables — works well for
// small/medium meshes, trades efficiency for simplicity.
//
// Packet layout (binary, sent as the LoRa payload):
//   [0]     msgType   - MSG_CHAT or MSG_PRESENCE
//   [1]     srcNode   - originating node id (ASCII char)
//   [2]     dstNode   - destination node id (ASCII char), or NODE_BROADCAST
//   [3]     msgId     - per-source sequence number, used for dedup
//   [4]     ttl       - hop budget, decremented on every relay, dropped at 0
//   [5..]   payload   - chat text, or "nickname" for presence beacons
//
// To add real routing tables later:
//   1. Track (neighborNode -> lastHeardFrom) when packets are relayed
//   2. Replace blind rebroadcast with next-hop lookup
//   3. Keep flood as fallback when no route is known

#include <Arduino.h>

#define NODE_BROADCAST       '*'      // dstNode value meaning "everyone"
#define MESH_MAX_PAYLOAD     200      // leaves headroom under typical LoRa MTU
#define MESH_HEADER_LEN      5
#define MESH_DEFAULT_TTL     6        // max hops before a packet is dropped
#define MESH_SEEN_CACHE_SIZE 32       // recently seen (srcNode,msgId) pairs
#define MESH_PRESENCE_INTERVAL_MS 30000UL
#define MESH_DIRECTORY_MAX_AGE_MS 120000UL // drop stale directory entries
#define MESH_DIRECTORY_SIZE  16

enum MeshMsgType : uint8_t {
    MSG_CHAT     = 0x01,
    MSG_PRESENCE = 0x02,
};

// Result of attempting to deliver/route an incoming mesh packet.
enum MeshRxAction {
    MESH_RX_FOR_ME,       // chat packet addressed to this node — deliver to phone
    MESH_RX_RELAYED,      // not for us, rebroadcast (or already rebroadcast)
    MESH_RX_DUPLICATE,    // already seen, dropped
    MESH_RX_PRESENCE,     // presence beacon, directory updated, possibly relayed
    MESH_RX_EXPIRED,      // ttl hit 0, dropped
};

// One entry in the nickname -> node directory.
struct DirectoryEntry {
    bool     used        = false;
    char     nodeId       = 0;
    char     nickname[16] = {0};
    unsigned long lastSeen = 0;
};

// Callback invoked when a chat packet addressed to this node arrives.
// senderNickname may be empty if the sender's nickname isn't known yet.
typedef void (*MeshChatCallback)(const String& senderNickname, char srcNode, const String& message);

// Callback invoked whenever the directory changes (new node/nickname seen,
// or an entry expires) — handy for UI updates.
typedef void (*MeshDirectoryCallback)();

class MeshLayer {
public:
    MeshLayer();

    // nodeId: this node's single-char identity, derived from BLE name.
    void begin(char nodeId);
    void update();   // call every loop() — handles presence beacon timing & expiry

    // Local phone registration (called from BLE handler when phone sets a nickname).
    void setLocalNickname(const String& nickname);
    String localNickname() const { return _localNickname; }
    char   nodeId() const { return _nodeId; }

    // Send a chat message from the locally-connected phone to a nickname.
    // Looks up nickname in the directory; if unknown, sends as broadcast so
    // it still has a chance to be picked up (and the directory to catch up).
    // Returns false only on a hard local failure (radio error).
    bool sendChat(const String& destNickname, const String& message);

    // Called by lora.cpp when a raw packet is received off the radio.
    // Returns the action taken so callers (main.cpp) can decide what to log/show.
    MeshRxAction handleIncomingPacket(const uint8_t* data, size_t len, int rssi, float snr);

    void onChatReceived(MeshChatCallback callback);
    void onDirectoryChanged(MeshDirectoryCallback callback);

    // Directory introspection (for UI / debugging).
    int  directoryCount() const;
    bool directoryEntryAt(int index, char& outNodeId, String& outNickname) const;
    bool resolveNickname(const String& nickname, char& outNodeId) const;

private:
    char     _nodeId;
    String   _localNickname;
    uint8_t  _nextMsgId;

    MeshChatCallback       _chatCallback;
    MeshDirectoryCallback  _directoryCallback;

    DirectoryEntry _directory[MESH_DIRECTORY_SIZE];

    // Dedup cache of recently seen (srcNode, msgId) pairs.
    struct SeenEntry { bool used = false; char srcNode = 0; uint8_t msgId = 0; };
    SeenEntry _seenCache[MESH_SEEN_CACHE_SIZE];
    uint8_t   _seenCacheIndex;

    unsigned long _lastPresenceBroadcast;

    bool _wasRecentlySeen(char srcNode, uint8_t msgId);
    void _markSeen(char srcNode, uint8_t msgId);

    void _updateDirectory(char nodeId, const String& nickname);
    void _expireDirectory();

    void _broadcastPresence();
    void _relayPacket(uint8_t* data, size_t len); // decrements ttl in-place, sends

    void _buildPacket(uint8_t* out, size_t& outLen, MeshMsgType type,
                       char srcNode, char dstNode, uint8_t msgId, uint8_t ttl,
                       const String& payload);
};

extern MeshLayer Mesh;