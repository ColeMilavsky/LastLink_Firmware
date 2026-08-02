#pragma once

// ─── Mesh Layer ────────────────────────────────────────────────────────────
//
// Flood-routing mesh over LoRa, with next-hop route learning layered on top.
// Each node has a single-character Node ID (derived from its BLE device
// name, e.g. "LastLink-A" -> 'A').
//
// A phone connects to exactly one node over BLE and registers a nickname.
// Each node periodically broadcasts a presence beacon (nodeId, nickname) so
// every node in range builds up a directory mapping nickname -> nodeId.
// When a phone wants to message another phone by nickname, the local node
// looks up the nickname's owning nodeId and addresses the packet to it.
//
// Routing strategy: every received packet (chat, presence, or ack) carries
// a prevHop field identifying whoever just (re)transmitted it, which lets
// any node passively learn "prevHop is a direct neighbor of mine" and "the
// original srcNode is reachable via prevHop, at this hop cost" — no separate
// discovery protocol needed, routes are learned from traffic already
// flowing. A nextHop field lets a sender (or relaying node) address a
// packet at a specific elected neighbor: only that neighbor re-relays it,
// everyone else just overhears it for route learning and drops it. When no
// route is known yet, nextHop is NODE_BROADCAST and every node in range
// relays (the original flood behavior) — this is the fallback path.
//
// Packet layout (binary, sent as the LoRa payload):
//   [0]     msgType   - MSG_CHAT, MSG_PRESENCE, or MSG_ACK
//   [1]     srcNode   - originating node id (ASCII char)
//   [2]     dstNode   - final destination node id (ASCII char), or NODE_BROADCAST
//   [3]     prevHop   - node id that just (re)transmitted this frame; rewritten
//                       to the relaying node's own id on every hop
//   [4]     nextHop   - node id elected to relay this next, or NODE_BROADCAST
//                       to mean "no route known, everyone should flood-relay"
//   [5]     msgId     - per-source sequence number, used for dedup
//   [6]     ttl       - hop budget, decremented on every relay, dropped at 0
//   [7..]   payload   - chat text, "nickname" for presence beacons, or a
//                       single byte (the acked msgId) for MSG_ACK
//
// Hop cost is derived, not transmitted: a packet arriving with ttl consumed
// (MESH_DEFAULT_TTL - ttl) hops has traveled that many relays already, so
// cost-to-srcNode = (MESH_DEFAULT_TTL - ttl) + 1 from the receiver's point
// of view. This keeps the header at 7 bytes instead of 8.

#include <Arduino.h>

#define NODE_BROADCAST       '*'      // dstNode/nextHop value meaning "everyone"
#define MESH_MAX_PAYLOAD     200      // leaves headroom under typical LoRa MTU
#define MESH_HEADER_LEN      7
#define MESH_DEFAULT_TTL     6        // max hops before a packet is dropped
#define MESH_SEEN_CACHE_SIZE 32       // recently seen (srcNode,msgId) pairs
#define MESH_PRESENCE_INTERVAL_MS 5000UL
// A node is considered dead after missing two consecutive heartbeats
// (~60s at the current interval), plus a small grace margin so ordinary
// scheduling/relay jitter doesn't evict a node that's only barely late.
#define MESH_NODE_TIMEOUT_MS (2 * MESH_PRESENCE_INTERVAL_MS + 5000UL)
#define MESH_DIRECTORY_SIZE  16
#define MESH_ROUTE_TABLE_SIZE 16
#define MESH_ROUTE_MAX_AGE_MS 20000UL     // route considered stale/replaceable
// A cheaper alternate next hop must be independently observed this many
// times before an active (non-stale) route actually switches to it — a
// single marginal/one-off reception (e.g. an occasional direct link that
// isn't really reliable) shouldn't be enough to shortcut a route away from
// an already-working multi-hop path. Does not apply to brand-new
// destinations or to replacing an already-stale entry — only to displacing
// a route that's currently active via a different next hop.
#define MESH_ROUTE_CONFIRM_COUNT 2
#define MESH_PENDING_ACK_SIZE 8
#define MESH_ACK_TIMEOUT_MS   30000UL      // absolute ceiling on an in-flight send, regardless of retry math
// Resend cadence scales with hop count: hopCount * this. When no route is
// known yet at send time, hop count defaults to MESH_DEFAULT_TTL (worst
// case) rather than assuming a fast direct link — see sendChat().
#define MESH_RETRY_INTERVAL_PER_HOP_MS 1000UL
#define MESH_MAX_RETRIES 15                 // total attempts = 1 original send + this many retries

enum MeshMsgType : uint8_t {
    MSG_CHAT     = 0x01,
    MSG_PRESENCE = 0x02,
    MSG_ACK      = 0x03,
};

// Result of attempting to deliver/route an incoming mesh packet.
enum MeshRxAction {
    MESH_RX_FOR_ME,       // chat packet addressed to this node — deliver to phone
    MESH_RX_RELAYED,      // not for us, rebroadcast (or already rebroadcast)
    MESH_RX_DUPLICATE,    // already seen, dropped
    MESH_RX_PRESENCE,     // presence beacon, directory updated, possibly relayed
    MESH_RX_EXPIRED,      // ttl hit 0, dropped
    MESH_RX_ACK,          // delivery acknowledgement received/relayed
};

// Delivery status for a chat message this node sent, tracked via ACK.
enum MeshDeliveryStatus {
    MESH_DELIVERY_ACKED,
    MESH_DELIVERY_FAILED, // timed out with no ACK
};

// One entry in the nickname -> node directory.
struct DirectoryEntry {
    bool     used        = false;
    char     nodeId       = 0;
    char     nickname[16] = {0};
    unsigned long lastSeen = 0;
};

// One entry in the destination -> next-hop routing table.
struct RouteEntry {
    bool     used         = false;
    char     destNode      = 0;
    char     nextHop       = 0;
    uint8_t  cost          = 0; // hop count to destNode
    unsigned long lastUpdated = 0;

    // Staging area for a cheaper candidate next hop that hasn't yet been
    // corroborated enough times to actually replace the active route — see
    // MeshLayer::_learnRoute().
    char    candidateNextHop = 0;
    uint8_t candidateCost    = 0;
    uint8_t candidateSeen    = 0;
};

// One in-flight chat message awaiting (or having received) an ACK. Doubles
// as the retry-tracking record: the same packet (same msgId, so the
// destination's dedup cache recognizes redundant copies) is retransmitted
// on an interval until acked, retries are exhausted, or the absolute
// timeout ceiling is hit.
struct PendingAck {
    bool          used            = false;
    char          destNode        = 0;
    uint8_t       msgId           = 0;
    unsigned long sentAt          = 0; // original send time — for round-trip-time reporting and the absolute timeout ceiling
    unsigned long lastSentAt      = 0; // time of the most recent (re)transmission — gates the next retry
    unsigned long retryIntervalMs = 0; // resend cadence for this message, frozen at send time (hopCount * MESH_RETRY_INTERVAL_PER_HOP_MS)
    uint8_t       retryCount      = 0; // retries sent so far (0 = only the original send)
    String        message;            // original chat text, so a retry can rebuild an identical packet with a freshly-looked-up next hop
};

// Callback invoked when a chat packet addressed to this node arrives.
// senderNickname may be empty if the sender's nickname isn't known yet.
typedef void (*MeshChatCallback)(const String& senderNickname, char srcNode, const String& message);

// Callback invoked whenever the directory changes (new node/nickname seen,
// or an entry expires) — handy for UI updates.
typedef void (*MeshDirectoryCallback)();

// Callback invoked when a chat message this node sent is acked or times out.
// elapsedMs is the round-trip time in milliseconds from send to ack — only
// meaningful when status==MESH_DELIVERY_ACKED; it is always 0 for
// MESH_DELIVERY_FAILED, since a timeout has no real round trip to report.
typedef void (*MeshDeliveryCallback)(char destNode, uint8_t msgId, MeshDeliveryStatus status, unsigned long elapsedMs);

// Callback invoked when the routing table gains/updates or loses an entry.
// Fired only on substantive changes (new destination, changed next hop/cost,
// or expiry) — not on every packet that merely refreshes a timestamp.
// removed=true means destNode's entry was dropped (nextHop/cost are 0).
typedef void (*MeshRouteCallback)(char destNode, char nextHop, uint8_t cost, bool removed);

class MeshLayer {
public:
    MeshLayer();

    // nodeId: this node's single-char identity, derived from BLE name.
    void begin(char nodeId);
    void update();   // call every loop() — handles presence beacon timing, expiry, ack timeouts

    // Local phone registration (called from BLE handler when phone sets a nickname).
    void setLocalNickname(const String& nickname);
    String localNickname() const { return _localNickname; }
    char   nodeId() const { return _nodeId; }

    // Send a chat message from the locally-connected phone to a nickname.
    // Looks up nickname in the directory; if unknown, sends as broadcast so
    // it still has a chance to be picked up (and the directory to catch up).
    // Unicast sends (resolved nickname) are tracked for delivery ACK.
    // outDestNode/outMsgId (if non-null) receive the resolved destination
    // node and assigned sequence number, so a caller (e.g. main.cpp) can
    // correlate a later delivery-status callback back to this specific send.
    // Returns false only on a hard local failure (radio error).
    bool sendChat(const String& destNickname, const String& message,
                  char* outDestNode = nullptr, uint8_t* outMsgId = nullptr);

    // Called by lora.cpp when a raw packet is received off the radio.
    // Returns the action taken so callers (main.cpp) can decide what to log/show.
    MeshRxAction handleIncomingPacket(const uint8_t* data, size_t len, int rssi, float snr);

    void onChatReceived(MeshChatCallback callback);
    void onDirectoryChanged(MeshDirectoryCallback callback);
    void onDeliveryStatus(MeshDeliveryCallback callback);
    void onRouteChanged(MeshRouteCallback callback);

    // Directory introspection (for UI / debugging).
    // outAgeMs is how long ago (in ms) this node's last heartbeat was heard,
    // for showing a "missed a beacon" indicator once it exceeds one interval.
    int  directoryCount() const;
    bool directoryEntryAt(int index, char& outNodeId, String& outNickname, unsigned long& outAgeMs) const;
    bool resolveNickname(const String& nickname, char& outNodeId) const;

    // Routing table introspection (for UI / debugging).
    // outAgeMs is how long ago (in ms) the entry was last refreshed, for
    // showing a fresh/stale indicator.
    int  routeCount() const;
    bool routeEntryAt(int index, char& outDest, char& outNextHop, uint8_t& outCost,
                       unsigned long& outAgeMs) const;

private:
    char     _nodeId;
    String   _localNickname;
    uint8_t  _nextMsgId;

    MeshChatCallback       _chatCallback;
    MeshDirectoryCallback  _directoryCallback;
    MeshDeliveryCallback   _deliveryCallback;
    MeshRouteCallback      _routeCallback;

    DirectoryEntry _directory[MESH_DIRECTORY_SIZE];
    RouteEntry     _routes[MESH_ROUTE_TABLE_SIZE];
    PendingAck     _pendingAcks[MESH_PENDING_ACK_SIZE];

    // Dedup cache of recently seen (msgType, srcNode, msgId) triples. msgType
    // is part of the key so a chat and an ack (or any other type) from the
    // same node reusing the same sequence number are never mistaken for each
    // other — a node's _nextMsgId counter is shared across every packet type
    // it sends, so the type alone can't be inferred from srcNode+msgId.
    struct SeenEntry { bool used = false; uint8_t msgType = 0; char srcNode = 0; uint8_t msgId = 0; };
    SeenEntry _seenCache[MESH_SEEN_CACHE_SIZE];
    uint8_t   _seenCacheIndex;

    unsigned long _lastPresenceBroadcast;

    bool _wasRecentlySeen(uint8_t msgType, char srcNode, uint8_t msgId);
    void _markSeen(uint8_t msgType, char srcNode, uint8_t msgId);

    void _updateDirectory(char nodeId, const String& nickname);
    void _expireDirectory();

    // Drops any routing table entry that leads to or through a node that
    // just timed out — called from _expireDirectory() on eviction.
    void _purgeRoutesForNode(char deadNode);

    // Learns routes from any received packet: prevHop as a direct 1-hop
    // neighbor, and srcNode as reachable via prevHop at the derived cost.
    void _learnRoute(char destNode, char viaNextHop, uint8_t cost);
    // outCost (if non-null) receives the route's hop count — used by
    // sendChat() to scale the retry interval to hop count.
    bool _lookupRoute(char destNode, char& outNextHop, uint8_t* outCost = nullptr) const;
    void _expireRoutes();

    void _addPendingAck(char destNode, uint8_t msgId, const String& message, unsigned long retryIntervalMs);
    void _resolvePendingAck(char destNode, uint8_t msgId);

    // Called every update(): retransmits any in-flight send whose retry
    // interval has elapsed (up to MESH_MAX_RETRIES times), or fails it once
    // retries are exhausted or the absolute MESH_ACK_TIMEOUT_MS ceiling hits.
    void _servicePendingAcks();

    // Rebuilds and resends a pending send's packet — same msgId (so the
    // destination's dedup cache recognizes and drops the extra copy if the
    // original already arrived), fresh ttl, and a freshly-looked-up next hop
    // in case routing has changed since the message was first sent.
    void _retransmitChat(const PendingAck& pending);

    void _sendAck(char toNode, uint8_t ackedMsgId);

    void _broadcastPresence();

    // Relays a packet: decrements ttl, rewrites prevHop to us, and sets
    // nextHop to our own route-table lookup for dstNode (or broadcast if we
    // don't have one either). Sends unconditionally — caller has already
    // decided this node should relay.
    void _relayPacket(const uint8_t* data, size_t len, char dstNode);

    void _buildPacket(uint8_t* out, size_t& outLen, MeshMsgType type,
                       char srcNode, char dstNode, char nextHop, uint8_t msgId, uint8_t ttl,
                       const uint8_t* payload, size_t payloadLen);
    void _buildPacket(uint8_t* out, size_t& outLen, MeshMsgType type,
                       char srcNode, char dstNode, char nextHop, uint8_t msgId, uint8_t ttl,
                       const String& payload);
};

extern MeshLayer Mesh;
