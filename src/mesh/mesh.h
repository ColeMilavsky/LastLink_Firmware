#pragma once

// ─── Mesh Layer ────────────────────────────────────────────────────────────
//
// Flood-routing mesh over LoRa with next-hop route learning layered on top.
// Each node has a single-character Node ID (derived from its BLE device name).
//
// A phone connects to exactly one node over BLE and registers a nickname.
// Nodes broadcast presence beacons so every node builds a nickname -> nodeId
// directory; sending to a nickname resolves it to a nodeId and addresses the
// packet there.
//
// Routes are learned passively: every packet carries a prevHop (whoever just
// relayed it), so any node overhearing it learns "prevHop is a neighbor" and
// "srcNode is reachable via prevHop". A nextHop field elects one relayer per
// packet; NODE_BROADCAST means no route is known yet and everyone floods.
//
// MSG_ROUTE_ADV packets add path-vector info (BGP AS-PATH style) on top of
// that, so nodes eventually learn full hop chains, not just next-hop+cost.
// Never relayed — each node periodically re-advertises its own known routes
// to its direct neighbors, and multi-hop knowledge propagates by each hop
// folding what it hears into its own table and re-advertising that. Loop
// prevention: reject any advertised path that already contains our own id.
//
// Packet layout (binary, sent as the LoRa payload):
//   [0]     msgType   - MSG_CHAT, MSG_PRESENCE, MSG_ACK, or MSG_ROUTE_ADV
//   [1]     srcNode   - originating node id (ASCII char)
//   [2]     dstNode   - final destination node id (ASCII char), or NODE_BROADCAST
//   [3]     prevHop   - node id that just (re)transmitted this frame
//   [4]     nextHop   - elected relayer, or NODE_BROADCAST to mean "flood"
//   [5]     msgId     - per-source sequence number, used for dedup
//   [6]     ttl       - hop budget, decremented on every relay, dropped at 0
//   [7..]   payload   - chat text, nickname (presence), acked msgId (ack), or
//                       for MSG_ROUTE_ADV: [entryCount, then per entry:
//                       destNode, pathLen, path[pathLen]] (path excludes the
//                       advertiser, which is already srcNode in the header)
//
// Hop cost is derived, not transmitted: cost-to-srcNode = (MESH_DEFAULT_TTL -
// ttl) + 1 from the receiver's point of view. Keeps the header at 7 bytes.

#include <Arduino.h>

#define NODE_BROADCAST       '*'      // dstNode/nextHop value meaning "everyone"
#define MESH_MAX_PAYLOAD     200      // leaves headroom under typical LoRa MTU
#define MESH_HEADER_LEN      7
#define MESH_DEFAULT_TTL     6        // max hops before a packet is dropped
#define MESH_SEEN_CACHE_SIZE 32       // recently seen (srcNode,msgId) pairs
#define MESH_PRESENCE_INTERVAL_MS 5000UL
#define MESH_NODE_TIMEOUT_MS (2 * MESH_PRESENCE_INTERVAL_MS + 5000UL) // ~2 missed beacons + grace margin
#define MESH_DIRECTORY_SIZE  16
#define MESH_ROUTE_TABLE_SIZE 16
#define MESH_ROUTE_MAX_AGE_MS 20000UL     // route considered stale/replaceable
#define MESH_ROUTE_CONFIRM_COUNT 2        // times a cheaper next hop must be reseen before it displaces an active route
#define MESH_ROUTE_MIN_PROMOTE_SNR 0.0f   // min link quality (dB) for a next hop to displace an active route
#define MESH_LINK_QUALITY_EWMA_ALPHA 0.3f // weight of each new SNR sample in the rolling link-quality average
#define MESH_LINK_QUALITY_FAIL_PENALTY_DB 10.0f // subtracted from a neighbor's link quality on a real delivery failure
#define MESH_PENDING_ACK_SIZE 8
#define MESH_ACK_TIMEOUT_MS   30000UL      // absolute ceiling on an in-flight send, regardless of retry math
#define MESH_RETRY_INTERVAL_PER_HOP_MS 1000UL // resend cadence = hopCount * this
#define MESH_MAX_RETRIES 15                 // total attempts = 1 original send + this many retries
#define MESH_ROUTE_ADV_INTERVAL_MS 10000UL  // how often a node broadcasts its known routes to neighbors

enum MeshMsgType : uint8_t {
    MSG_CHAT       = 0x01,
    MSG_PRESENCE   = 0x02,
    MSG_ACK        = 0x03,
    MSG_ROUTE_ADV  = 0x04, // path-vector route advertisement; never relayed
};

// Result of attempting to deliver/route an incoming mesh packet.
enum MeshRxAction {
    MESH_RX_FOR_ME,       // chat packet addressed to this node — deliver to phone
    MESH_RX_RELAYED,      // not for us, rebroadcast (or already rebroadcast)
    MESH_RX_DUPLICATE,    // already seen, dropped
    MESH_RX_PRESENCE,     // presence beacon, directory updated, possibly relayed
    MESH_RX_EXPIRED,      // ttl hit 0, dropped
    MESH_RX_ACK,          // delivery acknowledgement received/relayed
    MESH_RX_ROUTE_ADV,    // route advertisement received and processed (never relayed)
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

    // Staged candidate next hop awaiting confirmation — see MeshLayer::_learnRoute().
    char    candidateNextHop = 0;
    uint8_t candidateCost    = 0;
    uint8_t candidateSeen    = 0;

    // Rolling link-quality signal (EWMA of SNR in dB); only meaningful on a
    // direct-neighbor entry (destNode == nextHop). hasQuality false until first sample.
    float linkQuality = 0;
    bool  hasQuality   = false;

    // Full path to destNode, one char per hop, not including this node.
    // pathLen==0 means only next-hop+cost are known (passive-learning fallback).
    char    path[MESH_DEFAULT_TTL] = {0};
    uint8_t pathLen = 0;
};

// One in-flight chat message awaiting an ACK; also the retry-tracking record.
struct PendingAck {
    bool          used            = false;
    char          destNode        = 0;
    uint8_t       msgId           = 0;
    unsigned long sentAt          = 0; // original send time, for RTT + absolute timeout
    unsigned long lastSentAt      = 0; // gates the next retry
    unsigned long retryIntervalMs = 0; // resend cadence, frozen at send time
    uint8_t       retryCount      = 0;
    String        message;            // original text, so a retry can rebuild the packet
};

// Callback invoked when a chat packet addressed to this node arrives.
typedef void (*MeshChatCallback)(const String& senderNickname, char srcNode, const String& message);

// Callback invoked whenever the directory changes (new/changed nickname, or eviction).
typedef void (*MeshDirectoryCallback)(char nodeId, const String& nickname, bool removed);

// Callback invoked when a sent chat is acked or times out (elapsedMs is RTT, 0 if failed).
typedef void (*MeshDeliveryCallback)(char destNode, uint8_t msgId, MeshDeliveryStatus status, unsigned long elapsedMs);

// Callback invoked when the routing table gains/updates or loses an entry.
typedef void (*MeshRouteCallback)(char destNode, char nextHop, uint8_t cost, bool removed);

class MeshLayer {
public:
    MeshLayer();

    void begin(char nodeId);
    void update();   // call every loop() — handles beacon/route-adv timing, expiry, ack timeouts

    // Local phone registration (called from BLE handler when phone sets a nickname).
    void setLocalNickname(const String& nickname);
    String localNickname() const { return _localNickname; }
    char   nodeId() const { return _nodeId; }

    // Sends a chat to a nickname (broadcasts if unresolved); tracks unicast sends for delivery ACK.
    bool sendChat(const String& destNickname, const String& message,
                  char* outDestNode = nullptr, uint8_t* outMsgId = nullptr);

    // Called by lora.cpp on a raw radio reception; returns the action taken.
    MeshRxAction handleIncomingPacket(const uint8_t* data, size_t len, int rssi, float snr);

    void onChatReceived(MeshChatCallback callback);
    void onDirectoryChanged(MeshDirectoryCallback callback);
    void onDeliveryStatus(MeshDeliveryCallback callback);
    void onRouteChanged(MeshRouteCallback callback);

    // Directory introspection (for UI / debugging).
    int  directoryCount() const;
    bool directoryEntryAt(int index, char& outNodeId, String& outNickname, unsigned long& outAgeMs) const;
    bool resolveNickname(const String& nickname, char& outNodeId) const;

    // Routing table introspection (for UI / debugging).
    int  routeCount() const;
    bool routeEntryAt(int index, char& outDest, char& outNextHop, uint8_t& outCost,
                       unsigned long& outAgeMs) const;

    // Full known path to destNode, if established; empty if only next-hop+cost are known so far.
    bool routePathFor(char destNode, String& outPath) const;

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

    // Dedup cache of recently seen (msgType, srcNode, msgId) triples.
    struct SeenEntry { bool used = false; uint8_t msgType = 0; char srcNode = 0; uint8_t msgId = 0; };
    SeenEntry _seenCache[MESH_SEEN_CACHE_SIZE];
    uint8_t   _seenCacheIndex;

    unsigned long _lastPresenceBroadcast;
    unsigned long _lastRouteAdvBroadcast;

    bool _wasRecentlySeen(uint8_t msgType, char srcNode, uint8_t msgId);
    void _markSeen(uint8_t msgType, char srcNode, uint8_t msgId);

    void _updateDirectory(char nodeId, const String& nickname);
    void _expireDirectory();

    // Drops any route leading to/through a node that just timed out, and re-advertises if anything was removed.
    void _purgeRoutesForNode(char deadNode);

    // Learns/updates a route to destNode via viaNextHop; path/pathLen attach path-vector info when known.
    void _learnRoute(char destNode, char viaNextHop, uint8_t cost,
                      const char* path = nullptr, uint8_t pathLen = 0);
    bool _lookupRoute(char destNode, char& outNextHop, uint8_t* outCost = nullptr) const;
    void _expireRoutes();

    // Broadcasts this node's current route advertisement (routes it has a full path for).
    void _broadcastRouteAdvertisement();

    // Parses a neighbor's route advertisement, applies the path-vector loop check, and feeds accepted routes to _learnRoute().
    void _handleRouteAdvertisement(char advertiser, const uint8_t* payload, size_t payloadLen);

    // Blends a fresh SNR reading into a direct neighbor's rolling link-quality EWMA.
    void _updateNeighborQuality(char neighborId, float snr);
    bool _neighborQuality(char neighborId, float& outSnr) const;

    // A real delivery failure invalidates destNode's route and dings the next hop's link quality.
    void _penalizeRouteFailure(char destNode);

    void _addPendingAck(char destNode, uint8_t msgId, const String& message, unsigned long retryIntervalMs);
    void _resolvePendingAck(char destNode, uint8_t msgId);

    // Retransmits or fails any in-flight send whose retry interval/timeout has elapsed.
    void _servicePendingAcks();

    // Rebuilds and resends a pending send's packet with a freshly-looked-up next hop.
    void _retransmitChat(const PendingAck& pending);

    void _sendAck(char toNode, uint8_t ackedMsgId);

    void _broadcastPresence();

    // Relays a packet: decrements ttl, rewrites prevHop, elects our own next hop.
    void _relayPacket(const uint8_t* data, size_t len, char dstNode);

    void _buildPacket(uint8_t* out, size_t& outLen, MeshMsgType type,
                       char srcNode, char dstNode, char nextHop, uint8_t msgId, uint8_t ttl,
                       const uint8_t* payload, size_t payloadLen);
    void _buildPacket(uint8_t* out, size_t& outLen, MeshMsgType type,
                       char srcNode, char dstNode, char nextHop, uint8_t msgId, uint8_t ttl,
                       const String& payload);
};

extern MeshLayer Mesh;
