#include "mesh.h"
#include "../lora/lora.h"

MeshLayer Mesh;

MeshLayer::MeshLayer()
    : _nodeId(0),
      _nextMsgId(0),
      _chatCallback(nullptr),
      _directoryCallback(nullptr),
      _deliveryCallback(nullptr),
      _routeCallback(nullptr),
      _seenCacheIndex(0),
      _lastPresenceBroadcast(0),
      _lastRouteAdvBroadcast(0) {}

// Stores the node id and arms an immediate presence beacon for the first update() call.
void MeshLayer::begin(char nodeId) {
    _nodeId = nodeId;
    _lastPresenceBroadcast = 0; // force an immediate beacon on first update()
    Serial.printf("[Mesh] Node ID: %c\n", _nodeId);
}

// Per-loop tick: presence beacon, route advertisement, and expiry/retry sweeps.
void MeshLayer::update() {
    unsigned long now = millis();

    if (now - _lastPresenceBroadcast >= MESH_PRESENCE_INTERVAL_MS || _lastPresenceBroadcast == 0) {
        _broadcastPresence();
        _lastPresenceBroadcast = now;
    }

    if (now - _lastRouteAdvBroadcast >= MESH_ROUTE_ADV_INTERVAL_MS || _lastRouteAdvBroadcast == 0) {
        _broadcastRouteAdvertisement();
        _lastRouteAdvBroadcast = now;
    }

    _expireDirectory();
    _expireRoutes();
    _servicePendingAcks();
}

// Records the local nickname and announces it immediately instead of waiting for the timer.
void MeshLayer::setLocalNickname(const String& nickname) {
    _localNickname = nickname;
    Serial.println("[Mesh] Local nickname set to: " + nickname);

    _updateDirectory(_nodeId, _localNickname);
    _broadcastPresence();
    _lastPresenceBroadcast = millis();
}

// Resolves the nickname, sends a MSG_CHAT packet, and registers a pending ACK for unicast sends.
bool MeshLayer::sendChat(const String& destNickname, const String& message,
                          char* outDestNode, uint8_t* outMsgId) {
    char destNode = NODE_BROADCAST;
    bool resolved = resolveNickname(destNickname, destNode);

    if (!resolved) {
        Serial.println("[Mesh] Nickname \"" + destNickname + "\" not in directory yet — broadcasting");
        destNode = NODE_BROADCAST;
    }

    // Prefer a known route; fall back to flooding with a worst-case hop count for retry pacing.
    char    chosenNextHop = NODE_BROADCAST;
    uint8_t hopCount = MESH_DEFAULT_TTL;
    if (destNode != NODE_BROADCAST) {
        uint8_t routeCost = 0;
        if (_lookupRoute(destNode, chosenNextHop, &routeCost)) {
            hopCount = routeCost;
        }
    }
    if (hopCount == 0) hopCount = 1; // defensive floor — never a zero-length retry interval

    uint8_t packet[MESH_HEADER_LEN + MESH_MAX_PAYLOAD];
    size_t  packetLen = 0;
    uint8_t msgId = _nextMsgId++;

    _buildPacket(packet, packetLen, MSG_CHAT, _nodeId, destNode, chosenNextHop, msgId, MESH_DEFAULT_TTL, message);

    _markSeen(MSG_CHAT, _nodeId, msgId);

    // Only unicast sends expect an ACK — there's no single "delivered" for a broadcast.
    if (destNode != NODE_BROADCAST) {
        unsigned long retryIntervalMs = (unsigned long)hopCount * MESH_RETRY_INTERVAL_PER_HOP_MS;
        _addPendingAck(destNode, msgId, message, retryIntervalMs);
    }

    if (outDestNode) *outDestNode = destNode;
    if (outMsgId)    *outMsgId    = msgId;

    int state = loraSendRaw(packet, packetLen);
    return state == 0; // RADIOLIB_ERR_NONE
}

// Parses/dedups/learns-from an incoming packet, then dispatches by msgType.
MeshRxAction MeshLayer::handleIncomingPacket(const uint8_t* data, size_t len, int rssi, float snr) {
    if (len < MESH_HEADER_LEN) {
        return MESH_RX_DUPLICATE; // malformed, treat as noise
    }

    uint8_t msgType = data[0];
    char    srcNode  = (char)data[1];
    char    dstNode  = (char)data[2];
    char    prevHop  = (char)data[3];
    char    nextHop  = (char)data[4];
    uint8_t msgId    = data[5];
    uint8_t ttl      = data[6];

    String payload;
    for (size_t i = MESH_HEADER_LEN; i < len; i++) {
        payload += (char)data[i];
    }

    // Ignore our own transmissions echoing back.
    if (srcNode == _nodeId) {
        return MESH_RX_DUPLICATE;
    }

    if (_wasRecentlySeen(msgType, srcNode, msgId)) {
        // Likely our ack was lost, not the original chat — re-send it so the sender's retry can succeed.
        if (msgType == MSG_CHAT && dstNode == _nodeId) {
            _sendAck(srcNode, msgId);
        }
        return MESH_RX_DUPLICATE;
    }
    _markSeen(msgType, srcNode, msgId);

    // prevHop is always a direct neighbor; srcNode is reachable via prevHop at the derived hop cost.
    if (prevHop != _nodeId) {
        _learnRoute(prevHop, prevHop, 1);
        _updateNeighborQuality(prevHop, snr);
        if (srcNode != prevHop) {
            uint8_t hopsTraveled = MESH_DEFAULT_TTL - ttl;
            _learnRoute(srcNode, prevHop, hopsTraveled + 1);
        }
    }

    bool eligibleRelay = (nextHop == _nodeId) || (nextHop == NODE_BROADCAST);

    if (msgType == MSG_PRESENCE) {
        _updateDirectory(srcNode, payload);

        if (eligibleRelay && ttl > 1) {
            _relayPacket(data, len, dstNode);
        }
        return MESH_RX_PRESENCE;
    }

    if (msgType == MSG_ROUTE_ADV) {
        // Never relayed, regardless of nextHop/ttl — see mesh.h's top-of-file comment.
        _handleRouteAdvertisement(srcNode, data + MESH_HEADER_LEN, len - MESH_HEADER_LEN);
        return MESH_RX_ROUTE_ADV;
    }

    if (msgType == MSG_ACK) {
        if (dstNode == _nodeId) {
            if (len > MESH_HEADER_LEN) {
                uint8_t ackedMsgId = data[MESH_HEADER_LEN];
                _resolvePendingAck(srcNode, ackedMsgId);
            }
            return MESH_RX_ACK;
        }

        if (eligibleRelay) {
            if (ttl > 1) {
                _relayPacket(data, len, dstNode);
                return MESH_RX_ACK;
            }
            return MESH_RX_EXPIRED;
        }
        return MESH_RX_ACK; // seen and learned from, not ours to relay
    }

    if (msgType == MSG_CHAT) {
        bool forUs = (dstNode == _nodeId) || (dstNode == NODE_BROADCAST);

        if (forUs) {
            if (_chatCallback) {
                String senderNickname;
                for (int i = 0; i < MESH_DIRECTORY_SIZE; i++) {
                    if (_directory[i].used && _directory[i].nodeId == srcNode) {
                        senderNickname = _directory[i].nickname;
                        break;
                    }
                }
                _chatCallback(senderNickname, srcNode, payload);
            }

            if (dstNode == _nodeId) {
                _sendAck(srcNode, msgId);
                return MESH_RX_FOR_ME;
            }
        }

        // Broadcast chat, or addressed to someone else: relay only if we're the elected next hop.
        if (eligibleRelay) {
            if (ttl > 1) {
                _relayPacket(data, len, dstNode);
                return forUs ? MESH_RX_FOR_ME : MESH_RX_RELAYED;
            }
            return forUs ? MESH_RX_FOR_ME : MESH_RX_EXPIRED;
        }

        return forUs ? MESH_RX_FOR_ME : MESH_RX_RELAYED;
    }

    return MESH_RX_DUPLICATE; // unknown type, drop
}

void MeshLayer::onChatReceived(MeshChatCallback callback) {
    _chatCallback = callback;
}

void MeshLayer::onDirectoryChanged(MeshDirectoryCallback callback) {
    _directoryCallback = callback;
}

void MeshLayer::onDeliveryStatus(MeshDeliveryCallback callback) {
    _deliveryCallback = callback;
}

void MeshLayer::onRouteChanged(MeshRouteCallback callback) {
    _routeCallback = callback;
}

int MeshLayer::directoryCount() const {
    int count = 0;
    for (int i = 0; i < MESH_DIRECTORY_SIZE; i++) {
        if (_directory[i].used) count++;
    }
    return count;
}

bool MeshLayer::directoryEntryAt(int index, char& outNodeId, String& outNickname, unsigned long& outAgeMs) const {
    int count = 0;
    for (int i = 0; i < MESH_DIRECTORY_SIZE; i++) {
        if (!_directory[i].used) continue;
        if (count == index) {
            outNodeId   = _directory[i].nodeId;
            outNickname = _directory[i].nickname;
            outAgeMs    = millis() - _directory[i].lastSeen;
            return true;
        }
        count++;
    }
    return false;
}

bool MeshLayer::resolveNickname(const String& nickname, char& outNodeId) const {
    for (int i = 0; i < MESH_DIRECTORY_SIZE; i++) {
        if (_directory[i].used && nickname.equalsIgnoreCase(_directory[i].nickname)) {
            outNodeId = _directory[i].nodeId;
            return true;
        }
    }
    return false;
}

int MeshLayer::routeCount() const {
    int count = 0;
    for (int i = 0; i < MESH_ROUTE_TABLE_SIZE; i++) {
        if (_routes[i].used) count++;
    }
    return count;
}

bool MeshLayer::routeEntryAt(int index, char& outDest, char& outNextHop, uint8_t& outCost,
                              unsigned long& outAgeMs) const {
    int count = 0;
    for (int i = 0; i < MESH_ROUTE_TABLE_SIZE; i++) {
        if (!_routes[i].used) continue;
        if (count == index) {
            outDest    = _routes[i].destNode;
            outNextHop = _routes[i].nextHop;
            outCost    = _routes[i].cost;
            outAgeMs   = millis() - _routes[i].lastUpdated;
            return true;
        }
        count++;
    }
    return false;
}

bool MeshLayer::routePathFor(char destNode, String& outPath) const {
    for (int i = 0; i < MESH_ROUTE_TABLE_SIZE; i++) {
        if (_routes[i].used && _routes[i].destNode == destNode) {
            outPath = "";
            for (uint8_t k = 0; k < _routes[i].pathLen; k++) {
                outPath += _routes[i].path[k];
            }
            return true;
        }
    }
    return false;
}

// ─── Internal helpers ──────────────────────────────────────────────────────

bool MeshLayer::_wasRecentlySeen(uint8_t msgType, char srcNode, uint8_t msgId) {
    for (int i = 0; i < MESH_SEEN_CACHE_SIZE; i++) {
        if (_seenCache[i].used && _seenCache[i].msgType == msgType &&
            _seenCache[i].srcNode == srcNode && _seenCache[i].msgId == msgId) {
            return true;
        }
    }
    return false;
}

// Writes into the ring-buffer dedup cache, evicting the oldest entry.
void MeshLayer::_markSeen(uint8_t msgType, char srcNode, uint8_t msgId) {
    _seenCache[_seenCacheIndex].used    = true;
    _seenCache[_seenCacheIndex].msgType = msgType;
    _seenCache[_seenCacheIndex].srcNode = srcNode;
    _seenCache[_seenCacheIndex].msgId   = msgId;
    _seenCacheIndex = (_seenCacheIndex + 1) % MESH_SEEN_CACHE_SIZE;
}

// Updates or inserts nodeId's directory entry, firing the callback on a real change.
void MeshLayer::_updateDirectory(char nodeId, const String& nickname) {
    if (nickname.length() == 0) return;

    for (int i = 0; i < MESH_DIRECTORY_SIZE; i++) {
        if (_directory[i].used && _directory[i].nodeId == nodeId) {
            bool changed = !nickname.equals(_directory[i].nickname);
            strncpy(_directory[i].nickname, nickname.c_str(), sizeof(_directory[i].nickname) - 1);
            _directory[i].nickname[sizeof(_directory[i].nickname) - 1] = '\0';
            _directory[i].lastSeen = millis();
            if (changed && _directoryCallback) _directoryCallback(nodeId, nickname, false);
            return;
        }
    }

    // New node — find a free slot (or overwrite the oldest entry if full).
    int slot = -1;
    for (int i = 0; i < MESH_DIRECTORY_SIZE; i++) {
        if (!_directory[i].used) { slot = i; break; }
    }
    if (slot == -1) {
        unsigned long oldest = millis();
        for (int i = 0; i < MESH_DIRECTORY_SIZE; i++) {
            if (_directory[i].lastSeen <= oldest) {
                oldest = _directory[i].lastSeen;
                slot = i;
            }
        }
    }

    _directory[slot].used     = true;
    _directory[slot].nodeId    = nodeId;
    strncpy(_directory[slot].nickname, nickname.c_str(), sizeof(_directory[slot].nickname) - 1);
    _directory[slot].nickname[sizeof(_directory[slot].nickname) - 1] = '\0';
    _directory[slot].lastSeen = millis();

    Serial.printf("[Mesh] Directory + node %c -> %s\n", nodeId, nickname.c_str());
    if (_directoryCallback) _directoryCallback(nodeId, nickname, false);
}

// Evicts directory entries that missed their heartbeat deadline, purging their routes too.
void MeshLayer::_expireDirectory() {
    unsigned long now = millis();
    for (int i = 0; i < MESH_DIRECTORY_SIZE; i++) {
        if (_directory[i].used && _directory[i].nodeId != _nodeId &&
            now - _directory[i].lastSeen > MESH_NODE_TIMEOUT_MS) {
            char   deadNode = _directory[i].nodeId;
            String lastNickname = _directory[i].nickname; // captured before used=false, for the callback's benefit
            Serial.printf("[Mesh] Directory - node %c -> %s (heartbeat timeout)\n",
                          deadNode, _directory[i].nickname);
            _directory[i].used = false;
            if (_directoryCallback) _directoryCallback(deadNode, lastNickname, true);
            _purgeRoutesForNode(deadNode);
        }
    }
}

// Drops any route to/through deadNode (checked against the full path, not just nextHop) and re-advertises.
void MeshLayer::_purgeRoutesForNode(char deadNode) {
    bool removedAny = false;
    for (int i = 0; i < MESH_ROUTE_TABLE_SIZE; i++) {
        if (!_routes[i].used) continue;

        bool inPath = false;
        for (uint8_t k = 0; k < _routes[i].pathLen; k++) {
            if (_routes[i].path[k] == deadNode) { inPath = true; break; }
        }

        if (_routes[i].destNode == deadNode || _routes[i].nextHop == deadNode || inPath) {
            char destNode = _routes[i].destNode;
            Serial.printf("[Mesh] Route - dest %c (via dead node %c)\n", destNode, deadNode);
            _routes[i].used = false;
            if (_routeCallback) _routeCallback(destNode, 0, 0, true);
            removedAny = true;
        }
    }
    if (removedAny) _broadcastRouteAdvertisement();
}

// Commits/stages a candidate route to destNode via viaNextHop, gated by hysteresis + link quality; see mesh.h.
void MeshLayer::_learnRoute(char destNode, char viaNextHop, uint8_t cost,
                            const char* path, uint8_t pathLen) {
    if (destNode == _nodeId || destNode == NODE_BROADCAST) return;

    char autoPath[1];
    if (viaNextHop == destNode && path == nullptr) {
        autoPath[0] = destNode;
        path        = autoPath;
        pathLen     = 1;
    }
    if (pathLen > MESH_DEFAULT_TTL) pathLen = MESH_DEFAULT_TTL; // defensive clamp

    unsigned long now = millis();

    for (int i = 0; i < MESH_ROUTE_TABLE_SIZE; i++) {
        if (_routes[i].used && _routes[i].destNode == destNode) {
            bool sameNextHop = _routes[i].nextHop == viaNextHop;
            bool stale       = (now - _routes[i].lastUpdated) > MESH_ROUTE_MAX_AGE_MS;

            if (sameNextHop) {
                bool valueChanged = _routes[i].cost != cost;
                _routes[i].cost          = cost;
                _routes[i].lastUpdated   = now;
                _routes[i].candidateSeen = 0; // active path just reconfirmed itself; drop any pending candidate
                if (path && pathLen > 0) {
                    memcpy(_routes[i].path, path, pathLen);
                    _routes[i].pathLen = pathLen;
                }
                if (valueChanged && _routeCallback) _routeCallback(destNode, viaNextHop, cost, false);
                return;
            }

            if (stale) {
                _routes[i].nextHop       = viaNextHop;
                _routes[i].cost          = cost;
                _routes[i].lastUpdated   = now;
                _routes[i].candidateSeen = 0;
                _routes[i].pathLen       = 0;
                if (path && pathLen > 0) {
                    memcpy(_routes[i].path, path, pathLen);
                    _routes[i].pathLen = pathLen;
                }
                if (_routeCallback) _routeCallback(destNode, viaNextHop, cost, false);
                return;
            }

            if (cost < _routes[i].cost) {
                float candidateSnr;
                bool  qualityKnown = _neighborQuality(viaNextHop, candidateSnr);
                bool  qualityOk    = !qualityKnown || candidateSnr >= MESH_ROUTE_MIN_PROMOTE_SNR;
                if (!qualityOk) {
                    return; // marginal link — don't even stage it as a candidate
                }

                if (_routes[i].candidateNextHop == viaNextHop) {
                    _routes[i].candidateSeen++;
                } else {
                    _routes[i].candidateNextHop = viaNextHop;
                    _routes[i].candidateCost    = cost;
                    _routes[i].candidateSeen    = 1;
                }

                if (_routes[i].candidateSeen >= MESH_ROUTE_CONFIRM_COUNT) {
                    _routes[i].nextHop       = _routes[i].candidateNextHop;
                    _routes[i].cost          = _routes[i].candidateCost;
                    _routes[i].lastUpdated   = now;
                    _routes[i].candidateSeen = 0;
                    _routes[i].pathLen       = 0;
                    if (path && pathLen > 0) {
                        memcpy(_routes[i].path, path, pathLen);
                        _routes[i].pathLen = pathLen;
                    }
                    Serial.printf("[Mesh] Route ~ dest %c switching to %c (cost %u, confirmed)\n",
                                  destNode, _routes[i].nextHop, _routes[i].cost);
                    if (_routeCallback) _routeCallback(destNode, _routes[i].nextHop, _routes[i].cost, false);
                }
            }
            // Otherwise (cost >= existing, different next hop, not stale): ignore — the active route is at least as good.
            return;
        }
    }

    // New destination — find a free slot, or evict the oldest entry.
    int slot = -1;
    for (int i = 0; i < MESH_ROUTE_TABLE_SIZE; i++) {
        if (!_routes[i].used) { slot = i; break; }
    }
    if (slot == -1) {
        unsigned long oldest = now;
        for (int i = 0; i < MESH_ROUTE_TABLE_SIZE; i++) {
            if (_routes[i].lastUpdated <= oldest) {
                oldest = _routes[i].lastUpdated;
                slot = i;
            }
        }
    }

    _routes[slot].used             = true;
    _routes[slot].destNode         = destNode;
    _routes[slot].nextHop          = viaNextHop;
    _routes[slot].cost             = cost;
    _routes[slot].lastUpdated      = now;
    _routes[slot].candidateNextHop = 0;
    _routes[slot].candidateCost    = 0;
    _routes[slot].candidateSeen    = 0;
    _routes[slot].linkQuality      = 0;
    _routes[slot].hasQuality       = false;
    _routes[slot].pathLen          = 0;
    if (path && pathLen > 0) {
        memcpy(_routes[slot].path, path, pathLen);
        _routes[slot].pathLen = pathLen;
    }

    Serial.printf("[Mesh] Route + dest %c via %c (cost %u)\n", destNode, viaNextHop, cost);
    if (_routeCallback) _routeCallback(destNode, viaNextHop, cost, false);
}

// Blends snr into neighborId's rolling link-quality EWMA (its direct-neighbor route entry).
void MeshLayer::_updateNeighborQuality(char neighborId, float snr) {
    for (int i = 0; i < MESH_ROUTE_TABLE_SIZE; i++) {
        if (_routes[i].used && _routes[i].destNode == neighborId && _routes[i].nextHop == neighborId) {
            if (_routes[i].hasQuality) {
                _routes[i].linkQuality = MESH_LINK_QUALITY_EWMA_ALPHA * snr +
                                         (1.0f - MESH_LINK_QUALITY_EWMA_ALPHA) * _routes[i].linkQuality;
            } else {
                _routes[i].linkQuality = snr;
                _routes[i].hasQuality  = true;
            }
            return;
        }
    }
}

bool MeshLayer::_neighborQuality(char neighborId, float& outSnr) const {
    for (int i = 0; i < MESH_ROUTE_TABLE_SIZE; i++) {
        if (_routes[i].used && _routes[i].destNode == neighborId && _routes[i].nextHop == neighborId &&
            _routes[i].hasQuality) {
            outSnr = _routes[i].linkQuality;
            return true;
        }
    }
    return false;
}

bool MeshLayer::_lookupRoute(char destNode, char& outNextHop, uint8_t* outCost) const {
    for (int i = 0; i < MESH_ROUTE_TABLE_SIZE; i++) {
        if (_routes[i].used && _routes[i].destNode == destNode) {
            outNextHop = _routes[i].nextHop;
            if (outCost) *outCost = _routes[i].cost;
            return true;
        }
    }
    return false;
}

// Drops routes not refreshed within MESH_ROUTE_MAX_AGE_MS and re-advertises if anything was removed.
void MeshLayer::_expireRoutes() {
    unsigned long now = millis();
    bool removedAny = false;
    for (int i = 0; i < MESH_ROUTE_TABLE_SIZE; i++) {
        if (_routes[i].used && now - _routes[i].lastUpdated > MESH_ROUTE_MAX_AGE_MS) {
            Serial.printf("[Mesh] Route - dest %c (stale)\n", _routes[i].destNode);
            char destNode = _routes[i].destNode;
            _routes[i].used = false;
            if (_routeCallback) _routeCallback(destNode, 0, 0, true);
            removedAny = true;
        }
    }
    if (removedAny) _broadcastRouteAdvertisement();
}

// Records an in-flight send awaiting an ACK, evicting the oldest pending entry if the table is full.
void MeshLayer::_addPendingAck(char destNode, uint8_t msgId, const String& message, unsigned long retryIntervalMs) {
    int slot = -1;
    for (int i = 0; i < MESH_PENDING_ACK_SIZE; i++) {
        if (!_pendingAcks[i].used) { slot = i; break; }
    }
    if (slot == -1) {
        unsigned long oldest = millis();
        for (int i = 0; i < MESH_PENDING_ACK_SIZE; i++) {
            if (_pendingAcks[i].sentAt <= oldest) {
                oldest = _pendingAcks[i].sentAt;
                slot = i;
            }
        }
    }

    unsigned long now = millis();
    _pendingAcks[slot].used            = true;
    _pendingAcks[slot].destNode        = destNode;
    _pendingAcks[slot].msgId           = msgId;
    _pendingAcks[slot].sentAt          = now;
    _pendingAcks[slot].lastSentAt      = now;
    _pendingAcks[slot].retryIntervalMs = retryIntervalMs;
    _pendingAcks[slot].retryCount      = 0;
    _pendingAcks[slot].message         = message;
}

// Clears the matching pending entry and fires the delivery callback with the round-trip time.
void MeshLayer::_resolvePendingAck(char destNode, uint8_t msgId) {
    for (int i = 0; i < MESH_PENDING_ACK_SIZE; i++) {
        if (_pendingAcks[i].used && _pendingAcks[i].destNode == destNode && _pendingAcks[i].msgId == msgId) {
            unsigned long elapsedMs = (unsigned long)(millis() - _pendingAcks[i].sentAt);
            _pendingAcks[i].used = false;
            if (_deliveryCallback) _deliveryCallback(destNode, msgId, MESH_DELIVERY_ACKED, elapsedMs);
            return;
        }
    }
}

// Retries or fails each in-flight send whose interval/timeout has elapsed.
void MeshLayer::_servicePendingAcks() {
    unsigned long now = millis();
    for (int i = 0; i < MESH_PENDING_ACK_SIZE; i++) {
        if (!_pendingAcks[i].used) continue;

        char    destNode = _pendingAcks[i].destNode;
        uint8_t msgId    = _pendingAcks[i].msgId;

        if (now - _pendingAcks[i].sentAt > MESH_ACK_TIMEOUT_MS) {
            _pendingAcks[i].used = false;
            _penalizeRouteFailure(destNode);
            if (_deliveryCallback) _deliveryCallback(destNode, msgId, MESH_DELIVERY_FAILED, 0);
            continue;
        }

        if (now - _pendingAcks[i].lastSentAt < _pendingAcks[i].retryIntervalMs) {
            continue; // still within this attempt's window
        }

        if (_pendingAcks[i].retryCount >= MESH_MAX_RETRIES) {
            Serial.printf("[Mesh] Msg %u to %c: giving up after %u retries\n", msgId, destNode, (unsigned)_pendingAcks[i].retryCount);
            _pendingAcks[i].used = false;
            _penalizeRouteFailure(destNode);
            if (_deliveryCallback) _deliveryCallback(destNode, msgId, MESH_DELIVERY_FAILED, 0);
            continue;
        }

        _pendingAcks[i].retryCount++;
        _pendingAcks[i].lastSentAt = now;
        Serial.printf("[Mesh] Msg %u to %c: retry %u/%u\n", msgId, destNode, (unsigned)_pendingAcks[i].retryCount, (unsigned)MESH_MAX_RETRIES);
        _retransmitChat(_pendingAcks[i]);
    }
}

// Invalidates destNode's route on a real delivery failure, dings the next hop's link quality, and re-advertises.
void MeshLayer::_penalizeRouteFailure(char destNode) {
    for (int i = 0; i < MESH_ROUTE_TABLE_SIZE; i++) {
        if (_routes[i].used && _routes[i].destNode == destNode) {
            char usedNextHop = _routes[i].nextHop;

            if (usedNextHop != destNode) {
                for (int j = 0; j < MESH_ROUTE_TABLE_SIZE; j++) {
                    if (_routes[j].used && _routes[j].destNode == usedNextHop &&
                        _routes[j].nextHop == usedNextHop && _routes[j].hasQuality) {
                        _routes[j].linkQuality -= MESH_LINK_QUALITY_FAIL_PENALTY_DB;
                        break;
                    }
                }
            }

            Serial.printf("[Mesh] Route - dest %c (delivery failed via %c)\n", destNode, usedNextHop);
            _routes[i].used = false;
            if (_routeCallback) _routeCallback(destNode, 0, 0, true);
            _broadcastRouteAdvertisement();
            return;
        }
    }
}

// Rebuilds and resends a pending chat with the same msgId and a freshly-looked-up next hop.
void MeshLayer::_retransmitChat(const PendingAck& pending) {
    char chosenNextHop = NODE_BROADCAST;
    _lookupRoute(pending.destNode, chosenNextHop);

    uint8_t packet[MESH_HEADER_LEN + MESH_MAX_PAYLOAD];
    size_t  packetLen = 0;
    _buildPacket(packet, packetLen, MSG_CHAT, _nodeId, pending.destNode, chosenNextHop,
                 pending.msgId, MESH_DEFAULT_TTL, pending.message);

    loraSendRaw(packet, packetLen);
}

// Builds and transmits a MSG_ACK packet, unicast via a known route or flooded.
void MeshLayer::_sendAck(char toNode, uint8_t ackedMsgId) {
    char chosenNextHop = NODE_BROADCAST;
    _lookupRoute(toNode, chosenNextHop);

    uint8_t packet[MESH_HEADER_LEN + 1];
    size_t  packetLen = 0;
    uint8_t msgId = _nextMsgId++;
    uint8_t payload[1] = { ackedMsgId };

    _buildPacket(packet, packetLen, MSG_ACK, _nodeId, toNode, chosenNextHop, msgId, MESH_DEFAULT_TTL,
                 payload, 1);

    _markSeen(MSG_ACK, _nodeId, msgId);
    loraSendRaw(packet, packetLen);
}

// Builds and floods a MSG_PRESENCE packet; no-op if no nickname has been set yet.
void MeshLayer::_broadcastPresence() {
    if (_localNickname.length() == 0) return; // nothing to announce yet

    uint8_t packet[MESH_HEADER_LEN + MESH_MAX_PAYLOAD];
    size_t  packetLen = 0;
    uint8_t msgId = _nextMsgId++;

    _buildPacket(packet, packetLen, MSG_PRESENCE, _nodeId, NODE_BROADCAST, NODE_BROADCAST, msgId, MESH_DEFAULT_TTL, _localNickname);

    _markSeen(MSG_PRESENCE, _nodeId, msgId);
    loraSendRaw(packet, packetLen);
}

// Broadcasts a MSG_ROUTE_ADV listing every route this node has a full path for; never relayed by listeners.
void MeshLayer::_broadcastRouteAdvertisement() {
    uint8_t payload[1 + MESH_ROUTE_TABLE_SIZE * (2 + MESH_DEFAULT_TTL)];
    size_t  payloadLen = 1; // byte 0 reserved for entryCount, filled in below
    uint8_t entryCount = 0;

    for (int i = 0; i < MESH_ROUTE_TABLE_SIZE; i++) {
        if (!_routes[i].used || _routes[i].pathLen == 0) continue;

        payload[payloadLen++] = (uint8_t)_routes[i].destNode;
        payload[payloadLen++] = _routes[i].pathLen;
        memcpy(payload + payloadLen, _routes[i].path, _routes[i].pathLen);
        payloadLen += _routes[i].pathLen;
        entryCount++;
    }
    payload[0] = entryCount;

    uint8_t packet[MESH_HEADER_LEN + sizeof(payload)];
    size_t  packetLen = 0;
    uint8_t msgId = _nextMsgId++;

    // ttl=1 documents "never relayed" — handleIncomingPacket() also hard-codes this regardless of ttl/nextHop.
    _buildPacket(packet, packetLen, MSG_ROUTE_ADV, _nodeId, NODE_BROADCAST, NODE_BROADCAST, msgId, 1,
                 payload, payloadLen);

    _markSeen(MSG_ROUTE_ADV, _nodeId, msgId);
    loraSendRaw(packet, packetLen);
}

// Applies the path-vector loop check to each advertised entry and feeds accepted routes to _learnRoute().
void MeshLayer::_handleRouteAdvertisement(char advertiser, const uint8_t* payload, size_t payloadLen) {
    if (payloadLen < 1) return;

    uint8_t entryCount = payload[0];
    size_t  offset     = 1;

    for (uint8_t e = 0; e < entryCount; e++) {
        if (offset + 2 > payloadLen) break; // malformed/truncated, stop parsing
        char    destNode = (char)payload[offset++];
        uint8_t pathLen  = payload[offset++];
        if (offset + pathLen > payloadLen) break;

        const uint8_t* advertisedPath = payload + offset;
        offset += pathLen;

        if (destNode == _nodeId) continue; // no point routing to ourselves

        bool loop = (advertiser == _nodeId);
        for (uint8_t k = 0; k < pathLen && !loop; k++) {
            if ((char)advertisedPath[k] == _nodeId) loop = true;
        }
        if (loop) continue;

        uint8_t myPathLen = pathLen + 1;
        if (myPathLen > MESH_DEFAULT_TTL) continue; // longer than our own hop budget could ever use

        char myPath[MESH_DEFAULT_TTL];
        myPath[0] = advertiser;
        memcpy(myPath + 1, advertisedPath, pathLen);

        _learnRoute(destNode, advertiser, myPathLen, myPath, myPathLen);
    }
}

// Relays a packet: rewrites prevHop to us, elects our own next hop, decrements ttl, and jitters the resend.
void MeshLayer::_relayPacket(const uint8_t* data, size_t len, char dstNode) {
    uint8_t relay[MESH_HEADER_LEN + MESH_MAX_PAYLOAD];
    memcpy(relay, data, len);

    char chosenNextHop = NODE_BROADCAST;
    _lookupRoute(dstNode, chosenNextHop);

    relay[3] = (uint8_t)_nodeId;       // prevHop = us, we're the one transmitting now
    relay[4] = (uint8_t)chosenNextHop;
    relay[6] = relay[6] - 1;           // ttl decrement

    delay(random(20, 150)); // spread relays out in time to reduce collisions
    loraSendRaw(relay, len);
}

// Encodes header + payload into out/outLen; prevHop is set to srcNode (an original transmission, not a relay).
void MeshLayer::_buildPacket(uint8_t* out, size_t& outLen, MeshMsgType type,
                              char srcNode, char dstNode, char nextHop, uint8_t msgId, uint8_t ttl,
                              const uint8_t* payload, size_t payloadLen) {
    if (payloadLen > MESH_MAX_PAYLOAD) payloadLen = MESH_MAX_PAYLOAD;

    out[0] = (uint8_t)type;
    out[1] = (uint8_t)srcNode;
    out[2] = (uint8_t)dstNode;
    out[3] = (uint8_t)srcNode; // prevHop = originator on first transmission
    out[4] = (uint8_t)nextHop;
    out[5] = msgId;
    out[6] = ttl;
    if (payloadLen > 0) {
        memcpy(out + MESH_HEADER_LEN, payload, payloadLen);
    }

    outLen = MESH_HEADER_LEN + payloadLen;
}

// Convenience wrapper for text payloads (chat messages, presence nicknames).
void MeshLayer::_buildPacket(uint8_t* out, size_t& outLen, MeshMsgType type,
                              char srcNode, char dstNode, char nextHop, uint8_t msgId, uint8_t ttl,
                              const String& payload) {
    _buildPacket(out, outLen, type, srcNode, dstNode, nextHop, msgId, ttl,
                 (const uint8_t*)payload.c_str(), payload.length());
}
