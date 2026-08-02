#include "mesh.h"
#include "../lora/lora.h"

MeshLayer Mesh;

// In: none. Out: none.
// Constructs the mesh layer with an unset node id and empty callbacks/caches.
MeshLayer::MeshLayer()
    : _nodeId(0),
      _nextMsgId(0),
      _chatCallback(nullptr),
      _directoryCallback(nullptr),
      _deliveryCallback(nullptr),
      _routeCallback(nullptr),
      _seenCacheIndex(0),
      _lastPresenceBroadcast(0) {}

// In: nodeId - this node's single-character identity. Out: none.
// Stores the node id and arms an immediate presence beacon for the first update() call.
void MeshLayer::begin(char nodeId) {
    _nodeId = nodeId;
    _lastPresenceBroadcast = 0; // force an immediate beacon on first update()
    Serial.printf("[Mesh] Node ID: %c\n", _nodeId);
}

// In: none. Out: none.
// Per-loop tick: fires a presence beacon on interval, then sweeps stale
// directory entries, stale routes, and timed-out pending acks.
void MeshLayer::update() {
    unsigned long now = millis();

    if (now - _lastPresenceBroadcast >= MESH_PRESENCE_INTERVAL_MS || _lastPresenceBroadcast == 0) {
        _broadcastPresence();
        _lastPresenceBroadcast = now;
    }

    _expireDirectory();
    _expireRoutes();
    _servicePendingAcks();
}

// In: nickname - the phone-supplied display name for this node. Out: none.
// Records the local nickname, seeds it into our own directory entry, and
// broadcasts a presence beacon immediately instead of waiting for the timer.
void MeshLayer::setLocalNickname(const String& nickname) {
    _localNickname = nickname;
    Serial.println("[Mesh] Local nickname set to: " + nickname);

    // Make sure we're in our own directory immediately, and announce right away
    // rather than waiting for the next periodic beacon.
    _updateDirectory(_nodeId, _localNickname);
    _broadcastPresence();
    _lastPresenceBroadcast = millis();
}

// In: destNickname - target phone's nickname (empty/unknown means broadcast);
//     message - chat text to send; outDestNode/outMsgId - optional (may be
//     null) out-params receiving the resolved destination and assigned msgId.
// Out: true if the radio accepted the transmission, false on a hard radio error.
// Resolves the nickname to a node id and next hop (if a route is known),
// builds and sends a MSG_CHAT packet, and registers a pending ACK for
// unicast sends — including the retry interval (hop count * 500ms, or a
// worst-case 6-hop assumption if no route is known yet) — so it gets
// retransmitted until acked, retried out, or timed out.
bool MeshLayer::sendChat(const String& destNickname, const String& message,
                          char* outDestNode, uint8_t* outMsgId) {
    char destNode = NODE_BROADCAST;
    bool resolved = resolveNickname(destNickname, destNode);

    if (!resolved) {
        Serial.println("[Mesh] Nickname \"" + destNickname + "\" not in directory yet — broadcasting");
        destNode = NODE_BROADCAST;
    }

    // Prefer a known route; falls back to NODE_BROADCAST (flood) if we don't
    // have one yet, or if the destination is a broadcast in the first place.
    // hopCount defaults to the full TTL budget (worst case) when no route is
    // known — a flood delivery can legitimately take longer to complete than
    // a known direct route, and retrying too aggressively before it's had a
    // chance to succeed just wastes airtime.
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

    // Remember our own (srcNode,msgId) so if this packet echoes back to us
    // via a relay loop we drop it instead of re-processing it.
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

// In: data/len - raw packet bytes off the radio; rssi/snr - signal quality
//     of the reception (currently unused here, available for future logging).
// Out: the MeshRxAction describing what was done with the packet.
// Parses the header, drops malformed/self-echo packets; a recognized
// duplicate chat addressed to us gets its ack re-sent (in case the original
// ack was lost, not the chat) before being dropped, since sends now retry.
// Learns routing-table entries from prevHop/srcNode, then dispatches by
// msgType: updates the directory (presence), resolves a pending ack (ack),
// or delivers/acks/relays a chat message — relaying only if this node is
// the elected next hop or no route was known yet (flood fallback).
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
        // A retried copy of a chat we've already delivered usually means our
        // original ack was the thing lost in transit, not the chat — re-send
        // it so the sender's retry can still succeed instead of eventually
        // giving up on a message that actually arrived. Only for chat
        // addressed specifically to us (broadcasts have no single ack to
        // resend); we don't re-deliver to the phone or re-relay, just re-ack.
        if (msgType == MSG_CHAT && dstNode == _nodeId) {
            _sendAck(srcNode, msgId);
        }
        return MESH_RX_DUPLICATE;
    }
    _markSeen(msgType, srcNode, msgId);

    // Learn routes from this packet regardless of type or whether it's for
    // us — prevHop is always a direct neighbor, and srcNode is reachable
    // via prevHop at the hop cost the packet has already traveled.
    if (prevHop != _nodeId) {
        _learnRoute(prevHop, prevHop, 1);
        if (srcNode != prevHop) {
            uint8_t hopsTraveled = MESH_DEFAULT_TTL - ttl;
            _learnRoute(srcNode, prevHop, hopsTraveled + 1);
        }
    }

    bool eligibleRelay = (nextHop == _nodeId) || (nextHop == NODE_BROADCAST);

    if (msgType == MSG_PRESENCE) {
        _updateDirectory(srcNode, payload);

        // Presence always targets everyone (dstNode is NODE_BROADCAST), so
        // relay eligibility here just comes down to hop budget.
        if (eligibleRelay && ttl > 1) {
            _relayPacket(data, len, dstNode);
        }
        return MESH_RX_PRESENCE;
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

            // Addressed specifically to us (not broadcast): ack the sender and
            // stop — no further relay needed.
            if (dstNode == _nodeId) {
                _sendAck(srcNode, msgId);
                return MESH_RX_FOR_ME;
            }
        }

        // Broadcast chat, or addressed to someone else: relay onward only if
        // we're the elected next hop (or nobody's elected yet, i.e. flood).
        if (eligibleRelay) {
            if (ttl > 1) {
                _relayPacket(data, len, dstNode);
                return forUs ? MESH_RX_FOR_ME : MESH_RX_RELAYED;
            }
            return forUs ? MESH_RX_FOR_ME : MESH_RX_EXPIRED;
        }

        // Not our turn to relay — we've already learned from it above.
        return forUs ? MESH_RX_FOR_ME : MESH_RX_RELAYED;
    }

    return MESH_RX_DUPLICATE; // unknown type, drop
}

// In: callback - function to invoke when a chat packet addressed to us arrives.
// Out: none.
void MeshLayer::onChatReceived(MeshChatCallback callback) {
    _chatCallback = callback;
}

// In: callback - function to invoke whenever the nickname directory changes.
// Out: none.
void MeshLayer::onDirectoryChanged(MeshDirectoryCallback callback) {
    _directoryCallback = callback;
}

// In: callback - function to invoke when a sent chat is acked or times out.
// Out: none.
void MeshLayer::onDeliveryStatus(MeshDeliveryCallback callback) {
    _deliveryCallback = callback;
}

// In: callback - function to invoke when the routing table gains, updates,
//     or loses an entry. Out: none.
void MeshLayer::onRouteChanged(MeshRouteCallback callback) {
    _routeCallback = callback;
}

// In: none. Out: number of currently-populated directory entries.
int MeshLayer::directoryCount() const {
    int count = 0;
    for (int i = 0; i < MESH_DIRECTORY_SIZE; i++) {
        if (_directory[i].used) count++;
    }
    return count;
}

// In: index - zero-based position among used entries only.
// Out: outNodeId/outNickname/outAgeMs populated on success (outAgeMs is
//      milliseconds since this node's last heartbeat); returns true if index
//      was valid, false if it's out of range (fewer entries than requested).
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

// In: nickname - name to look up (case-insensitive).
// Out: outNodeId set to the owning node id on success; returns true if found,
//      false if the nickname isn't in the directory.
bool MeshLayer::resolveNickname(const String& nickname, char& outNodeId) const {
    for (int i = 0; i < MESH_DIRECTORY_SIZE; i++) {
        if (_directory[i].used && nickname.equalsIgnoreCase(_directory[i].nickname)) {
            outNodeId = _directory[i].nodeId;
            return true;
        }
    }
    return false;
}

// In: none. Out: number of currently-populated routing table entries.
int MeshLayer::routeCount() const {
    int count = 0;
    for (int i = 0; i < MESH_ROUTE_TABLE_SIZE; i++) {
        if (_routes[i].used) count++;
    }
    return count;
}

// In: index - zero-based position among used entries only.
// Out: outDest/outNextHop/outCost/outAgeMs populated on success (outAgeMs is
//      milliseconds since the entry was last refreshed); returns true if
//      index was valid, false if it's out of range.
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

// ─── Internal helpers ──────────────────────────────────────────────────────

// In: msgType/srcNode/msgId - the (type, source, sequence number) triple
//     identifying a packet.
// Out: true if that triple is in the recent-seen cache (i.e. a duplicate of
//      a packet of the same type from the same source).
bool MeshLayer::_wasRecentlySeen(uint8_t msgType, char srcNode, uint8_t msgId) {
    for (int i = 0; i < MESH_SEEN_CACHE_SIZE; i++) {
        if (_seenCache[i].used && _seenCache[i].msgType == msgType &&
            _seenCache[i].srcNode == srcNode && _seenCache[i].msgId == msgId) {
            return true;
        }
    }
    return false;
}

// In: msgType/srcNode/msgId - the (type, source, sequence number) triple to
//     remember. Out: none. Writes into the ring-buffer dedup cache, evicting
//     the oldest entry.
void MeshLayer::_markSeen(uint8_t msgType, char srcNode, uint8_t msgId) {
    _seenCache[_seenCacheIndex].used    = true;
    _seenCache[_seenCacheIndex].msgType = msgType;
    _seenCache[_seenCacheIndex].srcNode = srcNode;
    _seenCache[_seenCacheIndex].msgId   = msgId;
    _seenCacheIndex = (_seenCacheIndex + 1) % MESH_SEEN_CACHE_SIZE;
}

// In: nodeId - node the nickname belongs to; nickname - its announced name.
// Out: none. Updates the existing entry for nodeId, or inserts a new one
// (evicting the oldest entry if the table is full); fires the directory
// callback if the nickname changed or a new node was added.
void MeshLayer::_updateDirectory(char nodeId, const String& nickname) {
    if (nickname.length() == 0) return;

    // Update existing entry for this node if present.
    for (int i = 0; i < MESH_DIRECTORY_SIZE; i++) {
        if (_directory[i].used && _directory[i].nodeId == nodeId) {
            bool changed = !nickname.equals(_directory[i].nickname);
            strncpy(_directory[i].nickname, nickname.c_str(), sizeof(_directory[i].nickname) - 1);
            _directory[i].nickname[sizeof(_directory[i].nickname) - 1] = '\0';
            _directory[i].lastSeen = millis();
            if (changed && _directoryCallback) _directoryCallback();
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
    if (_directoryCallback) _directoryCallback();
}

// In: none. Out: none.
// Removes directory entries (other than our own) that have missed two
// consecutive heartbeats (MESH_NODE_TIMEOUT_MS), firing the directory
// callback for each removal and purging any routing table entries that led
// to or through the now-dead node.
void MeshLayer::_expireDirectory() {
    unsigned long now = millis();
    for (int i = 0; i < MESH_DIRECTORY_SIZE; i++) {
        if (_directory[i].used && _directory[i].nodeId != _nodeId &&
            now - _directory[i].lastSeen > MESH_NODE_TIMEOUT_MS) {
            char deadNode = _directory[i].nodeId;
            Serial.printf("[Mesh] Directory - node %c -> %s (heartbeat timeout)\n",
                          deadNode, _directory[i].nickname);
            _directory[i].used = false;
            if (_directoryCallback) _directoryCallback();
            _purgeRoutesForNode(deadNode);
        }
    }
}

// In: deadNode - node id that just missed two consecutive heartbeats and was
//     evicted from the directory. Out: none.
// Drops any routing table entry that leads TO deadNode, and any entry that
// goes THROUGH deadNode as its next hop — a route depending on a next hop
// that no longer exists is worse than useless, it's actively misleading.
// Fires the route callback (removed=true) for each entry dropped so UI/BLE
// listeners stay in sync; a fresh route can always be relearned later from
// traffic/beacons if the node comes back.
void MeshLayer::_purgeRoutesForNode(char deadNode) {
    for (int i = 0; i < MESH_ROUTE_TABLE_SIZE; i++) {
        if (_routes[i].used && (_routes[i].destNode == deadNode || _routes[i].nextHop == deadNode)) {
            char destNode = _routes[i].destNode;
            Serial.printf("[Mesh] Route - dest %c (via dead node %c)\n", destNode, deadNode);
            _routes[i].used = false;
            if (_routeCallback) _routeCallback(destNode, 0, 0, true);
        }
    }
}

// In: destNode - node the route leads to; viaNextHop - neighbor to reach it
//     through; cost - hop count for this candidate route.
// Out: none. Ignores routes to ourselves/broadcast. For an existing entry:
// a sighting via the SAME next hop always refreshes it immediately (that's
// just confirming the active path is still alive), and replacing an already
// STALE entry also happens immediately (there's no working path to protect).
// But a cheaper route via a DIFFERENT next hop is staged as a candidate and
// only actually committed once it's been independently observed
// MESH_ROUTE_CONFIRM_COUNT times — a single marginal/one-off reception (e.g.
// an occasional direct link that isn't really reliable) must not be able to
// instantly "shortcut" a route away from an already-working multi-hop path;
// a genuinely reliable alternative will keep showing up and get promoted
// within the next beacon cycle or two, a fluke won't repeat and never will.
// Inserts a new entry outright if destNode isn't known yet (evicting the
// oldest if the table is full) — no corroboration needed when there's no
// existing route to protect. Fires the route callback only when the
// committed next hop or cost actually changes (or the entry is new) — a
// plain timestamp refresh, or a candidate that hasn't been promoted yet,
// does not fire it, so UI/BLE listeners aren't spammed.
void MeshLayer::_learnRoute(char destNode, char viaNextHop, uint8_t cost) {
    if (destNode == _nodeId || destNode == NODE_BROADCAST) return;

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
                if (valueChanged && _routeCallback) _routeCallback(destNode, viaNextHop, cost, false);
                return;
            }

            if (stale) {
                _routes[i].nextHop       = viaNextHop;
                _routes[i].cost          = cost;
                _routes[i].lastUpdated   = now;
                _routes[i].candidateSeen = 0;
                if (_routeCallback) _routeCallback(destNode, viaNextHop, cost, false);
                return;
            }

            if (cost < _routes[i].cost) {
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

    _routes[slot].used            = true;
    _routes[slot].destNode        = destNode;
    _routes[slot].nextHop         = viaNextHop;
    _routes[slot].cost            = cost;
    _routes[slot].lastUpdated     = now;
    _routes[slot].candidateNextHop = 0;
    _routes[slot].candidateCost    = 0;
    _routes[slot].candidateSeen    = 0;

    Serial.printf("[Mesh] Route + dest %c via %c (cost %u)\n", destNode, viaNextHop, cost);
    if (_routeCallback) _routeCallback(destNode, viaNextHop, cost, false);
}

// In: destNode - node to find a route to.
// Out: outNextHop set to the known next hop on success; outCost (if
//      non-null) set to the route's hop count; returns true if a route
//      exists, false if destNode isn't in the table (caller should fall back
//      to NODE_BROADCAST/flood).
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

// In: none. Out: none.
// Drops routing table entries not refreshed within MESH_ROUTE_MAX_AGE_MS,
// firing the route callback (removed=true) for each one dropped.
void MeshLayer::_expireRoutes() {
    unsigned long now = millis();
    for (int i = 0; i < MESH_ROUTE_TABLE_SIZE; i++) {
        if (_routes[i].used && now - _routes[i].lastUpdated > MESH_ROUTE_MAX_AGE_MS) {
            Serial.printf("[Mesh] Route - dest %c (stale)\n", _routes[i].destNode);
            char destNode = _routes[i].destNode;
            _routes[i].used = false;
            if (_routeCallback) _routeCallback(destNode, 0, 0, true);
        }
    }
}

// In: destNode - node the chat was sent to; msgId - its sequence number;
//     message - the chat text (kept so a retry can rebuild an identical
//     packet); retryIntervalMs - resend cadence for this message (hop count
//     scaled). Out: none. Records an in-flight send awaiting an ACK,
//     evicting the oldest pending entry if the table is full.
void MeshLayer::_addPendingAck(char destNode, uint8_t msgId, const String& message, unsigned long retryIntervalMs) {
    int slot = -1;
    for (int i = 0; i < MESH_PENDING_ACK_SIZE; i++) {
        if (!_pendingAcks[i].used) { slot = i; break; }
    }
    if (slot == -1) {
        // Table full — evict the oldest in-flight entry. Best-effort tracking;
        // worst case the phone just never sees a status for that one send.
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

// In: destNode - node that sent the ACK; msgId - the original msgId it acks.
// Out: none. Clears the matching pending entry and fires the delivery
// callback with MESH_DELIVERY_ACKED and the round-trip time (millis() minus
// the recorded send timestamp, via unsigned subtraction so it stays correct
// even if millis() has wrapped around since the message was sent); no-op if
// no matching entry is found.
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

// In: none. Out: none.
// Called every update(). For each in-flight send: if the absolute
// MESH_ACK_TIMEOUT_MS ceiling has elapsed since the original send, fail it
// regardless of retry state (a backstop against a badly-wrong hop-count
// assumption). Otherwise, once its retry interval has elapsed with no ack:
// retransmit (up to MESH_MAX_RETRIES times) and reset the interval, or — if
// retries are already exhausted — fail it. Failing fires the delivery
// callback with MESH_DELIVERY_FAILED and elapsedMs=0 (a timeout/give-up has
// no real round-trip time to report).
void MeshLayer::_servicePendingAcks() {
    unsigned long now = millis();
    for (int i = 0; i < MESH_PENDING_ACK_SIZE; i++) {
        if (!_pendingAcks[i].used) continue;

        char    destNode = _pendingAcks[i].destNode;
        uint8_t msgId    = _pendingAcks[i].msgId;

        if (now - _pendingAcks[i].sentAt > MESH_ACK_TIMEOUT_MS) {
            _pendingAcks[i].used = false;
            if (_deliveryCallback) _deliveryCallback(destNode, msgId, MESH_DELIVERY_FAILED, 0);
            continue;
        }

        if (now - _pendingAcks[i].lastSentAt < _pendingAcks[i].retryIntervalMs) {
            continue; // still within this attempt's window
        }

        if (_pendingAcks[i].retryCount >= MESH_MAX_RETRIES) {
            Serial.printf("[Mesh] Msg %u to %c: giving up after %u retries\n", msgId, destNode, (unsigned)_pendingAcks[i].retryCount);
            _pendingAcks[i].used = false;
            if (_deliveryCallback) _deliveryCallback(destNode, msgId, MESH_DELIVERY_FAILED, 0);
            continue;
        }

        _pendingAcks[i].retryCount++;
        _pendingAcks[i].lastSentAt = now;
        Serial.printf("[Mesh] Msg %u to %c: retry %u/%u\n", msgId, destNode, (unsigned)_pendingAcks[i].retryCount, (unsigned)MESH_MAX_RETRIES);
        _retransmitChat(_pendingAcks[i]);
    }
}

// In: pending - the in-flight send to retransmit (destNode/msgId/message
//     already recorded). Out: none.
// Rebuilds and resends an identical MSG_CHAT packet — same msgId, so the
// destination's dedup cache recognizes and drops the extra copy if the
// original already arrived, rather than re-delivering it — using a
// freshly-looked-up next hop in case routing has improved, or the original
// next hop went stale/was evicted, since this message was first sent.
void MeshLayer::_retransmitChat(const PendingAck& pending) {
    char chosenNextHop = NODE_BROADCAST;
    _lookupRoute(pending.destNode, chosenNextHop);

    uint8_t packet[MESH_HEADER_LEN + MESH_MAX_PAYLOAD];
    size_t  packetLen = 0;
    _buildPacket(packet, packetLen, MSG_CHAT, _nodeId, pending.destNode, chosenNextHop,
                 pending.msgId, MESH_DEFAULT_TTL, pending.message);

    loraSendRaw(packet, packetLen);
}

// In: toNode - node to send the ACK to (the original chat's sender);
//     ackedMsgId - the msgId of the chat packet being acknowledged.
// Out: none. Builds and transmits a MSG_ACK packet, unicast via a known
// route if one exists, otherwise flooded.
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

// In: none. Out: none.
// Builds and floods a MSG_PRESENCE packet announcing this node's id and
// local nickname; no-op if no nickname has been set yet.
void MeshLayer::_broadcastPresence() {
    if (_localNickname.length() == 0) return; // nothing to announce yet

    uint8_t packet[MESH_HEADER_LEN + MESH_MAX_PAYLOAD];
    size_t  packetLen = 0;
    uint8_t msgId = _nextMsgId++;

    _buildPacket(packet, packetLen, MSG_PRESENCE, _nodeId, NODE_BROADCAST, NODE_BROADCAST, msgId, MESH_DEFAULT_TTL, _localNickname);

    _markSeen(MSG_PRESENCE, _nodeId, msgId);
    loraSendRaw(packet, packetLen);
}

// In: data/len - the packet as received (header + payload); dstNode - its
//     final destination, used to pick our own next hop.
// Out: none. Rewrites prevHop to us, sets nextHop from our routing table (or
// NODE_BROADCAST if we don't have one), decrements ttl, and retransmits
// after a small random delay to reduce collisions with other relayers.
void MeshLayer::_relayPacket(const uint8_t* data, size_t len, char dstNode) {
    // Naive flood routing means every node in range rebroadcasts at once,
    // which collides on a shared channel. A small random delay spreads
    // relays out in time so they don't all key up simultaneously. This is
    // a cheap mitigation, not a real MAC layer — fine for small meshes.
    uint8_t relay[MESH_HEADER_LEN + MESH_MAX_PAYLOAD];
    memcpy(relay, data, len);

    // Elect our own next hop toward dstNode if we know one; otherwise keep
    // flooding (NODE_BROADCAST) so the rest of the mesh still gets it.
    char chosenNextHop = NODE_BROADCAST;
    _lookupRoute(dstNode, chosenNextHop);

    relay[3] = (uint8_t)_nodeId;       // prevHop = us, we're the one transmitting now
    relay[4] = (uint8_t)chosenNextHop;
    relay[6] = relay[6] - 1;           // ttl decrement

    delay(random(20, 150));
    loraSendRaw(relay, len);
}

// In: type/srcNode/dstNode/nextHop/msgId/ttl - header fields; payload/payloadLen
//     - raw payload bytes (truncated to MESH_MAX_PAYLOAD if longer).
// Out: out - filled with the encoded packet; outLen - its total length
//      (header + payload). prevHop is set to srcNode (this is always an
//      original transmission, never a relay).
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

// In: same as the raw-bytes overload, but payload is a String.
// Out: same as the raw-bytes overload.
// Convenience wrapper for text payloads (chat messages, presence nicknames).
void MeshLayer::_buildPacket(uint8_t* out, size_t& outLen, MeshMsgType type,
                              char srcNode, char dstNode, char nextHop, uint8_t msgId, uint8_t ttl,
                              const String& payload) {
    _buildPacket(out, outLen, type, srcNode, dstNode, nextHop, msgId, ttl,
                 (const uint8_t*)payload.c_str(), payload.length());
}
