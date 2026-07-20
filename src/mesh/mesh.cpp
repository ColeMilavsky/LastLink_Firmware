#include "mesh.h"
#include "../lora/lora.h"

MeshLayer Mesh;

MeshLayer::MeshLayer()
    : _nodeId(0),
      _nextMsgId(0),
      _chatCallback(nullptr),
      _directoryCallback(nullptr),
      _seenCacheIndex(0),
      _lastPresenceBroadcast(0) {}

void MeshLayer::begin(char nodeId) {
    _nodeId = nodeId;
    _lastPresenceBroadcast = 0; // force an immediate beacon on first update()
    Serial.printf("[Mesh] Node ID: %c\n", _nodeId);
}

void MeshLayer::update() {
    unsigned long now = millis();

    if (now - _lastPresenceBroadcast >= MESH_PRESENCE_INTERVAL_MS || _lastPresenceBroadcast == 0) {
        _broadcastPresence();
        _lastPresenceBroadcast = now;
    }

    _expireDirectory();
}

void MeshLayer::setLocalNickname(const String& nickname) {
    _localNickname = nickname;
    Serial.println("[Mesh] Local nickname set to: " + nickname);

    // Make sure we're in our own directory immediately, and announce right away
    // rather than waiting for the next periodic beacon.
    _updateDirectory(_nodeId, _localNickname);
    _broadcastPresence();
    _lastPresenceBroadcast = millis();
}

bool MeshLayer::sendChat(const String& destNickname, const String& message) {
    char destNode = NODE_BROADCAST;
    bool resolved = resolveNickname(destNickname, destNode);

    if (!resolved) {
        Serial.println("[Mesh] Nickname \"" + destNickname + "\" not in directory yet — broadcasting");
        destNode = NODE_BROADCAST;
    }

    uint8_t packet[MESH_HEADER_LEN + MESH_MAX_PAYLOAD];
    size_t  packetLen = 0;
    uint8_t msgId = _nextMsgId++;

    _buildPacket(packet, packetLen, MSG_CHAT, _nodeId, destNode, msgId, MESH_DEFAULT_TTL, message);

    // Remember our own (srcNode,msgId) so if this packet echoes back to us
    // via a relay loop we drop it instead of re-processing it.
    _markSeen(_nodeId, msgId);

    int state = loraSendRaw(packet, packetLen);
    return state == 0; // RADIOLIB_ERR_NONE
}

MeshRxAction MeshLayer::handleIncomingPacket(const uint8_t* data, size_t len, int rssi, float snr) {
    if (len < MESH_HEADER_LEN) {
        return MESH_RX_DUPLICATE; // malformed, treat as noise
    }

    uint8_t msgType = data[0];
    char    srcNode  = (char)data[1];
    char    dstNode  = (char)data[2];
    uint8_t msgId    = data[3];
    uint8_t ttl      = data[4];

    String payload;
    for (size_t i = MESH_HEADER_LEN; i < len; i++) {
        payload += (char)data[i];
    }

    // Ignore our own transmissions echoing back.
    if (srcNode == _nodeId) {
        return MESH_RX_DUPLICATE;
    }

    if (_wasRecentlySeen(srcNode, msgId)) {
        return MESH_RX_DUPLICATE;
    }
    _markSeen(srcNode, msgId);

    if (msgType == MSG_PRESENCE) {
        _updateDirectory(srcNode, payload);

        // Relay presence beacons too, so the directory propagates beyond
        // direct radio range — but only if there's hop budget left.
        if (ttl > 1) {
            uint8_t relay[MESH_HEADER_LEN + MESH_MAX_PAYLOAD];
            memcpy(relay, data, len);
            relay[4] = ttl - 1;
            _relayPacket(relay, len);
        }
        return MESH_RX_PRESENCE;
    }

    if (msgType == MSG_CHAT) {
        bool forUs = (dstNode == _nodeId) || (dstNode == NODE_BROADCAST);

        if (dstNode == _nodeId || dstNode == NODE_BROADCAST) {
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
        }

        // If addressed specifically to us (not broadcast), we're done — don't relay.
        if (dstNode == _nodeId) {
            return MESH_RX_FOR_ME;
        }

        // Broadcast chat or addressed to someone else: relay onward if TTL allows.
        if (ttl > 1) {
            uint8_t relay[MESH_HEADER_LEN + MESH_MAX_PAYLOAD];
            memcpy(relay, data, len);
            relay[4] = ttl - 1;
            _relayPacket(relay, len);
            return forUs ? MESH_RX_FOR_ME : MESH_RX_RELAYED;
        }

        return forUs ? MESH_RX_FOR_ME : MESH_RX_EXPIRED;
    }

    return MESH_RX_DUPLICATE; // unknown type, drop
}

void MeshLayer::onChatReceived(MeshChatCallback callback) {
    _chatCallback = callback;
}

void MeshLayer::onDirectoryChanged(MeshDirectoryCallback callback) {
    _directoryCallback = callback;
}

int MeshLayer::directoryCount() const {
    int count = 0;
    for (int i = 0; i < MESH_DIRECTORY_SIZE; i++) {
        if (_directory[i].used) count++;
    }
    return count;
}

bool MeshLayer::directoryEntryAt(int index, char& outNodeId, String& outNickname) const {
    int count = 0;
    for (int i = 0; i < MESH_DIRECTORY_SIZE; i++) {
        if (!_directory[i].used) continue;
        if (count == index) {
            outNodeId   = _directory[i].nodeId;
            outNickname = _directory[i].nickname;
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

// ─── Internal helpers ──────────────────────────────────────────────────────

bool MeshLayer::_wasRecentlySeen(char srcNode, uint8_t msgId) {
    for (int i = 0; i < MESH_SEEN_CACHE_SIZE; i++) {
        if (_seenCache[i].used && _seenCache[i].srcNode == srcNode && _seenCache[i].msgId == msgId) {
            return true;
        }
    }
    return false;
}

void MeshLayer::_markSeen(char srcNode, uint8_t msgId) {
    _seenCache[_seenCacheIndex].used    = true;
    _seenCache[_seenCacheIndex].srcNode = srcNode;
    _seenCache[_seenCacheIndex].msgId   = msgId;
    _seenCacheIndex = (_seenCacheIndex + 1) % MESH_SEEN_CACHE_SIZE;
}

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

void MeshLayer::_expireDirectory() {
    unsigned long now = millis();
    for (int i = 0; i < MESH_DIRECTORY_SIZE; i++) {
        if (_directory[i].used && _directory[i].nodeId != _nodeId &&
            now - _directory[i].lastSeen > MESH_DIRECTORY_MAX_AGE_MS) {
            Serial.printf("[Mesh] Directory - node %c -> %s (stale)\n",
                          _directory[i].nodeId, _directory[i].nickname);
            _directory[i].used = false;
            if (_directoryCallback) _directoryCallback();
        }
    }
}

void MeshLayer::_broadcastPresence() {
    if (_localNickname.length() == 0) return; // nothing to announce yet

    uint8_t packet[MESH_HEADER_LEN + MESH_MAX_PAYLOAD];
    size_t  packetLen = 0;
    uint8_t msgId = _nextMsgId++;

    _buildPacket(packet, packetLen, MSG_PRESENCE, _nodeId, NODE_BROADCAST, msgId,
                 MESH_DEFAULT_TTL, _localNickname);

    _markSeen(_nodeId, msgId);
    loraSendRaw(packet, packetLen);
}

void MeshLayer::_relayPacket(uint8_t* data, size_t len) {
    // Naive flood routing means every node in range rebroadcasts at once,
    // which collides on a shared channel. A small random delay spreads
    // relays out in time so they don't all key up simultaneously. This is
    // a cheap mitigation, not a real MAC layer — fine for small meshes.
    delay(random(20, 150));
    loraSendRaw(data, len);
}

void MeshLayer::_buildPacket(uint8_t* out, size_t& outLen, MeshMsgType type,
                              char srcNode, char dstNode, uint8_t msgId, uint8_t ttl,
                              const String& payload) {
    size_t payloadLen = payload.length();
    if (payloadLen > MESH_MAX_PAYLOAD) payloadLen = MESH_MAX_PAYLOAD;

    out[0] = (uint8_t)type;
    out[1] = (uint8_t)srcNode;
    out[2] = (uint8_t)dstNode;
    out[3] = msgId;
    out[4] = ttl;
    memcpy(out + MESH_HEADER_LEN, payload.c_str(), payloadLen);

    outLen = MESH_HEADER_LEN + payloadLen;
}