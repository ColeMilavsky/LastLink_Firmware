#pragma once

// ─── Mesh Layer (Future) ──────────────────────────────────────────────────────
//
// This module will handle multi-hop mesh routing between nodes.
// For now it is a stub — messages are broadcast point-to-point.
//
// Planned features:
//   - Node ID assignment (based on MAC or manual config)
//   - Packet header: [src_id | dst_id | hop_count | payload]
//   - Flood routing / simple source routing
//   - Duplicate packet filtering (seen-packet cache)
//
// To add mesh support later:
//   1. Define a MeshPacket struct with header fields
//   2. Wrap LoRaRadio::send() to prepend the header
//   3. In the RX callback, inspect dst_id — forward if not ours, deliver if ours

#include <Arduino.h>

// Placeholder — currently a pass-through
class MeshLayer {
public:
    void begin() {}       // No-op for now
    void update() {}      // No-op for now
};

extern MeshLayer Mesh;
