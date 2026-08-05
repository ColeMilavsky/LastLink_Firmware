#ifndef LASTLINK_CONFIG_H
#define LASTLINK_CONFIG_H

#include <Arduino.h>
#include <RadioLib.h>

// ─── LoRa Pin Definitions (Heltec V3) ────────────────────────────────────────
#define LORA_SCK        9
#define LORA_MISO       11
#define LORA_MOSI       10
#define LORA_CS         8
#define LORA_RST        12
#define LORA_DIO0       14    // Not used on SX1262 but kept for reference
#define LORA_DIO1       14
#define LORA_BUSY       13

// ─── LoRa Radio Settings ──────────────────────────────────────────────────────
#define LORA_FREQUENCY      915E6   // 915 MHz for North America (use 868E6 for EU)
#define LORA_BANDWIDTH      125E3   // 125 kHz bandwidth
#define LORA_SPREAD_FACTOR  7       // SF7 = faster, shorter range (SF12 = slowest, max range)
#define LORA_CODING_RATE    5       // 4/5 coding rate
#define LORA_SYNC_WORD      0x34    // Custom network sync word (0x34 = private network)
#define LORA_TX_POWER       20      // dBm (max 20 for SX1276)
#define LORA_PREAMBLE       8       // Preamble length in symbols

// ─── Serial Settings ──────────────────────────────────────────────────────────
#define SERIAL_BAUD         115200
#define SERIAL_TIMEOUT_MS   10      // How long to collect serial chars before sending

// ─── Message Settings ─────────────────────────────────────────────────────────
#define MSG_MAX_LEN         250     // Max message length in bytes
#define MSG_DELIMITER       '\n'    // Character that triggers a send

// Decode RadioLib status codes to readable strings
inline String stateDecode(const int16_t result) {
  switch (result) {
    case RADIOLIB_ERR_NONE: return "ERR_NONE";
    case RADIOLIB_ERR_CHIP_NOT_FOUND: return "ERR_CHIP_NOT_FOUND";
    case RADIOLIB_ERR_PACKET_TOO_LONG: return "ERR_PACKET_TOO_LONG";
    case RADIOLIB_ERR_RX_TIMEOUT: return "ERR_RX_TIMEOUT";
    case RADIOLIB_ERR_MIC_MISMATCH: return "ERR_MIC_MISMATCH";
    case RADIOLIB_ERR_INVALID_BANDWIDTH: return "ERR_INVALID_BANDWIDTH";
    case RADIOLIB_ERR_INVALID_SPREADING_FACTOR: return "ERR_INVALID_SPREADING_FACTOR";
    case RADIOLIB_ERR_INVALID_CODING_RATE: return "ERR_INVALID_CODING_RATE";
    case RADIOLIB_ERR_INVALID_FREQUENCY: return "ERR_INVALID_FREQUENCY";
    case RADIOLIB_ERR_INVALID_OUTPUT_POWER: return "ERR_INVALID_OUTPUT_POWER";
    case RADIOLIB_ERR_NETWORK_NOT_JOINED: return "RADIOLIB_ERR_NETWORK_NOT_JOINED";
    case RADIOLIB_ERR_DOWNLINK_MALFORMED: return "RADIOLIB_ERR_DOWNLINK_MALFORMED";
    case RADIOLIB_ERR_INVALID_REVISION: return "RADIOLIB_ERR_INVALID_REVISION";
    case RADIOLIB_ERR_INVALID_PORT: return "RADIOLIB_ERR_INVALID_PORT";
    case RADIOLIB_ERR_NO_RX_WINDOW: return "RADIOLIB_ERR_NO_RX_WINDOW";
    case RADIOLIB_ERR_INVALID_CID: return "RADIOLIB_ERR_INVALID_CID";
    case RADIOLIB_ERR_UPLINK_UNAVAILABLE: return "RADIOLIB_ERR_UPLINK_UNAVAILABLE";
    case RADIOLIB_ERR_COMMAND_QUEUE_FULL: return "RADIOLIB_ERR_COMMAND_QUEUE_FULL";
    case RADIOLIB_ERR_COMMAND_QUEUE_ITEM_NOT_FOUND: return "RADIOLIB_ERR_COMMAND_QUEUE_ITEM_NOT_FOUND";
    case RADIOLIB_ERR_JOIN_NONCE_INVALID: return "RADIOLIB_ERR_JOIN_NONCE_INVALID";
    case RADIOLIB_ERR_DWELL_TIME_EXCEEDED: return "RADIOLIB_ERR_DWELL_TIME_EXCEEDED";
    case RADIOLIB_ERR_CHECKSUM_MISMATCH: return "RADIOLIB_ERR_CHECKSUM_MISMATCH";
    case RADIOLIB_ERR_NO_JOIN_ACCEPT: return "RADIOLIB_ERR_NO_JOIN_ACCEPT";
    case RADIOLIB_LORAWAN_SESSION_RESTORED: return "RADIOLIB_LORAWAN_SESSION_RESTORED";
    case RADIOLIB_LORAWAN_NEW_SESSION: return "RADIOLIB_LORAWAN_NEW_SESSION";
    case RADIOLIB_ERR_NONCES_DISCARDED: return "RADIOLIB_ERR_NONCES_DISCARDED";
    case RADIOLIB_ERR_SESSION_DISCARDED: return "RADIOLIB_ERR_SESSION_DISCARDED";
  }
  return "See https://jgromes.github.io/RadioLib/group__status__codes.html";
}

#endif