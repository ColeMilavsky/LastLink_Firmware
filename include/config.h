#pragma once

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
