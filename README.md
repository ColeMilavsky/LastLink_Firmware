# LastLink Firmware

Custom LoRa communication firmware for the **Heltec WiFi LoRa 32 V3** (ESP32-S3 + SX1262).

Two devices running this firmware can send messages to each other over LoRa via a serial terminal.

---

## Project Structure

```
LastLink_Firmware/
├── platformio.ini              # Build config, board, dependencies
├── include/
│   └── config.h                # Pin defs, LoRa settings, tuneable constants
├── src/
│   └── main/
│       ├── main.cpp            # setup() / loop() entry point
│       ├── lora/
│       │   ├── lora_radio.h    # LoRa send/receive API
│       │   └── lora_radio.cpp
│       ├── serial/
│       │   ├── serial_handler.h  # Serial line reader
│       │   └── serial_handler.cpp
│       └── mesh/
│           ├── mesh.h          # Mesh routing stub (future)
│           └── mesh.cpp
├── lib/                        # Local libraries (place custom libs here)
├── test/                       # Unit tests (PlatformIO test framework)
└── scripts/                    # Helper scripts (OTA, flash, etc.)
```

---

## Quick Start

### Prerequisites
- [PlatformIO](https://platformio.org/) (VSCode extension or CLI)
- Two Heltec WiFi LoRa 32 V3 boards

### Build & Flash
```bash
# Flash to connected board
pio run --target upload

# Open serial monitor
pio device monitor --baud 115200
```

### Usage
1. Flash both boards with the same firmware.
2. Open a serial monitor on each (115200 baud).
3. Type a message on one board and press **Enter**.
4. The other board will print the received message with RSSI and SNR info.

---

## Configuration (`include/config.h`)

| Setting | Default | Notes |
|---|---|---|
| `LORA_FREQUENCY` | `915E6` | Change to `868E6` for EU |
| `LORA_SPREAD_FACTOR` | `7` | SF7=fast/short, SF12=slow/long range |
| `LORA_TX_POWER` | `20` | Max 20 dBm |
| `LORA_SYNC_WORD` | `0x34` | Devices must match to hear each other |
| `MSG_MAX_LEN` | `250` | Max message bytes |

---

## Roadmap

- [ ] Mesh routing in `mesh/` (multi-hop, node addressing)
- [ ] OLED display support (show last RX message)
- [ ] Acknowledged messages (ACK/NACK)
- [ ] OTA firmware updates
