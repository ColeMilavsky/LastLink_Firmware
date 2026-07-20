#include "UI.h"
#include <Wire.h>
#include <SSD1306Wire.h>
#include "../mesh/mesh.h"

#define OLED_SDA   17
#define OLED_SCL   18
#define OLED_RST   21

static SSD1306Wire display(0x3c, OLED_SDA, OLED_SCL);

UiHandler Ui;

// In: none. Out: none. Constructs the handler with BLE shown as disconnected.
UiHandler::UiHandler() : _bleConnected(false) {}

// In: none. Out: none.
// Resets and initializes the OLED display, then draws the idle screen.
void UiHandler::begin() {
    pinMode(OLED_RST, OUTPUT);
    digitalWrite(OLED_RST, LOW);
    delay(50);
    digitalWrite(OLED_RST, HIGH);

    display.init();
    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_10);

    showIdle();
}

// In: none. Out: none. Reserved for future use (nothing to poll today).
void UiHandler::update() {
    // Reserved for future use
}

// In: label - text shown next to the direction arrow; sending - true for a
//     TX screen (">>"), false for RX ("<<"). Out: none.
// Clears the display and draws the shared header bar used by the send/receive screens.
void UiHandler::_drawHeader(const char* label, bool sending) {
    display.clear();
    display.setFont(ArialMT_Plain_10);

    String arrow = sending ? "TX >>" : "RX <<";
    display.drawString(0, 0, arrow);
    display.drawString(40, 0, label);
    display.drawLine(0, 12, 128, 12);
}

// In: text - message to display; startY - vertical pixel offset to start at.
// Out: none. Word-wraps text to the OLED's character width, draws each line
// (stopping once it runs off the bottom of the screen), then flushes to the display.
void UiHandler::_wrapAndPrint(const String& text, int startY) {
    const int maxCharsPerLine = 21;
    const int lineHeight = 12;

    String remaining = text;
    int y = startY;

    while (remaining.length() > 0 && y < 64) {
        String line;
        if ((int)remaining.length() <= maxCharsPerLine) {
            line = remaining;
            remaining = "";
        } else {
            int splitAt = maxCharsPerLine;
            int lastSpace = remaining.substring(0, maxCharsPerLine).lastIndexOf(' ');
            if (lastSpace > 0) splitAt = lastSpace;
            line = remaining.substring(0, splitAt);
            remaining = remaining.substring(splitAt);
            remaining.trim();
        }
        display.drawString(0, y, line);
        y += lineHeight;
    }
    display.display();
}

// In: deviceName - name of the connected phone/device. Out: none.
// Marks BLE as connected and draws the "Phone Connected" screen.
void UiHandler::showBleConnected(const String& deviceName) {
    _bleConnected = true;
    _lastBleDevice = deviceName;

    display.clear();
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, "Phone");
    display.drawString(0, 18, "Connected");
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 40, deviceName);
    display.display();
}

// In: none. Out: none. Marks BLE as disconnected and falls back to the idle screen.
void UiHandler::showBleDisconnected() {
    _bleConnected = false;
    showIdle();
}

// In: message - text being sent; source - label describing where it's going
//     (e.g. a nickname or "BLE"/"SERIAL"). Out: none.
// Draws the "SENDING..." screen with the message body.
void UiHandler::showSending(const String& message, const String& source) {
    _drawHeader("SENDING...", true);
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 16, "via " + source);
    _wrapAndPrint(message, 28);
}

// In: message - text that was sent; source - label describing where it went.
// Out: none. Draws the "SENT" confirmation screen.
void UiHandler::showSendComplete(const String& message, const String& source) {
    _drawHeader("SENT", true);
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 16, "via " + source);
    _wrapAndPrint(message, 28);
}

// In: message - text that failed to send. Out: none.
// Draws the "SEND FAILED" screen.
void UiHandler::showSendFailed(const String& message) {
    _drawHeader("SEND FAILED", true);
    display.setFont(ArialMT_Plain_10);
    _wrapAndPrint(message, 16);
}

// In: none. Out: none. Draws the "RECEIVING..." screen.
void UiHandler::showReceiving() {
    _drawHeader("RECEIVING...", false);
    display.display();
}

// In: message - received text; rssi/snr - signal quality of the reception.
// Out: none. Draws the "RECEIVED" screen with signal stats and the message body.
void UiHandler::showReceiveComplete(const String& message, int rssi, float snr) {
    _drawHeader("RECEIVED", false);
    display.setFont(ArialMT_Plain_10);

    char statsBuf[32];
    snprintf(statsBuf, sizeof(statsBuf), "RSSI:%d SNR:%.1f", rssi, snr);
    display.drawString(0, 16, statsBuf);

    _wrapAndPrint(message, 28);
}

// In: none. Out: none.
// Draws the idle/ready screen, showing BLE connection state.
void UiHandler::showIdle() {
    display.clear();
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, "LastLink");
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 20, "Ready");
    display.drawString(0, 36, _bleConnected ? ("BLE: " + _lastBleDevice) : "BLE: waiting...");
    display.display();
}

// In: none. Out: none.
// Draws the known nickname directory (node id -> nickname, marking our own
// entry), pulling current entries from the global Mesh instance.
void UiHandler::showMeshDirectory() {
    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, "Mesh Directory");
    display.drawLine(0, 12, 128, 12);

    int count = Mesh.directoryCount();
    if (count == 0) {
        display.drawString(0, 18, "(no nodes seen yet)");
    } else {
        int y = 16;
        for (int i = 0; i < count && y < 64; i++) {
            char   nodeId;
            String nickname;
            if (Mesh.directoryEntryAt(i, nodeId, nickname)) {
                String line = String(nodeId) + ": " + nickname;
                if (nodeId == Mesh.nodeId()) line += " (me)";
                display.drawString(0, y, line);
                y += 12;
            }
        }
    }
    display.display();
}