#include "UI.h"
#include <Wire.h>
#include <SSD1306Wire.h>
#include "../mesh/mesh.h"

#define OLED_SDA   17
#define OLED_SCL   18
#define OLED_RST   21

static SSD1306Wire display(0x3c, OLED_SDA, OLED_SCL);

UiHandler Ui;

UiHandler::UiHandler() : _bleConnected(false) {}

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

void UiHandler::update() {
    // Reserved for future use
}

void UiHandler::_drawHeader(const char* label, bool sending) {
    display.clear();
    display.setFont(ArialMT_Plain_10);

    String arrow = sending ? "TX >>" : "RX <<";
    display.drawString(0, 0, arrow);
    display.drawString(40, 0, label);
    display.drawLine(0, 12, 128, 12);
}

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

void UiHandler::showBleDisconnected() {
    _bleConnected = false;
    showIdle();
}

void UiHandler::showSending(const String& message, const String& source) {
    _drawHeader("SENDING...", true);
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 16, "via " + source);
    _wrapAndPrint(message, 28);
}

void UiHandler::showSendComplete(const String& message, const String& source) {
    _drawHeader("SENT", true);
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 16, "via " + source);
    _wrapAndPrint(message, 28);
}

void UiHandler::showSendFailed(const String& message) {
    _drawHeader("SEND FAILED", true);
    display.setFont(ArialMT_Plain_10);
    _wrapAndPrint(message, 16);
}

void UiHandler::showReceiving() {
    _drawHeader("RECEIVING...", false);
    display.display();
}

void UiHandler::showReceiveComplete(const String& message, int rssi, float snr) {
    _drawHeader("RECEIVED", false);
    display.setFont(ArialMT_Plain_10);

    char statsBuf[32];
    snprintf(statsBuf, sizeof(statsBuf), "RSSI:%d SNR:%.1f", rssi, snr);
    display.drawString(0, 16, statsBuf);

    _wrapAndPrint(message, 28);
}

void UiHandler::showIdle() {
    display.clear();
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, "LastLink");
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 20, "Ready");
    display.drawString(0, 36, _bleConnected ? ("BLE: " + _lastBleDevice) : "BLE: waiting...");
    display.display();
}

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