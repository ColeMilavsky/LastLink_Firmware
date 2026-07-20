#include "UI.h"
#include <Wire.h>
#include <SSD1306Wire.h>
#include "../mesh/mesh.h"

#define OLED_SDA   17
#define OLED_SCL   18
#define OLED_RST   21
#define OLED_VEXT  36  // Heltec V3: gates power to the OLED (and other external
                       // peripherals); active-low, must be driven LOW to enable it

static SSD1306Wire display(0x3c, OLED_SDA, OLED_SCL);

UiHandler Ui;

// In: none. Out: none.
// Constructs the handler with no identity set yet; begin() puts it into
// SCREEN_HOME once the display is ready.
UiHandler::UiHandler()
    : _screen(SCREEN_HOME), _screenExpiresAt(0), _nodeId(0),
      _meshPage(0), _lastMeshPageChange(0), _ackDestNode(0), _ackDelivered(false) {}

// In: none. Out: none.
// Enables the Heltec V3's Vext power rail (GPIO36, active-low) that feeds
// the OLED — left untouched, the display stays unpowered and never lights
// up no matter what I2C/reset sequencing follows. Then resets and
// initializes the display driver (unchanged from the previous fix), and
// enters the Home splash screen for UI_HOME_SCREEN_MS.
void UiHandler::begin() {
    pinMode(OLED_VEXT, OUTPUT);
    digitalWrite(OLED_VEXT, LOW); // LOW enables Vext on Heltec V3
    delay(100); // let the rail stabilize before driving the display

    pinMode(OLED_RST, OUTPUT);
    digitalWrite(OLED_RST, LOW);
    delay(50);
    digitalWrite(OLED_RST, HIGH);

    display.init();
    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_10);

    _screen = SCREEN_HOME;
    _screenExpiresAt = millis() + UI_HOME_SCREEN_MS;
    _render();
}

// In: none. Out: none.
// The only place timed transitions fire: expires Home into Mesh (once,
// never re-entered), auto-advances Mesh's page on its interval (skipped
// entirely when there's only one page, to avoid a wasted full-screen
// redraw), and auto-returns any other (transient event) screen to Mesh once
// its timer elapses. Screens with _screenExpiresAt==0 (Mesh itself, and
// Sending, which is always immediately followed by a Sent/Failed call) are left alone.
void UiHandler::update() {
    unsigned long now = millis();

    if (_screen == SCREEN_HOME) {
        if (now >= _screenExpiresAt) _enterMesh();
        return;
    }

    if (_screen == SCREEN_MESH) {
        int totalPages = _meshPageCount();

        if (totalPages > 1 && now - _lastMeshPageChange >= UI_MESH_PAGE_INTERVAL_MS) {
            _meshPage++;
            _lastMeshPageChange = now;
            _renderMesh();
        }
        return;
    }

    if (_screenExpiresAt != 0 && now >= _screenExpiresAt) {
        _enterMesh();
    }
}

// In: nodeId - this node's single-character identity; nickname - the
//     connected phone's registered nickname (empty if none yet). Out: none.
// Stores the identity for the Home splash and every screen's footer, and
// re-renders immediately if Home or Mesh is currently showing so it's
// visible right away rather than waiting for the next unrelated event.
void UiHandler::setIdentity(char nodeId, const String& nickname) {
    _nodeId = nodeId;
    _localNickname = nickname;
    if (_screen == SCREEN_HOME || _screen == SCREEN_MESH) {
        _render();
    }
}

// In: deviceName - name of the connected phone/device. Out: none.
// Fires the "phone connected" transient event (5s, then back to Mesh).
void UiHandler::notifyBleConnected(const String& deviceName) {
    _lastBleDevice = deviceName;
    _screen = SCREEN_BLE_CONNECTED;
    _screenExpiresAt = millis() + UI_BLE_EVENT_MS;
    _render();
}

// In: none. Out: none.
// Fires the "phone disconnected" transient event (5s, then back to Mesh).
void UiHandler::notifyBleDisconnected() {
    _screen = SCREEN_BLE_DISCONNECTED;
    _screenExpiresAt = millis() + UI_BLE_EVENT_MS;
    _render();
}

// In: message - text being sent; source - label describing where it's going.
// Out: none. Enters SCREEN_SENDING with no auto-expiry — the caller always
// follows this immediately with notifySendComplete()/notifySendFailed()
// once the radio call returns, which is what actually times out and returns
// to Mesh.
void UiHandler::notifySending(const String& message, const String& source) {
    _sendMessage = message;
    _sendSource  = source;
    _screen = SCREEN_SENDING;
    _screenExpiresAt = 0;
    _render();
}

// In: message - text that was sent; source - label describing where it went.
// Out: none. Fires the "sent" transient event (5s, then back to Mesh).
void UiHandler::notifySendComplete(const String& message, const String& source) {
    _sendMessage = message;
    _sendSource  = source;
    _screen = SCREEN_SENT;
    _screenExpiresAt = millis() + UI_SEND_STATUS_MS;
    _render();
}

// In: message - text that failed to send. Out: none.
// Fires the "send failed" transient event (5s, then back to Mesh).
void UiHandler::notifySendFailed(const String& message) {
    _sendMessage = message;
    _screen = SCREEN_SEND_FAILED;
    _screenExpiresAt = millis() + UI_SEND_STATUS_MS;
    _render();
}

// In: from - sender's nickname or node id; message - the chat text. Out: none.
// Fires the "message received" event: interrupts whatever is currently
// showing immediately and displays for UI_MESSAGE_SCREEN_MS before
// update() returns to Mesh.
void UiHandler::notifyMessageReceived(const String& from, const String& message) {
    _msgFrom = from;
    _msgText = message;
    _screen = SCREEN_MESSAGE;
    _screenExpiresAt = millis() + UI_MESSAGE_SCREEN_MS;
    _render();
}

// In: destNode - node the original chat was sent to; delivered - true if
//     acked, false if it timed out. Out: none.
// Fires the "delivery status" event: interrupts whatever is currently
// showing immediately and displays for UI_ACK_SCREEN_MS before update()
// returns to Mesh.
void UiHandler::notifyAckReceived(char destNode, bool delivered) {
    _ackDestNode = destNode;
    _ackDelivered = delivered;
    _screen = SCREEN_ACK;
    _screenExpiresAt = millis() + UI_ACK_SCREEN_MS;
    _render();
}

// In: none. Out: none.
// Redraws the Mesh screen in place (without resetting its page or timer) if
// it's the currently active screen; a no-op otherwise, since routing/
// directory data is pulled live from Mesh whenever the screen is next entered.
void UiHandler::refreshMesh() {
    if (_screen == SCREEN_MESH) {
        _renderMesh();
    }
}

// In: none. Out: none.
// Dispatches to the render function for whatever _screen currently is —
// the single centralized place screen content is drawn.
void UiHandler::_render() {
    switch (_screen) {
        case SCREEN_HOME:             _renderHome();             break;
        case SCREEN_MESH:             _renderMesh();             break;
        case SCREEN_MESSAGE:          _renderMessage();          break;
        case SCREEN_ACK:              _renderAck();              break;
        case SCREEN_SENDING:          _renderSending();          break;
        case SCREEN_SENT:             _renderSendResult(true);   break;
        case SCREEN_SEND_FAILED:      _renderSendResult(false);  break;
        case SCREEN_BLE_CONNECTED:    _renderBleConnected();     break;
        case SCREEN_BLE_DISCONNECTED: _renderBleDisconnected();  break;
    }
}

// In: none. Out: none.
// Transitions to the Mesh screen — the fallback state every timed event
// returns to — resetting its page to the start and rendering immediately.
void UiHandler::_enterMesh() {
    _screen = SCREEN_MESH;
    _screenExpiresAt = 0;
    _meshPage = 0;
    _lastMeshPageChange = millis();
    _renderMesh();
}

// In: none. Out: total Mesh-screen page count (routing-table pages, each
// holding up to UI_ROUTES_PER_PAGE entries, followed by directory pages,
// each holding up to UI_NODES_PER_PAGE entries). Each category reserves at
// least one page even when empty, so its "none yet" message still gets a turn.
int UiHandler::_meshPageCount() const {
    int routeCount = Mesh.routeCount();
    int dirCount   = Mesh.directoryCount();
    int routePages = (routeCount == 0) ? 1 : (routeCount + UI_ROUTES_PER_PAGE - 1) / UI_ROUTES_PER_PAGE;
    int dirPages   = (dirCount == 0)   ? 1 : (dirCount + UI_NODES_PER_PAGE - 1) / UI_NODES_PER_PAGE;
    return routePages + dirPages;
}

// In: label - text shown next to the direction arrow; sending - true for a
//     TX screen (">>"), false for RX ("<<"). Out: none.
// Clears the display and draws the shared header bar used by send/receive-style screens.
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
// (stopping before the footer at y=52), then flushes to the display.
void UiHandler::_wrapAndPrint(const String& text, int startY) {
    const int maxCharsPerLine = 21;
    const int lineHeight = 12;

    String remaining = text;
    int y = startY;

    while (remaining.length() > 0 && y < 52) {
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

// In: none. Out: none.
// Draws the persistent one-line footer ("A:Bob", or just "A" with no
// nickname yet) shared by every screen so the current node identity is
// always visible regardless of what else is on screen.
void UiHandler::_drawStatusBar() {
    display.setFont(ArialMT_Plain_10);
    display.drawLine(0, 52, 128, 52);

    String bar = String(_nodeId ? _nodeId : '?');
    if (_localNickname.length() > 0) {
        bar += ":" + _localNickname;
    }
    display.drawString(0, 54, bar);
}

// In: none. Out: none.
// Draws the boot splash: "LastLink" branding plus this node's id/nickname.
// Shown exactly once, for UI_HOME_SCREEN_MS, then update() moves to Mesh
// and never returns here.
void UiHandler::_renderHome() {
    display.clear();
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, "LastLink");

    display.setFont(ArialMT_Plain_10);
    String idLine = "Node ";
    idLine += (_nodeId ? _nodeId : '?');
    if (_localNickname.length() > 0) {
        idLine += " (" + _localNickname + ")";
    }
    display.drawString(0, 24, idLine);

    _drawStatusBar();
    display.display();
}

// In: none. Out: none.
// Draws the current Mesh-screen page: the routing table's pages first
// (destination > next hop, cost, age), then the known-nodes directory's
// pages, cycling back to the start — this is the persistent resting state
// every timed event returns to. Page count is (re)computed from live Mesh
// data each call, so it self-corrects if entries were added/removed since
// the last render.
void UiHandler::_renderMesh() {
    display.clear();
    display.setFont(ArialMT_Plain_10);

    int routeCount = Mesh.routeCount();
    int dirCount   = Mesh.directoryCount();
    int routePages = (routeCount == 0) ? 1 : (routeCount + UI_ROUTES_PER_PAGE - 1) / UI_ROUTES_PER_PAGE;
    int totalPages = _meshPageCount();
    if (_meshPage >= totalPages) _meshPage = 0;

    bool onRoutesPage = _meshPage < routePages;

    if (onRoutesPage) {
        display.drawString(0, 0, "Routes");
        display.drawLine(0, 12, 128, 12);

        if (routeCount == 0) {
            display.drawString(0, 18, "(no routes yet)");
        } else {
            int start = _meshPage * UI_ROUTES_PER_PAGE;
            int y = 16;
            for (int i = start; i < routeCount && i < start + UI_ROUTES_PER_PAGE; i++) {
                char          dest, nextHop;
                uint8_t       cost;
                unsigned long ageMs;
                if (Mesh.routeEntryAt(i, dest, nextHop, cost, ageMs)) {
                    char line[24];
                    snprintf(line, sizeof(line), "%c>%c c%u %lus", dest, nextHop, (unsigned)cost, ageMs / 1000);
                    display.drawString(0, y, line);
                    y += 12;
                }
            }
        }
    } else {
        int dirPage = _meshPage - routePages;
        display.drawString(0, 0, "Known Nodes");
        display.drawLine(0, 12, 128, 12);

        if (dirCount == 0) {
            display.drawString(0, 18, "(none yet)");
        } else {
            int start = dirPage * UI_NODES_PER_PAGE;
            int y = 16;
            for (int i = start; i < dirCount && i < start + UI_NODES_PER_PAGE; i++) {
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
    }

    if (totalPages > 1) {
        char pageLabel[8];
        snprintf(pageLabel, sizeof(pageLabel), "%d/%d", _meshPage + 1, totalPages);
        display.drawString(96, 0, pageLabel);
    }

    _drawStatusBar();
    display.display();
}

// In: none. Out: none.
// Draws the received-message screen (sender + content) using the shared
// RX header, wrapping the body text above the footer.
void UiHandler::_renderMessage() {
    _drawHeader("RECEIVED", false);
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 16, "from " + _msgFrom);
    _drawStatusBar();
    _wrapAndPrint(_msgText, 26);
}

// In: none. Out: none.
// Draws the delivery-confirmation screen: a large DELIVERED/FAILED label
// plus which node it was to/from.
void UiHandler::_renderAck() {
    display.clear();
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 8, _ackDelivered ? "DELIVERED" : "FAILED");

    display.setFont(ArialMT_Plain_10);
    String line = "to ";
    line += _ackDestNode;
    display.drawString(0, 32, line);

    _drawStatusBar();
    display.display();
}

// In: none. Out: none.
// Draws the "SENDING..." screen with the outgoing message body.
void UiHandler::_renderSending() {
    _drawHeader("SENDING...", true);
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 16, "via " + _sendSource);
    _drawStatusBar();
    _wrapAndPrint(_sendMessage, 26);
}

// In: success - true to draw the "SENT" screen, false for "SEND FAILED".
// Out: none. Shares layout with _renderSending() since both describe the
// same outgoing message.
void UiHandler::_renderSendResult(bool success) {
    _drawHeader(success ? "SENT" : "SEND FAILED", true);
    display.setFont(ArialMT_Plain_10);
    if (success) {
        display.drawString(0, 16, "via " + _sendSource);
        _drawStatusBar();
        _wrapAndPrint(_sendMessage, 26);
    } else {
        _drawStatusBar();
        _wrapAndPrint(_sendMessage, 16);
    }
}

// In: none. Out: none.
// Draws the "Phone Connected" banner with the connected device's name.
void UiHandler::_renderBleConnected() {
    display.clear();
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, "Phone");
    display.drawString(0, 18, "Connected");
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 38, _lastBleDevice);
    _drawStatusBar();
    display.display();
}

// In: none. Out: none.
// Draws the "Phone Disconnected" banner.
void UiHandler::_renderBleDisconnected() {
    display.clear();
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 8, "Phone");
    display.drawString(0, 26, "Disconnected");
    _drawStatusBar();
    display.display();
}
