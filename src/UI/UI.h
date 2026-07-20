#pragma once

#include <Arduino.h>

// ─── OLED screen state machine ─────────────────────────────────────────────
//
// Four states as designed: HOME (splash, boot-only) -> MESH (persistent
// resting state, routing table + known nodes) -> interrupted at any time by
// MESSAGE or ACK (5s each), which always return to MESH when their timer
// elapses. A few pre-existing behaviors (BLE connect/disconnect, outgoing
// send status) are folded in as the same kind of timed transient event,
// so there is exactly one state enum and one render dispatch for the whole
// display — main.cpp and the mesh callbacks only call a notify*() entry
// point to report something happened; they never draw anything themselves.
//
// Interrupt policy: newest event always wins immediately. Every notify*()
// call unconditionally overwrites whatever screen/timer is active and
// renders right away — there is no queue, so a second event arriving while
// an event screen is still showing simply replaces it with a fresh timer.

#define UI_HOME_SCREEN_MS         10000UL // splash duration, shown once at boot
#define UI_MESSAGE_SCREEN_MS       5000UL // received-chat screen duration
#define UI_ACK_SCREEN_MS           5000UL // delivery-confirmation screen duration
#define UI_SEND_STATUS_MS          5000UL // SENT / SEND FAILED screen duration
#define UI_BLE_EVENT_MS            5000UL // BLE connected/disconnected screen duration
#define UI_MESH_PAGE_INTERVAL_MS   3000UL // how long each Mesh-screen page is shown
#define UI_ROUTES_PER_PAGE         3       // routing-table entries per Mesh page
#define UI_NODES_PER_PAGE          3       // directory entries per Mesh page

class UiHandler {
public:
    UiHandler();

    void begin();
    void update();

    // Sets this node's identity (id + connected phone's nickname, if any),
    // shown on the Home splash and in every screen's footer status bar.
    // Call once at startup and again whenever the nickname changes.
    void setIdentity(char nodeId, const String& nickname);

    // ── Event entry points — main.cpp/mesh callbacks call these to report
    //    something happened; UiHandler alone decides how/whether to render it.
    void notifyBleConnected(const String& deviceName);
    void notifyBleDisconnected();

    void notifySending(const String& message, const String& source);
    void notifySendComplete(const String& message, const String& source);
    void notifySendFailed(const String& message);

    void notifyMessageReceived(const String& from, const String& message);
    void notifyAckReceived(char destNode, bool delivered);

    // Redraws the Mesh screen (routing table + known nodes) in place if it's
    // currently showing; no-op otherwise (the data is pulled live from Mesh
    // whenever the screen is next entered, so nothing is lost by skipping this).
    void refreshMesh();

private:
    enum Screen {
        SCREEN_HOME,
        SCREEN_MESH,
        SCREEN_MESSAGE,
        SCREEN_ACK,
        SCREEN_SENDING,
        SCREEN_SENT,
        SCREEN_SEND_FAILED,
        SCREEN_BLE_CONNECTED,
        SCREEN_BLE_DISCONNECTED,
    };
    Screen        _screen;
    unsigned long _screenExpiresAt; // 0 means "no auto-expiry for this screen"

    char   _nodeId;
    String _localNickname;

    int           _meshPage;
    unsigned long _lastMeshPageChange;

    String _msgFrom, _msgText;
    char   _ackDestNode;
    bool   _ackDelivered;
    String _sendMessage, _sendSource;
    String _lastBleDevice;

    // Central dispatcher: draws whatever _screen currently is. Every
    // notify*()/setIdentity() call ends by calling this — it is the only
    // place screen content actually gets rendered.
    void _render();

    void _renderHome();
    void _renderMesh();
    void _renderMessage();
    void _renderAck();
    void _renderSending();
    void _renderSendResult(bool success);
    void _renderBleConnected();
    void _renderBleDisconnected();

    // Transitions to the Mesh screen (the fallback every timed event
    // returns to): resets its page/timer and renders it immediately.
    void _enterMesh();

    // In: none. Out: total number of Mesh-screen pages (routing-table pages
    // plus directory pages), computed live from current Mesh state. Always
    // >= 2 (each category reserves at least one page, even when empty, to
    // show its "none yet" message).
    int _meshPageCount() const;

    void _drawHeader(const char* label, bool sending);
    void _wrapAndPrint(const String& text, int startY);

    // Draws the persistent one-line footer ("A:Bob") shown on every screen.
    void _drawStatusBar();
};

extern UiHandler Ui;
