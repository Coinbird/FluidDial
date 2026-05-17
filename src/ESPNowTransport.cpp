// Copyright (c) 2024 FluidDial contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#ifdef USE_ESPNOW

#include "ESPNowTransport.h"
#include "NVS.h"
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <driver/uart.h>
#include <atomic>
#include <cstring>

// Provided by SystemArduino.cpp — the ESP-IDF UART port handle.
extern uart_port_t fnc_uart_port;

static bool _uart_mode = true;

// ── TX ────────────────────────────────────────────────────────────────────────

static const uint8_t kBroadcast[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
// _tx_mac starts as broadcast; switches to unicast when "[FluidNC: Connected]"
// is received.  Written in the WiFi task (on_recv), read in main task (flush_tx).
// The transition happens exactly once at startup so we don't need a mutex.
static uint8_t _tx_mac[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

static uint8_t _tx_buf[250];
static size_t  _tx_len = 0;

static std::atomic<int> _tx_inflight { 0 };
static uint32_t         _tx_drops = 0;

static constexpr int kMaxInflight = 4;  // ESP-NOW internal queue is ~7

static void on_send(const uint8_t* /*mac*/, esp_now_send_status_t /*status*/) {
    _tx_inflight.fetch_sub(1, std::memory_order_relaxed);
}

static void flush_tx() {
    if (_tx_len == 0) {
        return;
    }
    // Back-pressure: wait briefly if the radio queue is saturated.
    for (int attempt = 0; attempt < 3; attempt++) {
        if (_tx_inflight.load(std::memory_order_relaxed) < kMaxInflight) {
            break;
        }
        delay(2);
    }
    if (_tx_inflight.load(std::memory_order_relaxed) >= kMaxInflight) {
        _tx_drops++;
        _tx_len = 0;
        return;
    }
    if (esp_now_send(_tx_mac, _tx_buf, _tx_len) == ESP_OK) {
        _tx_inflight.fetch_add(1, std::memory_order_relaxed);
    } else {
        _tx_drops++;
    }
    _tx_len = 0;
}

void espnow_putchar(uint8_t c) {
    if (_tx_len < sizeof(_tx_buf)) {
        _tx_buf[_tx_len++] = c;
    }
    // Flush immediately for all realtime commands:
    //  - High-byte (>= 0x80): 0x84 SafetyDoor, 0x85 JogCancel, 0x90+ overrides, etc.
    //  - Control chars (< 0x20): 0x18 Reset, 0x11 XON, 0x0A newline, etc.
    //  - ASCII single-char realtime: '?' StatusReport, '!' FeedHold, '~' CycleStart
    bool is_realtime = (c < 0x20 || c >= 0x80 || c == '?' || c == '!' || c == '~');
    if (is_realtime || _tx_len >= sizeof(_tx_buf)) {
        flush_tx();
    }
}

// ── RX (lock-free SPSC ring) ──────────────────────────────────────────────────

static const uint32_t kRxBufSize = 512;
static const uint32_t kRxBufMask = kRxBufSize - 1;

static std::atomic<uint32_t> _rx_head { 0 };
static std::atomic<uint32_t> _rx_tail { 0 };
static uint8_t               _rx_buf[kRxBufSize];

static void rx_push(uint8_t c) {
    uint32_t head = _rx_head.load(std::memory_order_relaxed);
    uint32_t next = (head + 1) & kRxBufMask;
    if (next == _rx_tail.load(std::memory_order_acquire)) {
        return;  // ring full — drop
    }
    _rx_buf[head] = c;
    _rx_head.store(next, std::memory_order_release);
}

int espnow_getchar() {
    uint32_t tail = _rx_tail.load(std::memory_order_relaxed);
    if (tail == _rx_head.load(std::memory_order_acquire)) {
        return -1;
    }
    uint8_t c = _rx_buf[tail];
    _rx_tail.store((tail + 1) & kRxBufMask, std::memory_order_release);
    return static_cast<int>(c);
}

// ── RX callback ───────────────────────────────────────────────────────────────

// _channel_alive: set when "[FluidNC: ?]" reply arrives during channel scan.
// _connected:     set when "[FluidNC: Connected]" arrives during handshake.
// Both are written in the WiFi task and read in the main task at startup only,
// so volatile is sufficient.
static volatile bool _channel_alive = false;
static volatile bool _connected     = false;
static volatile int  _pendant_id    = 0;

static void on_recv(const uint8_t* mac, const uint8_t* data, int len) {
    if (len <= 0) {
        return;
    }

    // Detect "[FluidNC: cmd]" control frames.
    static const char kPrefix[]  = "[FluidNC: ";
    static const int  kPrefixLen = sizeof(kPrefix) - 1;

    if (len > kPrefixLen && data[0] == '[' && data[len - 1] == ']' &&
        memcmp(data, kPrefix, kPrefixLen) == 0) {
        int cmdLen = len - kPrefixLen - 1;

        if (cmdLen == 1 && data[kPrefixLen] == '?') {
            // "[FluidNC: ?]" — server acknowledged our channel probe.
            _channel_alive = true;

        } else if (cmdLen >= 9 && memcmp(data + kPrefixLen, "Connected", 9) == 0) {
            // "[FluidNC: Connected]" or "[FluidNC: Connected id=N]"
            // Latch MAC, switch TX to unicast, store assigned ID.
            memcpy(_tx_mac, mac, 6);
            if (!esp_now_is_peer_exist(mac)) {
                esp_now_peer_info_t peer = {};
                memcpy(peer.peer_addr, mac, 6);
                peer.channel = 0;
                peer.encrypt = false;
                esp_now_add_peer(&peer);
            }
            // Parse optional "id=N" suffix.
            const char* id_str = strstr(reinterpret_cast<const char*>(data + kPrefixLen), "id=");
            _pendant_id        = id_str ? atoi(id_str + 3) : 0;
            _connected         = true;
        }
        // All other [FluidNC: ...] frames (Busy, broadcast status, etc.) are
        // silently ignored on the controller side.
        return;
    }

    // Raw GCode response from FluidNC — push to ring for the GCode parser.
    for (int i = 0; i < len; i++) {
        rx_push(data[i]);
    }
}

// ── Mode query ────────────────────────────────────────────────────────────────

bool espnow_use_uart_mode() {
    return _uart_mode;
}

// ── Channel scanning ──────────────────────────────────────────────────────────
// FluidNC lives on whatever WiFi channel its router gave it.  We probe each
// candidate with a standard '?' status query.  FluidNC echoes "[FluidNC: ?]"
// back to confirm it's alive on that channel without requiring a full connection.
// The result is cached in NVS so subsequent boots skip the scan.
// After the channel is confirmed, connect_to_server() sends [FluidNC: Connect]
// to establish the unicast link.

static nvs_handle_t _espnow_nvs = 0;

static bool try_channel(uint8_t channel) {
    if (esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
        return false;
    }
    delay(20);  // let the radio settle

    _channel_alive = false;

    const char probe[] = "?";
    esp_now_send(kBroadcast, reinterpret_cast<const uint8_t*>(probe), sizeof(probe) - 1);

    uint32_t deadline = millis() + 200;
    while ((int32_t)(millis() - deadline) < 0) {
        if (_channel_alive) {
            return true;
        }
        delay(5);
    }
    return false;
}

static uint8_t scan_for_channel() {
    // Try the cached channel first — usually correct across reboots.
    int32_t cached = 0;
    if (_espnow_nvs) {
        nvs_get_i32(_espnow_nvs, "channel", &cached);
    }
    if (cached >= 1 && cached <= 13) {
        if (try_channel(static_cast<uint8_t>(cached))) {
            return static_cast<uint8_t>(cached);
        }
    }
    // Common non-overlapping channels first, then the rest.
    static const uint8_t order[] = { 1, 6, 11, 2, 3, 4, 5, 7, 8, 9, 10, 12, 13 };
    for (uint8_t ch : order) {
        if (static_cast<int32_t>(ch) == cached) {
            continue;  // already tried
        }
        if (try_channel(ch)) {
            return ch;
        }
    }
    return 1;  // fallback; user can reboot once FluidNC is visible
}

static void connect_to_server() {
    // Drain any probe artifacts from the ring before entering GCode mode.
    _rx_tail.store(_rx_head.load(std::memory_order_acquire), std::memory_order_relaxed);

    _connected = false;
    static const char kConnect[] = "[FluidNC: Connect]";
    esp_now_send(kBroadcast,
                 reinterpret_cast<const uint8_t*>(kConnect),
                 sizeof(kConnect) - 1);

    uint32_t deadline = millis() + 500;
    while ((int32_t)(millis() - deadline) < 0) {
        if (_connected) {
            return;
        }
        delay(5);
    }
    // No reply — FluidNC may be older firmware without [FluidNC: ...] support.
    // TX stays in broadcast mode; jog watchdog won't fire but basic operation works.
}

// ── Reconnect ─────────────────────────────────────────────────────────────────
// Called every ~6 s by fnc_is_connected() while the link is down. A controller
// reboot can bring FluidNC back on a different WiFi channel, which would leave
// us broadcasting [FluidNC: Connect] on a dead channel forever. So: first retry
// cheaply on the current channel; if that keeps failing, re-scan all channels.

void espnow_request_connect() {
    static uint32_t last_full_scan = 0;

    memcpy(_tx_mac, kBroadcast, 6);  // accept a fresh MAC from the response
    _pendant_id = 0;

    // Cheap attempt: Connect on the channel we're already on.
    connect_to_server();
    if (_connected) {
        return;
    }

    // Still down. Re-acquire the controller's channel — throttled to once
    // per 20 s so the multi-channel scan doesn't keep freezing the UI while
    // the controller is simply powered off.
    uint32_t now = millis();
    if ((now - last_full_scan) < 20000) {
        return;
    }
    last_full_scan = now;

    uint8_t locked = scan_for_channel();
    esp_wifi_set_channel(locked, WIFI_SECOND_CHAN_NONE);
    if (_espnow_nvs) {
        nvs_set_i32(_espnow_nvs, "channel", static_cast<int>(locked));
    }
    connect_to_server();
}

int espnow_pendant_id() {
    return _pendant_id;
}

// ── Init ──────────────────────────────────────────────────────────────────────

static void init_radio() {
    // Station mode without connecting to an AP is required for ESP-NOW.
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK) {
        return;
    }
    esp_now_register_recv_cb(on_recv);
    esp_now_register_send_cb(on_send);

    // Register broadcast peer so we can probe before any server replies.
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, kBroadcast, 6);
    peer.channel = 0;  // 0 = inherit current WiFi channel
    peer.encrypt = false;
    esp_now_add_peer(&peer);

    _espnow_nvs    = nvs_init("espnow");
    uint8_t locked = scan_for_channel();
    esp_wifi_set_channel(locked, WIFI_SECOND_CHAN_NONE);
    if (_espnow_nvs) {
        nvs_set_i32(_espnow_nvs, "channel", static_cast<int>(locked));
    }

    connect_to_server();
}

void espnow_transport_init() {
    // Active UART probe: send '?' and wait for FluidNC to respond.
    // If no response within the timeout, fall through to ESP-NOW mode.
    const uint32_t kTimeoutMs = 1500;

    // Flush any stale RX bytes before probing.
    char dummy;
    while (uart_read_bytes(fnc_uart_port, &dummy, 1, 0) == 1) {}

    const char probe[] = "?\n";
    uart_write_bytes(fnc_uart_port, probe, sizeof(probe) - 1);

    uint32_t start = millis();
    while (millis() - start < kTimeoutMs) {
        char c;
        if (uart_read_bytes(fnc_uart_port, &c, 1, 0) == 1) {
            _uart_mode = true;
            return;
        }
        delay(10);
    }
    _uart_mode = false;
    init_radio();
}

#endif  // USE_ESPNOW
