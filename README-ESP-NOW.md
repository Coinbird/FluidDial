# ESP-NOW FluidDial Pendant Transport Layer

Connects a FluidDial M5Dial pendant to a FluidNC Controller (Only tested on V1E Jackpot v1) wirelessly
using ESP-NOW for a transport layer, with automatic wired/wireless detection at boot.
Potential to work with the CYD pendant and other FluidNC controllers.

This is an "either/or" with the FluidDial WiFi/websocket functionality (ESP-NOW/ WiFi Mode could potentially be selectable if
there is enough space in memory?)

(CYD support is wired up — a `cyd_base` plus `_espnow` profiles — but is untested beyond
compiling.)

## Build Notes

Using PlatformIO, build both the FluidDial and FluidNC forks that include the ESP-NOW additions.

### FluidDial
Use the `m5dial_espnow` profile (CYD profiles exist but are untested). Via VS Code PlatformIO UI, select that environment. Also build and upload the filesystem on first flash (it includes the ESP-NOW status icon).
```
pio run -e m5dial_espnow
pio run -e m5dial_espnow -t upload
# first time or after filesystem changes:
pio run -e m5dial_espnow -t buildfs
pio run -e m5dial_espnow -t uploadfs
```

### FluidNC
Use the `wifi` or `wifi_s3` profile (only ones tested on Jackpot). Build and flash normally. Add the `espnow_channel:` stanza and update `I2S_STATIC` → `I2S` engine in `config.yaml` (see Configure section below).


## Terminology

- **Controller** — the FluidNC machine controller (tested on V1E Jackpot v1)
- **Remote** — a pendant (FluidDial). FluidNC assigns each remote a unique numeric ID on
  connect (`id=1`, `id=2`, …).

## How it works

ESP-NOW supports two addressing modes:

- **Broadcast** (FF:FF:FF:FF:FF:FF) — fire-and-forget; any device on the same channel receives
  the frame. No acknowledgement, no retry. Used for channel discovery (`?` probe), the
  `[FluidNC: Connect]` handshake, and periodic status reports to display-only devices.
- **Unicast** (specific MAC) — the 802.11 layer automatically acknowledges every frame and
  retries on failure. Used for all GCode traffic once a remote connects. This makes the ESP-NOW direct link
  substantially more reliable for jog and feed commands.

At startup FluidDial scans channels until FluidNC responds, then sends `[FluidNC: Connect]`.
FluidNC replies `[FluidNC: Connected id=N]`, both sides latch each other's MAC, and all
subsequent traffic switches to unicast. 

The broadcast status channel stays active so display-only devices (no connection needed) keep receiving status updates, unless the broadcast_interval_ms is set to 0 (disables that feature.)

## Pairing sequence
* See time sequence mermaid diagram at the bottom of this document.

## Hardware (as tested)
- **Controller**: FluidNC Controller running this branch of FluidNC (tested on [V1E Jackpot](https://docs.v1e.com/electronics/jackpot/)) This is a fork of the Portability branch.
- **Pendant**: M5Dial (ESP32-S3) running FluidDial `m5dial_espnow` build

---
## Configure config.yaml
If you have I2S_STATIC in your `config.yaml`, change to:
```yaml
stepping: 
  engine: I2S
```
(This is a change from the Portability branch)


For ESP-NOW support, add this to the FluidNC `config.yaml` alongside the uart_channel sections:
```yaml
espnow_channel:
  report_interval_ms: 100
  broadcast_interval_ms: 200
```
- `report_interval_ms` — how often FluidNC pushes a status report to a *connected* remote
  (e.g. FluidDial). 100–200 ms gives a responsive UI. In ESP-NOW mode FluidDial does **not**
  send `$RI=` — the YAML value is authoritative.
- `broadcast_interval_ms` — how often (ms) FluidNC broadcasts status as `[FluidNC: <...>]` for
  passive display devices. Optional; **defaults to 200 ms** if omitted. Set to `0` to disable
  broadcasting. Display devices are passive listeners — they never connect and never set this;
  it is config-authoritative.

---
## Architecture notes

### Wired vs Wireless detection (FluidDial side)
At boot, `espnow_transport_init()` in `ESPNowTransport.cpp` actively probes the UART: it sends
`?\n` and waits up to **1500 ms** for any response. If a byte arrives → stay in UART mode.
If timeout → switch to ESP-NOW mode. The active probe works whether FluidNC is freshly booted
or already running idle, because `?` always elicits a status report. Only one transport is
active at a time.

### FluidDial transport routing (`SystemArduino.cpp`)
`fnc_putchar()` / `fnc_getchar()` already route to UART or WebSocket. ESP-NOW is a third path,
selected when `USE_ESPNOW` is defined and `espnow_use_uart_mode()` returns false.

### `[FluidNC: ...]` protocol
A lightweight framing layer for connection management. Raw GCode bytes flow without framing
once connected.

| Frame | Direction | Meaning |
|-------|-----------|---------|
| `[FluidNC: Connect]` | Remote → FluidNC | Request a unicast channel |
| `[FluidNC: Connected id=N]` | FluidNC → Remote | Confirmed; remote ID N assigned, MAC latched for unicast |
| `[FluidNC: Busy]` | FluidNC → Remote | All 4 remote slots occupied |
| `[FluidNC: Disconnect]` | Remote → FluidNC | Release the slot (graceful disconnect; optional) |
| `[FluidNC: <Idle\|...>]` | FluidNC → all | Periodic broadcast status (display mode), gated by `broadcast_interval_ms` |
| `[FluidNC: ?]` | FluidNC → all | Broadcast "I'm here" reply to a bare `?` probe — used by FluidDial during WiFi channel scanning to confirm FluidNC is alive on a given channel before attempting to connect |

### FluidNC side (`ESPNowServer` + `ESPNowClient` + `ESPNowBroadcastChannel`)
Split into classes following the TelnetServer/TelnetClient pattern:
- **`ESPNowServer`** (`Configurable`): owns radio init, broadcast peer setup, and static
  recv/send callbacks. Parses `[FluidNC: ...]` frames and manages up to 4 simultaneous remote
  slots. Init is deferred until **after** WiFi is up (after the Modules loop in `Main.cpp`)
  because `esp_now_init()` requires WiFi to be running.
- **`ESPNowClient`** (`Channel`): one instance per connected remote, keyed to that MAC and
  carrying its assigned ID. Created on the first `Connect`; **never deleted at runtime** — a
  reconnecting remote reuses its existing object via `reactivate()` (deleting a `Channel` from
  the ESP-NOW recv callback would race the main task's channel poll). TX is always unicast.
- **`ESPNowBroadcastChannel`** (`Channel`, internal to `ESPNowServer.cpp`): registered with
  `allChannels`; broadcasts status as `[FluidNC: <...>]` every `broadcast_interval_ms` ms.
  It **overrides `autoReport()`** to drop the base class's motion-state gate — the base
  `Channel::autoReport()` only emits periodic reports while the machine is *moving*
  (Cycle/Homing/Jog), so a passive display would see nothing while the machine sits idle.
  The override emits a full status report unconditionally at the configured interval.
  Enables any number of display-only devices with no connection required; disabled when
  `broadcast_interval_ms` is 0.

### Keepalive, disconnect detection, and the jog watchdog
The remote sends nothing periodically while merely connected, so FluidNC infers liveness from
remote traffic:

- **Idle disconnect detection** — `ESPNowClient::autoReport()` flags the client `_stale` and
  logs `espnow: remote (id=N) disconnected` if no packet arrives for `kIdleTimeoutMs` (4 s).
- **Ping** — FluidDial's `fnc_is_connected()` sends a `?` status request every
  `ping_interval_ms` (1500 ms) as a fallback keepalive. This must stay well below the 4 s
  idle timeout to avoid false-positive disconnects.
- **Jog watchdog** — a jog is a single `$J=` command followed by radio silence while the
  move executes (a button-held continuous jog, or a single MPG/dial click of a long
  distance), so FluidDial sends a `?` keepalive every 250 ms whenever the machine is in
  `Jog` state (`MultiJogScene`). FluidNC's `ESPNowClient::pollLine()` — armed **only during
  `State::Jog`** — injects a `JogCancel` (0x85) if that keepalive stops for >500 ms, so a
  jog can't run away after a remote drops mid-jog. The keepalive must cover MPG jogs too:
  without it, a long single-click move (e.g. 100 mm) gets cancelled mid-stroke.
  The watchdog only fires if the pendant sent a packet **after** the current jog started,
  so jogging from the WebUI or over UART is unaffected — those channels never feed
  `_last_rx_ms` and the guard condition stays false for the duration of the move.

### Connection lifecycle (FluidNC log messages)
| Event | Log |
|-------|-----|
| Server ready (broadcast enabled) | `espnow: server ready (broadcasting status every N ms)` |
| Server ready (broadcast disabled) | `espnow: server ready (broadcast disabled; waiting for remotes)` |
| New remote connects | `espnow: remote (id=N) connected` |
| Remote reconnects (object reused) | `espnow: remote (id=N) reconnected` |
| Remote times out / sends Disconnect | `espnow: remote (id=N) disconnected` |
| All slots full | `espnow: rejected remote (slots full)` |

### WiFi channel discovery and reconnect
FluidNC can be on any 2.4 GHz channel. FluidDial finds it automatically:

1. **Channel scan** — steps through channels 1–13, sending a `?` probe on each. FluidNC
   replies `[FluidNC: ?]` confirming it is alive. Result cached in NVS for fast reboots.
2. **Connection** — sends `[FluidNC: Connect]`; FluidNC creates a unicast `ESPNowClient` and
   replies `[FluidNC: Connected id=N]`. FluidDial latches FluidNC's MAC and switches to unicast.

While disconnected, `espnow_request_connect()` runs every ~6 s: it first retries cheaply on
the current channel, and — throttled to once per 8 s — re-scans every channel. This handles a
controller reboot that brings FluidNC back on a **different** WiFi channel; without the
re-scan the remote would broadcast `Connect` on a dead channel forever.

### Multiple remotes
Up to 4 FluidDial (or other) remotes can connect simultaneously with no special configuration.
Each gets its own `ESPNowClient` slot and unique ID. A `Busy` reply is sent if all slots are
occupied.
- Both devices can send GCode; the FluidNC channel system arbitrates (no motion locking).
- Status responses are broadcast to all registered channels, so all remotes stay in sync.
- Each remote's jog watchdog fires independently of the others.

### Note on WebUI coexistence
- WebUI keeps working because ESP-NOW frames piggyback on the same radio at the same channel
  as the WiFi stack — they don't fight for it.
- If the WiFi channel changes (router roam, WebUI config, controller reboot), the link drops
  briefly; FluidDial's periodic re-scan re-acquires the new channel automatically.

## Primary files changed

### FluidDial
| File | Change |
|------|--------|
| `platformio.ini` | Added `m5dial_espnow` environment; excludes `WiFiConnection.cpp` |
| `src/ESPNowTransport.h` | Declares transport API |
| `src/ESPNowTransport.cpp` | Boot UART probe, TX buffer, RX ring buffer, ESP-NOW init; channel scan (`?` probe), `[FluidNC: Connect]` handshake, unicast MAC latch + remote-ID parse on `Connected`, reconnect with channel re-scan |
| `src/SystemArduino.cpp` | Routes `fnc_putchar/getchar` through ESP-NOW when in wireless mode |
| `src/HardwareM5Dial.cpp` | Calls `espnow_transport_init()` at startup |
| `src/FluidNCModel.cpp` | Calls `espnow_request_connect()` on disconnect; `ping_interval_ms` keepalive; skips `$RI=` in ESP-NOW mode |
| `src/MultiJogScene.cpp` | Sends a `?` keepalive every 250 ms while the machine is in `Jog` state (covers button-held and MPG/dial jogs) |
| `src/Drawing.h` / `src/Drawing.cpp` | `drawESPNowIndicator()` — ESP-NOW status icon |
| `src/StatusScene.cpp` / `src/PieMenu.cpp` | Call `drawESPNowIndicator()` on round display |
| `src/BrightnessScene.cpp` | Falls back to `statusScene` when `USE_WIFI` not defined |

### FluidNC
| File | Change |
|------|--------|
| `FluidNC/esp32/ESPNowServer.h/.cpp` | Radio init, `[FluidNC: ...]` frame parsing, multi-remote management with unique IDs; `ESPNowBroadcastChannel` with an `autoReport()` override for state-independent broadcasts; `broadcast_interval_ms` config item |
| `FluidNC/src/ESPNowClient.h/.cpp` | Channel: RX ring, unicast TX, idle-timeout disconnect detection, jog watchdog in `pollLine()`, `reactivate()` for in-place reconnect |
| `FluidNC/src/Machine/MachineConfig.h` | Added `ESPNowServer* _espnow_server` member |
| `FluidNC/src/Machine/MachineConfig.cpp` | Added `handler.section("espnow_channel", ...)` (YAML key unchanged) |
| `FluidNC/src/Main.cpp` | Calls `_espnow_server->init()` after Modules loop |

---

## Time Sequence Diagrams

Solid arrows = **unicast** (802.11 ACK/retry); dashed arrows = **broadcast** (fire-and-forget).

```mermaid
sequenceDiagram
    participant P as FluidDial Pendant
    participant C as FluidNC Controller

    Note over P,C: ── Boot: channel discovery (broadcast) ──
    P->>P: espnow_transport_init()<br/>UART probe times out → ESP-NOW mode
    loop scan_for_channel(): WiFi ch 1..13 until a reply
        P-->>C: "?" probe (broadcast)
        C-->>P: [FluidNC: ?] (broadcast, no client yet)
    end
    Note over P: channel locked, cached in NVS

    Note over P,C: ── Pairing handshake (broadcast → unicast) ──
    P-->>C: [FluidNC: Connect] (broadcast)
    C->>C: createClient(mac) → assign id=1<br/>add pendant MAC as unicast peer
    C->>P: [FluidNC: Connected id=1] (unicast)
    Note over C: log: espnow: remote (id=1) connected
    P->>P: latch _tx_mac = controller MAC<br/>add unicast peer, _pendant_id = 1

    Note over P,C: ── Steady state (all unicast) ──
    loop while connected
        C->>P: <Idle|MPos:…> status (unicast, every report_interval_ms)
        P->>C: GCode / jog / "?" keepalive (unicast)
    end

    Note over P,C: ── Pendant lost (power off / out of range) ──
    P-xC: (silence — no more packets)
    C->>C: ESPNowClient::autoReport()<br/>no RX for kIdleTimeoutMs (4 s) → _stale = true
    Note over C: log: espnow: remote (id=1) disconnected

    Note over P,C: ── Reconnect (pendant back, or controller rebooted) ──
    P->>P: espnow_request_connect()<br/>_tx_mac → broadcast
    opt controller rebooted on a different WiFi channel
        P-->>C: re-scan channels ("?" probe)
        C-->>P: [FluidNC: ?] (broadcast)
        Note over P: re-lock channel, update NVS
    end
    P-->>C: [FluidNC: Connect] (broadcast)
    C->>C: findClient(mac) exists → reactivate()<br/>reuse object in place, keep id=1
    C->>P: [FluidNC: Connected id=1] (unicast)
    Note over C: log: espnow: remote (id=1) reconnected
    P->>P: re-latch _tx_mac → unicast resumes
```
