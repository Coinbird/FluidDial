// Copyright (c) 2024 FluidDial contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#ifdef USE_ESPNOW

#include <stdint.h>

// Called from init_hardware() after init_fnc_uart().
// Waits up to 1500 ms for data on the UART. If data arrives the pendant
// is wired and UART mode is kept. If nothing arrives, ESP-NOW is initialised
// and the pendant operates wirelessly.
void espnow_transport_init();

// Returns true when a wired UART connection was detected at boot.
bool espnow_use_uart_mode();

// Transport primitives used by fnc_putchar / fnc_getchar routing in SystemArduino.cpp.
void espnow_putchar(uint8_t c);
int  espnow_getchar();  // returns -1 if nothing available

// Send [FluidNC: Connect] broadcast and reset MAC state so we re-latch on the
// response. Non-blocking — on_recv handles the [FluidNC: Connected id=N] reply.
// Call at boot (done inside init_radio) and whenever disconnect is detected.
void espnow_request_connect();

// Returns the pendant ID assigned by FluidNC (0 = not yet connected).
int espnow_pendant_id();

#endif  // USE_ESPNOW
