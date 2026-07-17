// Physical I/O assignments — THE ONE FILE COMMISSIONING EDITS.
//
// Every assignment marked TODO(wiring) is a PLACEHOLDER: plausible, but not
// confirmed against the actual panel. Confirm each one during the bench
// bring-up and delete the TODO as you go. Everything else in the firmware is
// wiring-independent.

#pragma once

#include "ClearCore.h"

// ---------------------------------------------------------------- network
// STATIC IP, deliberately. The legacy setup ran DHCP and two different
// clients ended up addressing the ClearCore at two different IPs (.17 vs
// .18, see cell-config.yaml discrepancies) — at most one of them worked.
// The orchestrator config (cell-config network.clearcore.modbus_host) must
// match this address.
#define CC_IP_ADDRESS 192, 168, 1, 18
#define CC_NETMASK 255, 255, 255, 0
#define CC_GATEWAY 192, 168, 1, 1
#define MODBUS_TCP_PORT 502

// ----------------------------------------------------------------- motors
// Step/dir open-loop stepper drivers on all axes (NOT ClearPath — confirmed
// 2026-07-17). Legacy mapping was M0 = old main conveyor, M1 = feed, M2 =
// brush. The rewrite needs two zone belts + the feed.
#define MOTOR_ZONE1 ConnectorM0  // TODO(wiring): INQ<->IF belt
#define MOTOR_ZONE2 ConnectorM2  // TODO(wiring): S<->FD<->OUT belt
#define MOTOR_FEED ConnectorM1   // TODO(wiring): INQ queue belt (legacy M1)

// Step polarity, carried from the legacy firmware (ino:196-207).
#define INVERT_STEP_ENABLE true
#define INVERT_STEP_DIRECTION false

// ---------------------------------------------------------------- sensors
// All NPN presence sensors, debounced in firmware (DEBOUNCE_MS below) — the
// legacy line had no debounce anywhere, and sensor edges now STOP BELTS.
#define PIN_IF_PRESENT ConnectorIO0   // TODO(wiring)
#define PIN_S_PRESENT ConnectorIO1    // TODO(wiring)
#define PIN_FD_PRESENT ConnectorIO2   // TODO(wiring)
#define PIN_HANDOFF_TO_Z2 ConnectorIO3  // TODO(wiring): part fully on zone 2
#define PIN_HANDOFF_TO_Z1 ConnectorIO4  // TODO(wiring): part fully on zone 1
#define PIN_SHUTTER_OPEN_SW ConnectorDI6   // TODO(wiring): reed, open end
#define PIN_SHUTTER_CLOSED_SW ConnectorDI7 // TODO(wiring): reed, closed end

// ---------------------------------------------------------------- outputs
#define PIN_IF_FAN ConnectorIO5       // TODO(wiring): relay, fail-ON logic in firmware
#define PIN_FD_FAN ConnectorA9        // TODO(wiring): relay
#define PIN_SHUTTER_SOLENOID ConnectorA10  // TODO(wiring): energise = open (confirm!)

// ----------------------------------------------------------------- tuning
// Debounce: a level must hold this long before the firmware believes it.
// Budget: at 1600 steps/s (~53 mm/s) each ms of debounce adds ~0.05 mm to
// the stop chain — 10 ms costs ~0.5 mm, well inside the 12 mm inset budget.
#define DEBOUNCE_MS 10

// Watchdog: orchestrator heartbeats at ~5 Hz (heartbeat register 330); this
// must comfortably exceed the heartbeat period. line-config watchdog section.
#define WATCHDOG_TIMEOUT_MS 2000

// Defaults if the orchestrator has not written velocity/accel yet
// (cell-config conveyor: the tuned legacy values).
#define DEFAULT_VELOCITY_SPS 1600
#define DEFAULT_ACCEL_SPS2 16000
