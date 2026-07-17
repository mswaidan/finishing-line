// Physical I/O assignments — implements docs/hardware.md exactly.
//
// Decided 2026-07-17 (all 13 I/O points used; outputs may only live on
// IO0-IO5). TODO(wiring) marks assignments still to be confirmed against the
// physical panel at bench bring-up.

#pragma once

#include "ClearCore.h"

// ---------------------------------------------------------------- network
// STATIC IP, deliberately. The legacy setup ran DHCP and two different
// clients ended up addressing the ClearCore at two different IPs (.17 vs
// .18, see cell-config.yaml discrepancies). The orchestrator config must
// match this address.
#define CC_IP_ADDRESS 192, 168, 1, 18
#define CC_NETMASK 255, 255, 255, 0
#define CC_GATEWAY 192, 168, 1, 1

// ----------------------------------------------------------------- motors
// Step/dir open-loop stepper drivers on all axes (NOT ClearPath — confirmed
// 2026-07-17).
#define MOTOR_ZONE1 ConnectorM0  // TODO(wiring): INQ<->IF belt
#define MOTOR_FEED ConnectorM1   // TODO(wiring): INQ queue belt (legacy M1)
#define MOTOR_ZONE2 ConnectorM2  // TODO(wiring): S<->FD<->OUT belt
#define MOTOR_BRUSH ConnectorM3  // TODO(wiring): brush, legacy coil 108

// Legacy tuned brush values (cell-config conveyor.brush_motor).
#define BRUSH_VELOCITY_SPS 10000
#define BRUSH_ACCEL_SPS2 100000

// Step polarity, carried from the legacy firmware (ino:196-207).
#define INVERT_STEP_ENABLE true
#define INVERT_STEP_DIRECTION false

// ---------------------------------------------------------------- inputs
// All presence eyes NPN, debounced in firmware — edges stop belts.
#define PIN_IF_PRESENT ConnectorIO0        // TODO(wiring)
#define PIN_S_PRESENT ConnectorIO1         // TODO(wiring)
#define PIN_FD_PRESENT ConnectorDI6        // TODO(wiring)
#define PIN_SHUTTER_OPEN_SW ConnectorDI7   // TODO(wiring): end switch
#define PIN_SHUTTER_CLOSED_SW ConnectorDI8 // TODO(wiring): end switch
#define PIN_HANDOFF_TO_Z2 ConnectorA9      // TODO(wiring): part fully on zone 2
#define PIN_HANDOFF_TO_Z1 ConnectorA10     // TODO(wiring): part fully on zone 1
#define PIN_INQ_PRESENT ConnectorA11       // TODO(wiring): queue-head eye
#define PIN_OUT_PRESENT ConnectorA12       // TODO(wiring): outfeed occupancy eye

// ---------------------------------------------------------------- outputs
// IO0-IO5 are the only output-capable pins; four are spoken for here.
#define PIN_IF_FAN ConnectorIO2            // TODO(wiring): relay/contactor coil
#define PIN_FD_FAN ConnectorIO3            // TODO(wiring): relay/contactor coil
// Double-solenoid 5/2 detented valve: holds last position de-energised
// (§7: shutter holds state on fault). Commanded side energised continuously.
#define PIN_SHUTTER_OPEN_SOL ConnectorIO4  // TODO(wiring)
#define PIN_SHUTTER_CLOSE_SOL ConnectorIO5 // TODO(wiring)

// ----------------------------------------------------------------- tuning
// Debounce: a level must hold this long before the firmware believes it.
// ~0.05 mm of stop-chain error per ms at 53 mm/s; 10 ms costs ~0.5 mm.
#define DEBOUNCE_MS 10

// Watchdog: must comfortably exceed the orchestrator's ~5 Hz heartbeat.
#define WATCHDOG_TIMEOUT_MS 2000

// Defaults until the orchestrator writes velocity/accel (legacy tuned).
#define DEFAULT_VELOCITY_SPS 1600
#define DEFAULT_ACCEL_SPS2 16000
