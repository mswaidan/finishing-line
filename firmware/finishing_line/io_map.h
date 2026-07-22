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
//
// .19, NOT .18: the legacy production unit still lives at 192.168.1.18 and
// runs during the transition, so the rewrite unit takes .19 to avoid an
// address collision on the shared LAN. The orchestrator/monitor default and
// registers.py CLEARCORE_HOST track this value.
#define CC_IP_ADDRESS 192, 168, 1, 19
#define CC_NETMASK 255, 255, 255, 0
#define CC_GATEWAY 192, 168, 1, 1

// ----------------------------------------------------------------- motors
// Step/dir open-loop stepper drivers on all axes (NOT ClearPath — confirmed
// 2026-07-17).
#define Z1_BELT ConnectorM0  // TODO(wiring): IN<->F1 belt
#define IN_BELT ConnectorM1   // TODO(wiring): IN queue belt (legacy M1)
#define Z2_BELT ConnectorM2  // TODO(wiring): O<->F2<->OUT belt
#define O_BRUSH ConnectorM3  // TODO(wiring): brush, legacy coil 108

// Legacy tuned brush values (cell-config conveyor.brush_motor).
#define BRUSH_VELOCITY_SPS 10000
#define BRUSH_ACCEL_SPS2 100000

// Step polarity, carried from the legacy firmware (ino:196-207).
#define INVERT_STEP_ENABLE true
#define INVERT_STEP_DIRECTION false

// ---------------------------------------------------------------- inputs
// All presence eyes NPN, debounced in firmware — edges stop belts.
//
// Polarity: the F18 diffuse eyes read ACTIVE-LOW as wired (product present pulls
// the input low), so the firmware inverts every presence/handoff eye to make the
// discrete register mean "present". Set false if the eyes are taught light-on.
// The shutter end-switches are separate mechanical inputs and are NOT inverted.
#define EYES_ACTIVE_LOW true

#define PIN_F1_EYE ConnectorIO0        // TODO(wiring)
#define PIN_O_EYE ConnectorIO1         // TODO(wiring)
#define PIN_F2_EYE ConnectorDI6        // TODO(wiring)
#define PIN_SH_OPEN_EYE ConnectorDI7   // TODO(wiring): end switch
#define PIN_SH_CLOSED_EYE ConnectorDI8 // TODO(wiring): end switch
#define PIN_Z2_EYE ConnectorA9      // TODO(wiring): part fully on Z2
#define PIN_Z1_EYE ConnectorA10     // TODO(wiring): part fully on Z1
#define PIN_IN_EYE ConnectorA11       // TODO(wiring): queue-head eye
#define PIN_OUT_EYE ConnectorA12       // TODO(wiring): outfeed occupancy eye

// ---------------------------------------------------------------- outputs
// IO0-IO5 are the only output-capable pins; four are spoken for here.
#define PIN_F1_FAN ConnectorIO2            // TODO(wiring): relay/contactor coil
#define PIN_F2_FAN ConnectorIO3            // TODO(wiring): relay/contactor coil
// Double-solenoid 5/2 detented valve: holds last position de-energised
// (§7: shutter holds state on fault). Commanded side energised continuously.
#define PIN_SH_OPEN_SOL ConnectorIO4  // TODO(wiring)
#define PIN_SH_CLOSE_SOL ConnectorIO5 // TODO(wiring)

// ----------------------------------------------------------------- tuning
// Debounce: a level must hold this long before the firmware believes it.
// ~0.05 mm of stop-chain error per ms at 53 mm/s; 10 ms costs ~0.5 mm.
#define DEBOUNCE_MS 10

// Watchdog: must comfortably exceed the orchestrator's ~5 Hz heartbeat.
#define WATCHDOG_TIMEOUT_MS 2000

// Defaults until the orchestrator writes velocity/accel (legacy tuned).
#define DEFAULT_VELOCITY_SPS 1600
#define DEFAULT_ACCEL_SPS2 16000
