"""ClearCore Modbus register map.

Existing vocabulary is reproduced verbatim from cell-config.yaml — the old
Polyscope program must stay loadable as rollback (CLAUDE.md, Constraints), so
no existing address may move. New registers are strictly additive.

TWO WARNINGS
------------
1.  **Naming is inverted between the two sides.** The robot's `*_LOCAL`
    (100-108) is the firmware's `*_REMOTE`, and the robot's `*_REM` (200-208) is
    the firmware's `*_LOCAL`. The names below follow the ROBOT's convention,
    matching cell-config.yaml. The 200-block is an echo the master polls to
    confirm a command landed before moving.

2.  **Modbus TCP slaves accept multiple masters.** Nothing in the protocol stops
    the old Polyscope program and this orchestrator from writing these registers
    at the same time, and neither would complain. The handoff needs a
    procedural or firmware interlock — see MASTER_HANDOFF below.
"""

from __future__ import annotations

from enum import IntEnum

CLEARCORE_HOST = "192.168.1.18"
CLEARCORE_UNIT_ID = 255


class Status(IntEnum):
    """ClearCore -> master. Input registers / discrete inputs."""

    SERVER_STATE = 1   # 0 = not ready, 1 = ready, 2 = moving
    JOB = 2
    RUN = 3
    WORK_AT_ZERO = 4
    OFFLOAD = 5
    ONLOAD = 6


class Command(IntEnum):
    """master -> ClearCore. Holding registers / coils. (firmware: *_REMOTE)"""

    MOTION_MODE = 100
    DIRECTION = 101
    VELOCITY = 102
    ACCELERATION = 103
    DISTANCE = 104
    POSITION = 105
    REQUEST_ID = 106
    FEED_CONVEYOR = 107
    BRUSH_ON = 108


class Echo(IntEnum):
    """ClearCore -> master. The handshake: poll until echo == command.

    A distance move is only recognised when REQUEST_ID changes (firmware
    ino:305), so every move needs a fresh id.
    """

    MOTION_MODE = 200
    DIRECTION = 201
    VELOCITY = 202
    ACCELERATION = 203
    DISTANCE = 204
    POSITION = 205
    REQUEST_ID = 206
    FEED_CONVEYOR = 207
    BRUSH_ON = 208


class New(IntEnum):
    """Additive registers for the rewrite. ADDRESSES ARE PROPOSED, NOT ASSIGNED.

    TODO: confirm against firmware before wiring. Chosen to sit clear of the
    existing 1-6 / 100-108 / 200-208 blocks with room to grow.

    Zone motors: the old firmware drives M0 (main conveyor), M1 (feed), M2
    (brush). The rewrite needs ZONE1 (INQ<->IF) + ZONE2 (S<->FD<->OUT) + 2 fans
    + shutter. M3 is free, but which physical motor becomes which zone is an
    OPEN hardware mapping question.

    The IF<->S handoff sensors (407/408) confirm the sensor-terminated crossing
    between the two belts — see process/train.py for the manoeuvre.

    sim/fake_clearcore.py implements this map's behaviour (echo handshake,
    zone move lifecycle, shutter actuation, watchdog) and is the executable
    reference for the real firmware changes.
    """

    # Shutter
    SHUTTER_CMD = 300        # 0 = closed, 1 = open
    SHUTTER_FEEDBACK = 400   # 0 = closed, 1 = open, 2 = moving/unknown

    # Fans. Fail ON: firmware forces these on if the heartbeat stops.
    IF_FAN_CMD = 301
    FD_FAN_CMD = 302
    IF_FAN_FEEDBACK = 401
    FD_FAN_FEEDBACK = 402

    # Presence sensors at the tracked stations
    IF_PRESENT = 403
    S_PRESENT = 404
    FD_PRESENT = 405
    INQ_COUNT = 406

    # IF<->S handoff confirmation — the crossing is sensor-terminated (both
    # belts run together until the part is confirmed on the receiving belt).
    # One per direction.
    HANDOFF_TO_Z2 = 407   # part has fully entered zone 2 (downstream crossing)
    HANDOFF_TO_Z1 = 408   # part has fully entered zone 1 (the P2->P3 retreat)

    # Zone motion. Distance is UNSIGNED steps; sign travels on the direction
    # coil, exactly like the legacy conveyor block.
    ZONE1_MOTION_MODE = 310
    ZONE1_DISTANCE = 311
    ZONE1_REQUEST_ID = 312
    ZONE1_DIRECTION = 313    # coil: 1 = downstream
    #: Sensor-stop target for MODE_SENSOR_STOP (see SensorTarget below).
    ZONE1_TARGET = 314
    ZONE2_MOTION_MODE = 320
    ZONE2_DISTANCE = 321
    ZONE2_REQUEST_ID = 322
    ZONE2_DIRECTION = 323    # coil: 1 = downstream
    ZONE2_TARGET = 324
    ZONE1_STATE = 410
    ZONE2_STATE = 411
    # Move-acceptance ack: firmware mirrors the REQUEST_ID it last RECOGNISED.
    # A Modbus write only confirms the register changed, not that the firmware
    # loop acted on it — so "state == READY" right after a write is ambiguous
    # (done, or not yet noticed?). The client polls ack == id before trusting
    # state transitions. Same job the legacy 200-block echo did.
    ZONE1_REQID_ACK = 413
    ZONE2_REQID_ACK = 414
    #: 1 while the firmware watchdog has tripped (heartbeat stale): zones
    #: halted, fans forced ON. Clears when the heartbeat advances again.
    WATCHDOG_TRIPPED = 412

    # Watchdog. Orchestrator increments; firmware halts zones and forces fans ON
    # if it stops advancing for watchdog.clearcore_timeout_s. ARMS ON THE FIRST
    # HEARTBEAT, not at power-on — a powered-but-idle line with no orchestrator
    # yet must not sit with fans forced on and zones locked. Once armed, never
    # disarms; a resumed heartbeat clears a trip.
    HEARTBEAT = 330


class SensorTarget(IntEnum):
    """Encoding for the ZONE*_TARGET registers (MODE_SENSOR_STOP = 4).

    Low bits pick the sensor; adding FALLING (8) stops on the falling edge
    instead of the rising one. EDGES, not levels: the firmware records the
    sensor's state when the move is recognised and stops on the first
    TRANSITION to the target polarity. Level-triggered stops are wrong for
    every vacate-then-fill sequence — the destination sensor is still held by
    the departing part when the move starts, so a level check trips instantly.
    The firmware watches its own inputs at loop rate (~10 ms), which is the
    whole point: the stop is a reflex, with no Modbus round-trip (~20-50 ms of
    variance at the orchestrator) in the positioning chain. Positioning truth
    on this line is sensors + open-loop step counting — there are no encoders
    and no closed-loop motors — so the sensor edge deserves the tightest stop
    we can give it.
    """

    IF_PRESENT = 1
    S_PRESENT = 2
    FD_PRESENT = 3
    HANDOFF_TO_Z1 = 4
    HANDOFF_TO_Z2 = 5
    #: Flag: add to a sensor code to stop on its FALLING edge.
    FALLING = 8


#: Zone motion modes. 0-3 are the legacy vocabulary (cell-config enums);
#: 4 is new: run in DIRECTION until the ZONE*_TARGET sensor edge, stop in
#: firmware, report READY. Uses the REQUEST_ID/ack lifecycle like distance.
MODE_DISTANCE = 0
MODE_POSITION = 1
MODE_CONTINUOUS = 2
MODE_IDLE = 3
MODE_SENSOR_STOP = 4


MASTER_HANDOFF = """\
Rollback procedure. The ClearCore is a Modbus SLAVE in both the old and new
designs — only the master changes (robot -> PC). Modbus TCP does not arbitrate
masters, so both can write concurrently and silently corrupt each other.

To run the old Polyscope program:
  1. Stop the orchestrator service. Confirm its Modbus session is closed.
  2. Confirm HEARTBEAT has stopped advancing; the firmware halts zones.
  3. Load and play the old program on the pendant.

To return to the orchestrator:
  1. Stop the Polyscope program and confirm the robot is at a safe pose.
  2. Start the orchestrator; it performs an occupancy scan before scheduling.

TODO: enforce this in firmware rather than by procedure — e.g. a single-master
lease on HEARTBEAT that rejects command writes from any other source.
"""
