"""FastAPI + websocket layer — the HMI's only interface.

The HMI never talks to the robot or the ClearCore directly (CLAUDE.md,
Architecture). Everything goes through the orchestrator.

TODO(step 5): implement once the state machine is commissioned.

Intended surface:

    GET  /state            -> LineState + per-part timers + blocked_by
    POST /batch            -> operator declares parts at INQ (identity enters
                              the system here; sensors only ever report counts)
    POST /run  /stop
    POST /fault/ack        -> operator confirms reconstructed occupancy after a
                              sensor mismatch (§7 recovery)
    WS   /events           -> state deltas, ~2 Hz

`blocked_by` is not a nicety. A line sitting still with no stated reason is how
operators learn to bypass interlocks — `StepResult.blocked_by` carries the
reason from the guard that stopped it and should be on the HMI's main view.
"""

from __future__ import annotations
