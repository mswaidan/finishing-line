"""Flash-time accounting.

The single rule this module exists to enforce: **a flash timer advances only
while the part is at a fan station AND that fan is running.**

Wall-clock at a fan position is not flash time. This is the decision that makes
the §6 guarantee ("parts may over-flash safely, never under-flash") literally
true rather than approximately true, and it is why P3 stretches when the F1 fan
pauses for a spray burst. See the P3 note in schedule.py.
"""

from __future__ import annotations

from ..config.loader import ProcessConfig
from .model import FAN_STATIONS, FanState, LineState, PartState


def advance_flash_timers(state: LineState, dt: float, cfg: ProcessConfig) -> dict[str, PartState]:
    """Return `state.parts` with flash timers advanced by `dt` seconds.

    Only parts standing at a fan station whose fan is ON accumulate time. A part
    at F1 while the F1 fan is paused mid-spray-burst banks nothing — that is the
    whole point.

    Wetness clears HERE, the moment the active flash completes — not when the
    part later moves. Wet means "coated and flash incomplete"; tying it to
    movement left fully-flashed parts tagged wet until the next transition,
    which both misled the HMI and kept the §7 F1-fan pause armed for a part
    that was already dry.
    """
    from dataclasses import replace

    parts = dict(state.parts)
    for station in FAN_STATIONS:
        if state.fan_state(station) is not FanState.ON:
            continue
        part = state.part_at(station)
        if part is None:
            continue
        # An uncoated part at F1 is STAGED, not flashing (§2: "F1 doubles as a
        # staging slot when the fan is off"). Banking time for it would let it
        # skip its real flash 1 later, having already 'served' 180 s dry.
        if part.coats_applied == 0:
            continue
        advanced = part.with_flash_advanced(dt)
        if advanced.is_wet and flash_complete(advanced, cfg):
            advanced = replace(advanced, is_wet=False)
        parts[part.part_id] = advanced
    return parts


def flash_complete(part: PartState, cfg: ProcessConfig) -> bool:
    """Has the part banked its full flash on whichever coat it is waiting out?"""
    return part.active_flash_seconds() >= cfg.flash_seconds


def may_receive_coat_2(part: PartState, cfg: ProcessConfig) -> bool:
    """§6 guard: coat 2 requires a complete flash 1."""
    return part.coats_applied == 1 and part.flash_1_s >= cfg.flash_seconds


def may_outfeed(part: PartState, cfg: ProcessConfig) -> bool:
    """§6 guard: outfeed requires both coats and a complete flash 2."""
    return part.coats_applied >= cfg.coats and part.flash_2_s >= cfg.flash_seconds


def may_leave_fan(part: PartState, cfg: ProcessConfig) -> bool:
    """§6 guard: a part may leave a fan position only once its active flash is done.

    An uncoated part is merely staged and is free to move — F1 is a staging slot
    as well as a flash position (§2), and only a part carrying wet finish owes
    the fan any time.

    For everything else this is deliberately unconditional: no 'close enough'
    tolerance, because tolerance here means shipping a part with a soft finish.
    This is the guard that stretches P3.
    """
    if part.coats_applied == 0:
        return True
    return flash_complete(part, cfg)
