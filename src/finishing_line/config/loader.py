"""Typed access to the two config files.

`cell-config.yaml` holds constants extracted from the old line and proven by
200 parts/week. `line-config.yaml` holds new process parameters, most of which
are assumed rather than measured.

The `provenance_*` keys are not decoration. `ProcessConfig.unmeasured()` exists
so the HMI and commissioning checklist can show, at a glance, which numbers the
line is currently running on faith.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

REPO_ROOT = Path(__file__).resolve().parents[3]
CELL_CONFIG = REPO_ROOT / "cell-config.yaml"
LINE_CONFIG = REPO_ROOT / "line-config.yaml"


@dataclass(frozen=True, slots=True)
class ProcessConfig:
    """Process parameters governing the schedule."""

    flash_seconds: float
    coats: int
    spray_burst_pause_s: float
    transfer_s: float
    robot_coat1_s: float
    robot_coat2_s: float
    clean_gun_enabled: bool
    clean_gun_duration_s: float
    provenance: dict[str, str]

    def nominal_period_s(self) -> float:
        """Predicted period for one pair (2 parts).

        Three nominal beats plus P3, which stretches by the burst pause because
        flash timers bank fan-on seconds only. This is a prediction for the
        simulator and the HMI — the controller never enforces it.
        """
        beat = self.flash_seconds + self.transfer_s
        return beat * 3 + (beat + self.spray_burst_pause_s)

    def nominal_seconds_per_part(self) -> float:
        return self.nominal_period_s() / 2

    def unmeasured(self) -> tuple[str, ...]:
        """Parameters not yet measured on the real line.

        Anything listed here is a number the line is running on faith.
        """
        return tuple(k for k, v in self.provenance.items() if v != "measured")


@dataclass(frozen=True, slots=True)
class ProductSpec:
    name: str
    legacy_job_id: int
    width_mm: int
    height_mm: int
    depth_mm: int


@dataclass(frozen=True, slots=True)
class SandConfig:
    """Tuned sanding constants from the old line (cell-config.yaml).

    These are the values that produce the accepted finish. The rewrite reuses
    them rather than re-deriving them (CLAUDE.md, build order step 1).
    """

    z_force_n: float
    width_inset_mm: int
    movel_a: float
    movel_v: float
    contact_search_distance_m: float
    stopl_on_contact: float
    stopl_on_force_end: float
    ft_wait_steady_ms: int


def _load(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def load_process_config(path: Path | None = None) -> ProcessConfig:
    raw = _load(path or LINE_CONFIG)["process"]
    nominal = raw["nominal"]
    return ProcessConfig(
        flash_seconds=float(raw["flash_seconds"]),
        coats=int(raw["coats"]),
        spray_burst_pause_s=float(raw["spray_burst_pause_s"]),
        transfer_s=float(nominal["transfer_s"]),
        robot_coat1_s=float(nominal["robot_coat1_s"]),
        robot_coat2_s=float(nominal["robot_coat2_s"]),
        clean_gun_enabled=bool(raw["clean_gun"]["enabled"]),
        clean_gun_duration_s=float(raw["clean_gun"]["duration_s"]),
        provenance={
            "flash_seconds": raw["provenance_flash"],
            "spray_burst_pause_s": raw["provenance_burst"],
            "transfer_s": nominal["provenance_transfer"],
            "clean_gun": raw["clean_gun"]["provenance"],
        },
    )


def load_products(path: Path | None = None) -> dict[str, ProductSpec]:
    raw = _load(path or LINE_CONFIG)["products"]
    return {
        name: ProductSpec(
            name=name,
            legacy_job_id=spec["legacy_job_id"],
            width_mm=spec["width_mm"],
            height_mm=spec["height_mm"],
            depth_mm=spec["depth_mm"],
        )
        for name, spec in raw.items()
    }


@dataclass(frozen=True, slots=True)
class ConveyorKinematics:
    """The tuned mm -> steps conversion (cell-config conveyor.kinematics).

    Computed from the drive mechanics plus the 400/381 empirical calibration —
    ~29.9962 microsteps/mm. The legacy program floors the result; keep that,
    bit-for-bit, so distances land where the old line put them.
    """

    microsteps_per_mm: float
    velocity_steps_per_sec: int
    acceleration_steps_per_sec2: int

    def mm_to_steps(self, mm: float) -> int:
        import math

        return math.floor(abs(mm) * self.microsteps_per_mm)


def load_conveyor_kinematics(path: Path | None = None) -> ConveyorKinematics:
    raw = _load(path or CELL_CONFIG)["conveyor"]
    k = raw["kinematics"]
    per_mm = (
        360.0
        / k["pulley_teeth"]
        / k["degrees_per_step"]
        / k["belt_pitch_mm"]
        * k["microsteps"]
        * k["calibration_adjustment"]
    )
    return ConveyorKinematics(
        microsteps_per_mm=per_mm,
        velocity_steps_per_sec=int(raw["velocity_steps_per_sec"]),
        acceleration_steps_per_sec2=int(raw["acceleration_steps_per_sec2"]),
    )


@dataclass(frozen=True, slots=True)
class Waypoint:
    """A tuned pose: p is the TCP target, q the joint seed for inverse kin.

    The legacy program's movej was get_inverse_kin(p, qnear=q) — p wins, q only
    steers the solver away from alternate elbow solutions. Preserve that.
    """

    p: tuple[float, ...]
    q: tuple[float, ...]


@dataclass(frozen=True, slots=True)
class RobotSetup:
    """TCP, payload, and default motion params from cell-config.

    `tcp` is the sanding/default tool frame; `spray_tcp` is the distinct frame
    the legacy program set before every spray waypoint (script:2859). The spray
    waypoints only solve correctly under it, so URClient switches between them.
    """

    tcp: tuple[float, ...]
    spray_tcp: tuple[float, ...]
    payload_mass_kg: float
    payload_cog_m: tuple[float, ...]
    movej_a: float
    movej_v: float
    waypoints: dict[str, Waypoint]


def load_robot_setup(path: Path | None = None) -> RobotSetup:
    raw = _load(path or CELL_CONFIG)["robot"]
    motion = _load(path or CELL_CONFIG)["motion"]
    return RobotSetup(
        tcp=tuple(raw["tcp"]),
        spray_tcp=tuple(raw["spray_tcp"]),
        payload_mass_kg=float(raw["payload"]["mass_kg"]),
        payload_cog_m=tuple(raw["payload"]["cog_m"]),
        movej_a=float(motion["movej_default"]["a"]),
        movej_v=float(motion["movej_default"]["v"]),
        waypoints={
            name: Waypoint(p=tuple(w["p"]), q=tuple(w["q"]))
            for name, w in raw["waypoints"].items()
        },
    )


@dataclass(frozen=True, slots=True)
class SprayConfig:
    """Tuned spray constants (cell-config). Spraying is NON-CONTACT (no force
    mode); the gun is DO5. Distances mirror sanding (width - inset, height).
    """

    width_inset_mm: int
    approach_z_m: float   # browser vertical: base-Z+ standoff before the raster
    approach_a: float
    approach_v: float
    height_a: float       # moveHeight passes (movel_process)
    height_v: float


def load_spray_config(path: Path | None = None) -> SprayConfig:
    raw = _load(path or CELL_CONFIG)
    moves, motion = raw["moves"], raw["motion"]
    return SprayConfig(
        width_inset_mm=int(moves["width_inset_mm"]),
        approach_z_m=float(moves["spray_vertical"]["approach_z_m_job2"]),
        approach_a=float(motion["movel_spray_approach"]["a"]),
        approach_v=float(motion["movel_spray_approach"]["v"]),
        height_a=float(motion["movel_process"]["a"]),
        height_v=float(motion["movel_process"]["v"]),
    )


@dataclass(frozen=True, slots=True)
class BrushConfig:
    """Tuned gun-clean constants (cell-config). The HVLP tip contacts the
    rotating brush (legacy coil 108) to stay clean before a coat — NOT a product
    denib. Non-force: contact-detect, back off, hold while the brush spins.
    """

    contact_v: float          # base-Z+ contact search speed (movel_process)
    retract_off_mm: float     # back off the hard-contact point before the brush
    retract_a: float          # movel_retract_brush
    retract_v: float
    settle_before_on_s: float
    duration_s: float         # brush run time (legacy Thread_2 kill at 30 s)
    settle_after_off_s: float


@dataclass(frozen=True, slots=True)
class FanMode:
    """Legacy-mod fan control mode (line-config legacy_mode.fans).

    kind: 'always_on' (hardwired — flash banks wall-clock), 'none' (not
    mounted — wall-clock + loud warning), or 'robot_do' (relay on a robot
    digital output — full schedule semantics incl. the P3 burst pause).
    """

    kind: str
    do: int | None = None

    @property
    def controllable(self) -> bool:
        return self.kind == "robot_do"


def load_legacy_fans(path: Path | None = None) -> dict[str, FanMode]:
    raw = _load(path or LINE_CONFIG)["legacy_mode"]["fans"]
    fans: dict[str, FanMode] = {}
    for name, spec in raw.items():
        if isinstance(spec, dict):
            fans[name] = FanMode(kind="robot_do", do=int(spec["robot_do"]))
        elif spec in ("always_on", "none"):
            fans[name] = FanMode(kind=spec)
        else:
            raise ValueError(f"unknown fan mode for {name}: {spec!r}")
    return fans


def load_brush_config(path: Path | None = None) -> BrushConfig:
    raw = _load(path or CELL_CONFIG)
    moves, motion, timings = raw["moves"], raw["motion"], raw["timings"]
    return BrushConfig(
        contact_v=float(motion["movel_process"]["v"]),
        retract_off_mm=float(moves["brush_clean"]["retract_off_mm"]),
        retract_a=float(motion["movel_retract_brush"]["a"]),
        retract_v=float(motion["movel_retract_brush"]["v"]),
        settle_before_on_s=float(timings["brush_settle_before_on_s"]),
        duration_s=float(timings["brush_duration_s"]),
        settle_after_off_s=float(timings["brush_settle_after_off_s"]),
    )


def load_sand_config(path: Path | None = None) -> SandConfig:
    """Pull the tuned sanding constants out of the archaeology file."""
    raw = _load(path or CELL_CONFIG)
    return SandConfig(
        z_force_n=float(raw["force"]["z_force_newtons"]["main_sand"]),
        width_inset_mm=int(raw["moves"]["width_inset_mm"]),
        movel_a=float(raw["motion"]["movel_process"]["a"]),
        movel_v=float(raw["motion"]["movel_process"]["v"]),
        contact_search_distance_m=float(raw["motion"]["tool_contact"]["search_distance_m"]),
        stopl_on_contact=float(raw["motion"]["stopl"]["on_tool_contact"]),
        stopl_on_force_end=float(raw["motion"]["stopl"]["on_force_mode_end"]),
        ft_wait_steady_ms=int(raw["force"]["ft_sensor"]["wait_steady_ms"]),
    )
