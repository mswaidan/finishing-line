"""Runnable orchestrator: `python -m finishing_line.api [--sim | --cc HOST | --ur HOST]`

--sim       Full simulation: fake ClearCore + physics + FakeRobot, compressed
            process times. The whole line runs visibly in a browser with zero
            hardware — the Stage C demo.
--cc HOST   Conveyor commissioning: REAL ClearCore at HOST, FakeRobot standing
            in for the UR5e. For belt/sensor/shutter bring-up.
--ur HOST   Full hardware: REAL ClearCore (--cc-host, default CLEARCORE_HOST)
            + REAL UR5e at HOST via ur_rtde (Linux/WSL2 only; the arm must
            already be in Remote control mode). sand + conveyor are live;
            spray/clean_gun are not yet implemented (URRobot raises), so a full
            schedule run awaits the Sprayer composite — this mode is for
            robot/conveyor bring-up until then.

Serves the HMI at http://localhost:8000 — operators only, shop LAN only.
"""

from __future__ import annotations

import argparse
import functools
from dataclasses import replace

import uvicorn

from ..config.loader import load_process_config
from ..core.model import LineState
from ..devices.clearcore import ClearCoreClient
from ..process.controller import LineController
from ..process.executor import Executor
from ..process.supervisor import Supervisor
from ..process.train import TrainMover
from ..sim.fake_robot import FakeRobot
from .app import create_app


def _build_sim(cfg):
    """Fake ClearCore + physics + fake robot, times compressed so a full
    period takes about a minute instead of thirteen.
    """
    from ..sim.fake_clearcore import FakeClearCore
    from ..sim.physics import PhysicsSim

    cfg = replace(
        cfg, flash_seconds=10.0, spray_burst_pause_s=2.0, transfer_s=1.0,
        robot_coat1_s=4.0, robot_coat2_s=3.0, clean_gun_duration_s=1.0,
    )
    fake = FakeClearCore(port=15030, watchdog_timeout_s=3.0, shutter_actuation_s=0.3).start()
    physics = PhysicsSim(fake, in_count=0, transfer_s=1.0).start()
    cc = ClearCoreClient("127.0.0.1", port=15030).connect()
    robot = FakeRobot(work_s=3.0, spray_s=2.0, retract_s=0.3)
    return cfg, cc, robot, physics


def _run_legacy(args, cfg) -> None:
    """Legacy-mod route: interleave on the unmodified legacy firmware."""
    from ..config.loader import load_legacy_fans, load_products
    from ..devices.legacy_clearcore import LegacyClearCoreClient
    from ..process.legacy_controller import LegacyController
    from ..process.legacy_sequencer import LegacySequencer
    from ..process.legacy_train import LegacyTrain

    cc = LegacyClearCoreClient(args.legacy).connect()
    train = LegacyTrain(cc)
    fans = load_legacy_fans()
    fan_do = None

    if args.no_robot:
        from ..sim.fake_robot import FakeRobot
        robot = FakeRobot(work_s=5.0, spray_s=5.0)
        print(f"LEGACY MODE (belt only): legacy ClearCore at {args.legacy}, FAKE robot")
    else:
        if not args.ur_host:
            raise SystemExit("--legacy with a real robot needs --ur-host (or pass --no-robot)")
        from ..config.loader import load_brush_config, load_robot_setup, load_sand_config, load_spray_config
        from ..devices.legacy_belt import LegacyBeltAdapter
        from ..devices.ur import Dashboard, URClient
        from ..process.gun_clean import GunClean
        from ..process.robot_ur import URRobot
        from ..process.sander import Sander
        from ..process.sprayer import Sprayer

        print(f"LEGACY MODE: legacy ClearCore at {args.legacy}, real UR5e at {args.ur_host}")
        Dashboard(args.ur_host).connect().power_on_and_release()
        ur = URClient(args.ur_host, load_robot_setup())
        belt = LegacyBeltAdapter(cc)
        products = load_products()
        robot = URRobot(
            ur, Sander(ur, belt, load_sand_config()),
            Sprayer(ur, belt, load_spray_config()),
            GunClean(ur, belt, load_brush_config()),
            lambda pid: products[sequencer.parts[pid].product.value],
        )
        if any(f.controllable for f in fans.values()):
            fan_do = ur.set_digital_out

    sequencer = LegacySequencer(train, robot, cfg, fans, fan_do=fan_do)
    controller = LegacyController(sequencer, cfg, state_file=args.state_file).start()
    if sequencer.fault is not None:
        print(f"RESTORED with parts on the line: {sequencer.fault}")
    fan_desc = ", ".join(f"{k}={v.kind}{'' if v.do is None else f'(DO{v.do})'}"
                         for k, v in fans.items())
    print(f"fans: {fan_desc}")
    print(f"HMI: http://localhost:{args.port}")
    try:
        uvicorn.run(create_app(controller), host="0.0.0.0", port=args.port,
                    log_level="warning")
    finally:
        controller.close()  # Ctrl-C included: stop the loop AND both belts


def main() -> None:
    ap = argparse.ArgumentParser(prog="finishing_line.api")
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--sim", action="store_true", help="fully simulated line")
    mode.add_argument("--cc", metavar="HOST", help="real ClearCore, fake robot")
    mode.add_argument("--ur", metavar="UR_HOST",
                      help="real ClearCore + real UR5e at UR_HOST (needs ur_rtde: Linux/WSL2)")
    mode.add_argument("--legacy", metavar="CC_HOST",
                      help="LEGACY-MOD route: unmodified legacy ClearCore firmware at "
                           "CC_HOST (production: 192.168.1.18), direct-entry interleave "
                           "(docs/legacy-mod-choreography.md). Robot via --ur-host, or "
                           "--no-robot for belt-only sessions.")
    ap.add_argument("--ur-host", default=None,
                    help="UR5e host for --legacy (required unless --no-robot)")
    ap.add_argument("--no-robot", action="store_true",
                    help="legacy mode with FakeRobot — automatic choreography with the "
                         "HMI, no arm motion (the belt-only bring-up session)")
    ap.add_argument("--cc-host", default=None,
                    help="ClearCore host for --ur (default: registers.CLEARCORE_HOST)")
    ap.add_argument("--flash-seconds", type=float, default=None,
                    help="override process flash_seconds (bench/testing — e.g. 8 to "
                         "compress the 180 s flash while manually triggering sensors)")
    ap.add_argument("--bench", action="store_true",
                    help="bench manual-sensor testing (real ClearCore only): stub the "
                         "shutter as follows-command (no end-switches wired yet) and "
                         "loosen the train post-shift verify to human pace")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument(
        "--state-file", default="var/line-state.json",
        help="snapshot path; parts on the line survive a restart via the "
             "confirm-and-resume flow (default: %(default)s)",
    )
    args = ap.parse_args()
    if args.bench and args.sim:
        ap.error("--bench applies to real hardware (--cc/--ur), not --sim")

    cfg = load_process_config()
    if args.flash_seconds is not None:
        cfg = replace(cfg, flash_seconds=args.flash_seconds)

    if args.legacy:
        _run_legacy(args, cfg)
        return

    physics = None
    if args.sim:
        cfg, cc, robot, physics = _build_sim(cfg)
        print("simulated line: fake ClearCore + physics + fake robot (compressed times)")
    elif args.cc:
        cc = ClearCoreClient(args.cc, stub_shutter=args.bench).connect()
        robot = FakeRobot(work_s=5.0, spray_s=5.0)
        print(f"conveyor commissioning: real ClearCore at {args.cc}, FAKE robot")
    else:  # --ur: real ClearCore + real UR5e (needs ur_rtde on this host)
        from ..config.loader import (
            load_brush_config, load_products, load_robot_setup, load_sand_config,
            load_spray_config,
        )
        from ..devices.registers import CLEARCORE_HOST
        from ..devices.ur import Dashboard, URClient
        from .gun_clean import GunClean
        from .robot_ur import URRobot
        from .sander import Sander
        from .sprayer import Sprayer

        cc_host = args.cc_host or CLEARCORE_HOST
        cc = ClearCoreClient(cc_host, stub_shutter=args.bench).connect()
        print(f"FULL HARDWARE: real ClearCore at {cc_host}, real UR5e at {args.ur}")
        Dashboard(args.ur).connect().power_on_and_release()  # bring the arm to RUNNING
        ur = URClient(args.ur, load_robot_setup())           # RTDE; sets TCP + payload
        sander = Sander(ur, cc, load_sand_config())
        sprayer = Sprayer(ur, cc, load_spray_config())
        gun_clean = GunClean(ur, cc, load_brush_config())
        products = load_products()
        # Resolver reads live part identity from the supervisor (built just
        # below); late-bound, so it resolves at operation time.
        robot = URRobot(
            ur, sander, sprayer, gun_clean,
            lambda pid: products[supervisor.state.parts[pid].product.value],
        )

    from ..process.persistence import StateStore

    if args.bench:
        print("BENCH: shutter stubbed (follows command); train post-shift verify -> 30 s")
    executor = Executor(cc, robot, TrainMover(cc, post_shift_timeout_s=30.0 if args.bench else 2.0))
    supervisor = Supervisor(cc=cc, robot=robot, executor=executor, cfg=cfg, state=LineState())
    store = StateStore(args.state_file)
    controller = LineController(supervisor, executor, store=store).start()
    if supervisor.state.fault is not None:
        print(f"RESTORED with parts on the line: {supervisor.state.fault}")
        print("Use the HMI fault panel to confirm occupancy and resume.")

    if physics is not None:
        # In sim mode, a declared batch must also appear on the fake infeed —
        # physics owns the physical queue, the controller owns identity.
        orig = controller.declare_batch

        @functools.wraps(orig)
        def declare_and_feed(product: str, part_ids: list[str]) -> list[str]:
            staged = orig(product, part_ids)
            physics.in_count += len(staged)
            return staged

        controller.declare_batch = declare_and_feed  # type: ignore[method-assign]

    print(f"HMI: http://localhost:{args.port}")
    uvicorn.run(create_app(controller), host="0.0.0.0", port=args.port, log_level="warning")


if __name__ == "__main__":
    main()
