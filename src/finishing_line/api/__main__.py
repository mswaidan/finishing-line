"""Runnable orchestrator: `python -m finishing_line.api [--sim | --cc HOST | --ur HOST]`

--sim       Full simulation: fake ClearCore + physics + FakeRobot, compressed
            process times. The whole line runs visibly in a browser with zero
            hardware — the Stage C demo.
--cc HOST   Conveyor commissioning: REAL ClearCore at HOST, FakeRobot standing
            in for the UR5e. For belt/sensor/shutter bring-up.
--ur HOST   Full hardware: REAL ClearCore (--cc-host, default CLEARCORE_HOST)
            + REAL UR5e at HOST via ur_rtde (Linux/WSL2 only; the arm must
            already be in Remote control mode). sand + conveyor are live;
            spray/denib are not yet implemented (URRobot raises), so a full
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
        robot_coat1_s=4.0, robot_coat2_s=3.0, denib_duration_s=1.0,
    )
    fake = FakeClearCore(port=15030, watchdog_timeout_s=3.0, shutter_actuation_s=0.3).start()
    physics = PhysicsSim(fake, in_count=0, transfer_s=1.0).start()
    cc = ClearCoreClient("127.0.0.1", port=15030).connect()
    robot = FakeRobot(work_s=3.0, spray_s=2.0, retract_s=0.3)
    return cfg, cc, robot, physics


def main() -> None:
    ap = argparse.ArgumentParser(prog="finishing_line.api")
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--sim", action="store_true", help="fully simulated line")
    mode.add_argument("--cc", metavar="HOST", help="real ClearCore, fake robot")
    mode.add_argument("--ur", metavar="UR_HOST",
                      help="real ClearCore + real UR5e at UR_HOST (needs ur_rtde: Linux/WSL2)")
    ap.add_argument("--cc-host", default=None,
                    help="ClearCore host for --ur (default: registers.CLEARCORE_HOST)")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument(
        "--state-file", default="var/line-state.json",
        help="snapshot path; parts on the line survive a restart via the "
             "confirm-and-resume flow (default: %(default)s)",
    )
    args = ap.parse_args()

    cfg = load_process_config()
    physics = None
    if args.sim:
        cfg, cc, robot, physics = _build_sim(cfg)
        print("simulated line: fake ClearCore + physics + fake robot (compressed times)")
    elif args.cc:
        cc = ClearCoreClient(args.cc).connect()
        robot = FakeRobot(work_s=5.0, spray_s=5.0)
        print(f"conveyor commissioning: real ClearCore at {args.cc}, FAKE robot")
    else:  # --ur: real ClearCore + real UR5e (needs ur_rtde on this host)
        from ..config.loader import load_products, load_robot_setup, load_sand_config
        from ..devices.registers import CLEARCORE_HOST
        from ..devices.ur import Dashboard, URClient
        from .robot_ur import URRobot
        from .sander import Sander

        cc_host = args.cc_host or CLEARCORE_HOST
        cc = ClearCoreClient(cc_host).connect()
        print(f"FULL HARDWARE: real ClearCore at {cc_host}, real UR5e at {args.ur}")
        Dashboard(args.ur).connect().power_on_and_release()  # bring the arm to RUNNING
        ur = URClient(args.ur, load_robot_setup())           # RTDE; sets TCP + payload
        sander = Sander(ur, cc, load_sand_config())
        products = load_products()
        # Resolver reads live part identity from the supervisor (built just
        # below); late-bound, so it resolves at sand time when supervisor exists.
        robot = URRobot(
            ur, sander,
            lambda pid: products[supervisor.state.parts[pid].product.value],
        )

    from ..process.persistence import StateStore

    executor = Executor(cc, robot, TrainMover(cc))
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
