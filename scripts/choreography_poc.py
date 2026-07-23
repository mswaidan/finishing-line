"""Belt-choreography proof of concept — the interleaved schedule on ONE belt,
driven against the UNMODIFIED legacy ClearCore firmware. No robot, no fans,
no Polyscope: just the P1..P4 zone motions from the spec, keypress-paced,
with real workpieces you place and watch.

WHY THIS IS VALID: the two zones of the rewrite always move together in the
same direction (core/model.py Zone docstring) — the split exists for the
baffle, not for scheduling. So on the existing single belt every transition
collapses to one distance move of +/- one station pitch, which is exactly the
kind of move the legacy firmware has executed in production for years.

    python scripts/choreography_poc.py probe --host 192.168.1.18
    python scripts/choreography_poc.py jog   --host ... --mm 25
    python scripts/choreography_poc.py feed  --host ... --seconds 2
    python scripts/choreography_poc.py idle  --host ...
    python scripts/choreography_poc.py run   --host ... --pitch-mm 500 [--parts 4]

SAFETY: this moves the PRODUCTION belt. Line must be idle, legacy robot
program NOT running (single Modbus master), belt clear of tools, e-stop in
reach. Motion commands ask for explicit GO.

Suggested pitch for tonight: the distance from the work-zero position to the
existing fan's sweet spot — that is what F1's fan placement must mirror
upstream. Measure it, pass --pitch-mm.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from finishing_line.core.model import Station  # noqa: E402
from finishing_line.core.schedule import BEATS, SCHEDULE, next_beat  # noqa: E402
from finishing_line.devices.legacy_clearcore import (  # noqa: E402
    LegacyClearCoreClient,
    LegacyClearCoreError,
)

BELT_STATIONS = (Station.F1, Station.O, Station.F2)  # stations ON the main belt


def sensors_line(cc: LegacyClearCoreClient) -> str:
    d = cc.read_inputs()
    return (f"state={d.server_state}  WORK_AT_ZERO={'●' if d.work_at_zero else '·'}  "
            f"ONLOAD={'●' if d.onload else '·'}  OFFLOAD={'●' if d.offload else '·'}")


def confirm_go(what: str) -> bool:
    print(f"\n!! {what}")
    print("   Line idle, legacy robot program STOPPED, belt clear, e-stop in reach.")
    return input("   Type GO to proceed: ").strip() == "GO"


def occupancy_str(occ: dict, queue: list[str]) -> str:
    stations = "   ".join(f"{s.name}:{occ.get(s, '-') or '-'}" for s in BELT_STATIONS)
    return f"IN queue:{len(queue)}   {stations}"


# ------------------------------------------------------------------ commands


def cmd_probe(cc: LegacyClearCoreClient, args) -> None:
    print("legacy ClearCore at", args.host)
    print(" ", sensors_line(cc))
    for name, addr in (("mode", 200), ("direction", None), ("velocity", 202),
                       ("accel", 203), ("distance", 204), ("request_id", 206)):
        if addr:
            print(f"  echo {name:10}: {cc._read_input_reg(addr)}")
    print("read-only probe complete — nothing was commanded.")


def cmd_jog(cc: LegacyClearCoreClient, args) -> None:
    if not confirm_go(f"JOG the main belt {args.mm:+.0f} mm ({'DOWNSTREAM' if args.mm >= 0 else 'UPSTREAM'} if convention holds)"):
        return
    cc.set_params()
    print("  before:", sensors_line(cc))
    steps = cc.move_mm(args.mm)
    print(f"  moved {args.mm:+.0f} mm ({steps} steps) — belt should have gone "
          f"{'DOWNSTREAM (toward fan/offload)' if args.mm >= 0 else 'UPSTREAM (toward infeed)'}")
    print("  after :", sensors_line(cc))


def cmd_feed(cc: LegacyClearCoreClient, args) -> None:
    if not confirm_go(f"RUN the feed conveyor for {args.seconds:.1f} s"):
        return
    cc.set_params()
    cc.set_feed(True)
    try:
        time.sleep(args.seconds)
    finally:
        cc.set_feed(False)
    print("  feed pulsed.", sensors_line(cc))


def cmd_idle(cc: LegacyClearCoreClient, args) -> None:
    cc.move_idle()
    print("  main conveyor idled.", sensors_line(cc))


def _feed_recovery(cc: LegacyClearCoreClient, part: str) -> None:
    """ONLOAD never completed its pass — let the operator pulse the feed."""
    print(f"  !! ONLOAD did not complete its HI->LO pass — {part} may be mid-junction.")
    while True:
        ans = input("     f+ENTER = pulse feed 1 s · ENTER = continue (part placed/ok): ").strip().lower()
        if ans != "f":
            return
        cc.set_feed(True)
        time.sleep(1.0)
        cc.set_feed(False)
        print("     pulsed.", sensors_line(cc))


def cmd_run(cc: LegacyClearCoreClient, args) -> None:
    """Direct-entry interleave — no F1 staging stop, no commanded pitch.

        load    : C1 queue->O (continuous belt, work-zero stop)
        P1->P2  : C2 ENTERS queue->O · C1 carried out to F2
        P2->P3  : C1 retreats F2->O (pass + re-approach) · C2 carried to F1
        P3->P4  : C2 returns F1->O · C1 carried to F2 · next lead STAGES after
        P4->P1' : C3 ENTERS queue->O · C2 to F2 · C1 to OUT

    Every arrival at O is sensor-stopped (continuous belt). Spacing is whatever
    the entry geometry produces — the printed travel per transition — and the
    fans belong wherever parts naturally rest. Staging is timed so the junction
    is always CLEAR when a retreat runs: each entry stages the part behind it,
    and the P3->P4 return stages the next lead afterwards. Blind moves remain
    only for fill/drain gaps with no O-arrival.
    """
    queue = []
    for i in range(args.parts):
        pair, role = i // 2 + 1, ("L" if i % 2 == 0 else "T")
        queue.append(f"{role}{pair}")
    occ: dict[Station, str] = {}
    completed: list[str] = []
    beat = BEATS[0]
    mmps = cc.kinematics.velocity_steps_per_sec / cc.kinematics.microsteps_per_mm
    last_spacing: float | None = None
    warned_retreat = False

    print(__doc__.split("SAFETY")[0])
    print(f"{args.parts} parts ({', '.join([*queue])})"
          + ("  · FEED mode: entries ride queue->O" if args.feed else "  · hand-place mode"))
    print("No marks needed — the belt travel printed per transition IS the spacing.\n")
    if not confirm_go("RUN the direct-entry choreography (sensor-determined moves"
                      + (", feed conveyor live" if args.feed else "") + ")"):
        return
    cc.set_params()

    def report(res, label) -> float:
        est = res["seconds"] * mmps
        print(f"  belt ran {res['seconds']:.1f} s (~{est:.0f} mm)   {sensors_line(cc)}")
        if res["arrived"] is True:
            print(f"  {label}: arrived at work-zero — travel/spacing ~{est:.0f} mm")
        elif res["arrived"] is False:
            print(f"  !! {label}: WORK_AT_ZERO chain did not complete — belt idled; investigate.")
        return est

    def stage_jit(part: str) -> bool:
        """Just-in-time staging, run immediately BEFORE an entry (beat-end:
        the O part's work is done and it departs next move, so the nudge's
        δ-slide is harmless). Feed-only bulk, sensor-stopped belt NUDGE only
        if the crossing stalls — δ is measured and reported every time."""
        print(f"  staging {part} just-in-time (feed-only; belt-nudge if it stalls)...")
        st = cc.stage_next()
        if st["staged"]:
            if st["nudged"]:
                d = st["nudge_s"] * mmps
                print(f"  staged ✓ via NUDGE ~{d:.0f} mm (beat-end slide — harmless)")
                if d > 150:
                    print("  !! nudge > 150 mm — retreat margin at risk: check the junction "
                          "mechanically or add eye margin downstream")
            else:
                print("  staged ✓ (feed alone reached the eye)")
        else:
            print("  !! staging FAILED even with the nudge — junction needs eyes on it")
        return st["staged"]

    # ---- startup: first part straight to O; the part behind it stages.
    first = queue.pop(0)
    if args.feed:
        input(f"\n>> {first} will LOAD queue->O (continuous, work-zero stop; feed cuts "
              "at the FIRST ONLOAD HI — aboard). ENTER... ")
        res = cc.transition_move(900.0, stop_on_work_zero=True, feed=True, continuous=True,
                                 timeout_s=120.0)
        est = report(res, "load")
        if res["arrived"]:
            last_spacing = est
        if res["entered"] is False:
            _feed_recovery(cc, first)
    else:
        input(f"\n>> Place part {first} at O (on the work-zero eye), then ENTER... ")
    occ[Station.O] = first
    print("  ", occupancy_str(occ, queue))
    # No staging here: staging is just-in-time, immediately before each entry.
    # The eye and junction stay clear at all other times by construction.

    while True:
        spec = SCHEDULE[beat]
        nxt = next_beat(beat)
        print(f"\n===== beat {beat} =====")
        robot_part = occ.get(Station.O)
        print(f"  [{beat}] robot would: {spec.robot.role.name} "
              f"{'gun-clean' if spec.robot.clean_gun else 'sand'}+coat{spec.robot.coat} "
              f"on {robot_part or '(empty — skipped)'}   "
              f"fans: F1={spec.f1_fan.name} F2={spec.f2_fan.name}")

        blind_mm = last_spacing or args.pitch_mm
        # ---- choose this transition's mechanics from occupancy + the scheme
        if beat in ("P1", "P4"):          # downstream; entries happen here
            if occ.get(Station.F1):
                action, desc = "return", f"{occ[Station.F1]} returns F1->O (sensor-stopped)"
            elif queue and args.feed:
                action, desc = "entry", f"{queue[0]} ENTERS queue->O (continuous + feed)"
            elif queue:
                action, desc = "entry_hand", f"blind +{blind_mm:.0f} mm, then hand-place {queue[0]} at O"
            elif occ:
                action, desc = "blind_down", f"drain shuffle: blind +{blind_mm:.0f} mm"
            else:
                action, desc = "none", "nothing to do"
        elif beat == "P2":                # upstream retreat
            if occ.get(Station.F2):
                action, desc = "retreat", (f"{occ[Station.F2]} retreats F2->O (pass + re-approach); "
                                           f"{occ.get(Station.O) or 'nothing'} carried to F1")
            else:
                action, desc = "none", "no part at F2 — lone part keeps O for coat 2"
        else:                             # P3: downstream return; stage next after
            if occ.get(Station.F1):
                action, desc = "return", f"{occ[Station.F1]} returns F1->O (sensor-stopped)"
            elif occ:
                action, desc = "blind_down", f"drain shuffle: blind +{blind_mm:.0f} mm"
            else:
                action, desc = "none", "nothing on the belt"

        print(f"  [{beat}->{nxt}] {desc}")
        if action == "retreat" and not warned_retreat:
            print("  ** FIRST RETREAT: watch the trail vs the junction/feed belt — this is")
            print("     the one physical constraint of direct entry. E-stop if it crowds.")
            warned_retreat = True

        ans = input("  ENTER = execute · q = quit: ").strip().lower()
        if ans == "q":
            break

        if action == "entry":
            # Phase 0 — JIT staging: park the enterer at the eye right now
            # (beat-end, δ-slide harmless). Phase 1 — the entry ride, feed OFF
            # (enterer is aboard at the eye); feed boarding-assist only as a
            # fallback if staging failed outright.
            staged = cc.read_inputs().onload or stage_jit(queue[0])
            res = cc.transition_move(900.0, stop_on_work_zero=True,
                                     o_occupied=occ.get(Station.O) is not None,
                                     feed=not staged, continuous=True, timeout_s=120.0)
            est = report(res, "entry")
            if res["arrived"]:
                last_spacing = est
            if res["entered"] is False:
                _feed_recovery(cc, queue[0])
            if occ.get(Station.F2):
                completed.append(occ.pop(Station.F2))
                print(f"  >> {completed[-1]} should now be at OUT — REMOVE it")
            if occ.get(Station.O):
                occ[Station.F2] = occ.pop(Station.O)
            occ[Station.O] = queue.pop(0)
        elif action == "entry_hand":
            cc.transition_move(blind_mm)
            if occ.get(Station.F2):
                completed.append(occ.pop(Station.F2))
                print(f"  >> {completed[-1]} should now be at OUT — REMOVE it")
            if occ.get(Station.O):
                occ[Station.F2] = occ.pop(Station.O)
            part = queue.pop(0)
            input(f"  >> place {part} at O (on the eye), then ENTER... ")
            occ[Station.O] = part
        elif action == "return":
            res = cc.transition_move(900.0, stop_on_work_zero=True,
                                     o_occupied=occ.get(Station.O) is not None,
                                     continuous=True, timeout_s=90.0)
            est = report(res, "return")
            if res["arrived"]:
                last_spacing = est
            if occ.get(Station.F2):
                completed.append(occ.pop(Station.F2))
                print(f"  >> {completed[-1]} should now be at OUT — REMOVE it")
            if occ.get(Station.O):
                occ[Station.F2] = occ.pop(Station.O)
            occ[Station.O] = occ.pop(Station.F1)
        elif action == "retreat":
            res = cc.transition_move(-900.0, stop_on_work_zero=True,
                                     o_occupied=occ.get(Station.O) is not None,
                                     pass_through=True, continuous=True, timeout_s=90.0)
            est = report(res, "retreat")
            if res["arrived"]:
                last_spacing = est
            # With ONLOAD one cube downstream of the junction, the retreated
            # trail rests nose-at-the-eye — ONLOAD doubles as the F1 position
            # check (junction is clear during retreats, so HI is unambiguous).
            if occ.get(Station.O):  # a trail was carried back to F1
                on = cc.read_inputs().onload
                print("  F1 check (ONLOAD): "
                      + ("HI ✓ trail resting at the eye — F1 fan mounts here"
                         if on else "LO — trail short of the eye; note where it rests"))
            if occ.get(Station.O):
                occ[Station.F1] = occ.pop(Station.O)
            occ[Station.O] = occ.pop(Station.F2)
        elif action == "blind_down":
            cc.transition_move(blind_mm)
            print(f"  blind +{blind_mm:.0f} mm done   {sensors_line(cc)}")
            if occ.get(Station.F2):
                completed.append(occ.pop(Station.F2))
                print(f"  >> {completed[-1]} should now be at OUT — REMOVE it")
            if occ.get(Station.O):
                occ[Station.F2] = occ.pop(Station.O)
            if occ.get(Station.F1):
                occ[Station.O] = occ.pop(Station.F1)


        if occ.get(Station.O) is not None and action in ("entry", "return", "retreat"):
            wz = cc.read_inputs().work_at_zero
            print(f"  WORK_AT_ZERO check: {'HI ✓' if wz else 'LO !! part not on the eye'}")

        print("  expected:", occupancy_str(occ, queue), f"  done: {completed or '-'}")
        if not occ and not queue:
            print(f"\nline drained — {len(completed)} parts completed: {completed}")
            break
        beat = nxt

    cc.move_idle()
    print("\nbelt idled. POC session over.")


def main() -> int:
    ap = argparse.ArgumentParser(prog="choreography_poc", description=__doc__)
    ap.add_argument("--host", required=True, help="legacy ClearCore (production: 192.168.1.18)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("probe")
    j = sub.add_parser("jog"); j.add_argument("--mm", type=float, required=True)
    f = sub.add_parser("feed"); f.add_argument("--seconds", type=float, default=2.0)
    sub.add_parser("idle")
    r = sub.add_parser("run")
    r.add_argument("--pitch-mm", type=float, default=900.0,
                   help="fallback distance for BLIND fill/drain moves only — steady "
                        "transitions are sensor-determined (default: 900, or the last "
                        "measured spacing once one exists)")
    r.add_argument("--parts", type=int, default=4, help="simulated parts (default 4)")
    r.add_argument("--feed", action="store_true",
                   help="entries ride queue->O with the feed running (ONLOAD pass "
                        "semantics); staging is timed so the junction is clear "
                        "whenever a retreat runs")
    args = ap.parse_args()

    try:
        with LegacyClearCoreClient(args.host) as cc:
            {"probe": cmd_probe, "jog": cmd_jog, "feed": cmd_feed,
             "idle": cmd_idle, "run": cmd_run}[args.cmd](cc, args)
    except LegacyClearCoreError as exc:
        print(f"\nFAILED: {exc}")
        return 1
    except KeyboardInterrupt:
        print("\ninterrupted — sending idle")
        try:
            LegacyClearCoreClient(args.host).connect().move_idle()
        except Exception:
            print("could not idle the belt — verify manually / e-stop if it is moving")
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
