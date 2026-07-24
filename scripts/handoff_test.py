"""Z1/Z2 handoff test — the intake sequence, nothing else.

Setup: nothing on Z2 (main belt), parts queued on Z1 (feed). One
feed-assisted transition, then the persistent feed watch:

  - Z2 stops when part 1 trips the STAGING eye (the intake park).
  - Z1 cuts ONLY on the junction chain: ONLOAD HI (part 1's nose) -> LO
    (tail clears) -> HI (part 2's nose) — whether that completes during the
    move or after Z2 has parked. NO timeout: an empty queue keeps Z1
    feeding until a part shows (Ctrl-C stops it).

Usage (from the repo root, line idle, legacy robot program STOPPED):
    python scripts/handoff_test.py [--host 192.168.1.18] [--mm 1500]

Then phase 2 (on confirm): part 1 rides staging -> WORK_AT_ZERO (Z2 only,
feed off, sensor-stopped) while part 2 stays put at the junction — watch
whether Z2 drags part 2's nose.

Expected physical outcome: part 1 at O on the WZ eye; part 2 unmoved,
nose-at-junction (ONLOAD still blocked).
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from finishing_line.devices.legacy_clearcore import LegacyClearCoreClient


def sensors_line(cc: LegacyClearCoreClient) -> str:
    d = cc.read_inputs()
    return (f"state={d.server_state}  WORK_AT_ZERO={'●' if d.work_at_zero else '·'}  "
            f"ONLOAD={'●' if d.onload else '·'}  STAGING={'●' if d.staging else '·'}  "
            f"OFFLOAD={'●' if d.offload else '·'}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--host", default="192.168.1.18")
    ap.add_argument("--mm", type=float, default=1500.0,
                    help="main-belt cap for the run (default 1500, max 2184)")
    ap.add_argument("--entry-mm", type=float, default=750.0,
                    help="nominal for the staging->O entry (cap = this + 400; "
                         "measured eye->WZ is ~625)")
    ap.add_argument("--from-phase", type=int, default=1, choices=(1, 4, 6),
                    help="start mid-sequence when cubes are already in place "
                         "from an earlier run (4 = entry + retreat, 6 = "
                         "return + stage cube 3)")
    args = ap.parse_args()

    with LegacyClearCoreClient(args.host) as cc:
        print("connected.", sensors_line(cc))
        if args.from_phase == 1:
            print("\nSetup: Z2 empty, 2+ parts queued on Z1 (first part clear of the eye).")
            print("Line idle, legacy robot program STOPPED, e-stop in reach.")
            if input("Type GO to run the handoff: ").strip() != "GO":
                print("aborted.")
                return

            res = cc.transition_move(args.mm, feed=True, stop_on_staging=True)
            print(f"\nran {res['seconds']:.1f} s   arrived={res['arrived']}   "
                  f"entered={res['entered']}   feed_running={res['feed_running']}   "
                  + sensors_line(cc))
            if not res["arrived"]:
                print("!! Z2 hit the distance cap without part 1 tripping STAGING — "
                      "raise --mm or check the staging eye.")
            if res["entered"]:
                print("Z1 cut during the move: follower nose-at-junction (ONLOAD ●).")
            elif res["feed_running"]:
                print("Z2 parked; Z1 STILL FEEDING until a follower trips ONLOAD "
                      "(no timeout — Ctrl-C to stop the feed)...")
                try:
                    while cc.feed_tick() is False:
                        time.sleep(0.05)
                    print("follower at the junction — Z1 cut.", sensors_line(cc))
                except KeyboardInterrupt:
                    cc.set_feed(False)
                    print("\nfeed stopped by operator.", sensors_line(cc))
                    return

            # ---- phase 2: cube 1 staging -> O; cube 2 must not move.
            print("\nPhase 2: cube 1 rides to WORK_AT_ZERO (Z2 only, feed off). "
                  "Cube 2 should stay nose-at-junction — watch it.")
            if input("Type GO to run the entry: ").strip() != "GO":
                print("stopped after phase 1.")
                return
            res = cc.transition_move(args.entry_mm, stop_on_work_zero=True)
            print(f"\nran {res['seconds']:.1f} s   arrived={res['arrived']}   "
                  + sensors_line(cc))
            if res["arrived"]:
                print("cube 1 at O (WZ ●). Cube 2 unmoved? ONLOAD should still be ● "
                      "and the part exactly where it stopped.")
            else:
                print("!! Z2 hit the cap without a WZ arrival — raise --entry-mm or "
                      "check the WORK_AT_ZERO eye.")
                return

            # ---- phase 3: cube 2 junction -> staging (stage_next: feed-only,
            # then the Z2 nudge on a stall — cube 1 slides downstream by the
            # nudge travel; fan location is not precise, accepted).
            print("\nPhase 3: cube 2 stages (feed-only; Z2 nudge if it stalls).")
            if input("Type GO to stage cube 2: ").strip() != "GO":
                print("stopped after phase 2.")
                return
            st = cc.stage_next(feed_timeout_s=10.0)
            mmps = cc.kinematics.velocity_steps_per_sec / cc.kinematics.microsteps_per_mm
            slide = st["nudge_s"] * mmps
            print(f"\nstaged={st['staged']}   nudged={st['nudged']}   "
                  f"slide~{slide:.0f} mm   " + sensors_line(cc))
            if st["staged"]:
                print("cube 2 at STAGING"
                      + (f" — cube 1 slid ~{slide:.0f} mm past WZ." if st["nudged"]
                         else " on feed alone — cube 1 untouched."))
            else:
                print("!! staging failed even with the nudge — cube 2 likely "
                      "mid-junction; check mechanically before moving anything.")
            if st["feed_running"]:
                print("Z1 STILL FEEDING until cube 3 trips ONLOAD (no timeout — "
                      "Ctrl-C to stop if only 2 cubes are queued)...")
                try:
                    while cc.feed_tick() is False:
                        time.sleep(0.05)
                    print("cube 3 at the junction — Z1 cut.", sensors_line(cc))
                except KeyboardInterrupt:
                    cc.set_feed(False)
                    print("\nfeed stopped by operator.", sensors_line(cc))
        else:
            print(f"\nStarting at phase {args.from_phase} — cubes assumed in "
                  "place from the earlier run, feed off.")

        if args.from_phase <= 4:
            # ---- phase 4: cube 2 staging -> O; cube 1 departs O downstream to
            # F2 on the same belt run (WZ chain: depart HI->LO, then arrive HI).
            print("\nPhase 4: cube 2 rides to WORK_AT_ZERO; cube 1 is carried "
                  "downstream to F2. Cube 3 stays at the junction (feed off).")
            if input("Type GO to run the entry: ").strip() != "GO":
                print("stopped after phase 3.")
                return
            res = cc.transition_move(args.entry_mm, stop_on_work_zero=True,
                                     o_occupied=True)
            print(f"\nran {res['seconds']:.1f} s   arrived={res['arrived']}   "
                  + sensors_line(cc))
            if not res["arrived"]:
                print("!! no WZ arrival — check where the cubes rest before retrying.")
                return
            print("cube 2 at O; note cube 1's rest position — that is F2, where "
                  "the downstream fan mounts. Cube 3 moved at all?")

            # ---- phase 5: the retreat — cube 1 comes back upstream to O;
            # cube 2 is pushed back to its staging point on the same run.
            # Legacy return-to-zero: pass WZ HI->LO, then re-approach until HI.
            print("\nPhase 5: RETREAT — belt runs upstream; cube 1 F2 -> O "
                  "(pass + re-approach), cube 2 back toward staging.")
            if input("Type GO to run the retreat: ").strip() != "GO":
                print("stopped after phase 4.")
                return
            res = cc.transition_move(-args.entry_mm, stop_on_work_zero=True,
                                     o_occupied=True, pass_through=True)
            print(f"\nran {res['seconds']:.1f} s   arrived={res['arrived']}   "
                  + sensors_line(cc))
            if res["arrived"]:
                print("cube 1 back at O (approached downstream — WZ ●). Cube 2 "
                      "should rest ~at the staging eye; cube 3 untouched at the "
                      "junction.")
            else:
                print("!! retreat did not land back on WZ — note where cube 1 "
                      "rests; the re-approach cap may need widening.")
                return

        # ---- phase 6: the return — cube 2 staging -> O downstream; cube 1
        # carried out to F2 on the same run (same WZ chain as phase 4).
        print("\nPhase 6: RETURN — cube 2 rides to WORK_AT_ZERO; cube 1 "
              "carried downstream to F2. Feed off, cube 3 stays put.")
        if input("Type GO to run the return: ").strip() != "GO":
            print("stopped after phase 5.")
            return
        res = cc.transition_move(args.entry_mm, stop_on_work_zero=True,
                                 o_occupied=True)
        print(f"\nran {res['seconds']:.1f} s   arrived={res['arrived']}   "
              + sensors_line(cc))
        if not res["arrived"]:
            print("!! no WZ arrival — check where the cubes rest.")
            return
        print("cube 2 at O, cube 1 at F2.")

        # ---- phase 7: stage cube 3. It starts ON the ONLOAD eye, so its
        # chain is LO -> HI; with no cube 4 queued, Z1 runs indefinitely
        # (the designed no-timeout behavior) — Ctrl-C to stop it.
        print("\nPhase 7: stage cube 3 (feed-only; Z2 nudge on stall). "
              "No cube 4: expect Z1 to KEEP RUNNING after cube 3 stages.")
        if input("Type GO to stage cube 3: ").strip() != "GO":
            print("stopped after phase 6.")
            return
        st = cc.stage_next(feed_timeout_s=10.0)
        mmps = cc.kinematics.velocity_steps_per_sec / cc.kinematics.microsteps_per_mm
        slide = st["nudge_s"] * mmps
        print(f"\nstaged={st['staged']}   nudged={st['nudged']}   "
              f"slide~{slide:.0f} mm   feed_running={st['feed_running']}   "
              + sensors_line(cc))
        if st["staged"]:
            print("cube 3 at STAGING"
                  + (f" — cubes 1/2 slid ~{slide:.0f} mm downstream." if st["nudged"]
                     else " on feed alone — belt untouched."))
        if st["feed_running"]:
            print("Z1 running indefinitely (no cube 4 — correct). "
                  "Ctrl-C to cut the feed and finish.")
            try:
                while cc.feed_tick() is False:
                    time.sleep(0.05)
                print("a cube 4 appeared?! Z1 cut.", sensors_line(cc))
            except KeyboardInterrupt:
                cc.set_feed(False)
                print("\nfeed stopped by operator. Sequence complete.",
                      sensors_line(cc))


if __name__ == "__main__":
    main()
