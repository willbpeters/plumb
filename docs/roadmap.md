# Roadmap to 11 December — DRAFT

**Status:** draft for Will to revise, 2026-09-26. Nothing here is decided until he says so.
**Sources:** parent spec §13 (milestones), §10.1 (validation tiers), §5.5 (tap test gate);
`HANDOFF.md` for state.

**Week numbering (an assumption).** §13 gives 15 weeks ending Friday 11 December, so week 1 is
taken to start Monday 31 August. Today, Saturday 26 September, is the end of week 4. If the
semester actually started on another date, shift everything by the difference.

**The hard edge:** a Spring 2027 semester abroad ends physical work, so the validation study
must be finished in Fall (§13). Eleven working weeks remain, and one of them is Thanksgiving.

---

## 1. Where things stand against §13

| Phase (§13 target) | Status | Evidence |
|---|---|---|
| **0. Bring-up** (wk 1–2) | **Mostly done.** IMU streams; display and touch render the real screens. **Open:** battery power and charging, blocked on a cell. | `docs/bringup-results.md` |
| **1. Mount** (wk 2–4) | **Behind, and it is the hard gate.** No base printed, tap test not started; grip survey done for the blade only. | `HANDOFF.md` "Blocked on Will" |
| **2. Logging** (wk 4–6) | **Not started.** The capture tools exist for a tethered board; on-device logging (§11) needs the ESP-IDF firmware. | — |
| **3. Tempo** (wk 6–7) | **Algorithm done in Python, not on device.** Tempo worst error 0.009 against 0.05, on synthetic strokes. | `analysis/` |
| **4. Face angle** (wk 7–11) | **Algorithm done in Python; C port started** (`quat.c`, bit-identical). No validation tiers run. | `test_c_port.py` |
| **5. Path + UI** (wk 11–13) | **Ahead.** Path fit done in Python (session means within 1% of truth on synthetic strokes); UI screens done and on the panel. | `test_pipeline.py`, `test_ui.py` |
| **6. Study** (wk 13–15) | **Not started.** Scope reduced by Will: optical motion capture deprioritised, tiers 0–2 remain. | `HANDOFF.md` "Validation" |

Everything proven so far is proven **on synthetic strokes or on a bench**. Nothing has measured
a real putt yet, which is what the remaining weeks are for.

## 2. The critical path

```
print base ─► tap test (§5.5 gate) ─► mounted logging ─► corpus ─► thresholds (inv. 5)
                                                             │
ESP-IDF firmware + C port ───────────────────────────────────┴─► on-device metrics ─► tiers 0–2 ─► report
```

Two chains, which can run in parallel until they meet at the corpus:

- **The hardware chain is on Will**, and it is behind. The printed base gates the tap test,
  the tap test gates everything mounted, and real strokes gate the thresholds that invariant 5
  forbids guessing.
- **The software chain needs no hardware**: the ESP-IDF firmware skeleton, the rest of the C
  port, and logging. It should run flat out while the base is printed, so that the day the
  mount passes, the firmware is ready to log strokes on it.

## 3. Proposed order of work

| Week | Will (hardware, decisions) | Code (no hardware needed unless marked) |
|---|---|---|
| **5** · Sep 28 – Oct 2 | Print the base, and the Tier 0 protractor plate on the same run. Run the accelerometer tumble (5 min). Order a ≥ 1000 mAh MX1.25 cell. Decide 906.86 Hz. Check the mallet and zero-torque grips' butt caps (§5.6). | **ESP-IDF skeleton:** install the toolchain; IMU task on core 0 with direct polling; LVGL on core 1 with the display driver carried over from bring-up; invariant 8 enforced by the app task. |
| **6** · Oct 5 – 9 | **Tap test** (§5.5), on the printed base. | Tap test firmware: the test needs sampling above 1 kHz. Try 1793.6 Hz direct polling first (untried); the FIFO path still loses 21.9%. **§11 logging to LittleFS** and a USB dump tool. |
| **7** · Oct 12 – 16 | **If the tap test fails:** the §5.5 escalation (collet into the shaft bore) comes before anything else. Otherwise, log the first strokes with the blade. | `pivot.c`, then `pipeline.c`, each re-verified against the Python with the differential harness. |
| **8** · Oct 19 – 23 | Corpus: blade, then mallet and zero-torque, in a room that is representative of use (open defect 5: room vibration defeats stillness detection). | Notebook: derive the detection thresholds from the corpus (invariant 5). Re-run the algorithm on real strokes; the first real test of everything in `analysis/`. |
| **9** · Oct 26 – 30 | **Tier 0**: static angles on the protractor plate (±0.5° at 0, ±1, ±2, ±5°). If it fails, nothing downstream matters. | Tempo and face angle on the device from the ported C; the C re-verified on the real corpus. |
| **10** · Nov 2 – 6 | **Tier 1**: pendulum. **Tier 2**: phone at 240 fps. | Fix what the tiers find. |
| **11** · Nov 9 – 13 | Charging verified with the new cell (§4.4 amended: 1 A). | Path on the device: the pivot calibration across a session, the accelerometer calibration applied. |
| **12** · Nov 16 – 20 | — | Power (§9): sleep and wake on motion. Results screen wired to real strokes. |
| **13** · Nov 23 – 27 | *Thanksgiving: slack.* | *Slack.* |
| **14** · Nov 30 – Dec 4 | **Study runs**: tiers 0–2 across all three putters. | Bland–Altman analysis (§10.4). |
| **15** · Dec 7 – 11 | — | **Write-up**: validation report and accuracy study, deliverables of equal standing with the firmware (§15). |

## 4. Risks, in order of what they would cost

1. **The mount fails the tap test.** It is the highest-risk unknown in the build (§5.5), and it
   has not started. The escalation is destructive to the grip and cannot be used on borrowed
   putters. Every week the base is not printed comes straight off the end of the schedule.
2. **The tap test cannot sample fast enough.** It needs more than 1 kHz; the FIFO path loses
   21.9% and direct polling at 1793.6 Hz is untried. Settle it in week 6, before it blocks the
   gate.
3. **Real strokes break the synthetic results.** Every accuracy number so far agrees with the
   generator, not with a putt (`HANDOFF.md`). Weeks 8–9 are where that meets reality; budget
   for fixes, which is what week 10 is.
4. **Accelerometer bias.** Up to ~25% of arc on this unit until the tumble is run (open defect
   6). Five minutes of Will's time settles how big it is.
5. **Validation scope.** With optical motion capture set aside, tier 2 (a phone at 240 fps) is
   the only independent reference for face angle at impact. §1.3 and `CLAUDE.md` say a
   finished device with unvalidated numbers fails the goal. Raised here once, as the handoff
   asks.

## 5. Decisions only Will can make

- **Print date for the base.** Everything mounted waits on it.
- **906.86 Hz:** per-unit calibration, or the firmware measures its own rate at startup
  (`HANDOFF.md`, next task 2). Needed before `pipeline.c`.
- **The cell:** ≥ 1000 mAh, or change R15 on the board (§4.4 as amended).
- **Whether this plan is the plan.** In particular whether ESP-IDF comes before the C port, as
  proposed here, or after.
