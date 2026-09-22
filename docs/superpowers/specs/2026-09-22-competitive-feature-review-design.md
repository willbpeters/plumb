# Feature candidates from a competitive review — Plus Putt Path

**Date:** 2026-09-22
**Parent:** `2026-09-15-putting-analyzer-design.md`
**Source:** `plusputt.com/products/plus-putt-path`, feature list as published 2026-09-22.

Plus Putt Path is the closest existing product to this one: grip-mounted, round display,
reports on the device with no phone. That makes its feature list a useful mirror — not a
target. This document grades each of its features against **what a 6-axis IMU at the butt of
the grip can actually recover**, and against §1.2, which forbids adding features outside scope.

Nothing here is adopted by this document. Items are sorted into what the hardware can already
do, what it could do with work, and what it cannot do or should not.

Two corroborations worth recording before the list:

- **Their device is 33 g** (24 g device + 9 g adapter). §5.4 estimates ~30 g for this one and
  calls it "mild counterbalancing, generally perceived favorably." A shipping product landing
  at 33 g is independent evidence that the mass budget is realistic rather than optimistic.
- **They support SuperStroke Tech Port grips.** See §4 below — this is the most valuable thing
  in the review and it is not a feature at all.

---

## 1. Already computed, or nearly free

These need no new sensing. The pipeline produces the underlying quantities today and discards
them, or produces them one line away.

### 1.1 Face-to-path differential — **the strongest candidate**

Face angle minus path direction. Free: both terms already exist in `StrokeResult`.

It matters more than either number alone, because face-to-path is what actually determines
where a putt starts. And it is the more defensible of the two to report, for a reason specific
to this device: **it is a relationship between two things the device measures, not a
measurement against an external reference it does not have.**

*What it does not fix.* §3.3's limitation carries through undiminished. A golfer who addresses
2° open is told the face is square, so the differential inherits that same 2° error — the path
term is measured from the stroke's own travel and is unaffected, so nothing cancels. Report it
with the same address-relative caveat as face angle.

### 1.2 Face rotation rate at impact — **possibly the most accurate number the device can produce**

Degrees per second of face rotation at the moment of contact: the gyro reading at the impact
sample, projected onto measured gravity. One dot product.

Worth singling out because **it involves no integration at all.** Face angle is an integral
over ~1300 samples and its error budget is dominated by drift; rotation rate is a direct
reading of the sensor's best-characterised channel, against a measured 0.22–0.24 dps noise
floor. Every other metric in this project is derived; this one is measured.

It is also the number that distinguishes putter types — a zero-torque putter's whole design
claim is that this value is near zero. Reporting it is *not* a putter-type prior (invariant 1
forbids the algorithm keying off rotation, not the device reporting it).

### 1.3 Total face rotation through the stroke

`twist_angle` evaluated from stroke start to impact rather than only at impact. One extra
call. Pairs naturally with 1.2.

### 1.4 Absolute stroke timings, and the pause at the top

The state machine already records `i_backswing_start`, `i_transition` and `i_impact`; tempo
ratio is their quotient and the underlying milliseconds are thrown away. Backstroke time,
forward-stroke time and time-to-contact are free.

**Pause at the top** needs one new definition — the interval around the transition where the
perpendicular rate stays below the onset threshold — but it is timing, which is this device's
best-validated axis (tempo recovers to 0.009 against a 0.05 target).

### 1.5 Backswing and follow-through length, and their ratio

Distance travelled along the forward axis, start→transition and transition→end. Same
`_face_track` machinery as path.

**Much better conditioned than arc.** The noise that currently inflates a 6 mm arc by 24% is
the same few tenths of a millimetre against a backswing of a few hundred — the relative error
falls by roughly the ratio of the two lengths.

**The ratio is better still.** A pivot-offset error scales both lengths in nearly the same
proportion, so it largely cancels in the quotient. That makes length ratio reportable *even
while open defect 6 is unresolved*, which arc magnitude is not. Worth measuring on the harness
before claiming it, but the argument is the same one that makes tempo ratio robust.

### 1.6 Left-handed mirrored mode — **a real gap, and free**

Every screen decision in this project is framed as "looking down at the grip, left is the
target, the stroke animates right to left." For a left-handed golfer that is backwards.

Pure UI, one transform, no algorithm change. It is only on this list because the competitor's
feature list is what made its absence visible.

### 1.7 Compare-to-first-stroke ("Clone")

Store the session's first stroke, show every subsequent one against it. Pure software plus
on-device storage, which §1.2 permits — the non-goal is *cloud sync, accounts, and history
beyond on-device storage*, not local history.

### 1.8 Session history

Same reasoning as 1.7. 13 MB of LittleFS is far more than a trend of recent sessions needs.

---

## 2. Possible, with real work

### 2.1 Visual metronome that learns your tempo

Their version is a drill mode whose metronome adapts to the golfer's natural rhythm. Entirely
buildable and it leans on tempo, the metric this project recovers best.

**Constraint:** the board has no buzzer and no haptic motor, so it must be visual. That is a
weaker metronome than an audible one — a golfer over a putt is looking at the ball, not at the
grip. Consider whether a visual-only metronome is worth building before building it.

### 2.2 Rise / attack angle at impact

Angle of the face-point velocity vector out of the ground plane at impact. Computable:
`v_face` and `g₀` both exist. It inherits the translation error that path has, so it is gated
behind the same pivot work.

### 2.3 Configurable multi-stat screens

They offer 4-stat grids and 2-stat views. This project deliberately chose one metric per
swipeable screen, so this is a *decision to revisit*, not a gap. Noted and not recommended:
the one-metric decision was made for a reason and a 240×240 round display is small.

---

## 3. Cannot do, or should not

### 3.1 Strike location (toe / heel offset in mm) — the interesting one

They report where on the face contact happened. A 6-axis IMU at the butt is not obviously
blind to this: an off-centre strike applies a torque about the shaft, and the gyro would see
the resulting impulse, with toe and heel giving opposite signs.

**So it is not impossible in principle — it is blocked twice over:**

1. It lives entirely in the impact impulse, which needs sampling well above 1 kHz. That is
   open defect 1, and the only route to it is the FIFO path that currently loses 21.9%.
2. It requires the mount to transmit the impulse faithfully, which is exactly what §5.5's tap
   test exists to determine and which has not been run.

It would also need calibration against known strike locations — a reference this project does
not have, and §10.2's motion-capture tier has been deprioritised.

**Recommendation:** record as a research question, not a feature. It is the most technically
interesting thing on their list and the least defensible to claim without validation.

### 3.2 Ball speed adjusted for strike location

Depends entirely on 3.1.

### 3.3 Estimated roll distance

Already handled: §1.2.1 defers this, with reasoning that stands. Their shipping it is evidence
of commercial expectation, not evidence the physics got easier. §1.2.1's three obstacles —
strike quality, green speed, slope — are unchanged, and only green speed is addressed by
per-round calibration.

### 3.4 Putt quality score, 0–100 — **rejected, and record why**

This conflicts head-on with a product decision already made with Will: **report, never judge.**
No screen says "good" or "perfect", because a device that grades a stroke asserts a standard,
and 1.8° open is a miss for a tour player and a fine putt for most people.

A 0–100 score is that assertion with a number on it. Their "Combine" scored round is the same
thing at session scale.

Recording this as *rejected with a reason* rather than omitting it, so that nobody re-proposes
it in six months having seen the competitor do it.

### 3.5 Mini Golf, Lag Ladder

Both require distance estimation (§3.3 above) and both are scored (§3.4 above). Out on two
counts.

---

## 4. The most valuable finding is not a feature

**They list SuperStroke Tech Port grip compatibility.**

Tech Port grips ship with a manufacturer-designed port in the butt intended to accept a sensor.
That is directly relevant to §5.5, the highest-risk unknown in this project:

> The base anchors into rubber... any compliance between the base and the shaft means the IMU
> partly measures the plug flexing rather than the putter rotating.

§5.5's escalation path, if the tap test fails, is to open the grip butt with a step drill and
anchor an expanding collet into the steel shaft bore — explicitly **destructive to the grip and
unusable on borrowed or demo putters.**

A purpose-built port in a commercially available grip is a **third option that sits between the
barbed taper and the collet**: more rigid than a barb in rubber, non-destructive, and available
to anyone who buys that grip. It does not replace the barbed taper — §5.6's whole point is
tolerating grips the project does not control — but it would be a strong reference mount.

**Concrete proposal:** buy one SuperStroke Tech Port grip and run the §5.5 tap test against
both mounts. That turns the tap test from a pass/fail gate into a comparison, and if the barb
fails it provides an escalation path that does not destroy a grip. It also gives a rigid
reference against which the barb's compliance can be *measured* rather than just judged.

This should be added to the hardware purchasing spec.

---

## 5. Proposed amendments

None applied. In order of value:

1. **§1.2.1** — add face-to-path differential, face rotation rate at impact, total face
   rotation, absolute stroke timings, pause duration, and backswing/follow-through length and
   ratio as deferred-but-wanted, with the notes above. Several are close enough to free that
   they may belong in the first build; that is a scope decision, not a technical one.
2. **§5.5** — add the SuperStroke Tech Port mount as a third option and as a rigid reference
   for the tap test.
3. **§1.2** — add "numeric quality or skill scores" to the explicit non-goals list, with the
   report-never-judge reasoning, so it is written down in the contract rather than only in the
   handoff.
4. **§5.4** — note the competitor's 33 g as external corroboration of the mass budget.
5. **Hardware purchasing spec** — add a SuperStroke Tech Port grip.
6. **§8** — long putters (armlock, broomstick) change both the lever arm and the pivot offset.
   The putter profile already exists; note that its parameters have a much wider range than a
   conventional putter implies.
