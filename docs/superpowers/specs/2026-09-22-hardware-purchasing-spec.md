# Hardware purchasing spec — what to buy, and the envelope to design the base against

**Date:** 2026-09-22
**Purpose:** freeze the numbers needed to order parts and start CAD on the base and puck.
**Parent:** `2026-09-15-putting-analyzer-design.md` §4 (hardware) and §5 (mechanical).

> **Read section 1 before ordering a battery.** The parent spec's BOM and the board as
> manufactured are incompatible, and the incompatibility is the destructive kind. This was
> found by doing what parent §4.4 hazard 4 instructs — reading the charge current off the
> schematic — which had not been done.

---

## 1. The battery is blocked, and here is why

### What the board actually does

Parent §4.1 and §4.4 name the charger as an **ETA6096** rated to 800 mA, and worry that the
board might be programmed to 500 mA, which would put a 400 mAh cell at 1.25C.

The board does not carry an ETA6096. Read off the Waveshare schematic
(`ESP32-S3-Touch-LCD-1.28-Sch.pdf`, U6):

| | |
|---|---|
| Charger | **ETA6098** — a 2.5 A synchronous buck charger, not the ETA6096 |
| ISET resistor | **R15 = 160 kΩ, fitted** |
| Schematic's own selection table | `160K → 1 A`, `82K → 2 A`, `66K → 2.5 A` |

Cross-checked against the ETA6098 datasheet, which characterises `RISET = 82 kΩ → 2 A` and
`RISET = 150 kΩ → 1.2 A`. Waveshare's 82 K row matches the datasheet exactly, and 160 K sits
just above the 150 K row. **The board is programmed to roughly 1 A of fast-charge current.**

Also fixed in silicon, from the same datasheet: pre-charge 200 mA, termination at 130 mA,
end-of-charge 4.2 V, auto-restart 160 mV below EOC.

### Why that blocks the purchase

Parent §4.4 hazard 4 sets the rule: **charge current ≤ 1C for the fitted cell.**

- A 400 mAh cell — the BOM's choice — charges at **2.5C**. Two and a half times the spec's own
  limit, on a small pouch cell, with no thermal management inside a sealed printed shell.
- Satisfying ≤1C at 1 A needs a **≥1000 mAh** cell.

And a ≥1000 mAh cell does not fit. A Ø37.5 mm PCB circumscribes a square of about
**26 × 26 mm**; the largest sensible rectangle is roughly 20 × 32 mm. Common 1000 mAh pouches
are 34 × 50 mm or 30 × 40 mm — both need a circle well over 45 mm. Making the puck wide enough
abandons the form factor in §5.4 (~30 g, ~20 mm proud of the butt).

So the constraint triangle does not close: **1 A charge current, ≤1C, and a cell that fits
inside Ø37.5 mm — pick two.**

### The three ways out

| | What it means | Cost / risk |
|---|---|---|
| **A. Rework R15** | Swap the 160 kΩ for ~330–450 kΩ to bring charge current to 0.4–0.5 A, then a 400 mAh cell is at ~1C and everything else in the parent spec stands. | SMD rework on a $25 board. **The datasheet only characterises 82 K–150 K, so 400 kΩ is extrapolation** — confirm with ETA, or measure the actual charge current before trusting it. |
| **B. Bigger cell, bigger puck** | Keep the board untouched, fit ≥1000 mAh, let the puck overhang the board. | Kills the form factor and changes the mass that §5.4 calls "a controlled constant across the validation study". |
| **C. Cell rated for the C-rate** | Buy a 400 mAh cell explicitly rated ≥2.5C **charge** (not discharge — vendors quote discharge). | Rare at this size, and vendor claims at this end of the market are not reliable. |

**Recommendation: A**, and do not buy a cell until the charge current is measured after the
rework. Measuring it is a multimeter in series with the battery lead during a charge cycle.

**Do not buy the 400 mAh cell in the parent BOM and connect it to an unmodified board.**

### Proposed parent-spec amendments

Per the working agreement, these are proposed rather than applied:

1. §4.1 and §4.2 — charger is **ETA6098**, not ETA6096.
2. §4.4 hazard 4 — replace the speculative "if the board is set to 500 mA" with the measured
   fact: R15 = 160 kΩ, ~1 A, which is 2.5C on the BOM cell. Record that the hazard is **live**,
   not hypothetical.
3. §4.1 — the battery line cannot be specified until option A/B/C is chosen.

---

## 2. Board envelope — design against these

From Waveshare's official outline drawing (`Esp32-s3-touch-lcd-1.28-002.png`). Units mm.

### Front (display side)

| Feature | Value |
|---|---|
| Display glass, outer | **Ø38.51 ±0.05** |
| Bezel step | Ø35.67 ±0.05 |
| Visible area (VA) | Ø33.40 ±0.10 |
| Overall height incl. USB-C tab | **40.36** |

### Back (PCB side)

| Feature | Value |
|---|---|
| PCB circle | **R18.75 → Ø37.50** |
| Overall height incl. tab | **39.49** |
| USB-C tab, width | 25.28 |
| USB-C tab, depth below the circle | 7.48 |
| Tab corner radius | R2.00 |
| USB-C connector, as drawn | 9.92 |

**The glass is wider than the PCB** — Ø38.51 against Ø37.50. The shell must clear the glass,
not the board, or it will bind on the display.

### Not published, and you must measure

Waveshare gives no thickness anywhere — not on the wiki, not on the product page. For a
press-fit shell you need calipers on your own board regardless, because the drawing will not
tell you connector protrusion or assembled stack height. Measure and record:

1. **Board thickness**, glass front face to the tallest component on the back.
2. **PCB thickness** alone.
3. **USB-C shell protrusion** past the tab edge, and the port centreline height above the PCB.
4. **Battery connector (MX1.25) header height**, and where it sits — it sets the minimum gap
   between the PCB back and the cell.
5. **Tallest back-side component** and its position, so the shell can pocket around it.
6. **Grip butt hole diameter** on each putter you intend to fit, and its depth. §5.1 chose a
   barbed taper precisely because this varies between manufacturers — you need the range, not
   one number.

---

## 3. Buy list

Everything except the battery is unblocked.

| # | Item | Spec | Qty | Notes |
|---|---|---|---|---|
| 1 | Board | Waveshare ESP32-S3-Touch-LCD-1.28, **Touch version** | have it | Non-touch variant exists and is a different product. §4.2 wants the ESP32-S3**R2**, 2 MB quad PSRAM, 16 MB flash. |
| 2 | **Battery** | **BLOCKED — see section 1** | 1 | 3.7 V LiPo, integrated PCM, **MX1.25 (1.25 mm) connector**. Capacity depends on the charge-current decision. |
| 3 | Magnets | N52 neodymium discs — **dimension ambiguous, see below** | 4 | §5.2: safe here because the IMU is 6-axis with no magnetometer. |
| 4 | PETG filament | For base and puck | — | §5.4 mass budget assumes PETG. |
| 5 | Calipers | Digital, 0.01 mm | 1 | Section 2 is unusable without them. |
| 6 | Multimeter | Any | 1 | Required twice: §4.4 hazard 2 (meter cell polarity before first connection) and section 1 (verify charge current after rework). |
| 7 | Soft mallet | For the §5.5 tap test | 1 | Phase 1 gate. |
| 8 | **SuperStroke Tech Port grip** | Any model with the Tech Port butt | 1 | Added 2026-09-22 from the competitive feature review. A manufacturer-designed sensor port is more rigid than a barb in rubber, so it serves as the **known-good reference mount** for the §5.5 tap test — turning a pass/fail gate into a comparison. Also a non-destructive escalation path if the barb fails, unlike §5.5's step-drill-and-collet. Needs a spare putter or a re-grip. |

### Battery requirements, once unblocked

Non-negotiable regardless of which option you pick:

- **MX1.25, 1.25 mm pitch.** Most hobby cells ship JST-PH 2.0 mm, which will not mate (§4.4.1).
- **Integrated protection circuit (PCM).** Unprotected cells are not acceptable (§4.4.5).
- **Meter the polarity before connecting.** Two-pin LiPo polarity is not standardised. If
  reversed, re-pin the housing by lifting the retention tab — do not cut and splice, and never
  cut both leads at once (§4.4.2, §4.4.3).
- **Footprint ≤ ~26 × 26 mm**, or ≤ ~20 × 32 mm, to stay inside the Ø37.50 PCB circle.

### The magnet dimension is ambiguous

Parent §4.1 says *"N52 neodymium discs, 4 × 6 mm"*, which reads either as Ø4 × 6 mm tall or
Ø6 × 4 mm thick. These need different pockets and give different holding force, and four of
them retain the whole device on a swinging putter. **Decide and amend §4.1 before cutting
pockets.** Ø6 × 3 mm is the more common stock size and the more plausible reading.

---

## 4. Mass budget, for reference

From §5.4, unchanged by anything here except the battery:

| | |
|---|---|
| Base | ~3 g |
| Board | 13 g |
| Cell | ~7 g (at 400 mAh — a 1000 mAh cell is ~20 g) |
| Shell | ~7 g |
| **Total** | **~30 g**, standing ~20 mm proud of the butt |

§5.4 notes this is mild counterbalancing and generally perceived favourably, and that because
one puck serves every putter the added mass is a controlled constant across the validation
study rather than a per-putter confound. **Option B would break that**, roughly doubling the
cell mass mid-study if it were adopted late.

---

## 5. What is still open

1. **The charge-current decision (A / B / C).** Blocks the battery order. Everything else in
   the buy list can be ordered today.
2. **Magnet dimensions.** Blocks the magnet order and the pocket geometry.
3. **Board thickness and connector protrusion.** Blocks the puck, not the base.
4. **Whether to buy the Tech Port grip now.** It is only useful once you are running the
   tap test, but it changes what that test can tell you — see §5.5 as amended.
5. **Grip butt hole diameters.** Blocks the base's barb geometry — and this is the one only you
   can answer, because it depends on which putters you intend to fit.

Items 3 and 5 are calipers, not research.
