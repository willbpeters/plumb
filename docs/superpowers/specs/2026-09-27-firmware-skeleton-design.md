# ESP-IDF firmware skeleton — design

**Date:** 2026-09-27
**Parent spec:** `2026-09-15-putting-analyzer-design.md` (§6.2, §6.3, §6.4, §14; invariants 6, 7, 8)
**Roadmap:** `docs/roadmap.md`, week 5, code track
**Status:** approved with Will, 2026-09-27

## Goal

Stand up the real product firmware: an ESP-IDF project that samples the IMU on core 0 with zero
loss, runs the UI on core 1, and enforces invariant 8 with a gate. The stroke logic is not
ported yet, so nothing drives the gate automatically; everything the logic will need to plug
into exists and is measured.

**Done means** all five of these are measured and recorded:

1. Zero lost samples over 60 s at 896.8 Hz, counted from the sensor's own sample counter
   (parent §9.2, repeated on this firmware).
2. Sample timing jitter, with the screen animating against with the gate armed. This is the first
   measurement of invariant 8's premise.
3. The screens and swipes behave as in the display bring-up (checked by Will).
4. No Wi-Fi or Bluetooth code in the linked binary, checked by the build.
5. Parent spec §6.2 amended for data-ready pacing (below).

**Out of scope:** the stroke state machine, logging to flash (§11), sleep and wake-on-motion
(§9), and the pivot, face-angle and tempo code in C.

## Toolchain

ESP-IDF **v5.5.5**, installed at `C:\Users\willi\esp\v5.5.5`. This is the version the Arduino
core runs underneath, so the bring-up's `esp_lcd` display driver has already run on it. 6.x
reworked the LCD and driver APIs; moving to it is a separate decision.

## Acquisition: data-ready, in SyncSample mode

Parent §6.2 says acquisition is "hardware-timer driven". A timer runs on the ESP32's clock and
the IMU on its own; this unit's IMU runs at **906.86 Hz** against a nominal 896.8 (bring-up
§9.3). A timer at the nominal rate would drift against the sensor and duplicate or miss about 1
sample in 100. Instead:

- **DRDY on INT2 → GPIO3.** The QMI8658 pulses INT2 when a sample is ready (datasheet rev A
  §6.3, CTRL7.DRDY_DIS = 0, FIFO in bypass). Sampling is paced by the sensor's own clock.
- **SyncSample (locking) mode**, CTRL7 bit 7. In the normal mode §6.3 requires the host to read
  during the DRDY pulse's high level, "otherwise … the new data happens during the host reading
  process and causes data mismatch". The 17-byte I²C read takes about 500 µs, far longer than
  that pulse. In SyncSample mode, reading STATUSINT locks the current sample, and reading
  through GZ_H releases the lock (§13.2.3), so no read can be torn. Over I²C, internal AHB
  clock gating must first be disabled with CTRL9 command 0x12 (§13.2.1).
- **Loss counting** from the 24-bit TIMESTAMP counter in the same burst, the method that
  measured the FIFO's 21.9% loss.

This gets a marked amendment to parent §6.2 in the same commit as the driver.

## Architecture

```
firmware/
  CMakeLists.txt            IDF project; LVGL from third_party via EXTRA_COMPONENT_DIRS
  sdkconfig.defaults        16 MB flash, QSPI PSRAM, 240 MHz, Wi-Fi/BT off
  main/                     app_main: tasks, the gate, the serial console
  components/
    plumb/                  (exists) ported algorithm
    plumb_ui/               (exists) screens; gains an IDF CMakeLists
    board/                  qmi8658.c, gc9a01.c, cst816.c, pins.h
    acq/                    acquisition task + SPSC ring buffer
```

- **`board`**: the three drivers on ESP-IDF's I²C master and `esp_lcd`. The display driver is
  the bring-up's `gc9a01.c` (`MADCTL` 0x48, inversion on). The IMU driver carries over
  `imu_stream`'s hard-won register knowledge: ADDR_AI, the 17-byte burst from 0x30, the
  24-bit counter.
- **`acq`**: one task, **pinned to core 0, highest application priority.** The GPIO3 rising-edge
  interrupt notifies the task, which does a locked read and pushes one record to a single-producer
  single-consumer ring buffer (lock-free; core 1 is the only reader). Per record: the 24-bit
  timestamp, raw int16 accel and gyro, and `esp_timer` microseconds at the DRDY edge and at
  read completion.
- **`main`**: the app task, **pinned to core 1**. It drains the ring buffer and forwards
  records over UART in `imu_stream`'s binary framing, so `analysis/tools/capture.py` reads this
  firmware unchanged. It runs LVGL (partial buffers, 2 × 240 × 60, internal DMA SRAM,
  invariant 7) and touch.
- **The gate.** An atomic flag. While armed, the app task does not call `lv_timer_handler()` and
  does not touch the I²C bus for the touch controller (parent §6.4: touch polling suspended
  between arm and follow-through). For now a console command arms it; later the ported state
  machine will. The gate does not change when that happens.

**Shared I²C bus.** The IMU and touch controller share GPIO6/7. Only the acquisition task reads
the IMU; touch reads happen in the app task only while the gate is open. The bus driver's own
lock serialises the two. Under the gate the IMU has the bus to itself, which is the point.

## Console (UART, 921600 baud)

Single characters, as in `imu_stream`: `s` start/stop streaming, `b` binary, `c` CSV, `a` arm
gate, `o` open gate, `x` cycle the result screens every 100 ms (the jitter test's rendering
load), `j` print a jitter report, `?` status. The UI result screen shows the bring-up's example
result on `r`, `n`/`p` step screens, and swipes move between screens. `1` and `d` are
acknowledged, as the only rate and read path this firmware has; `9` and `f` are refused.

*Changed during the build.* This first read `d` for "disarm gate". `tools/board.py` sends `d` to
select the direct read path, and `capture.py` has to work unchanged, so the gate is `a`/`o`.

## Measurements

- **Loss:** 60 s streamed to `capture.py` at rest. The expected count comes from the TIMESTAMP
  span, and duplicates and gaps come from its steps. Pass is zero lost and zero duplicated.
- **Jitter:** per sample, the DRDY-to-read-complete latency and the interval between successive
  DRDY edges, as a histogram (min, median, p99, max), over 30 s each in two conditions: the UI
  animating continuously, and the gate armed. Printed by `j`. No pass or fail threshold is set in
  advance (invariant 5's spirit); the numbers go in `bringup-results.md` and inform the state
  machine port.
- **Wi-Fi absent:** a build step greps the linker map for `esp_wifi` / `esp_bt` / `wifi_` symbols
  and fails the build if any are present.

## Decisions made in the build

Recorded in the plan (`docs/superpowers/plans/2026-09-27-firmware-skeleton.md`); results in
`docs/bringup-results.md`, "ESP-IDF firmware skeleton".

- **LVGL's Kconfig is ignored** (`LV_KCONFIG_IGNORE`), so the device compiles LVGL from exactly
  the `lv_conf.h` the host bench uses, one configuration in both places.
- **The build contains only what `main` pulls in** (`set(COMPONENTS main)`), so `esp_wifi` and
  `bt` are not compiled at all. The map check matches radio *archives*; matching symbol names
  failed on ROM addresses and a clock helper that every build links.
- **Flash runs at 80 MHz QIO, not the 120 MHz of parent §6.3.** On the S3, 120 MHz needs
  `SPI_FLASH_HPM_ON` and a flash part that supports it, and the Kconfig help warns of random
  crashes after a ~20 °C temperature change in some modes. Nothing here depends on flash speed.
  **This disagrees with parent §6.3 and is left for Will.**
- **`LV_MEMCPY_MEMSET_STD` (parent §6.3) is an LVGL 8 name.** Its LVGL 9 equivalent belongs in
  the shared `lv_conf.h`, so it is left for a measured change.
- **Register facts rev A does not give:** INT2 needs `CTRL1` bit 4, and the CTRL9 handshake
  takes 3253 µs. Also, a reset in the middle of a read leaves the IMU holding SDA, and nine
  clocks and a STOP free it. All three are measured in `bringup-results.md`.

## Done-means, as measured

1. **Loss:** 0 of 54,653 in 60 s at rest. — met.
2. **Jitter:** recorded, animating against armed (the tables in `bringup-results.md`). Rendering
   pushes read latency past the sample period and, together with streaming, lost 0.72%; the gate
   removes both. — met.
3. **Screens and swipes:** not yet checked by eye on this firmware. — **open, needs Will.**
4. **No radio code linked:** checked on every link, and the check shown to fail a canary. — met.
5. **Parent §6.2 amended** for data-ready pacing. — met.
