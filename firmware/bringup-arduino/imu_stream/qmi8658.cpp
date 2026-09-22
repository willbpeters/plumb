// Register-level driver for the QMI8658 6-axis IMU. See qmi8658.h.
//
// Register map, bit fields and ODR/FIFO behaviour are taken from:
//   [DS-A]  QMI8658C Datasheet, QST Corporation, Document# 13-52-27, Rev A
//           (2022). https://www.qstcorp.com/upload/pdf/202210/13-52-27%20QMI8658C%20Datasheet%20Rev%20A%20(1).pdf
//   [DS-09] QMI8658C Datasheet, QST Corporation, Rev 0.9 (2022), an earlier
//           revision of the same document, used to cross-check DS-A.
//           https://qstcorp.com/upload/pdf/202202/QMI8658C%20datasheet%20rev%200.9.pdf
//   [SL]    lewisxhe/SensorLib, src/SensorQMI8658.hpp (open-source Arduino
//           driver in circulation for this exact part), used as the second
//           independent source the task asked for -- its register
//           #defines, full-scale enums, FIFO_CTRL bit layout and CTRL9
//           handshake sequence all agree with DS-A/DS-09.
//           https://github.com/lewisxhe/SensorLib/blob/master/src/SensorQMI8658.hpp
//
// DS-A and DS-09 agree on every register address and bit field used here.
// Where they differ (gFS setting 111 is "NA" in DS-A, "+-2048 dps" in
// DS-09), it doesn't matter -- this driver never uses that setting.

#include "qmi8658.h"

#include <Wire.h>

namespace {

constexpr uint8_t kAddress = 0x6B;      // confirmed on hardware by smoke_test
constexpr uint8_t kRegWhoAmI = 0x00;    // confirmed: returns 0x05
constexpr uint8_t kWhoAmIValue = 0x05;

// --- Register addresses --------------------------------------------------
// All confirmed identical in DS-A Table 19 (UI Register Overview) / Table 22
// (Configuration Registers) / Table 23 (FIFO Registers) / Table 24 (Status
// Registers), DS-09 (same tables), and SL's QMI8658_REG_* #defines.

constexpr uint8_t kRegCtrl1 = 0x02;         // Serial interface / FIFO INT pin select
constexpr uint8_t kRegCtrl2 = 0x03;         // Accelerometer: full scale + ODR
constexpr uint8_t kRegCtrl3 = 0x04;         // Gyroscope: full scale + ODR
constexpr uint8_t kRegCtrl5 = 0x06;         // Low-pass filter enables and modes
constexpr uint8_t kRegCtrl7 = 0x08;         // Enable sensors (aEN/gEN)
constexpr uint8_t kRegCtrl9 = 0x0A;         // Host command register (CTRL9 protocol)
constexpr uint8_t kRegFifoWtmTh = 0x13;     // FIFO watermark, in samples
constexpr uint8_t kRegFifoCtrl = 0x14;      // FIFO size / mode / read-mode bit
constexpr uint8_t kRegFifoSmplCntLsb = 0x15;// FIFO sample count, LSB (unit: word)
constexpr uint8_t kRegFifoStatus = 0x16;    // FIFO status + sample count MSBs
constexpr uint8_t kRegFifoData = 0x17;      // FIFO data pop register
constexpr uint8_t kRegStatusInt = 0x2D;     // STATUSINT: CTRL9 CmdDone flag
constexpr uint8_t kRegStatus0 = 0x2E;       // STATUS0: per-sensor data available
constexpr uint8_t kRegTimestampLow = 0x30;  // TIMESTAMP_L..H at 0x30..0x32
constexpr uint8_t kRegResetResult = 0x4D;   // reads 0x80 after a successful reset
constexpr uint8_t kRegReset = 0x60;         // soft reset (write-only)

// Soft reset. DS-A Table 27: "Write 0xB0 to this register from any modes, will
// trigger the sensor reset process immediately. The register 0x4D will equals
// to 0x80 if there is a successful reset."
constexpr uint8_t kResetCommand = 0xB0;
constexpr uint8_t kResetResultOk = 0x80;

// --- STATUS0 bits (0x2E), DS-A Table 24 ----------------------------------
// bits 7:2 reserved, bit 1 gDA, bit 0 aDA: "0: No updates since last read.
// 1: New data available." This is the direct read path's data-ready signal.
//
// Note what STATUS0 does NOT have: an over-run bit. Table 19 calls the
// register "Output Data Over Run and Data Availability", but the bit table in
// Table 24 defines only the two availability flags. So the direct path has no
// hardware loss flag at all -- a sample overwritten before the host read it is
// only detectable through the TIMESTAMP counter, which is the other reason
// every direct read carries it.
constexpr uint8_t kStatus0GyroDataBit = 0x02;  // bit 0, aDA, is not consumed --
                                               // see dataReady()

// --- CTRL9 host commands (DS-A/DS-09 Table 28, SL's writeCommand()) ------
constexpr uint8_t kCtrl9CmdAck = 0x00;      // end the CTRL9 handshake
constexpr uint8_t kCtrl9CmdRstFifo = 0x04;  // reset FIFO data/count/flags
constexpr uint8_t kCtrl9CmdReqFifo = 0x05;  // enter FIFO read mode

constexpr uint8_t kStatusIntCmdDoneBit = 0x80;  // STATUSINT bit 7

// --- FIFO_STATUS bits (0x16) ---------------------------------------------
// bit 7 (FIFO_FULL) isn't consumed separately -- dataReady()/drain() key off
// FIFO_WTM and FIFO_NOT_EMPTY, and a full FIFO always implies watermark hit
// (watermark is configured <= FIFO size, DS-A/DS-09 §8.5).
constexpr uint8_t kFifoStatusWtmBit = 0x40;       // bit 6: watermark reached
constexpr uint8_t kFifoStatusOvflowBit = 0x20;    // bit 5: overflow happened
constexpr uint8_t kFifoStatusNotEmptyBit = 0x10;  // bit 4
constexpr uint8_t kFifoStatusCountMsbMask = 0x03; // bits 1:0: sample count MSBs

// --- CTRL1 (0x02) bit 6, ADDR_AI -----------------------------------------
// Burst reads either walk the register address or sit on it, and which one you
// get is a configuration bit this driver never wrote. DS-A section 16.1:
//
//   "If ADDR_AI = 0, the register address will not increase... If ADDR_AI = 1,
//    the register address will automatically increase... Note that the default
//    value of ADDR_AI is 0, so it is recommended to set it to 1 from beginning,
//    in case of burst read/write is required."
//
// Table 19 confirms CTRL1's reset value is 0b00100000, so ADDR_AI is indeed 0
// out of reset. The two read paths need opposite settings, which is why this
// is configured per path rather than once in begin():
//
//   FIFO path   ADDR_AI = 0. Every read of FIFO_DATA pops the next byte
//               (DS-A section 8.8), so a burst that does NOT advance the
//               address is exactly the documented way to drain it. With
//               ADDR_AI = 1 the same burst would walk off 0x17 into the
//               reserved registers above it.
//   Direct path ADDR_AI = 1. The output registers are twelve consecutive
//               addresses; without auto-increment a 12-byte burst returns
//               twelve copies of AX_L, which would decode into plausible
//               numbers and be wrong in silence -- all six axes equal, and
//               a standard deviation that means nothing.
//
// Bit 5 (BE, byte order) is deliberately left alone. Table 22 gives its reset
// value as 1, which would mean big-endian, but the FIFO path's measured
// resting accelerometer magnitude -- 2019 counts against 2048 expected for 1 g
// at +-16 g full scale -- proves the host interface hands over the low byte
// first, as this driver's unpacking assumes. Something in that table is
// inconsistent with the part; read-modify-write means we depend on neither
// reading of it.
constexpr uint8_t kCtrl1AddrAiBit = 0x40;

// --- Accelerometer full scale, CTRL2 bits 6:4 (aFS<2:0>) -----------------
// 000=+-2g 001=+-4g 010=+-8g 011=+-16g. Identical in DS-A, DS-09, and SL
// (ACC_RANGE_16G is the 4th enum value, encoded 3 << 4).
constexpr uint8_t kAccelFs16g = 0x03;

// --- Gyroscope full scale, CTRL3 bits 6:4 (gFS<2:0>) ----------------------
// DS-A and DS-09 both list only doubling steps: 000=+-16, 001=+-32, 010=+-64,
// 011=+-128, 100=+-256, 101=+-512, 110=+-1024 dps. SL's GYR_RANGE_* enum
// independently confirms the same seven values with no 250 dps member.
//
// There is no +-250 dps setting on this part -- confirmed by two
// independent sources, not an omission. +-256 dps (setting 100) is the
// nearest available range to the requested +-250 dps. See the task report:
// this is a deliberate "nearest documented value" choice, not a guess.
constexpr uint8_t kGyroFs256dps = 0x04;  // +-256 dps

// --- ODR encoding, CTRL2 bits 3:0 (aODR<3:0>) / CTRL3 bits 3:0 (gODR<3:0>)
// DS-A/DS-09 Table 22. When accel and gyro are both enabled and both feed
// the FIFO, they must be programmed to the SAME setting value (DS-A/DS-09
// §8.2: "the sensors must be set at the same Output Data Rate"); the
// resulting physical rate is then the CTRL2 table's "(6DOF)" column, which
// equals the CTRL3 gyro-ODR column at that same setting. These are the
// datasheet's actual numbers, not rounded (note 13 in Table 22: the QMI8658
// gyro ODR steps are derived from the gyro's natural frequency, not round
// numbers).
//
//   setting  ODR (Hz), 6DOF / gyro column
//   0000     7174.4   <- maximum
//   0001     3587.2
//   0010     1793.6
//   0011      896.8   <- stroke rate (see below)
//   0100      448.4
//   0101      224.2
//   0110      112.1
//   0111       56.05
//   1000       28.025
//
// Stroke rate is 896.8 Hz, the step ABOVE the spec's original 500 Hz rather
// than the nearer 448.4 below it. 500 Hz is not on this table at all, so the
// choice was between the two neighbours, and it was made on measured evidence:
// the synthetic harness found tempo ratio to be the binding accuracy
// constraint, with far less headroom than face angle, and tempo error scales
// directly with sample resolution. Doubling the rate roughly halves it and
// quarters the integration error.
//
// The cost is current draw, which is NOT yet measured on this board -- it has
// battery voltage sense but no current sense, so it needs an inline power
// meter. It is bounded by duty cycle: the device sleeps between strokes
// (spec 9), so the higher rate runs only during the ~1.5 s a stroke is being
// measured, not continuously. If the section 4.3 power budget later proves
// tight, this is a one-constant change back.
constexpr uint8_t kOdrSettingStroke = 0x03;  // -> 896.8 Hz
constexpr uint8_t kOdrSettingMax = 0x00;     // -> 7174.4 Hz, the maximum

constexpr float kNominalHzStroke = 896.8f;
constexpr float kNominalHzMax = 7174.4f;

// --- CTRL5 (0x06): low-pass filters (DS-09 Table 26) ----------------------
// bit 4 gLPF_EN, bits 6:5 gLPF_MODE, bit 0 aLPF_EN, bits 2:1 aLPF_MODE.
// Both filters default to DISABLED, which is how this driver originally left
// them -- CTRL5 was never written at all. Unfiltered, the gyro runs at roughly
// ODR/2 of bandwidth, and measured resting noise was 2.17 dps worst-axis
// against a datasheet-typical near 0.21.
//
// gLPF_MODE bandwidths, as a percentage of ODR:
//   00  2.66%   01  3.63%   10  5.39%   11  13.37%
//
// Gyro filter ON, mode 00. At 896.8 Hz that is 23.9 Hz of bandwidth. A putting
// stroke is a ~1.5 s motion whose content sits well under 20 Hz, so this costs
// no signal, and noise scales with the square root of bandwidth: 448 Hz down to
// 23.9 Hz is a 4.3x reduction. It also attenuates the environmental vibration
// a desk transmits, which lives above 24 Hz.
constexpr uint8_t kCtrl5StrokeFilters = 0x10;  // gLPF_EN=1, mode 00; aLPF off

// Accelerometer filter deliberately OFF, at both rates.
//
// The accelerometer's job here is impact DETECTION, not measurement (parent
// spec 6.4). Impact is a ~4 ms impulse and the algorithm keys off its leading
// edge; a 23.9 Hz filter has a time constant far longer than the event and
// would smear exactly the edge that has to stay sharp. Saturation is already
// accepted for the same reason -- a clipped spike is a cleaner edge than an
// unclipped one.

// Tap test (spec 5.5) looks for mount resonance ABOVE 500 Hz, so every filter
// must be off or the answer is filtered away before it is measured.
constexpr uint8_t kCtrl5TapFilters = 0x00;

// --- CTRL7: enable sensors (DS-A/DS-09 Table 22) --------------------------
constexpr uint8_t kCtrl7EnableAccelGyro = 0x03;  // bit1 gEN=1, bit0 aEN=1

// --- FIFO configuration (DS-A/DS-09 Table 23, section 8.2/8.3) -----------
// FIFO_SIZE[1:0] (bits 3:2): 00=16, 01=32, 10=64, 11=128 samples. Use the
// largest size the part supports -- this is a capacity choice (headroom
// against jitter), not a tuned algorithmic threshold.
constexpr uint8_t kFifoSizeSetting128 = 0x03;
// FIFO_MODE[1:0] (bits 1:0): 00=Bypass, 01='FIFO' (stop filling & flag
// overflow once full), 10='Stream' (overwrite oldest once full). 'FIFO'
// mode is used so "overflow happened" has one unambiguous meaning matching
// the plan's acceptance criteria (dropped/overflow must be zero).
constexpr uint8_t kFifoModeFifo = 0x01;
constexpr uint8_t kFifoCtrlConfig =
    (uint8_t)((kFifoSizeSetting128 << 2) | kFifoModeFifo);  // 0x0D, rd_mode(bit7)=0
// Bypass, for the direct read path. Not merely "don't read the FIFO" -- the
// point of the experiment is that the FIFO is not in the signal path at all,
// so it cannot be filling, flagging, or being reset behind the measurement.
constexpr uint8_t kFifoModeBypass = 0x00;
constexpr uint8_t kFifoCtrlBypass =
    (uint8_t)((kFifoSizeSetting128 << 2) | kFifoModeBypass);

// FIFO watermark, in samples (FIFO_WTM_TH, 0x13). Half of the 128-sample
// FIFO: an I/O batching size (I2C burst efficiency vs. latency), not a
// detection threshold -- invariant 5 is about not guessing values that
// shape what the algorithm reports, which doesn't apply to a FIFO transfer
// size. Revisit if Task 8's measured-ODR run shows drops or overflows.
constexpr uint8_t kFifoWatermarkSamples = 64;

constexpr uint8_t kBytesPerSample = 12;  // ax,ay,az,gx,gy,gz x int16 (DS-A §8.9)

// Largest drain this driver will service in one call, and the staging buffer
// it needs. 128 is the part's maximum FIFO depth, so a single drain can always
// empty a full FIFO.
constexpr uint8_t kMaxDrainSamples = 128;

// Bytes per I2C transaction. The ESP32 Arduino Wire buffer defaults to 128
// bytes (I2C_BUFFER_LENGTH), so 120 is the largest multiple of 12 that fits
// without depending on Wire::setBufferSize. That turns a 64-sample drain from
// 64 transactions into 7.
constexpr uint8_t kMaxChunkBytes = 120;

// Bound on CTRL9 handshake polling. Each iteration is one 1-byte I2C
// register read (roughly tens of microseconds at 400 kHz); this bounds
// drain()'s worst case to a small, fixed amount of time rather than
// blocking forever if the device stops acking.
constexpr int kCtrl9PollIterations = 50;

// --- Turn-on timing (DS-A Tables 7 and 8) --------------------------------
// Two different delays, and the difference matters. An earlier note in
// docs/bringup-results.md recorded this as a single "150 ms turn-on time";
// the datasheet has two rows.
//
// System Turn On Time = 15 ms, in both Table 7 (accel) and Table 8 (gyro),
// with note 7/8: "System Turn-On Time defines the initialization duration...
// it starts from about 200us later than the release of POR or the Software
// Reset." Section 3.3.1 says what it is for: "Normally it takes within about
// 15ms... for QMI8658C to finish the Initialization and during which, there
// should be no write/configuration to QMI8658C, to prevent possible
// interference and failure."
//
// That is the intermittent-init defect exactly: begin() ran immediately after
// Wire.begin(), inside the window the datasheet says not to write in, and the
// first boot after flashing failed the config read-back while a reset cleared
// it. Reset-and-wait makes the initial state the same on every boot instead of
// depending on whether the sensor happened to be power-cycled with the MCU.
constexpr uint32_t kSystemTurnOnMs = 15;

// Gyro Turn On Time = 150 ms + 3/ODR (Table 8), measured from enabling the
// gyro. This one is not about configuration succeeding; it is about the data
// being worth anything. Sampling the gyro inside its 150 ms start-up would put
// a settling transient into the noise floor measurement, which is the single
// most consequential number this instrument produces.
constexpr uint32_t kGyroTurnOnMs = 150;

// --- Direct read path block (DS-A Tables 24-25) --------------------------
// One burst covers 0x30..0x40: TIMESTAMP_L/M/H, TEMP_L/H, then AX_L..GZ_H.
// Seventeen bytes instead of twelve, and the five extra are what buy the
// measurement its integrity:
//
//   The timestamp says WHICH sample this is. STATUS0 is polled, and between
//   the poll and the read a new sample can land -- so without an identity the
//   host cannot tell a fresh sample from the same one read twice. A duplicate
//   does not look like corruption; it looks like quiet. It would deflate the
//   very standard deviation being measured, in the flattering direction.
//
// Reading the timestamp in a separate transaction would reintroduce the
// ambiguity it exists to remove, so it shares the burst with the data.
constexpr uint8_t kRegGyroZHigh = 0x40;
constexpr uint8_t kDirectBlockBytes =
    (uint8_t)(kRegGyroZHigh - kRegTimestampLow + 1);  // 17
constexpr uint8_t kDirectSampleOffset = 5;  // AX_L, past timestamp(3) + temp(2)

// TIMESTAMP is a 24-bit circular counter (DS-A Table 24): "Count incremented
// by one for each sample (x, y, z data set) from sensor with highest ODR
// (circular register 0x0-0xFFFFFF)".
//
// MEASURED ON HARDWARE, 2026-09-21, and not in the datasheet: the counter
// advances ONLY while the FIFO is bypassed. Polled at 20 ms intervals with the
// FIFO in FIFO mode it read 84 eight times running; in bypass mode, over the
// same interval, it advanced 19 counts a step. It counts samples written to the
// OUTPUT REGISTERS, and in FIFO mode the samples go to the FIFO instead.
//
// The consequence is not a small one. It means the FIFO path has no way to
// count what the sensor produced, and therefore no way to measure its own
// sample loss from the inside: the part offers a latched overflow flag that
// says loss happened and nothing that says how much. That is why DrainResult
// carries `sensorCounted` -- an instrument that cannot measure something has
// to report that, not report a zero.
constexpr uint32_t kTimestampMask = 0x00FFFFFF;

Rate gCurrentRate = Rate::Stroke;
ReadPath gCurrentPath = ReadPath::Fifo;

// Sensor TIMESTAMP at the last resetSampleClock(), and the timestamp of the
// most recent sample handed to a caller. The second is how a re-read of an
// unchanged sample is rejected rather than reported as data.
uint32_t gTimestampBase = 0;
uint32_t gLastTimestamp = 0;

bool writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(kAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readReg(uint8_t reg, uint8_t& out) {
  Wire.beginTransmission(kAddress);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)kAddress, 1) != 1) return false;
  out = Wire.read();
  return true;
}

// Write, then read back and compare. A mistyped register address or a
// misread encoding table is the most likely failure in this whole module, and
// it is silent without this check -- the device simply runs at a rate nobody
// intended.
bool writeVerified(uint8_t reg, uint8_t value) {
  if (!writeReg(reg, value)) return false;
  uint8_t back = 0;
  if (!readReg(reg, back)) return false;
  return back == value;
}

// Burst-read `n` consecutive bytes from FIFO_DATA. Every read of FIFO_DATA
// pops the next byte (DS-A §8.8), so this is a genuine burst read: one
// address phase, then `n` bytes in one I2C transaction.
bool burstReadFifoData(uint8_t* buf, uint8_t n) {
  Wire.beginTransmission(kAddress);
  Wire.write(kRegFifoData);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)kAddress, (int)n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

// Burst-read `n` consecutive registers starting at `reg`. Requires ADDR_AI=1,
// so this is only valid on the direct path -- see the ADDR_AI note above.
bool burstReadRegs(uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire.beginTransmission(kAddress);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)kAddress, (int)n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

// Set or clear CTRL1.ADDR_AI, leaving every other bit of CTRL1 as the part
// has it -- read-modify-write rather than a whole-byte constant, so this
// cannot silently change the byte order bit or the FIFO interrupt mapping.
bool setAddrAutoIncrement(bool enable) {
  uint8_t ctrl1 = 0;
  if (!readReg(kRegCtrl1, ctrl1)) return false;
  const uint8_t wanted = enable ? (uint8_t)(ctrl1 | kCtrl1AddrAiBit)
                                : (uint8_t)(ctrl1 & (uint8_t)~kCtrl1AddrAiBit);
  if (wanted == ctrl1) return true;
  return writeVerified(kRegCtrl1, wanted);
}

// Read the 24-bit TIMESTAMP counter with single-register reads, so it works on
// either read path regardless of ADDR_AI.
//
// Three separate reads can straddle an increment: read L as 0xFF, the counter
// increments, and M comes back already carried, producing a value 256 too
// high. Re-reading L detects any increment during the window -- not just the
// carries -- and a retry costs four register reads on a counter that is read
// once per drain. Without this, roughly one drain in four thousand would
// report a phantom 256-sample gap, which is precisely the kind of number that
// gets believed.
bool readTimestamp(uint32_t& out) {
  for (int attempt = 0; attempt < 3; attempt++) {
    uint8_t low = 0, mid = 0, high = 0, lowAgain = 0;
    if (!readReg(kRegTimestampLow, low)) return false;
    if (!readReg((uint8_t)(kRegTimestampLow + 1), mid)) return false;
    if (!readReg((uint8_t)(kRegTimestampLow + 2), high)) return false;
    if (!readReg(kRegTimestampLow, lowAgain)) return false;
    if (lowAgain == low) {
      out = (uint32_t)low | ((uint32_t)mid << 8) | ((uint32_t)high << 16);
      return true;
    }
  }
  return false;
}

// Samples produced since the last resetSampleClock(). Unsigned arithmetic
// masked to 24 bits, so the counter's wrap is handled rather than noticed.
uint32_t producedSince(uint32_t timestamp) {
  return (timestamp - gTimestampBase) & kTimestampMask;
}

// Little-endian int16 pairs, in the order the part emits them: AX, AY, AZ,
// GX, GY, GZ. The same layout serves both paths -- DS-A section 8.9 gives it
// for the FIFO, Table 25 for the output registers.
void unpackSample(const uint8_t* b, Sample& s) {
  s.ax = (int16_t)((uint16_t)b[1] << 8 | b[0]);
  s.ay = (int16_t)((uint16_t)b[3] << 8 | b[2]);
  s.az = (int16_t)((uint16_t)b[5] << 8 | b[4]);
  s.gx = (int16_t)((uint16_t)b[7] << 8 | b[6]);
  s.gy = (int16_t)((uint16_t)b[9] << 8 | b[8]);
  s.gz = (int16_t)((uint16_t)b[11] << 8 | b[10]);
}

// Send a CTRL9 host command and run the documented handshake (DS-A/DS-09
// §5.10.4 "WCtrl9" / §5.10.5 "Ctrl9R" protocol description, independently
// confirmed against SL's writeCommand()): write the command byte to CTRL9,
// poll STATUSINT for CmdDone (bit 7), then write CTRL_CMD_ACK to end the
// protocol. CTRL9 is an ordinary rw register from the bus's point of view,
// so writeVerified's read-back-equals-what-was-written check would pass
// trivially without ever proving the command executed -- STATUSINT.CmdDone
// is the only signal that actually confirms success, which is why this
// command path exists instead of routing CTRL9 writes through
// writeVerified. Bounded to kCtrl9PollIterations so callers that must not
// block (drain()) return promptly either way.
bool sendCtrl9Command(uint8_t command) {
  if (!writeReg(kRegCtrl9, command)) return false;

  bool done = false;
  for (int i = 0; i < kCtrl9PollIterations; i++) {
    uint8_t status = 0;
    if (!readReg(kRegStatusInt, status)) return false;
    if (status & kStatusIntCmdDoneBit) {
      done = true;
      break;
    }
  }
  if (!done) return false;

  if (!writeReg(kRegCtrl9, kCtrl9CmdAck)) return false;
  // Best-effort: confirm CmdDone clears. Not required for the correctness
  // of the command that was just issued, so failure here isn't fatal.
  for (int i = 0; i < kCtrl9PollIterations; i++) {
    uint8_t status = 0;
    if (!readReg(kRegStatusInt, status)) break;
    if (!(status & kStatusIntCmdDoneBit)) break;
  }
  return true;
}

uint8_t odrSettingFor(Rate rate) {
  return rate == Rate::Max ? kOdrSettingMax : kOdrSettingStroke;
}

// Programs CTRL2 (accel FS+ODR) and CTRL3 (gyro FS+ODR) together -- they
// must carry the same ODR setting for 6DOF FIFO output (DS-A/DS-09 §8.2).
bool configureOdr(Rate rate) {
  const uint8_t odr = odrSettingFor(rate);
  const uint8_t ctrl2 = (uint8_t)((kAccelFs16g << 4) | odr);
  const uint8_t ctrl3 = (uint8_t)((kGyroFs256dps << 4) | odr);
  if (!writeVerified(kRegCtrl2, ctrl2)) return false;
  if (!writeVerified(kRegCtrl3, ctrl3)) return false;

  // Filters follow the rate, because the two rates exist for different jobs.
  // Stroke rate measures a slow motion and wants the noise gone; tap rate is
  // hunting a resonance above 500 Hz and must see the whole band.
  const uint8_t ctrl5 =
      (rate == Rate::Max) ? kCtrl5TapFilters : kCtrl5StrokeFilters;
  if (!writeVerified(kRegCtrl5, ctrl5)) return false;
  return true;
}

// Everything that differs between the two read paths, in one place: burst
// address behaviour, and whether the FIFO is in the signal path at all.
bool configureReadPath(ReadPath path) {
  if (!setAddrAutoIncrement(path == ReadPath::Direct)) return false;

  if (path == ReadPath::Direct) {
    // Bypass. No watermark to reach, no read mode to enter, nothing to reset.
    return writeVerified(kRegFifoCtrl, kFifoCtrlBypass);
  }

  if (!writeVerified(kRegFifoWtmTh, kFifoWatermarkSamples)) return false;
  if (!writeVerified(kRegFifoCtrl, kFifoCtrlConfig)) return false;
  return sendCtrl9Command(kCtrl9CmdRstFifo);
}

}  // namespace

namespace qmi8658 {

bool begin(Rate rate, ReadPath path) {
  // Soft reset first, so the part starts from the same state on every boot.
  // The MCU resets without power-cycling the sensor, which left begin()
  // configuring a part in whatever state the previous run had put it in --
  // the likely other half of the intermittent first-boot failure. A write to
  // the reset register cannot be read back to verify it (it is write-only and
  // self-clearing), so it is confirmed the way the datasheet says to confirm
  // it: 0x4D reads 0x80 after a successful reset.
  //
  // 0x4D is a general purpose register that "could be overwritten after later
  // operations, like enabling the sensor(s)" (DS-A 7.4), so it is checked
  // here, before anything else is configured, and never again.
  if (!writeReg(kRegReset, kResetCommand)) return false;
  delay(kSystemTurnOnMs);

  uint8_t resetResult = 0;
  if (!readReg(kRegResetResult, resetResult)) return false;
  if (resetResult != kResetResultOk) return false;

  uint8_t who = 0;
  if (!readReg(kRegWhoAmI, who)) return false;
  if (who != kWhoAmIValue) return false;

  if (!configureOdr(rate)) return false;
  if (!writeVerified(kRegCtrl7, kCtrl7EnableAccelGyro)) return false;
  // Gyro start-up. Everything after this point may be read; nothing sampled
  // before it should be believed.
  delay(kGyroTurnOnMs);

  gCurrentPath = path;
  if (!configureReadPath(path)) return false;

  gCurrentRate = rate;
  return resetSampleClock();
}

bool setRate(Rate rate) {
  if (!configureOdr(rate)) return false;
  // The gyro is being restarted at a new rate and its output is not
  // trustworthy until it has settled -- the same 150 ms as begin().
  delay(kGyroTurnOnMs);
  // Samples already queued at the old rate would corrupt the time base --
  // start clean at the new rate.
  if (!configureReadPath(gCurrentPath)) return false;
  gCurrentRate = rate;
  return resetSampleClock();
}

bool setReadPath(ReadPath path) {
  const ReadPath previous = gCurrentPath;
  gCurrentPath = path;
  if (!configureReadPath(path)) {
    gCurrentPath = previous;
    return false;
  }
  return resetSampleClock();
}

bool resetSampleClock() {
  if (gCurrentPath == ReadPath::Fifo) {
    // No sample clock to re-base -- the counter is frozen while the FIFO is
    // enabled. Anything already queued was produced before the capture began
    // and would be counted against a window it does not belong to, so clear
    // it and leave the base at zero.
    gTimestampBase = 0;
    gLastTimestamp = 0;
    return sendCtrl9Command(kCtrl9CmdRstFifo);
  }

  uint32_t timestamp = 0;
  if (!readTimestamp(timestamp)) return false;
  gTimestampBase = timestamp;
  gLastTimestamp = timestamp;
  return true;
}

bool dataReady() {
  if (gCurrentPath == ReadPath::Direct) {
    uint8_t status = 0;
    if (!readReg(kRegStatus0, status)) return false;
    // gDA only. Accelerometer and gyroscope run at a common ODR -- they must,
    // to share the FIFO (DS-A 8.2) -- so one sample event sets both flags.
    // Testing the gyro's is testing the sample's, and the gyro is the channel
    // every number this instrument exists to measure comes from.
    return (status & kStatus0GyroDataBit) != 0;
  }

  uint8_t status = 0;
  if (!readReg(kRegFifoStatus, status)) return false;
  return (status & kFifoStatusWtmBit) != 0;
}

// Direct path: one poll, one sample, and no buffer between the sensor and the
// bus.
//
// The output registers hold only the newest sample, so this keeps up only
// while the whole per-sample cycle fits inside one ODR period. At stroke rate
// that is 1.115 ms against roughly 500 us of bus time -- a 3-byte STATUS0 poll
// and an 18-byte block read at 400 kHz -- so it fits with room to spare. At
// maximum rate the period is 139 us and it cannot fit at all: the tap test
// keeps the FIFO, and this is a stroke-rate path. Nothing here enforces that.
// The timestamp reports it.
DrainResult drainDirect(Sample* out) {
  DrainResult result{0, false, 0, true};

  uint8_t block[kDirectBlockBytes];
  if (!burstReadRegs(kRegTimestampLow, block, kDirectBlockBytes)) return result;

  const uint32_t timestamp = (uint32_t)block[0] | ((uint32_t)block[1] << 8) |
                             ((uint32_t)block[2] << 16);
  result.produced = producedSince(timestamp);

  // Same sample as last time: STATUS0 said "new data" but the data has not
  // changed, so this is a re-read, not a measurement. Reporting it as a sample
  // would be reporting the instrument's own polling as sensor behaviour, and
  // duplicated samples pull a standard deviation DOWN -- the direction that
  // would look like success.
  if (timestamp == gLastTimestamp) return result;
  gLastTimestamp = timestamp;

  unpackSample(block + kDirectSampleOffset, out[0]);
  result.count = 1;
  return result;
}

DrainResult drain(Sample* out, uint8_t capacity) {
  DrainResult result{0, false, 0, false};
  if (out == nullptr || capacity == 0) return result;

  if (gCurrentPath == ReadPath::Direct) return drainDirect(out);

  // No produced count on this path. The obvious fix for "dropped: 0 alongside
  // overflow on 224 of 225 batches" was to number samples from the sensor's
  // TIMESTAMP counter instead of from the delivered count -- but the counter
  // is frozen while the FIFO is enabled (see the note on kTimestampMask), so
  // there is nothing to number them from. `sensorCounted` stays false, and the
  // caller numbers this path's samples by what it has been handed, which is
  // all anyone can honestly claim about them.

  uint8_t status = 0;
  if (!readReg(kRegFifoStatus, status)) return result;

  const bool overflowed = (status & kFifoStatusOvflowBit) != 0;

  if (!(status & kFifoStatusNotEmptyBit)) {
    // Nothing to read. Still clear a latched overflow via RST_FIFO so it
    // can't wedge a future drain into reporting a stale overflow forever --
    // FIFO_OVFLOW has no documented self-clear or "write 1 to clear" bit;
    // CTRL_CMD_RST_FIFO is the only documented way to force it (and the
    // sample count) back to a known state (DS-A/DS-09 §5.10.6.2).
    if (overflowed) sendCtrl9Command(kCtrl9CmdRstFifo);
    result.overflow = overflowed;
    return result;
  }

  uint8_t countLsb = 0;
  if (!readReg(kRegFifoSmplCntLsb, countLsb)) {
    result.overflow = overflowed;
    return result;
  }
  // FIFO_Sample_Count (bytes) = 2 * (msb[1:0] * 256 + lsb) (DS-A/DS-09 §8.4).
  const uint16_t availableBytes =
      (uint16_t)(2 * (((status & kFifoStatusCountMsbMask) << 8) | countLsb));
  const uint16_t availableSamples = availableBytes / kBytesPerSample;

  const uint16_t limit =
      (capacity < kMaxDrainSamples) ? capacity : kMaxDrainSamples;
  const uint8_t samplesToRead =
      (availableSamples > limit) ? (uint8_t)limit : (uint8_t)availableSamples;
  if (samplesToRead == 0) {
    result.overflow = overflowed;
    return result;
  }

  // Enter FIFO read mode (DS-A/DS-09 §8.7/§8.8): a CTRL9 command, not a
  // direct register write -- FIFO_CTRL.FIFO_RD_MODE "is automatically set
  // by using a CTRL9 command."
  if (!sendCtrl9Command(kCtrl9CmdReqFifo)) {
    result.overflow = overflowed;
    return result;
  }

  // Read the whole batch in as few I2C transactions as the Wire buffer allows,
  // then unpack. This is what parent spec 6.4 means by "FIFO batching is
  // mandatory"; the previous version issued one complete transaction --
  // address, register, repeated start, read, stop -- per 12-byte SAMPLE, so a
  // 64-sample drain cost 64 transactions.
  //
  // It matters more than ordinary I2C overhead because the FIFO does not fill
  // while the part is in read mode (DS-A/DS-09 §8.7/§8.8 -- exiting read mode
  // is what lets new samples "resume filling"). Every microsecond spent
  // draining is a microsecond of samples the sensor never stores. Measured
  // before this change: 652 Hz delivered against 896.8 Hz produced, a 27% loss,
  // with FIFO_OVFLOW set on 208 of 209 consecutive batches.
  uint8_t staging[kMaxDrainSamples * kBytesPerSample];
  const uint16_t wanted = (uint16_t)samplesToRead * kBytesPerSample;
  uint16_t got = 0;
  while (got < wanted) {
    const uint16_t remaining = (uint16_t)(wanted - got);
    const uint8_t chunk =
        (uint8_t)(remaining > kMaxChunkBytes ? kMaxChunkBytes : remaining);
    if (!burstReadFifoData(staging + got, chunk)) break;
    got = (uint16_t)(got + chunk);
  }

  const uint8_t samplesRead = (uint8_t)(got / kBytesPerSample);
  for (uint8_t i = 0; i < samplesRead; i++) {
    unpackSample(staging + (uint16_t)i * kBytesPerSample, out[i]);
  }
  result.count = samplesRead;

  // Exit FIFO read mode so new samples resume filling the FIFO (§8.7/§8.8
  // step 5). This is a direct register write (clearing bit 7), not a CTRL9
  // command, so it goes through writeVerified like any other config write.
  writeVerified(kRegFifoCtrl, kFifoCtrlConfig);

  if (overflowed) {
    // See the empty-FIFO branch above: RST_FIFO is the only documented way
    // to clear FIFO_OVFLOW. We've already pulled out everything available
    // this cycle, so resetting here doesn't discard anything beyond what
    // the overflow itself already dropped.
    sendCtrl9Command(kCtrl9CmdRstFifo);
  }

  result.overflow = overflowed;
  return result;
}

float nominalRateHz() {
  return gCurrentRate == Rate::Max ? kNominalHzMax : kNominalHzStroke;
}

}  // namespace qmi8658
