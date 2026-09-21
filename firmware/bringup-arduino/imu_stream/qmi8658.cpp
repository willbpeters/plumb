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

Rate gCurrentRate = Rate::Stroke;

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

}  // namespace

namespace qmi8658 {

bool begin(Rate rate) {
  uint8_t who = 0;
  if (!readReg(kRegWhoAmI, who)) return false;
  if (who != kWhoAmIValue) return false;

  if (!configureOdr(rate)) return false;
  if (!writeVerified(kRegCtrl7, kCtrl7EnableAccelGyro)) return false;
  if (!writeVerified(kRegFifoWtmTh, kFifoWatermarkSamples)) return false;
  if (!writeVerified(kRegFifoCtrl, kFifoCtrlConfig)) return false;

  // Start from a known-empty FIFO (CTRL_CMD_RST_FIFO clears data, sample
  // count and flags -- DS-A/DS-09 §5.10.6.2). This is a CTRL9 command, not
  // a directly-writable register, so it goes through sendCtrl9Command
  // rather than writeVerified.
  if (!sendCtrl9Command(kCtrl9CmdRstFifo)) return false;

  gCurrentRate = rate;
  return true;
}

bool setRate(Rate rate) {
  if (!configureOdr(rate)) return false;
  // Samples already queued at the old rate would corrupt the time base --
  // start clean at the new rate.
  if (!sendCtrl9Command(kCtrl9CmdRstFifo)) return false;
  gCurrentRate = rate;
  return true;
}

bool dataReady() {
  uint8_t status = 0;
  if (!readReg(kRegFifoStatus, status)) return false;
  return (status & kFifoStatusWtmBit) != 0;
}

DrainResult drain(Sample* out, uint8_t capacity) {
  DrainResult result{0, false};
  if (out == nullptr || capacity == 0) return result;

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
    const uint8_t* b = staging + (uint16_t)i * kBytesPerSample;
    out[i].ax = (int16_t)((uint16_t)b[1] << 8 | b[0]);
    out[i].ay = (int16_t)((uint16_t)b[3] << 8 | b[2]);
    out[i].az = (int16_t)((uint16_t)b[5] << 8 | b[4]);
    out[i].gx = (int16_t)((uint16_t)b[7] << 8 | b[6]);
    out[i].gy = (int16_t)((uint16_t)b[9] << 8 | b[8]);
    out[i].gz = (int16_t)((uint16_t)b[11] << 8 | b[10]);
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
