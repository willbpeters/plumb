"""Talking to the instrument board over serial.

One implementation of the connect-and-configure sequence, shared by every host
tool. Not a convenience: the sequence contains a measured hazard, and a second
copy of it would eventually be a second copy without the fix.

The hazard is that **opening the serial port resets this board only about half
the time.** Measured across four consecutive attempts, not assumed. When it
does not reset, the board is still streaming from the previous run -- and `s`
is a toggle, so sending it stops the stream instead of starting one. The
failure does not look like a failure: it yields a short capture of stale
in-flight frames, decoded out of order, which before the `repeats` counter
existed was reported as a clean capture.

So nothing here trusts the reset. It asks the board what it is doing, settles
it, and only then configures.
"""

import sys
import time

BOOT_SECONDS = 2.5          # setup() holds for 2 s, then begin() takes ~0.2 s
GYRO_SETTLE_SECONDS = 0.4   # 150 ms + 3/ODR (datasheet Table 8), plus margin
COMMAND_SECONDS = 0.2


def settle_stopped(port, attempts: int = 6, log=sys.stderr) -> bool:
    """Leave the board not streaming, whatever state it was found in.

    Returns False if it never said so, in which case the caller is about to
    capture something it cannot trust and should say so out loud.
    """
    from tools.capture import parse_streaming_state

    for _ in range(attempts):
        port.write(b"?")
        time.sleep(COMMAND_SECONDS)
        state = parse_streaming_state(port.read(8192).decode("utf-8", "replace"))
        if state is False:
            return True
        if state is True:
            port.write(b"s")
            time.sleep(COMMAND_SECONDS)
    if log:
        print("# WARNING: board never reported a stopped stream; anything "
              "captured now may be stale frames from a previous run", file=log)
    return False


def configure(port, rate: str = "stroke", read_path: str = "direct",
              binary: bool = True, log=sys.stderr) -> None:
    """Set rate, read path and wire format. Does not start the stream."""
    port.write(b"9" if rate == "max" else b"1")
    # Selecting a rate restarts the gyro, and the part specifies 150 ms + 3/ODR
    # before its output means anything. The firmware waits it out; this has to
    # wait for the firmware.
    time.sleep(GYRO_SETTLE_SECONDS)
    port.write(b"d" if read_path == "direct" else b"f")
    time.sleep(COMMAND_SECONDS)
    port.write(b"b" if binary else b"c")
    time.sleep(COMMAND_SECONDS)
    if read_path == "direct" and rate == "max" and log:
        print("# WARNING: direct reads cannot keep up at max rate; expect most "
              "samples to be lost", file=log)


def start(port) -> None:
    port.write(b"s")
    time.sleep(COMMAND_SECONDS)


def stop(port) -> None:
    """Leave the board quiet. A stream left running is what makes the next
    run's `s` stop it instead of starting it."""
    port.write(b"s")
    time.sleep(COMMAND_SECONDS)


def connect(port, rate: str = "stroke", read_path: str = "direct",
            binary: bool = True, log=sys.stderr) -> bool:
    """Wait out the boot, settle the stream, configure. Returns whether the
    board confirmed it was stopped before being configured."""
    if log:
        print("# resetting board, waiting for boot...", file=log)
    time.sleep(BOOT_SECONDS)
    port.reset_input_buffer()
    settled = settle_stopped(port, log=log)
    configure(port, rate, read_path, binary, log=log)
    return settled
