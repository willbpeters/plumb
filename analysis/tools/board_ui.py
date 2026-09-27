"""Build, flash and talk to the display bring-up sketch.

    uv run python -m tools.board_ui build
    uv run python -m tools.board_ui flash --port COM4
    uv run python -m tools.board_ui send t --port COM4 --seconds 2

The sketch (firmware/bringup-arduino/display) compiles LVGL from its submodule
and plumb_ui from firmware/components/plumb_ui, through one-line wrapper files.
What makes that work is here, in one place: LVGL passed as a library, and
lv_conf.h plus the component's include paths passed as build properties. The
build directory is fixed so rebuilds are incremental; a cold build is about
five minutes, most of it LVGL.
"""

import argparse
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
SKETCH = REPO / "firmware" / "bringup-arduino" / "display"
BUILD = SKETCH / "build"
LVGL = REPO / "firmware" / "third_party" / "lvgl"
UI = REPO / "firmware" / "components" / "plumb_ui"
BOARD = REPO / "firmware" / "components" / "board"

# Parent spec 14.2: 16 MB flash, QSPI PSRAM, and CDC-on-boot off (the CH343
# bridge carries Serial, not native USB).
FQBN = ("esp32:esp32:esp32s3:FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,"
        "PSRAM=enabled,CDCOnBoot=default")


def _flags() -> list[str]:
    ui, include, src = (p.as_posix() for p in (UI, UI / "include", UI / "src"))
    board, board_include = BOARD.as_posix(), (BOARD / "include").as_posix()
    extra = (f"-DLV_CONF_INCLUDE_SIMPLE -I{ui} -I{include} -I{src} "
             f"-I{board} -I{board_include}")
    return ["--library", LVGL.as_posix(),
            "--build-property", f"compiler.c.extra_flags={extra}",
            "--build-property", f"compiler.cpp.extra_flags={extra}"]


def build() -> None:
    subprocess.run(["arduino-cli", "compile", "--fqbn", FQBN,
                    "--build-path", str(BUILD), *_flags(), str(SKETCH)],
                   check=True)


def flash(port: str) -> None:
    build()
    subprocess.run(["arduino-cli", "upload", "--fqbn", FQBN, "-p", port,
                    "--input-dir", str(BUILD), str(SKETCH)], check=True)


def send(port: str, commands: str, seconds: float, baud: int = 115200) -> None:
    """Write console characters and print what comes back.

    DTR and RTS are held low before the port opens so that opening it does
    not pulse the auto-download circuit (parent spec 14.1) -- tools/board.py
    measured that a naive open resets this board about half the time, and a
    reset here would silently throw away the state being inspected.

    The display sketch's console runs at 115200; the product firmware's at
    921600, imu_stream's rate.
    """
    import serial

    port_obj = serial.Serial()
    port_obj.port = port
    port_obj.baudrate = baud
    port_obj.timeout = 0.1
    port_obj.dtr = False
    port_obj.rts = False
    port_obj.open()
    try:
        time.sleep(0.2)
        port_obj.reset_input_buffer()
        for c in commands:
            port_obj.write(c.encode("ascii"))
            time.sleep(0.15)
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            data = port_obj.read(4096)
            if data:
                sys.stdout.write(data.decode("utf-8", "replace"))
                sys.stdout.flush()
    finally:
        port_obj.close()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="action", required=True)
    sub.add_parser("build")
    f = sub.add_parser("flash")
    f.add_argument("--port", required=True)
    s = sub.add_parser("send")
    s.add_argument("commands", nargs="?", default="")
    s.add_argument("--port", required=True)
    s.add_argument("--seconds", type=float, default=2.0)
    s.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()
    if args.action == "build":
        build()
    elif args.action == "flash":
        flash(args.port)
    else:
        send(args.port, args.commands, args.seconds, args.baud)


if __name__ == "__main__":
    main()
