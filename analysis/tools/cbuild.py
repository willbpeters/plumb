"""Build the ported C for host testing, from any shell.

This exists because the first version was a shell script, and a shell script
needs a shell: run the differential test from PowerShell rather than Git Bash
and every case skipped, silently, because `sh` was not on PATH. A test that
looks like it ran and did nothing is worse than one that fails.

So the toolchain discovery lives here, in the language the test suite is
already written in, and the compiler is invoked directly rather than through a
shell. One implementation -- the same reasoning that put the board connect
sequence in tools/board.py.

Falls back to cc/gcc/clang wherever one exists, so this is not tied to Windows.
"""

import os
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
COMPONENT = REPO / "firmware" / "components" / "plumb"
HARNESS = REPO / "firmware" / "test"

SOURCES = [HARNESS / "portcheck.c", COMPONENT / "src" / "quat.c"]

MSVC_ROOT = Path(r"C:\Program Files\Microsoft Visual Studio\2022\Community"
                 r"\VC\Tools\MSVC")
SDK_ROOT = Path(r"C:\Program Files (x86)\Windows Kits\10")


class CompilerNotFound(Exception):
    """No host C compiler. The port cannot be verified on this machine."""


@dataclass
class BuildResult:
    executable: Path
    compiler: str


def _newest(directory: Path) -> Path | None:
    if not directory.is_dir():
        return None
    versions = sorted((p for p in directory.iterdir() if p.is_dir()),
                      key=lambda p: [int(n) if n.isdigit() else n
                                     for n in p.name.replace(".", " ").split()])
    return versions[-1] if versions else None


def _msvc():
    """cl.exe plus the INCLUDE and LIB it needs.

    vcvars64.bat would supply these, but it hangs when launched from Git Bash
    on this machine, so the paths are discovered instead. Discovered rather
    than hardcoded, so a Visual Studio update does not quietly break the build.
    """
    toolset = _newest(MSVC_ROOT)
    sdk = _newest(SDK_ROOT / "Include")
    if toolset is None or sdk is None:
        return None
    cl = toolset / "bin" / "Hostx64" / "x64" / "cl.exe"
    if not cl.is_file():
        return None
    sdk_version = sdk.name
    include = [toolset / "include",
               SDK_ROOT / "Include" / sdk_version / "ucrt",
               SDK_ROOT / "Include" / sdk_version / "um",
               SDK_ROOT / "Include" / sdk_version / "shared"]
    lib = [toolset / "lib" / "x64",
           SDK_ROOT / "Lib" / sdk_version / "ucrt" / "x64",
           SDK_ROOT / "Lib" / sdk_version / "um" / "x64"]
    return cl, include, lib


def describe_toolchain() -> str:
    """What this machine would build with, for reporting in a skip or a log."""
    unix = shutil.which("cc") or shutil.which("gcc") or shutil.which("clang")
    if unix:
        return f"unix: {unix}"
    msvc = _msvc()
    if msvc:
        return f"msvc: {msvc[0]}"
    return "none"


def build(name: str, *defines: str) -> BuildResult:
    """Compile the harness to `name` under firmware/test. Raises if it cannot.

    `defines` are passed through in the compiler's own spelling, e.g.
    "PLUMB_SINGLE_PRECISION".
    """
    HARNESS.mkdir(parents=True, exist_ok=True)
    output = HARNESS / name

    unix = shutil.which("cc") or shutil.which("gcc") or shutil.which("clang")
    if unix:
        command = [unix, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
                   # No fused multiply-add. Contracting a*b+c into one
                   # rounding changes the last bits, and the bit-for-bit
                   # comparison against NumPy (include/plumb/real.h) means
                   # nothing if the compiler is free to do it. x86-64
                   # baseline happens not to contract; the ESP32-S3 has
                   # MADD.S and GCC contracts by default, so the host build
                   # states the same rule the component's CMakeLists does.
                   "-ffp-contract=off",
                   f"-I{COMPONENT / 'include'}",
                   *(f"-D{d}" for d in defines),
                   *(str(s) for s in SOURCES), "-lm", "-o", str(output)]
        environment = None
        label = f"unix:{Path(unix).name}"
    else:
        msvc = _msvc()
        if msvc is None:
            raise CompilerNotFound(
                "no host C compiler found (looked for cc, gcc, clang, then "
                "MSVC under Program Files)")
        cl, include, lib = msvc
        objects = HARNESS / "obj"
        objects.mkdir(exist_ok=True)
        command = [str(cl), "-nologo", "-TC", "-O2", "-W4", "-WX",
                   # The default already, and stated so the intent is
                   # visible: /fp:precise does not contract to FMA in C on
                   # x64, which the bit-for-bit comparison depends on. See
                   # -ffp-contract=off on the unix path.
                   "-fp:precise",
                   # Silences MSVC deprecating sscanf, and applies to the
                   # harness only: the ported algorithm compiles clean at
                   # -W4 -WX, which is the property worth keeping true.
                   "-D_CRT_SECURE_NO_WARNINGS",
                   f"-I{COMPONENT / 'include'}",
                   *(f"-D{d}" for d in defines),
                   *(str(s) for s in SOURCES),
                   f"-Fe:{output}", f"-Fo:{objects}{os.sep}"]
        environment = dict(os.environ)
        environment["INCLUDE"] = os.pathsep.join(str(p) for p in include)
        environment["LIB"] = os.pathsep.join(str(p) for p in lib)
        label = "msvc"

    result = subprocess.run(command, capture_output=True, text=True,
                            env=environment, cwd=str(HARNESS))
    if result.returncode != 0 or not output.is_file():
        raise RuntimeError(
            f"compiling the port failed ({label}):\n"
            f"{result.stdout}\n{result.stderr}")
    return BuildResult(executable=output, compiler=label)


if __name__ == "__main__":
    print(f"toolchain: {describe_toolchain()}")
    print(f"built: {build('portcheck.exe').executable}")
