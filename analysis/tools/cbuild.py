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
LVGL = REPO / "firmware" / "third_party" / "lvgl"
UI = REPO / "firmware" / "components" / "plumb_ui"

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


def _unix_objects(cc: str, flags: list[str], sources: list[Path],
                  directory: Path) -> list[Path]:
    """Compile each source to an object, in parallel. cc has no -MP."""
    from concurrent.futures import ThreadPoolExecutor

    def one(source: Path):
        obj = directory / f"{source.stem}.o"
        result = subprocess.run([cc, *flags, "-c", str(source), "-o", str(obj)],
                                capture_output=True, text=True)
        return source, result

    with ThreadPoolExecutor() as pool:
        for source, result in pool.map(one, sources):
            if result.returncode != 0:
                raise RuntimeError(f"compiling {source} failed:\n"
                                   f"{result.stdout}\n{result.stderr}")
    return [directory / f"{s.stem}.o" for s in sources]


def _by_unique_name(sources: list[Path]) -> list[list[Path]]:
    """Split sources so no group holds two files with the same name.

    A compiler names each object after its source's file name, so two
    same-named sources compiled into one directory overwrite each other and
    the build links whichever finished last, silently. LVGL 9.6 has two
    vg_lite_matrix.c. Each group is compiled into its own directory.
    """
    groups: list[list[Path]] = []
    for source in sources:
        for group in groups:
            if all(other.stem != source.stem for other in group):
                group.append(source)
                break
        else:
            groups.append([source])
    return groups


def build_ui(name: str = "uisnap.exe") -> BuildResult:
    """Build the headless UI renderer: LVGL, the plumb_ui component, uisnap.c.

    LVGL is compiled once into firmware/test/obj-lvgl and reused until its
    sources, its version or lv_conf.h change. It is third-party code built
    with relaxed warnings; our code is held to -W4 -WX (or -Wall -Wextra
    -Werror), with LVGL's headers marked external so their warnings cannot
    fail our build.
    """
    if not (LVGL / "src").is_dir():
        raise RuntimeError("LVGL is missing: run `git submodule update --init` "
                           "in the repository root")
    lvgl_sources = sorted((LVGL / "src").rglob("*.c"))
    fonts = sorted((UI / "fonts").glob("pl_font_*.c"))
    ours = sorted((UI / "src").glob("*.c")) + [HARNESS / "uisnap.c"]

    output = HARNESS / name
    lib_dir = HARNESS / "obj-lvgl"
    our_dir = HARNESS / "obj-ui"
    our_dir.mkdir(parents=True, exist_ok=True)
    groups = _by_unique_name(lvgl_sources)
    group_dirs = [lib_dir / str(i) for i in range(len(groups))]
    for directory in group_dirs:
        directory.mkdir(parents=True, exist_ok=True)

    stamp = "\n".join([(UI / "lv_conf.h").read_text(),
                       (LVGL / "lv_version.h").read_text(),
                       describe_toolchain(),
                       *(str(s.relative_to(LVGL)) for s in lvgl_sources)])
    stamp_file = lib_dir / "stamp.txt"
    fresh = stamp_file.is_file() and stamp_file.read_text() == stamp
    common = ["-DLV_CONF_INCLUDE_SIMPLE"]

    unix = shutil.which("cc") or shutil.which("gcc") or shutil.which("clang")
    if unix:
        lib_flags = ["-std=c99", "-O2", "-w", *common, f"-I{UI}", f"-I{LVGL}"]
        if not fresh:
            for group, directory in zip(groups, group_dirs):
                _unix_objects(unix, lib_flags, group, directory)
            stamp_file.write_text(stamp)
        lib = [d / f"{s.stem}.o" for g, d in zip(groups, group_dirs) for s in g]
        font_objs = _unix_objects(unix, lib_flags, fonts, our_dir)
        command = [unix, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
                   *common, f"-I{UI / 'include'}", f"-I{UI / 'src'}",
                   f"-I{UI}", "-isystem", str(LVGL),
                   *(str(s) for s in ours), *(str(o) for o in font_objs + lib),
                   "-lm", "-o", str(output)]
        environment = None
        label = f"unix:{Path(unix).name}"
    else:
        msvc = _msvc()
        if msvc is None:
            raise CompilerNotFound(
                "no host C compiler found (looked for cc, gcc, clang, then "
                "MSVC under Program Files)")
        cl, include, lib_paths = msvc
        environment = dict(os.environ)
        environment["INCLUDE"] = os.pathsep.join(str(p) for p in include)
        environment["LIB"] = os.pathsep.join(str(p) for p in lib_paths)

        def cl_objects(flags: list[str], sources: list[Path],
                       directory: Path) -> list[Path]:
            if not sources:
                return []
            rsp = directory / "sources.rsp"
            rsp.write_text("\n".join(f'"{s}"' for s in sources))
            result = subprocess.run(
                [str(cl), "-nologo", "-TC", "-c", "-MP", *flags, f"@{rsp}",
                 f"-Fo:{directory}{os.sep}"],
                capture_output=True, text=True, errors="replace",
                env=environment, cwd=str(HARNESS))
            if result.returncode != 0:
                raise RuntimeError(f"compiling into {directory.name} failed:\n"
                                   f"{result.stdout}\n{result.stderr}")
            return [directory / f"{s.stem}.obj" for s in sources]

        lib_flags = ["-O2", "-W1", *common, f"-I{UI}", f"-I{LVGL}"]
        if not fresh:
            for group, directory in zip(groups, group_dirs):
                cl_objects(lib_flags, group, directory)
            stamp_file.write_text(stamp)
        lib = [d / f"{s.stem}.obj" for g, d in zip(groups, group_dirs) for s in g]
        font_objs = cl_objects(lib_flags, fonts, our_dir)
        link = our_dir / "link.rsp"
        link.write_text("\n".join(f'"{p}"' for p in [*ours, *font_objs, *lib]))
        # No -TC here, unlike every other cl call in this file: -TC means
        # "treat EVERY input as C", and this command also takes the 480 LVGL
        # objects to link. With it, cl compiled binary .obj files as C source:
        # 11,591 errors and ten minutes of CPU before anyone noticed. The .c
        # extension is enough to make our sources C.
        command = [str(cl), "-nologo", "-O2", "-W4", "-WX",
                   "-D_CRT_SECURE_NO_WARNINGS", *common,
                   f"-I{UI / 'include'}", f"-I{UI / 'src'}", f"-I{UI}",
                   f"-external:I{LVGL}", "-external:W0",
                   f"@{link}", f"-Fe:{output}", f"-Fo:{our_dir}{os.sep}"]
        label = "msvc"

    # Compiler output is in the console code page and can hold anything; a
    # failure has to be readable rather than crash the reader thread.
    result = subprocess.run(command, capture_output=True, text=True,
                            errors="replace", env=environment,
                            cwd=str(HARNESS))
    if result.returncode != 0 or not output.is_file():
        raise RuntimeError(f"building the UI bench failed ({label}):\n"
                           f"{result.stdout}\n{result.stderr}")
    return BuildResult(executable=output, compiler=label)


if __name__ == "__main__":
    print(f"toolchain: {describe_toolchain()}")
    print(f"built: {build('portcheck.exe').executable}")
