#!/bin/sh
# Build the differential harness for the host.
#
# Usage:  ./build.sh [output.exe] [extra compiler flags...]
#         ./build.sh portcheck-single.exe -DPLUMB_SINGLE_PRECISION
#
# MSVC is driven directly rather than through vcvars64.bat, which hangs in Git
# Bash on this machine. That means INCLUDE and LIB are set here; the paths are
# discovered rather than hardcoded so a toolchain update does not silently
# break the build. Falls back to cc/gcc/clang wherever one exists, so this is
# not tied to Windows.

set -e

HERE=$(cd "$(dirname "$0")" && pwd)
COMPONENT="$HERE/../components/plumb"
OUT=${1:-"$HERE/portcheck.exe"}
shift 2>/dev/null || true

SOURCES="$HERE/portcheck.c $COMPONENT/src/quat.c"

if command -v cc >/dev/null 2>&1 || command -v gcc >/dev/null 2>&1; then
    CC=$(command -v cc || command -v gcc)
    exec "$CC" -std=c99 -O2 -Wall -Wextra -Werror \
        -I"$COMPONENT/include" $SOURCES -lm -o "$OUT" "$@"
fi

VSROOT="/c/Program Files/Microsoft Visual Studio/2022/Community/VC/Tools/MSVC"
SDKROOT="/c/Program Files (x86)/Windows Kits/10"
VCVER=$(ls "$VSROOT" 2>/dev/null | sort -V | tail -1)
SDKVER=$(ls "$SDKROOT/Include" 2>/dev/null | sort -V | tail -1)

if [ -z "$VCVER" ] || [ -z "$SDKVER" ]; then
    echo "no C compiler found (looked for cc/gcc, then MSVC)" >&2
    exit 127
fi

VC="$VSROOT/$VCVER"
export INCLUDE="$(cygpath -w "$VC/include");$(cygpath -w "$SDKROOT/Include/$SDKVER/ucrt");$(cygpath -w "$SDKROOT/Include/$SDKVER/um");$(cygpath -w "$SDKROOT/Include/$SDKVER/shared")"
export LIB="$(cygpath -w "$VC/lib/x64");$(cygpath -w "$SDKROOT/Lib/$SDKVER/ucrt/x64");$(cygpath -w "$SDKROOT/Lib/$SDKVER/um/x64")"

# Dash-form flags, not slash-form: Git Bash rewrites /W4 into a filesystem
# path and MSVC then treats it as a source file.
#
# _CRT_SECURE_NO_WARNINGS silences MSVC deprecating sscanf, and it applies to
# this harness only -- the ported algorithm itself compiles clean at -W4 -WX,
# which is the property worth keeping true.
"$VC/bin/Hostx64/x64/cl.exe" -nologo -TC -O2 -W4 -WX -D_CRT_SECURE_NO_WARNINGS \
    -I"$(cygpath -w "$COMPONENT/include")" \
    "$@" \
    $(cygpath -w "$HERE/portcheck.c") \
    $(cygpath -w "$COMPONENT/src/quat.c") \
    -Fe:"$(cygpath -w "$OUT")" \
    -Fo:"$(cygpath -w "$HERE/obj/")"
