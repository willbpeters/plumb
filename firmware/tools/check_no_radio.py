"""Fail the firmware build if any Wi-Fi or Bluetooth code was linked.

CLAUDE.md invariant 6: Wi-Fi stays compiled out in every build, because the
Wi-Fi stack changes interrupt behaviour and therefore sampling jitter, which
the accuracy argument rests on. That is a property of the linked binary, not of
the source tree, so this reads the linker map: it records every archive member
pulled in and every symbol placed.

Run by firmware/CMakeLists.txt after every link:

    python check_no_radio.py build/plumb.map
"""

import re
import sys
from pathlib import Path

# Archives that exist only for the radios: the Wi-Fi driver and its binary
# blobs (net80211, pp, core), the supplicant, coexistence, and the Bluetooth
# controller and host.
_ARCHIVES = re.compile(
    r"\blib(esp_wifi|wpa_supplicant|net80211|pp|core|coexist|espnow|mesh|"
    r"smartconfig|wapi|bt|btdm_app|btbb|ble_app|esp_coex)\.a\b")
# Placed symbols from the radio APIs.
_SYMBOLS = re.compile(r"(?<![\w.])(esp_wifi_\w+|wifi_\w+|esp_bt_\w+|esp_ble_\w+)\b")


def offending(map_text: str) -> list[str]:
    """Lines of the map that show radio code in the image, deduplicated."""
    found: list[str] = []
    for line in map_text.splitlines():
        if _ARCHIVES.search(line) or _SYMBOLS.search(line):
            stripped = line.strip()
            if stripped not in found:
                found.append(stripped)
    return found


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print("usage: check_no_radio.py <linker map>", file=sys.stderr)
        return 2
    path = Path(argv[1])
    if not path.is_file():
        print(f"check_no_radio: no linker map at {path}", file=sys.stderr)
        return 2
    found = offending(path.read_text(errors="replace"))
    if found:
        print("INVARIANT 6 VIOLATED: radio code is linked into the firmware "
              "(CLAUDE.md). First lines of the map that show it:", file=sys.stderr)
        for line in found[:20]:
            print(f"  {line}", file=sys.stderr)
        return 1
    print(f"check_no_radio: no Wi-Fi or Bluetooth code in {path.name}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
