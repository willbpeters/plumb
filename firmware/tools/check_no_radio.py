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
# controller and host. Radio code can only reach the image from one of these.
#
# Archives, not symbol names. The first real build's map names Wi-Fi in lines
# that are not radio code: ROM function addresses from the chip's linker
# script (`wifi_get_macaddr = 0x40005ab4`), and esp_hw_support's
# wifi_bt_common_module_enable, the peripheral-clock helper every build links.
# Matching names failed that build; matching archives does not, and still
# catches a real esp_wifi link (shown on a canary build, docs/bringup-results.md).
_ARCHIVES = re.compile(
    r"\blib(esp_wifi|wpa_supplicant|net80211|pp|core|coexist|espnow|mesh|"
    r"smartconfig|wapi|bt|btdm_app|btbb|ble_app|esp_coex)\.a\b")


def offending(map_text: str) -> list[str]:
    """Lines of the map that show radio code in the image, deduplicated."""
    found: list[str] = []
    for line in map_text.splitlines():
        if _ARCHIVES.search(line):
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
