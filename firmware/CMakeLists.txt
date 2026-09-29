# Plumb product firmware, ESP-IDF v5.5.5.
#
#   ./idf.ps1 build        ./idf.ps1 -p COM4 flash
cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/third_party/lvgl")
# Build only what main pulls in. esp_wifi and bt are not compiled at all
# (invariant 6); tools/check_no_radio.py below proves it on every link.
set(COMPONENTS main)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# One LVGL configuration, host and device: plumb_ui/lv_conf.h. LVGL's Kconfig
# is ignored so options lv_conf.h does not list take the same defaults here
# as on the host bench, not Kconfig's.
idf_build_set_property(COMPILE_DEFINITIONS "LV_KCONFIG_IGNORE" APPEND)
idf_build_set_property(COMPILE_OPTIONS
    "-I${CMAKE_CURRENT_LIST_DIR}/components/plumb_ui" APPEND)

project(plumb)

idf_build_get_property(python PYTHON)
add_custom_command(TARGET ${CMAKE_PROJECT_NAME}.elf POST_BUILD
    COMMAND ${python} ${CMAKE_CURRENT_LIST_DIR}/tools/check_no_radio.py
            ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.map
    VERBATIM)
