# The UI as an ESP-IDF component. The same sources build on the host
# (analysis/tools/cbuild.py build_ui) with no ESP-IDF at all: the UI knows
# nothing about hardware (CLAUDE.md), and that is kept true by building it
# both ways.
file(GLOB FONTS "${CMAKE_CURRENT_LIST_DIR}/fonts/*.c")
idf_component_register(
    SRCS "src/geometry.c" "src/style.c" "src/ui.c" "src/screen_face.c"
         "src/screen_tempo.c" "src/screen_path.c" "src/screen_speed.c" ${FONTS}
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "src" "fonts" "."
    REQUIRES lvgl)
