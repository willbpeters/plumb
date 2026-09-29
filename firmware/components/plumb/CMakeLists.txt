# ESP-IDF component registration.
#
# The same sources build for the host, from analysis/tools/cbuild.py, with no
# ESP-IDF in sight -- which is the point. An algorithm that only compiles
# inside the IDF cannot be diffed against the Python it came from, and on a
# board with no JTAG (parent spec 14.3) that diff is the only debugger there is.
idf_component_register(
    SRCS "src/quat.c" "src/pivot.c" "src/pipeline.c" "src/session.c"
    INCLUDE_DIRS "include"
)

# No FMA contraction: the LX7 has MADD.S and GCC fuses by default, which would
# make the device diverge from the host build that was diffed (include/plumb/real.h).
target_compile_options(${COMPONENT_LIB} PRIVATE -ffp-contract=off)

# Precision (include/plumb/real.h). Single by default for the measurement
# builds; which one ships is Will's call, from the step cost measured on the
# board (stroke_status, `?`). `idf.py -DPLUMB_DOUBLE=ON` builds double.
#
# PUBLIC, not PRIVATE: pl_real appears in every struct the app shares with
# this component, so the two must agree on its size. A PRIVATE define would
# compile the component with float and main with double -- different struct
# layouts, no warning, and wrong numbers.
option(PLUMB_DOUBLE "Run the stroke pipeline in double precision" OFF)
if(NOT PLUMB_DOUBLE)
    target_compile_definitions(${COMPONENT_LIB} PUBLIC PLUMB_SINGLE_PRECISION)
endif()
