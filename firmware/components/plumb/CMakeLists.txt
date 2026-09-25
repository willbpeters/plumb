# ESP-IDF component registration.
#
# The same sources build for the host, from firmware/test/build.sh, with no
# ESP-IDF in sight -- which is the point. An algorithm that only compiles
# inside the IDF cannot be diffed against the Python it came from, and on a
# board with no JTAG (parent spec 14.3) that diff is the only debugger there is.
idf_component_register(
    SRCS "src/quat.c"
    INCLUDE_DIRS "include"
)

# No FMA contraction: the LX7 has MADD.S and GCC fuses by default, which would
# make the device diverge from the host build that was diffed (include/plumb/real.h).
target_compile_options(${COMPONENT_LIB} PRIVATE -ffp-contract=off)
