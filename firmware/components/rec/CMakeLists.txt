# The corpus log (parent spec 11, amended 2026-09-29). Pure C: flash is
# reached through callbacks, which main/rec_port.c supplies from esp_partition
# and firmware/test/reccheck.c from RAM. Proven on the host by
# analysis/tests/test_rec.py against plumb/corpus.py.
idf_component_register(
    SRCS "src/crc32.c" "src/block.c" "src/log.c"
    INCLUDE_DIRS "include")
