# Acquisition: the core-0 task (acq.c) and the pure-C pieces it and the app
# task share. ring, seqcount, jitter and frame have no ESP-IDF dependency and
# are proven on the host by analysis/tests/test_acq.py.
idf_component_register(
    SRCS "src/acq.c" "src/ring.c" "src/seqcount.c" "src/jitter.c" "src/frame.c"
    INCLUDE_DIRS "include"
    REQUIRES board esp_driver_gpio esp_timer)
