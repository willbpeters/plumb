idf_component_register(
    SRCS "app_main.c" "gate.c" "stream.c" "stroke.c" "stroke_config.c" "ui_port.c" "uart_io.c"
    INCLUDE_DIRS "."
    REQUIRES acq board plumb plumb_ui lvgl esp_driver_uart esp_timer)
