idf_component_register(
    SRCS "app_main.c" "gate.c" "stream.c" "stroke.c" "stroke_config.c" "ui_port.c" "uart_io.c" "rec_port.c"
    INCLUDE_DIRS "."
    REQUIRES acq board plumb plumb_ui rec lvgl esp_driver_uart esp_timer
             esp_partition nvs_flash esp_app_format)
