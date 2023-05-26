idf_component_register(
    SRCS "lvgl_port.c"
    INCLUDE_DIRS "."
    REQUIRES "lvgl" "esp_timer" "esp_lcd" "driver" "ui" "esp_lcd_touch_cst816t"
)
