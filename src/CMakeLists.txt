idf_component_register(SRCS "main.c" "RPLidar.c"
                       INCLUDE_DIRS "../include"
                       REQUIRES esp_driver_uart esp_driver_gpio)
