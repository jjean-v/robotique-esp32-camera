idf_component_register(SRCS take_picture.c aruco.c
                        PRIV_INCLUDE_DIRS .
                        PRIV_REQUIRES nvs_flash esp_psram fatfs sdmmc esp_timer) 
