include($ENV{IDF_PATH}/tools/cmake/version.cmake)

idf_component_register(SRCS "bme280.c"
                       INCLUDE_DIRS . )
