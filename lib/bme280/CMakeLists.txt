include($ENV{IDF_PATH}/tools/cmake/version.cmake)

set(srcs "src/bme280.c")

idf_component_register(SRCS "bme280.c"
                       INCLUDE_DIRS . )
