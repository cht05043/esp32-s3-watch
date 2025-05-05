file(GLOB_RECURSE SRC_UI ${CMAKE_SOURCE_DIR} "ui/*.c" "rtc/*.c" "qmi8658/*.c" "buzzer/*.c" "mywifi/*.c")

idf_component_register(SRCS "main.c" ${SRC_UI}
                    INCLUDE_DIRS "." "ui" "rtc" "qmi8658" "buzzer" "mywifi")