add_library(sensor_hdc3022 STATIC
    ${CMAKE_CURRENT_LIST_DIR}/hdc3022.c
)

target_include_directories(sensor_hdc3022
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/include
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(sensor_hdc3022 PUBLIC
    pico_stdlib
    hardware_i2c
)
