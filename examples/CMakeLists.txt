add_executable(example
    main.c
)

target_link_libraries(example
    pico_stdlib
    hardware_i2c
    sensor_hdc3022
)

pico_enable_stdio_usb(example 0)
pico_enable_stdio_uart(example 1)

pico_add_extra_outputs(example)
