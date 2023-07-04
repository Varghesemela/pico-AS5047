add_executable(${PROJECT_NAME} main.cpp)

# # target_sources(${PROJECT_NAME} PRIVATE main.cpp)

include_directories(${CMAKE_SOURCE_DIR}/inc)

target_include_directories(${PROJECT_NAME} PUBLIC
        ../lib/utils
        ../lib/flash
        ../lib/encoder
        ../inc
        )

target_link_libraries(${PROJECT_NAME} PUBLIC
        pico_stdlib
        pico_multicore
        pico_unique_id
        hardware_sync
        hardware_pio
        hardware_dma
        hardware_irq
        hardware_i2c
        hardware_uart
        hardware_flash
        hardware_pwm
        hardware_watchdog
        hardware_adc
        pico_printf
        utils
        flash
        encoder
        )


pico_add_extra_outputs(${PROJECT_NAME})
pico_enable_stdio_usb(${PROJECT_NAME} 0)
pico_enable_stdio_uart(${PROJECT_NAME} 1)