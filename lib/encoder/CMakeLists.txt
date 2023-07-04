project(encoder)

add_library(${PROJECT_NAME} encoder.cpp encoder.h)

pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/encoders.pio)

target_sources(${PROJECT_NAME} PRIVATE
        encoder.cpp
        encoder.h)

target_link_libraries(${PROJECT_NAME}   PUBLIC
        pico_stdlib

        hardware_dma
        hardware_pio
        hardware_spi
        )