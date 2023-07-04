#ifndef PINS_PICO_BOARD_H
#define PINS_PICO_BOARD_H


#define BOARD_TYPE "Pico"

/*
* GPIO Pins declaration for strobing
*/


#define SPI_PORT    spi1
#define SCK         10
#define MOSI        11
#define MISO        12
#define CS          13

#define ENC_SPI_PORT    SPI_PORT
#define ENC_SCK         SCK
#define ENC_MOSI        MOSI
#define ENC_MISO        MISO
#define ENC_CS          CS
#define ENC_Z       9


#define LED_STATUS  25
#define MONITOR_5V 29

#endif