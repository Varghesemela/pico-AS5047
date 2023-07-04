//
// Created by sanjay on 18/5/23.
//

#ifndef PICO_ENCODER_H
#define PICO_ENCODER_H


#include <cstdio>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "encoders.pio.h"

// Register Addresses
static uint16_t angle_addr = 0xFFFF; // Angle address with parity and read bit appended
static uint16_t zposm_reg_addr = 0x0016;
static uint16_t zposl_reg_addr = 0x0017;
static uint16_t settings1_reg_addr = 0x0018;

enum {
    NONE_type,
    ABEncoder,
    magneticEncoder
};

class encoder{
public:
    int32_t _encoder_resolution;
    float *_encoder_position;
    float _encoder_mm_per_revolution, _encoder_mm_per_click, encoder_value;
    pio_hw_t *_encoder_pio = nullptr;
    encoder(float *enc_pos, PIO encoder_pio, int32_t encoder_resolution, float encoder_mm_per_revolution, uint8_t Apin, uint8_t Bpin, int8_t Zpin);
    encoder(float *enc_pos, uint32_t BAUDRATE, int32_t encoder_resolution, float encoder_mm_per_revolution, spi_inst_t * SPIport, uint8_t MISOpin, uint8_t MOSIpin, uint8_t SCKpin , int8_t CSpin);
    uint16_t readAngle();
    bool writeZeroPosition(uint16_t angle_value);
    uint8_t parity_check(uint16_t data);
    void print_position() const;
    const spi_inst_t  *SPI_PORT_ = nullptr;
    const int PIN_MISO_ = -1, PIN_MOSI_ = -1, PIN_SCK_ = -1, PIN_CS_ = -1;
    const uint32_t SPI_BAUDRATE_ = 0;

    void spiInit();
    void dmaInit();

    friend void AB_dmahandler();
    friend void magnetic_dmahandler();
    volatile int32_t capture_buf = 0;
    uint16_t res = 0;
    uint16_t present_angle = 0;
    uint16_t previous_angle = 0;
    int32_t diff = 0;
    int16_t counter = 0;
    uint8_t encoder_type = NONE_type;
    float getEncoderValue() const;

    int8_t initialize();
    uint offset, unused_sm;//, unused_dma_channel;

private:

    const int _PIN_A = -1, _PIN_B = -1, _PIN_Z = -1;

    void inline enableCS() const;
    void inline disableCS() const;

};




#endif //PICO_ENCODER_H
