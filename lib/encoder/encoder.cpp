//
// Created by sanjay on 18/5/23.
//

#include "encoder.h"

#define DATA_LENGTH     1
static encoder *instance = nullptr;

// Select DMA channels - GLOBAL DECLARTION
static int req_chan = dma_claim_unused_channel(true);
static int resp_chan = dma_claim_unused_channel(true);
static int unused_dma_channel = dma_claim_unused_channel(true);

// DMA functions
void magnetic_dmahandler() {
    instance->present_angle = instance->res & 0x3FFF;
    instance->diff = instance->present_angle - instance->previous_angle;
    instance->previous_angle = instance->present_angle;

    if (instance->diff < -8000)
        instance->counter++;
    else if (instance->diff > 8000)
        instance->counter--;

    instance->capture_buf = (instance->present_angle + (instance->counter * 16384));
    *instance->_encoder_position = ((float)instance->capture_buf) * instance->_encoder_mm_per_click;
//    printf("Angle: %d\tinstance->counter: %d\tCapturebuf: %d\n", instance->instance->present_angle, instance->instance->counter, capture_buf);

    // Clear DMA channel interrupt
    dma_hw->ints0 = 1u << resp_chan;
    dma_channel_set_read_addr(req_chan, &angle_addr, true);
}

void AB_dmahandler() {
    *instance->_encoder_position = ((float)instance->capture_buf) * instance->_encoder_mm_per_click;
    dma_hw->ints0 = 1u << unused_dma_channel;
    dma_channel_set_read_addr(unused_dma_channel, &instance->_encoder_pio->rxf[instance->unused_sm], true);

}

void encoder::print_position() const{
    printf("Angle: %d\tcounter: %d\tCapturebuf: %d\n", this->present_angle, this->counter, this->capture_buf);
};

float encoder::getEncoderValue() const{
    return *this->_encoder_position;
}

void encoder::spiInit()
{
    // Initialize SPI channel (channel, baud rate set to 20MHz)
    spi_init((spi_inst_t *) SPI_PORT_,  SPI_BAUDRATE_) ;

    // Format SPI channel (channel, data bits per transfer, polarity, phase, order)
    spi_set_format((spi_inst_t *) SPI_PORT_, 16, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);


    // Map SPI signals to GPIO ports, acts like framed SPI with this CS mapping
    gpio_set_function(this->PIN_MOSI_, GPIO_FUNC_SPI);
    gpio_set_function(this->PIN_CS_, GPIO_FUNC_SPI) ;
    gpio_set_function(this->PIN_SCK_, GPIO_FUNC_SPI);
    gpio_set_function(this->PIN_MISO_, GPIO_FUNC_SPI);
}

// Encoder functions
uint16_t encoder::readAngle() {
    uint16_t angle_value = 0;
    // Read Angle from sensor

    disableCS();
    spi_write16_blocking((spi_inst_t *) SPI_PORT_, &angle_addr, 1);
    enableCS();
    busy_wait_us(150);

    disableCS();
    spi_read16_blocking((spi_inst_t *) SPI_PORT_, 0, &angle_value, 1);
    busy_wait_us(50);

    enableCS();
    busy_wait_us(150);

    // printf("Present Angle value: %d\n", angle_value);

    return angle_value & 0x3FFF;
}

bool encoder::writeZeroPosition(uint16_t angle_value) {
    if(this->encoder_type == ABEncoder){
        // TODO: write zero writing function for AB enco
    }
    else if(this->encoder_type == magneticEncoder){//------------------------------------------------------------------
        // ZPOSM write and read
        uint16_t reg = zposm_reg_addr;
        // Write ZPOSM register
        if (parity_check(zposm_reg_addr))
            reg = zposm_reg_addr | (1 << 15);
        // printf("ZPOSM address sent on SPI: %x\n", reg);
        disableCS();
        spi_write16_blocking((spi_inst_t *) SPI_PORT_, &reg, 1);
        enableCS();
        busy_wait_us(150);

        uint16_t zposm_reg_value = angle_value >> 6;
        // printf("ZPOSM value sent on SPI: %x\n", zposm_reg_value);
        if (parity_check(zposm_reg_value))
            zposm_reg_value = zposm_reg_value | (1 << 15);
        disableCS();
        spi_write16_blocking((spi_inst_t *) SPI_PORT_, &zposm_reg_value, 1);
        enableCS();
        busy_wait_us(150);

        //---------------------------------------
        // Read ZPOSM register
        reg = 0;
        reg = zposm_reg_addr | (1 << 14); // Read bit
        if (parity_check(reg))
            reg = reg | (1 << 15);
        disableCS();
        spi_write16_blocking((spi_inst_t *) SPI_PORT_, &reg, 1);
        enableCS();
        busy_wait_us(150);

        uint16_t res1;
        disableCS();
        spi_read16_blocking((spi_inst_t *) SPI_PORT_, 0, &res1, 1);
        busy_wait_us(50);
        enableCS();
        busy_wait_us(150);
        res1 = res1 & 0x00FF;
        // printf("ZPOSM value stored: %x\n", res1);

        //------------------------------------------------------------------
        // ZPOSL write and read
        reg = zposl_reg_addr;
        // Write POSL register
        if (parity_check(zposl_reg_addr))
            reg = zposl_reg_addr | (1 << 15);
        // printf("ZPOSL address sent on SPI: %x\n", reg);
        disableCS();
        spi_write16_blocking((spi_inst_t *) SPI_PORT_, &reg, 1);
        enableCS();
        busy_wait_us(150);

        uint16_t zposl_reg_value = angle_value & 0x003F;
        // printf("ZPOSL value sent on SPI: %x\n", zposl_reg_value);
        if (parity_check(zposl_reg_value))
            zposl_reg_value = zposl_reg_value | (1 << 15);
        disableCS();
        spi_write16_blocking((spi_inst_t *) SPI_PORT_, &zposl_reg_value, 1);
        enableCS();
        busy_wait_us(150);
        //---------------------------------------
        // Read ZPOSL register
        reg = 0;
        reg = zposl_reg_addr | (1 << 14); // Read bit
        if (parity_check(reg))
            reg = reg | (1 << 15);
        disableCS();
        spi_write16_blocking((spi_inst_t *) SPI_PORT_, &reg, 1);
        enableCS();
        busy_wait_us(150);

        uint16_t res2;
        disableCS();
        spi_read16_blocking((spi_inst_t *) SPI_PORT_, 0, &res2, 1);
        busy_wait_us(50);
        enableCS();
        busy_wait_us(150);
        res2 = res2 & 0x003F;
        // printf("ZPOSL value stored: %x\n", res2);

        uint16_t final_res = 0;
        final_res = (res1 << 6) | (res2);

        // printf("Zero Position set in sensor is: %d\n", final_res);

        zposm_reg_value = 0;
        zposl_reg_value = 0;

//        magnetic_dmahandler();

        if (final_res == angle_value)
            return true;
        else
            return false;
    }
    return PICO_OK;
}

void encoder::dmaInit() {
    if(this->encoder_type == magneticEncoder) {
        // Setup the request channel
        dma_channel_config c = dma_channel_get_default_config(req_chan);    // default configs
        channel_config_set_transfer_data_size(&c, DMA_SIZE_16);             // 16-bit txfers
        channel_config_set_read_increment(&c, false);                       // no read incrementing
        channel_config_set_write_increment(&c, false);                      // no write incrementing
        channel_config_set_chain_to(&c, resp_chan);                         // chain to response channel
        dma_channel_configure(
                req_chan,                                                       // Channel to be configured
                &c,                                                             // The configuration we just created
                &spi_get_hw(
                        (spi_inst_t *) this->SPI_PORT_)->dr,                                      // Write address (data channel read address)
                NULL,                                                           // Read address (POINTER TO AN ADDRESS)
                DATA_LENGTH,                                                    // Number of transfers
                false                                                           // Don't start immediately
        );

        // Set up the response channel
        dma_channel_config c2 = dma_channel_get_default_config(resp_chan);  // Default configs
        channel_config_set_transfer_data_size(&c2, DMA_SIZE_16);            // 16-bit txfers
        channel_config_set_read_increment(&c2, false);                      // yes read incrementing
        channel_config_set_write_increment(&c2, false);                     // no write incrementing
        channel_config_set_dreq(&c2, spi_get_dreq((spi_inst_t *) this->SPI_PORT_, false));        // DREQ paced by timer 0
        dma_channel_configure(
                resp_chan,                                                      // Channel to be configured
                &c2,                                                            // The configuration we just created
                &this->res,                                                           // write address (SPI data register)
                &spi_get_hw(
                        (spi_inst_t *) this->SPI_PORT_)->dr,                                      // The initial read address
                DATA_LENGTH,                                                    // Number of transfers
                false                                                           // Don't start immediately.
        );

        // Tell the DMA to raise IRQ line 0 when the channel finishes a block
        dma_channel_set_irq0_enabled(resp_chan, true);

        // Configure the processor to run AB_dmahandler() when DMA IRQ 0 is asserted
        irq_set_exclusive_handler(DMA_IRQ_0, magnetic_dmahandler);
        irq_set_enabled(DMA_IRQ_0, true);

        magnetic_dmahandler();
    }
    else if(this->encoder_type == ABEncoder){

        encoders_program_init(this->_encoder_pio, this->unused_sm, this->offset, this->_PIN_A, false);
        dma_channel_config c = dma_channel_get_default_config(unused_dma_channel);
        channel_config_set_read_increment(&c, false);
        channel_config_set_write_increment(&c, false);

        channel_config_set_dreq(&c, pio_get_dreq(this->_encoder_pio, this->unused_sm, false));
        dma_channel_configure(
                unused_dma_channel, &c,
                              &this->capture_buf,    // Destination pointer
                              &this->_encoder_pio->rxf[this->unused_sm],       // Source pointer
                              DATA_LENGTH,             // Number of transfers
                              true                // Start immediately
        );
        dma_channel_set_irq0_enabled(unused_dma_channel, true);
        irq_set_exclusive_handler(DMA_IRQ_0, AB_dmahandler);
        irq_set_enabled(DMA_IRQ_0, true);

    }
}

uint8_t encoder::parity_check(uint16_t data) {
    int count = 0;
    for (unsigned int i = 0; i < 15; i++) {
        if (data & (1 << i))
            count++;
    }
    if (count % 2 == 0)
        return 0;
    else
        return 1;
}

encoder::encoder(float* enc_pos, uint32_t BAUDRATE, int32_t encoder_resolution, float encoder_mm_per_revolution, spi_inst_t *SPIport, uint8_t MISOpin, uint8_t MOSIpin, uint8_t SCKpin , int8_t CSpin)
        : _encoder_resolution(encoder_resolution), _encoder_position(enc_pos), _encoder_mm_per_revolution(encoder_mm_per_revolution), SPI_PORT_(SPIport), PIN_MISO_(MISOpin), PIN_MOSI_(MOSIpin), PIN_SCK_(SCKpin), PIN_CS_(CSpin), SPI_BAUDRATE_(BAUDRATE), encoder_type(magneticEncoder) {

    instance = this;
    this->_encoder_mm_per_click = this->_encoder_mm_per_revolution / (float) this->_encoder_resolution;
}

encoder::encoder(float *enc_pos, PIO encoder_pio, int32_t encoder_resolution, float encoder_mm_per_revolution, uint8_t Apin, uint8_t Bpin, int8_t Zpin)
    :_encoder_resolution(encoder_resolution), _encoder_position(enc_pos), _encoder_mm_per_revolution(encoder_mm_per_revolution), _encoder_pio(encoder_pio), encoder_type(ABEncoder), _PIN_A(Apin), _PIN_B(Bpin), _PIN_Z(Zpin){

    instance = this;
    this->_encoder_mm_per_click = this->_encoder_mm_per_revolution / (float) this->_encoder_resolution;

}


int8_t encoder::initialize(){
    if(encoder_type == magneticEncoder) {
        // Init SPI communication
        spiInit();
        // TODO: wait for motor to be homes to set zero angle
        uint16_t new_zero_angle = 0;
        uint16_t final_angle = 0;
        do {
            uint16_t angle_read = 0;
            angle_read = this->readAngle();
            new_zero_angle += angle_read;
            this->writeZeroPosition(new_zero_angle);
            final_angle = this->readAngle();
        } while (final_angle != 0);
    }
    else if (encoder_type == ABEncoder){
        // Set up the state machine.
        this->offset = pio_add_program(this->_encoder_pio, &encoders_program);
        this->unused_sm = pio_claim_unused_sm(this->_encoder_pio, true);

    }
    else{
        return PICO_ERROR_INVALID_ARG;
    }
    dmaInit();
    return 0;
}


void inline encoder::enableCS() const{
    gpio_put(PIN_CS_, true);
}

void inline encoder::disableCS() const {
    gpio_put(PIN_CS_, false);
}
