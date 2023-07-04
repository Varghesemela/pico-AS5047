//#define PICO_FLASH_SPI_CLKDIV   4

#include <cmath>
#include <cstring>
#include <string>
using namespace std;

#include "project_config.h"
#include "pins_pico_board.h"
#include "encoder_config.h"
#include "functions.h"

#include "pico/printf.h"
#include <pico/multicore.h>
#include <pico/sem.h>

#include "hardware/adc.h"
#include "encoder.h"
// #include "pico/unique_id.h"
// pico_unique_board_id_t id_out;

float encoder_position;
typedef struct can2040_msg CANMsg;
struct repeating_timer timer1, timer2;

// vars
int servo_angle, servo_current_limit;

int directions = encoder_dir;

float homing_position = 0;               // the absolute position of the loader when it is homed
volatile float added_offset = 0, homing_offset = homing_position;

volatile int print_done = 0, reach_flag = 0, global_confirm;
int global_count;
volatile float global_position;
float encoder_mm_per_revolution = (DEFAULT_LEADSCREW_PITCH);

#define USING_AMT_ENCODER
//#define USING_MAGNETIC_ENCODER

#ifdef USING_MAGNETIC_ENCODER
int32_t enc_resolution = 16384;
    encoder encoder_dummy(&encoder_position, 1000000, enc_resolution, encoder_mm_per_revolution, SPI_PORT, MISO, MOSI, SCK, CS);

#endif
#ifdef USING_AMT_ENCODER
int32_t enc_resolution = encoder_PPR * 4;
encoder encoder_dummy(&encoder_position, pio0, enc_resolution, encoder_mm_per_revolution, 2, 3, -1);

#endif

semaphore_t sem_timer;



inline void vSafePrint(const char *format, ...) {
    /* Custom vSafePrint function that takes the same arguments as vSafePrint. */
    if (!sem_acquire_timeout_us(&sem_timer, 10)){
        return;
    }
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
    sem_release(&sem_timer);

}

void GPIO_init() {
    gpio_init(LED_STATUS);
    gpio_set_dir(LED_STATUS, GPIO_OUT);
    gpio_put(LED_STATUS, LOW);

    gpio_init(22);
    gpio_set_dir(22, GPIO_OUT);
    gpio_put(22, HIGH);


}

#define BUFFER_LENGTH   50
char buffer[BUFFER_LENGTH];
const char s[2] = ",";
const char start_of_frame = '<', end_of_frame = '>';
string command_str, subcommand_str;
string data_str;
uint16_t buffer_index = 0, EOF_index = 0;
bool SOF_flag = false, EOF_flag = false;

union {
    uint8_t axis_val[4];
    float axis_fval;
} buffer_union;

// Core 1 interrupt Handler
void core1_interrupt_handler() {
    // Receive Raw Value, Convert and Print Temperature Value
    while (multicore_fifo_rvalid()){
     printf("Core 1");
    }
    multicore_fifo_clear_irq(); // Clear interrupt
}

/**
 * \brief Core 1 main loop
 *
 * This function serves as the main loop for the secondary core when initialised in the primary core.
 *
 */
[[noreturn]] void core1_entry() {
    // Configure Core 1 Interrupt
    multicore_fifo_clear_irq();
    multicore_lockout_victim_init();
    // irq_set_exclusive_handler(SIO_IRQ_PROC1, core1_interrupt_handler);
    // irq_set_enabled(SIO_IRQ_PROC1, true);

    encoder_dummy.initialize();
    absolute_time_t timeStamp;
    uint64_t timeElapsed_in_us;
    while (true) {
        auto encoder_pos = (homing_offset + added_offset + (float) (pow(-1, !directions)) * -(encoder_position));
        vSafePrint("encoder position: %f\n", encoder_pos);
        tight_loop_contents();
        timeElapsed_in_us = time_us_32();
        update_us_since_boot(&timeStamp, timeElapsed_in_us+(uint64_t)TICK_RATE);
        busy_wait_until(timeStamp);

        gpio_xor_mask(1<<LED_STATUS);

    }
}


bool get_block(struct repeating_timer *t) {
    // buffer_index = 0;
    // while(true){
    int c = getchar_timeout_us(0);
    // printf("%c\n", c);
    if (c != PICO_ERROR_TIMEOUT && buffer_index < BUFFER_LENGTH && !EOF_flag) {
        if (c == start_of_frame) {
            SOF_flag = true;
            command_str.clear();
            subcommand_str.clear();
//            data_str.clear();
            std::fill( std::begin(data_str), std::end(data_str), '\0');
            std::fill( std::begin(buffer), std::end(buffer), '\0' );
            buffer_index = 0;
        } else if (c == end_of_frame) {
            EOF_flag = true;
            EOF_index = buffer_index;

            command_str = strtok(buffer, s);
            if (command_str == "#RP64209"){
                reset_usb_boot(0, 0);
            }
            if(!is_only_alphabets(command_str)){
                printf("Invalid command, change later\n");
            }

            // break;
        }
        if ((SOF_flag) && (!EOF_flag) && (c != start_of_frame)) {
            buffer[buffer_index++] = (c & 0xFF);
        }

    } else {
        // break;
    }
    // }
    return true;
}


int main() {
//    set_sys_clock_khz(250000, true);
    stdio_init_all();

    GPIO_init();

    adc_init();
    adc_gpio_init(26);// Make sure GPIO is high-impedance, no pullups etc
    adc_select_input(0); //Or Select ADC input 1 (GPIO27)
    adc_set_clkdiv(25600);

    sleep_ms(3000);
//    canbus_setup();

    printf("Status: %d\n", add_repeating_timer_us(-10 * TICK_RATE, get_block, NULL, &timer1));

    // watchdog_enable(5000, 1);
    // while(!stdio_usb_connected());
    printf("Hello\n");
    sem_init(&sem_timer, 1, 1);

    multicore_launch_core1(core1_entry);

    while (true) {
        tight_loop_contents();
        busy_wait_ms(100);
        if (!EOF_flag) {

        } else {
            if (command_str == "encoder_value") {
                printf("current encoder position: %f\n", encoder_position);
            } else if (command_str == "u2") {
                printf(BOARD_TYPE
                "\n");
                printf("ok\n");
            } else if (command_str == "setzero"){
//                writeZeroPosition()
            } else if (command_str == "help") {
                printCommandlist();
            }
            buffer_index = 0;
            EOF_index = 0;
            memset(buffer, '\0', sizeof(buffer));
            SOF_flag = false;
            EOF_flag = false;
        }
    }
}
