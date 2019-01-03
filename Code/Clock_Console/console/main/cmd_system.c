/* Console example — various system commands

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "esp_log.h"
#include "esp_console.h"
#include "esp_system.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "argtable3/argtable3.h"
#include "cmd_decl.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/i2c.h"

#include "driver/gpio.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#include "esp_types.h"
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"

#include "sdkconfig.h"

#ifdef CONFIG_FREERTOS_USE_STATS_FORMATTING_FUNCTIONS
#define WITH_TASKS_INFO 1
#endif

//Defined GPIO pins for each bit of output
#define B3 27
#define B2 33
#define B1 15
#define B0 32

//Define i2c parameters
#define DATA_LENGTH                        128              /*!<Data buffer length for test buffer*/

#define I2C_EXAMPLE_MASTER_SCL_IO          22               /*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO          23               /*!< gpio number for I2C master data  */
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_1        /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000           /*!< I2C master clock frequency */
#define ESP_SLAVE_ADDR                     0x70             /*!< ESP32 slave address, you can set any 7bit value */
#define WRITE_BIT                          I2C_MASTER_WRITE /*!< I2C master write */
#define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/


// v Define Servo parameters v
#define SERVO_MIN_PULSEWIDTH 500 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2500 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 180 //Maximum angle in degree upto which servo can rotate

static void register_free();
static void register_restart();
static void register_deep_sleep();
static void register_make();

// v Added the line below for assignment v
static void register_led();
static void register_hwclock();

// v Interrupt related volatiles v
volatile bool clock_stop = false;
volatile int secs = 0;
volatile int mins = 0;
volatile int hrs = 0;

// v GPIO interrupt parameters v
#define GPIO_INTERRUPT_PIN     33
#define GPIO_INPUT_PIN_SEL  (1ULL<<GPIO_INTERRUPT_PIN)
#define ESP_INTR_FLAG_DEFAULT 0

// v Timer interrupt parameters v
#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TIMER_INTERVAL_SEC   (1.0) // sample test interval for the first timer
#define TEST_WITHOUT_RELOAD   0
#define TEST_WITH_RELOAD      1

// v ISR Handlers v
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    // Includes 2 millisecond delay for debouncing
    vTaskDelay(2 / portTICK_RATE_MS);
    if (gpio_get_level(GPIO_INTERRUPT_PIN) == 0) {
        clock_stop = true;
    }
}

void IRAM_ATTR timer_group0_isr(void *para)
{
    // All we want to do is increase the seconds by 1 every trigger
    int timer_idx = (int) para;

    /* Clear the interrupt
       and update the alarm time for the timer with without reload */
    TIMERG0.int_clr_timers.t0 = 1;
    // TIMERG0.int_clr_timers.t1 = 1;

    /* After the alarm has been triggered
      we need enable it again, so it is triggered the next time */
    TIMERG0.hw_timer[timer_idx].config.alarm_en = TIMER_ALARM_EN;

    // ADD TO SECONDS.
    if (secs < 60){
        secs++;
    }
    if (secs == 60) {
        mins++;
        secs = 0;
    }
    if (mins == 60) {
        hrs++;
        mins = 0;
    }
    if (hrs == 13) {
        hrs = 1;
    }
}

static void example_tg0_timer_init(int timer_idx, 
    bool auto_reload, double timer_interval_sec)
{
    /* Select and initialize basic parameters of the timer */
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = auto_reload;
    timer_init(TIMER_GROUP_0, timer_idx, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(TIMER_GROUP_0, timer_idx, timer_interval_sec * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, timer_idx);
    timer_isr_register(TIMER_GROUP_0, timer_idx, timer_group0_isr, 
        (void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);

    timer_start(TIMER_GROUP_0, timer_idx);
}

#if WITH_TASKS_INFO
static void register_tasks();
#endif

static const uint16_t alphafonttable[] =  {

0b0000000000000001,
0b0000000000000010,
0b0000000000000100,
0b0000000000001000,
0b0000000000010000,
0b0000000000100000,
0b0000000001000000,
0b0000000010000000,
0b0000000100000000,
0b0000001000000000,
0b0000010000000000,
0b0000100000000000,
0b0001000000000000,
0b0010000000000000,
0b0100000000000000,
0b1000000000000000,
0b0000000000000000,
0b0000000000000000,
0b0000000000000000,
0b0000000000000000,
0b0000000000000000,
0b0000000000000000,
0b0000000000000000,
0b0000000000000000,
0b0001001011001001,
0b0001010111000000,
0b0001001011111001,
0b0000000011100011,
0b0000010100110000,
0b0001001011001000,
0b0011101000000000,
0b0001011100000000,
0b0000000000000000, //  
0b0000000000000110, // !
0b0000001000100000, // "
0b0001001011001110, // #
0b0001001011101101, // $
0b0000110000100100, // %
0b0010001101011101, // &
0b0000010000000000, // '
0b0010010000000000, // (
0b0000100100000000, // )
0b0011111111000000, // *
0b0001001011000000, // +
0b0000100000000000, // ,
0b0000000011000000, // -
0b0000000000000000, // .
0b0000110000000000, // /
0b0000110000111111, // 0
0b0000000000000110, // 1
0b0000000011011011, // 2
0b0000000010001111, // 3
0b0000000011100110, // 4
0b0010000001101001, // 5
0b0000000011111101, // 6
0b0000000000000111, // 7
0b0000000011111111, // 8
0b0000000011101111, // 9
0b0001001000000000, // :
0b0000101000000000, // ;
0b0010010000000000, // <
0b0000000011001000, // =
0b0000100100000000, // >
0b0001000010000011, // ?
0b0000001010111011, // @
0b0000000011110111, // A
0b0001001010001111, // B
0b0000000000111001, // C
0b0001001000001111, // D
0b0000000011111001, // E
0b0000000001110001, // F
0b0000000010111101, // G
0b0000000011110110, // H
0b0001001000000000, // I
0b0000000000011110, // J
0b0010010001110000, // K
0b0000000000111000, // L
0b0000010100110110, // M
0b0010000100110110, // N
0b0000000000111111, // O
0b0000000011110011, // P
0b0010000000111111, // Q
0b0010000011110011, // R
0b0000000011101101, // S
0b0001001000000001, // T
0b0000000000111110, // U
0b0000110000110000, // V
0b0010100000110110, // W
0b0010110100000000, // X
0b0001010100000000, // Y
0b0000110000001001, // Z
0b0000000000111001, // [
0b0010000100000000, // 
0b0000000000001111, // ]
0b0000110000000011, // ^
0b0000000000001000, // _
0b0000000100000000, // `
0b0001000001011000, // a
0b0010000001111000, // b
0b0000000011011000, // c
0b0000100010001110, // d
0b0000100001011000, // e
0b0000000001110001, // f
0b0000010010001110, // g
0b0001000001110000, // h
0b0001000000000000, // i
0b0000000000001110, // j
0b0011011000000000, // k
0b0000000000110000, // l
0b0001000011010100, // m
0b0001000001010000, // n
0b0000000011011100, // o
0b0000000101110000, // p
0b0000010010000110, // q
0b0000000001010000, // r
0b0010000010001000, // s
0b0000000001111000, // t
0b0000000000011100, // u
0b0010000000000100, // v
0b0010100000010100, // w
0b0010100011000000, // x
0b0010000000001100, // y
0b0000100001001000, // z
0b0000100101001001, // {
0b0001001000000000, // |
0b0010010010001001, // }
0b0000010100100000, // ~
0b0011111111111111,

};

void register_system()
{
    register_free();
    register_restart();
    register_deep_sleep();
    register_make();
    register_led();
    register_hwclock();
#if WITH_TASKS_INFO
    register_tasks();
#endif
}

static int restart(int argc, char** argv)
{
    ESP_LOGI(__func__, "Restarting");
    esp_restart();
}

static void register_restart()
{
    const esp_console_cmd_t cmd = {
        .command = "restart",
        .help = "Restart the program",
        .hint = NULL,
        .func = &restart,
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

/** 'led' command takes an integer value and flashes the number in binary on the LEDs */

static int led(int argc, char** argv)
{
    int lednum = atoi(argv[1]);
    int b3,b2,b1,b0;
    if (lednum > 15) {
    	printf("Number too large for number of bits available!\n");
    	return 0;
    }
    else {

    }
    gpio_pad_select_gpio(B3);
    gpio_pad_select_gpio(B2);
    gpio_pad_select_gpio(B1);
    gpio_pad_select_gpio(B0);
    gpio_set_direction(B3, GPIO_MODE_OUTPUT);
    gpio_set_direction(B2, GPIO_MODE_OUTPUT);
    gpio_set_direction(B1, GPIO_MODE_OUTPUT);
    gpio_set_direction(B0, GPIO_MODE_OUTPUT);
    b0 = lednum % 2;
    lednum = lednum / 2;
	b1 = lednum % 2;
	lednum = lednum / 2;
	b2 = lednum % 2;
	lednum = lednum / 2;
	b3 = lednum %2;
	gpio_set_level(B3, b3);
    gpio_set_level(B2, b2);
    gpio_set_level(B1, b1);
    gpio_set_level(B0, b0);
    return 0;
}

static void register_led()
{
    const esp_console_cmd_t cmd = {
        .command = "led",
        .help = "Display input number as binary on LEDs",
        .hint = NULL,
        .func = &led,
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

//i2c function

uint16_t * writeDigitAscii(uint8_t n, uint8_t a, uint16_t *displaybuffer){
    uint16_t font = (uint16_t)(alphafonttable[a]);

    displaybuffer[n] = font;

    return displaybuffer;
}

static esp_err_t i2c_example_master_write_slave_one(i2c_port_t i2c_num, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( ESP_SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

static void i2c_example_master_init()
{
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       0,0, 0);
}

void writeDisplay(uint16_t *displaybuffer){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( ESP_SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (uint8_t) 0x00, ACK_CHECK_EN);
    uint8_t i;
    for(i = 0; i < 8; i++){
        i2c_master_write_byte(cmd, displaybuffer[i] & 0xFF, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, displaybuffer[i] >> 8, ACK_CHECK_EN);
    }
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_1, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

}

static int hwclock(){
    // setup servos for minute display. (From example code)
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 15); // minutes
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, 32); // seconds
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);

    // setup GPIO pin for interrupt

    gpio_config_t io_conf;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    //negedge triggering since we're using a pull-up resistor
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode    
    io_conf.mode = GPIO_MODE_INPUT;
    gpio_config(&io_conf);
    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INTERRUPT_PIN, gpio_isr_handler, (void*) GPIO_INTERRUPT_PIN);

    int alhrs,almins;
    hrs = 0;
    mins = 0;
    alhrs = 0;
    almins = 0;

    //read only two characters from command line (one is the setting number, the other is the NULL character placed by scanf)
    char setting[2];
    char digits[3];
    uint32_t pwmm, pwms;
    example_tg0_timer_init(TIMER_0, TEST_WITH_RELOAD, TIMER_INTERVAL_SEC);

    //initialize i2c
    uint16_t* displaybuffer = (uint16_t*) malloc(DATA_LENGTH);

    i2c_example_master_init();

    i2c_example_master_write_slave_one(I2C_EXAMPLE_MASTER_NUM,(uint8_t)0x21);
    i2c_example_master_write_slave_one(I2C_EXAMPLE_MASTER_NUM, 0x80 | 0x01 | (0 << 1));

    // initialize LED gpio for alarm
    gpio_pad_select_gpio(12);
    gpio_set_direction(12, GPIO_MODE_OUTPUT);
    gpio_set_level(12, 0);

    while (1) {
        printf("\nPlease enter a number to choose from available commands:\n 0 = Exit\n 1 = Set time\n 2 = Set alarm\n 3 = Start clock\n");
        scanf("%1s",setting);
        int s = setting[0];
        clock_stop = false; // This resets the flag that the GPIO interrupt sets


        if (s == '0'){
            printf("\nGoodbye.\n");
            return 0;
        }

        else if (s == '1'){ // set time

            printf("Enter hours:\n");
            scanf("%2s",digits);
            if (atoi(digits) > 12) {
                printf("Invalid time!\n");
            } else {
                hrs = atoi(digits);
            }

            printf("Enter minutes:\n");
            scanf("%2s",digits);
            if (atoi(digits) > 59) {
                printf("Invalid time!\n");
            } else {
                mins = atoi(digits);
            }

            pwmm = (500 + ((1900 * (60 - mins) / 60)));
            pwms = (2400);
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, pwmm);
            mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, pwms);
            displaybuffer = writeDigitAscii(0, hrs/10 + 48, displaybuffer);
            displaybuffer = writeDigitAscii(1, hrs % 10 + 48, displaybuffer);
            displaybuffer = writeDigitAscii(2, mins/10 + 48, displaybuffer);
            displaybuffer = writeDigitAscii(3, mins % 10 + 48, displaybuffer);

            writeDisplay(displaybuffer);
        }

        else if (s == '2'){ // set alarm
            printf("Enter hours:\n");
            scanf("%2s",digits);
            if (atoi(digits) > 12) {
                printf("Invalid time!\n");
            } else {
                alhrs = atoi(digits);
            }

            printf("Enter minutes:\n");
            scanf("%2s",digits);
            if (atoi(digits) > 59) {
                printf("Invalid time!\n");
            } else {
                almins = atoi(digits);
            }
        }

        else if (s == '3'){ // start clock
            printf("\nClock started, press push-button at any time to stop.\n");
            // enable GPIO interrupts to stop clock after it starts.
            gpio_set_intr_type(GPIO_INTERRUPT_PIN, GPIO_INTR_NEGEDGE);
            timer_start(TIMER_GROUP_0, TIMER_0);
            secs = 0;
            while (!clock_stop) {
                // let the clock run
                // THREE LINE BELOW ADDED TO AVOID TASK WATCHDOG COMPLAINING
                TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
                TIMERG0.wdt_feed=1;
                TIMERG0.wdt_wprotect=0;
                // Send i2c stuff
                displaybuffer = writeDigitAscii(0, hrs/10 + 48, displaybuffer);
                displaybuffer = writeDigitAscii(1, hrs % 10 + 48, displaybuffer);
                displaybuffer = writeDigitAscii(2, mins/10 + 48, displaybuffer);
                displaybuffer = writeDigitAscii(3, mins % 10 + 48, displaybuffer);

                writeDisplay(displaybuffer);

                pwms = (500 + ((1900 * (60 - secs) / 60)));
                pwmm = (500 + ((1900 * (60 - mins) / 60)));
                mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, pwmm);
                mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, pwms);

                if ((mins == almins) && (hrs = alhrs)) {
                    gpio_set_level(12, 1);
                } else {
                    gpio_set_level(12, 0);
                }
            }
            printf("\nClock stoppped.\n");
        }

        else {
            printf("\nCommand not recognized.\n");
            // return 0;
        }
    }
    return 0;
}

static void register_hwclock(){
    const esp_console_cmd_t cmd = {
        .command = "hwclock",
        .help = "Set time on hardware clock.",
        .hint = NULL,
        .func = &hwclock,
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

static int free_mem(int argc, char** argv)
{
    printf("%d\n", esp_get_free_heap_size());
    return 0;
}

static void register_free()
{
    const esp_console_cmd_t cmd = {
        .command = "free",
        .help = "Get the total size of heap memory available",
        .hint = NULL,
        .func = &free_mem,
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

#if WITH_TASKS_INFO

static int tasks_info(int argc, char** argv)
{
    const size_t bytes_per_task = 40; /* see vTaskList description */
    char* task_list_buffer = malloc(uxTaskGetNumberOfTasks() * bytes_per_task);
    if (task_list_buffer == NULL) {
        ESP_LOGE(__func__, "failed to allocate buffer for vTaskList output");
        return 1;
    }
    fputs("Task Name\tStatus\tPrio\tHWM\tTask#", stdout);
#ifdef CONFIG_FREERTOS_VTASKLIST_INCLUDE_COREID
    fputs("\tAffinity", stdout);
#endif
    fputs("\n", stdout);
    vTaskList(task_list_buffer);
    fputs(task_list_buffer, stdout);
    free(task_list_buffer);
    return 0;
}

static void register_tasks()
{
    const esp_console_cmd_t cmd = {
        .command = "tasks",
        .help = "Get information about running tasks",
        .hint = NULL,
        .func = &tasks_info,
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

#endif

static struct {
    struct arg_int *wakeup_time;
    struct arg_int *wakeup_gpio_num;
    struct arg_int *wakeup_gpio_level;
    struct arg_end *end;
} deep_sleep_args;


static int deep_sleep(int argc, char** argv)
{
    int nerrors = arg_parse(argc, argv, (void**) &deep_sleep_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, deep_sleep_args.end, argv[0]);
        return 1;
    }
    if (deep_sleep_args.wakeup_time->count) {
        uint64_t timeout = 1000ULL * deep_sleep_args.wakeup_time->ival[0];
        ESP_LOGI(__func__, "Enabling timer wakeup, timeout=%lluus", timeout);
        ESP_ERROR_CHECK( esp_sleep_enable_timer_wakeup(timeout) );
    }
    if (deep_sleep_args.wakeup_gpio_num->count) {
        int io_num = deep_sleep_args.wakeup_gpio_num->ival[0];
        if (!rtc_gpio_is_valid_gpio(io_num)) {
            ESP_LOGE(__func__, "GPIO %d is not an RTC IO", io_num);
            return 1;
        }
        int level = 0;
        if (deep_sleep_args.wakeup_gpio_level->count) {
            level = deep_sleep_args.wakeup_gpio_level->ival[0];
            if (level != 0 && level != 1) {
                ESP_LOGE(__func__, "Invalid wakeup level: %d", level);
                return 1;
            }
        }
        ESP_LOGI(__func__, "Enabling wakeup on GPIO%d, wakeup on %s level",
                io_num, level ? "HIGH" : "LOW");

        ESP_ERROR_CHECK( esp_sleep_enable_ext1_wakeup(1ULL << io_num, level) );
    }
    rtc_gpio_isolate(GPIO_NUM_12);
    esp_deep_sleep_start();
}

static void register_deep_sleep()
{
    deep_sleep_args.wakeup_time =
            arg_int0("t", "time", "<t>", "Wake up time, ms");
    deep_sleep_args.wakeup_gpio_num =
            arg_int0(NULL, "io", "<n>",
                     "If specified, wakeup using GPIO with given number");
    deep_sleep_args.wakeup_gpio_level =
            arg_int0(NULL, "io_level", "<0|1>", "GPIO level to trigger wakeup");
    deep_sleep_args.end = arg_end(3);

    const esp_console_cmd_t cmd = {
        .command = "deep_sleep",
        .help = "Enter deep sleep mode. "
                "Two wakeup modes are supported: timer and GPIO. "
                "If no wakeup option is specified, will sleep indefinitely.",
        .hint = NULL,
        .func = &deep_sleep,
        .argtable = &deep_sleep_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

/** This command helps maintain sanity when testing console example from a console */

static int make(int argc, char** argv)
{
    int count = REG_READ(RTC_CNTL_STORE0_REG);
    if (++count >= 3) {
        printf("This is not the console you are looking for.\n");
        return 0;
    }
    REG_WRITE(RTC_CNTL_STORE0_REG, count);

    const char* make_output =
R"(LD build/console.elf
esptool.py v2.1-beta1
)";

    const char* flash_output[] = {
R"(Flashing binaries to serial port )" CONFIG_ESPTOOLPY_PORT R"( (app at offset 0x10000)...
esptool.py v2.1-beta1
Connecting....
)",
R"(Chip is ESP32D0WDQ6 (revision 0)
Uploading stub...
Running stub...
Stub running...
Changing baud rate to 921600
Changed.
Configuring flash size...
Auto-detected Flash size: 4MB
Flash params set to 0x0220
Compressed 15712 bytes to 9345...
)",
R"(Wrote 15712 bytes (9345 compressed) at 0x00001000 in 0.1 seconds (effective 1126.9 kbit/s)...
Hash of data verified.
Compressed 333776 bytes to 197830...
)",
R"(Wrote 333776 bytes (197830 compressed) at 0x00010000 in 3.3 seconds (effective 810.3 kbit/s)...
Hash of data verified.
Compressed 3072 bytes to 82...
)",
R"(Wrote 3072 bytes (82 compressed) at 0x00008000 in 0.0 seconds (effective 1588.4 kbit/s)...
Hash of data verified.
Leaving...
Hard resetting...
)"
    };

    const char* monitor_output =
R"(MONITOR
)" LOG_COLOR_W R"(--- idf_monitor on )" CONFIG_ESPTOOLPY_PORT R"( 115200 ---
--- Quit: Ctrl+] | Menu: Ctrl+T | Help: Ctrl+T followed by Ctrl+H --
)" LOG_RESET_COLOR;

    bool need_make = false;
    bool need_flash = false;
    bool need_monitor = false;
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "all") == 0) {
            need_make = true;
        } else if (strcmp(argv[i], "flash") == 0) {
            need_make = true;
            need_flash = true;
        } else if (strcmp(argv[i], "monitor") == 0) {
            need_monitor = true;
        } else if (argv[i][0] == '-') {
            /* probably -j option */
        } else if (isdigit((int) argv[i][0])) {
            /* might be an argument to -j */
        } else {
            printf("make: *** No rule to make target `%s'.  Stop.\n", argv[i]);
            /* Technically this is an error, but let's not spoil the output */
            return 0;
        }
    }
    if (argc == 1) {
        need_make = true;
    }
    if (need_make) {
        printf("%s", make_output);
    }
    if (need_flash) {
        size_t n_items = sizeof(flash_output) / sizeof(flash_output[0]);
        for (int i = 0; i < n_items; ++i) {
            printf("%s", flash_output[i]);
            vTaskDelay(200/portTICK_PERIOD_MS);
        }
    }
    if (need_monitor) {
        printf("%s", monitor_output);
        esp_restart();
    }
    return 0;
}

static void register_make()
{
    const esp_console_cmd_t cmd = {
        .command = "make",
        .help = NULL, /* hide from 'help' output */
        .hint = "all | flash | monitor",
        .func = &make,
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}


