#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "pico/time.h"

#include <stdint.h>
#include <math.h>

// Settings
#define SCREEN_TIMEOUT  30000
#define TOAST_TIME_INC  15
#define BAKE_TIME_INC   30
#define BAKE_TEMP_INC   25
#define LOOP_DELAY_MS   20
#define TEMP_HYSTERESIS 2.5
// LCD update interval (ms) to avoid blocking the main loop too long
#define LCD_UPDATE_MS 200

// Debug prints (set to 1 to enable)
#define DEBUG 1
#if DEBUG
#define DPRINTF(...) printf(__VA_ARGS__)
#else
#define DPRINTF(...) do {} while (0)
#endif

char* modes[] = {"     Toast      ", "      Bake      ", "    Passthru    "};
char* running_modes[] = {"  Toasting...   ", "   Baking...    ", "    Passthru    "};

int toast_time = 240; // seconds
int bake_time = 300;  // seconds
int bake_temp = 350;  // Fahrenheit
bool timer_counting = false;

absolute_time_t start_time;
int time_target = 0; // In milliseconds
int temp_target = 0; // Celsius
static absolute_time_t last_lcd_update;
static int last_display_seconds = -1;

// SPI Defines
#define SPI_PORT spi1
#define PIN_MISO 12
#define PIN_CS   13
#define PIN_SCK  10
#define PIN_MOSI 11

// I2C defines
#define I2C_PORT i2c0
#define I2C_SDA 4
#define I2C_SCL 5

#define PIN_RELAY 7

#define PIN_BTN_MODE  16
#define PIN_BTN_UP    17
#define PIN_BTN_DOWN  18
#define PIN_BTN_START 19

// commands
const int LCD_CLEARDISPLAY = 0x01;
const int LCD_RETURNHOME = 0x02;
const int LCD_ENTRYMODESET = 0x04;
const int LCD_DISPLAYCONTROL = 0x08;
const int LCD_CURSORSHIFT = 0x10;
const int LCD_FUNCTIONSET = 0x20;
const int LCD_SETCGRAMADDR = 0x40;
const int LCD_SETDDRAMADDR = 0x80;

// flags for display entry mode
const int LCD_ENTRYSHIFTINCREMENT = 0x01;
const int LCD_ENTRYLEFT = 0x02;

// flags for display and cursor control
const int LCD_BLINKON = 0x01;
const int LCD_CURSORON = 0x02;
const int LCD_DISPLAYON = 0x04;

// flags for display and cursor shift
const int LCD_MOVERIGHT = 0x04;
const int LCD_DISPLAYMOVE = 0x08;

// flags for function set
const int LCD_5x10DOTS = 0x04;
const int LCD_2LINE = 0x08;
const int LCD_8BITMODE = 0x10;

// flag for backlight control
const int LCD_BACKLIGHT = 0x08;
bool backlightEnabled = true;

const int LCD_ENABLE_BIT = 0x04;

// By default these LCD display drivers are on bus address 0x27
static int addr = 0x27;

// Modes for lcd_send_byte
#define LCD_CHARACTER  1
#define LCD_COMMAND    0

#define MAX_LINES      2
#define MAX_CHARS      16

/* Quick helper function for single byte transfers */
void i2c_write_byte(uint8_t val) {
#ifdef i2c_default
    i2c_write_blocking(i2c_default, addr, &val, 1, false);
#endif
}

void lcd_toggle_enable(uint8_t val) {
    // Toggle enable pin on LCD display
    // We cannot do this too quickly or things don't work
#define DELAY_US 600
    sleep_us(DELAY_US);
    i2c_write_byte(val | LCD_ENABLE_BIT);
    sleep_us(DELAY_US);
    i2c_write_byte(val & ~LCD_ENABLE_BIT);
    sleep_us(DELAY_US);
}

// The display is sent a byte as two separate nibble transfers
void lcd_send_byte(uint8_t val, int mode) {
    uint8_t high = mode | (val & 0xF0) | LCD_BACKLIGHT * backlightEnabled;
    uint8_t low = mode | ((val << 4) & 0xF0) | LCD_BACKLIGHT * backlightEnabled;

    i2c_write_byte(high);
    lcd_toggle_enable(high);
    i2c_write_byte(low);
    lcd_toggle_enable(low);
}

void lcd_clear(void) {
    lcd_send_byte(LCD_CLEARDISPLAY, LCD_COMMAND);
}

// go to location on LCD
void lcd_set_cursor(int line, int position) {
    int val = (line == 0) ? 0x80 + position : 0xC0 + position;
    lcd_send_byte(val, LCD_COMMAND);
}

static inline void lcd_char(char val) {
    lcd_send_byte(val, LCD_CHARACTER);
}

void lcd_string(const char *s) {
    while (*s) {
        lcd_char(*s++);
    }
}

void lcd_init() {
    lcd_send_byte(0x03, LCD_COMMAND);
    lcd_send_byte(0x03, LCD_COMMAND);
    lcd_send_byte(0x03, LCD_COMMAND);
    lcd_send_byte(0x02, LCD_COMMAND);

    lcd_send_byte(LCD_ENTRYMODESET | LCD_ENTRYLEFT, LCD_COMMAND);
    lcd_send_byte(LCD_FUNCTIONSET | LCD_2LINE, LCD_COMMAND);
    lcd_send_byte(LCD_DISPLAYCONTROL | LCD_DISPLAYON, LCD_COMMAND);
    lcd_clear();
}

void lcd_off() {
    backlightEnabled = false;
    lcd_send_byte(LCD_DISPLAYCONTROL, LCD_COMMAND);
}

void lcd_on() {
    backlightEnabled = true;
    lcd_send_byte(LCD_DISPLAYCONTROL | LCD_DISPLAYON, LCD_COMMAND);
}

float read_temp() {
    uint8_t buffer[2];

    gpio_put(PIN_CS, 0);
    sleep_ms(10);
    spi_read_blocking(SPI_PORT, 0, buffer, 2);
    sleep_ms(10);
    gpio_put(PIN_CS, 1);

    DPRINTF("Data: %d %d\n", buffer[0], buffer[1]);
    uint16_t raw = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
    raw = raw >> 3;
    return (float)raw * 0.25f;
}

void get_settings_str(uint8_t mode, uint8_t setting_option, char* str) {
    switch (mode) {
        case 0:
            snprintf(str, 17, "  Time: %02d:%02d   ", toast_time / 60, toast_time % 60);
            break;
        case 1:
            if (setting_option == 0)
                snprintf(str, 17, "   Temp: %3dF    ", bake_temp);
            else
                snprintf(str, 17, "   Time: %02d:%02d  ", bake_time / 60, bake_time % 60);
            break;
        default:
        case 2:
            snprintf(str, 17, "                ");
            break;
    }
}

void draw_lcd(uint8_t mode, uint8_t setting_option, bool running) {
    if (!running) {
        lcd_set_cursor(0, 0);
        lcd_string(modes[mode]);

        lcd_set_cursor(1, 0);
        char settings_str[17];
        get_settings_str(mode, setting_option, settings_str);
        lcd_string(settings_str);
    } else {
        lcd_set_cursor(0, 0);
        lcd_string(running_modes[mode]);

        lcd_set_cursor(1, 0);
        absolute_time_t current_time = get_absolute_time();
            int remaining_time = round((float)time_target / 1000.0f);
        switch (mode) {
            case 0:
                char time_str[17];
                snprintf(time_str, 17, "Time Left: %02d:%02d", remaining_time / 60, remaining_time % 60);
                lcd_string(time_str);
                break;
            case 1:
                break;
            case 2:
                lcd_string("   Running...   ");
                break;
        }
    }
}

// Force an immediate LCD update and refresh tracking state
static void lcd_force_update(uint8_t mode, uint8_t setting_option, bool running) {
    draw_lcd(mode, setting_option, running);
    last_lcd_update = get_absolute_time();
    last_display_seconds = running ? (int)roundf((float)time_target / 1000.0f) : -1;
}

// Update LCD only when visible seconds change or after a timeout
static void lcd_maybe_update(uint8_t mode, uint8_t setting_option, bool running) {
    int current_display_seconds = running ? (int)roundf((float)time_target / 1000.0f) : -1;
    int64_t since_lcd_ms = absolute_time_diff_us(last_lcd_update, get_absolute_time()) / 1000;
    if (!running || current_display_seconds != last_display_seconds || since_lcd_ms >= LCD_UPDATE_MS) {
        lcd_force_update(mode, setting_option, running);
    }
}

int main() {
    stdio_init_all();

    // SPI initialisation at 4MHz, 8 bits, mode 0 (CPOL=0, CPHA=0)
    spi_init(SPI_PORT, 4000000);
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    
    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
    // For more examples of SPI use see https://github.com/raspberrypi/pico-examples/tree/master/spi

    // Set up button pins
    gpio_set_function(PIN_BTN_MODE,  GPIO_FUNC_SIO);
    gpio_set_function(PIN_BTN_UP,    GPIO_FUNC_SIO);
    gpio_set_function(PIN_BTN_DOWN,  GPIO_FUNC_SIO);
    gpio_set_function(PIN_BTN_START, GPIO_FUNC_SIO);
    gpio_set_dir(PIN_BTN_MODE,  GPIO_IN);
    gpio_set_dir(PIN_BTN_UP,    GPIO_IN);
    gpio_set_dir(PIN_BTN_DOWN,  GPIO_IN);
    gpio_set_dir(PIN_BTN_START, GPIO_IN);
    gpio_pull_up(PIN_BTN_MODE);
    gpio_pull_up(PIN_BTN_UP);
    gpio_pull_up(PIN_BTN_DOWN);
    gpio_pull_up(PIN_BTN_START);

    gpio_set_function(PIN_RELAY, GPIO_FUNC_SIO);
    gpio_set_dir(PIN_RELAY, GPIO_OUT);

    // I2C Initialisation. Using it at 100Khz.
    i2c_init(I2C_PORT, 100000);
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    // For more examples of I2C use see https://github.com/raspberrypi/pico-examples/tree/master/i2c

    lcd_init();
    lcd_clear();
    last_lcd_update = get_absolute_time();
    last_display_seconds = -1;

    bool prev_mode_btn = false;
    bool mode_btn = false;

    bool prev_up_btn = false;
    bool up_btn = false;
    bool up_btn_stale = false;
    int up_btn_press_time = 0;

    bool prev_down_btn = false;
    bool down_btn = false;

    bool prev_start_btn = false;
    bool start_btn = false;

    uint8_t mode = 0;
    uint8_t setting_option = 0; // for bake mode: 0 = temp, 1 = time
    bool running = false;
    absolute_time_t screenTimeout = make_timeout_time_ms(SCREEN_TIMEOUT);

    absolute_time_t last_time = get_absolute_time();

    lcd_force_update(mode, setting_option, running);
    while (true) {
        last_time = get_absolute_time();
        sleep_ms(LOOP_DELAY_MS);

        if (!is_nil_time(screenTimeout) && absolute_time_min(get_absolute_time(), screenTimeout) == screenTimeout) {
            screenTimeout = nil_time;
            lcd_off();
        }

        prev_mode_btn = mode_btn;
        mode_btn = !gpio_get(PIN_BTN_MODE);
        
        prev_up_btn = up_btn;
        up_btn = !gpio_get(PIN_BTN_UP);
        up_btn_stale = up_btn_stale && prev_up_btn;
        up_btn_press_time = up_btn * (up_btn_press_time + absolute_time_diff_us(last_time, get_absolute_time()) / 1000);

        prev_down_btn = down_btn;
        down_btn = !gpio_get(PIN_BTN_DOWN);

        prev_start_btn = start_btn;
        start_btn = !gpio_get(PIN_BTN_START);

        if (!mode_btn && prev_mode_btn && !running) {
            if (is_nil_time(screenTimeout)) {
                lcd_on();
                screenTimeout = make_timeout_time_ms(SCREEN_TIMEOUT);
                continue;
            }
            screenTimeout = make_timeout_time_ms(SCREEN_TIMEOUT);

            mode = ++mode % (sizeof(modes) / sizeof(modes[0]));
            setting_option = 0;

            lcd_force_update(mode, setting_option, running);
        }

        DPRINTF("Up Button Stale: %d, Press Time: %d, Running: %d, Mode: %d\n", up_btn_stale, up_btn_press_time, running, mode);
        if (mode == 1 && !up_btn_stale && (up_btn_press_time >= 1000 / LOOP_DELAY_MS) && !running) {
            // Long press changes setting option in bake mode
            setting_option = (setting_option + 1) % 2;
            DPRINTF("long press happened\n");
            up_btn_stale = true;

            lcd_force_update(mode, setting_option, running);
        }

        if (!up_btn && prev_up_btn && !up_btn_stale && !running) {
            if (is_nil_time(screenTimeout)) {
                lcd_on();
                screenTimeout = make_timeout_time_ms(SCREEN_TIMEOUT);
                continue;
            }
            screenTimeout = make_timeout_time_ms(SCREEN_TIMEOUT);

            switch (mode) {
                case 0:
                    toast_time = MIN(toast_time + TOAST_TIME_INC, 600);
                    break;
                case 1:
                    if (setting_option == 0)
                        bake_temp = MIN(bake_temp + BAKE_TEMP_INC, 500);
                    else
                        bake_time = MIN(bake_time + BAKE_TIME_INC, 1200);
                    break;
            }

            lcd_force_update(mode, setting_option, running);
        }

        if (!down_btn && prev_down_btn && !running) {
            if (is_nil_time(screenTimeout)) {
                lcd_on();
                screenTimeout = make_timeout_time_ms(SCREEN_TIMEOUT);
                continue;
            }
            screenTimeout = make_timeout_time_ms(SCREEN_TIMEOUT);

            switch (mode) {
                case 0:
                    toast_time = MAX(toast_time - TOAST_TIME_INC, 30);
                    break;
                case 1:
                    if (setting_option == 0)
                        bake_temp = MAX(bake_temp - BAKE_TEMP_INC, 50);
                    else
                        bake_time = MAX(bake_time - BAKE_TIME_INC, 30);
                    break;
            }

            lcd_force_update(mode, setting_option, running);
        }

        if (!start_btn && prev_start_btn) {
            if (!running && is_nil_time(screenTimeout)) {
                lcd_on();
                screenTimeout = make_timeout_time_ms(SCREEN_TIMEOUT);
                continue;
            }
            if (!running) screenTimeout = make_timeout_time_ms(SCREEN_TIMEOUT);

            running = !running;
            if (running) {
                screenTimeout = nil_time;
                start_time = get_absolute_time();
                temp_target = (mode == 1) ? (int)((float)(bake_temp - 32) * (5.0f / 9.0f)) : 0;
                time_target = ((mode == 0) ? toast_time : bake_time) * 1000; // convert seconds -> ms
                timer_counting = mode == 0;
            } else {
                gpio_put(PIN_RELAY, 0);
                lcd_force_update(mode, setting_option, running);
                screenTimeout = make_timeout_time_ms(SCREEN_TIMEOUT);
            }
        }

        /* perform periodic LCD updates (also updated immediately on button events)
           Only redraw when the displayed seconds would change, or as a fallback
           at least every LCD_UPDATE_MS. This prevents a large blocking I2C write
           from happening every few loops and causing visible jumps. */
        lcd_maybe_update(mode, setting_option, running);

        /* compute elapsed time for the whole iteration and apply when running */
        int32_t delta_us = absolute_time_diff_us(last_time, get_absolute_time());
        int32_t delta_ms = (int32_t)((delta_us + 500) / 1000); // round to nearest ms
        if (running) {
            time_target -= delta_ms;
            DPRINTF("Timing: %d %d\n", to_ms_since_boot(last_time), to_ms_since_boot(get_absolute_time()));
            DPRINTF("Time Target: %d (%d) diff: %d\n", time_target, (int)roundf((float)time_target / 1000.0f), delta_ms);
            /* Diagnostic: compare real elapsed time (ms) with counted-down elapsed time (ms) */
            int32_t initial_ms = ((mode == 0) ? toast_time : bake_time) * 1000;
            int64_t real_elapsed_ms = absolute_time_diff_us(start_time, get_absolute_time()) / 1000;
            int32_t counted_elapsed_ms = initial_ms - time_target;
            DPRINTF("Elapsed real: %lld ms, counted: %d ms\n", (long long)real_elapsed_ms, counted_elapsed_ms);

            if (time_target <= 0) {
                gpio_put(PIN_RELAY, 0);
                running = false;
                lcd_force_update(mode, setting_option, running);
            }
        }
    }
}
