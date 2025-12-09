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
#define LONG_PRESS_MS   200
// LCD update interval (ms) to avoid blocking the main loop too long
#define LCD_UPDATE_MS   200

#define MIN_TEMP_REFRESH_US 220000

#define ACTION_BEEP_LENGTH   50
#define START_BEEP_LENGTH    200 
#define COMPLETE_BEEP_LENGTH 500

// Debug prints (set to 1 to enable). Keep disabled by default to avoid
// expensive blocking stdio calls in tight loops.
#define DEBUG 1
#if DEBUG
#define DPRINTF(...) printf(__VA_ARGS__)
#else
#define DPRINTF(...) do {} while (0)
#endif

/* --- Global configuration and state --- */
static const char* const modes[] = {"     Toast      ", "      Bake      ", "    Passthru    "};
static const char* const running_modes[] = {"  Toasting...   ", "   Baking...    ", "    Passthru    "};

static int toast_time = 10; // seconds
static int bake_time = 300;  // seconds
static int bake_temp = 82;  // Fahrenheit

/**
 * 0: Preheating
 * 1: Ready
 * 2: Cooking
 */
static uint8_t heating_stage = 0;
static uint8_t prev_heating_stage = 0;

static absolute_time_t start_time;
static int time_target = 0; // In milliseconds
static int temp_target = 0; // Celsius
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

#define PIN_BUZZER 20

// LCD constants
enum {
    LCD_CLEARDISPLAY = 0x01,
    LCD_RETURNHOME = 0x02,
    LCD_ENTRYMODESET = 0x04,
    LCD_DISPLAYCONTROL = 0x08,
    LCD_CURSORSHIFT = 0x10,
    LCD_FUNCTIONSET = 0x20,
    LCD_SETCGRAMADDR = 0x40,
    LCD_SETDDRAMADDR = 0x80,

    LCD_ENTRYSHIFTINCREMENT = 0x01,
    LCD_ENTRYLEFT = 0x02,

    LCD_BLINKON = 0x01,
    LCD_CURSORON = 0x02,
    LCD_DISPLAYON = 0x04,

    LCD_MOVERIGHT = 0x04,
    LCD_DISPLAYMOVE = 0x08,

    LCD_5x10DOTS = 0x04,
    LCD_2LINE = 0x08,
    LCD_8BITMODE = 0x10,

    LCD_BACKLIGHT = 0x08,
    LCD_ENABLE_BIT = 0x04,
};

static bool backlightEnabled = true;
static int lcd_addr = 0x27; // default I2C addr

#define LCD_CHARACTER  1
#define LCD_COMMAND    0

#define MAX_LINES      2
#define MAX_CHARS      16

static float current_temp = -1;
static absolute_time_t last_temp_check = 0;

/**
 * Updates the current temperature
 * @returns Whether the temperature was update (Irrespective of whether it was changed)
 */
static bool update_temp() {
    if (!is_nil_time(last_temp_check) && absolute_time_diff_us(last_temp_check, get_absolute_time()) < MIN_TEMP_REFRESH_US) {
        return false;
    }
    uint8_t buffer[2];

    gpio_put(PIN_CS, 0);
    spi_read_blocking(SPI_PORT, 0, buffer, 2);
    gpio_put(PIN_CS, 1);

    last_temp_check = get_absolute_time();

    uint16_t raw = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
    raw = raw >> 3;
    current_temp = (float)raw * 0.25f;
    return true;
}

/* --- Button helper structure and functions --- */
typedef struct ButtonState {
    uint pin;
    bool prev;
    bool cur;
    bool stale; // used for the up-button long-press logic
    int press_time_ms;
} ButtonState;

static inline void button_init(ButtonState *b, uint pin) {
    b->pin = pin;
    b->prev = false;
    b->cur = false;
    b->stale = false;
    b->press_time_ms = 0;
}

static inline void button_update(ButtonState *b, int delta_ms) {
    b->prev = b->cur;
    b->cur = !gpio_get(b->pin); // active-low buttons
    // maintain stale flag similar to original behaviour
    b->stale = b->stale && b->prev;
    if (b->cur) {
        b->press_time_ms += delta_ms;
    } else {
        b->press_time_ms = 0;
    }
}

/* --- Minimal I2C helper (single byte) --- */
void i2c_write_byte(uint8_t val) {
#ifdef i2c_default
    i2c_write_blocking(i2c_default, lcd_addr, &val, 1, false);
#endif
}

/* --- LCD helpers --- */
static void lcd_toggle_enable(uint8_t val) {
    const uint32_t DELAY_US = 600;
    sleep_us(DELAY_US);
    i2c_write_byte(val | LCD_ENABLE_BIT);
    sleep_us(DELAY_US);
    i2c_write_byte(val & ~LCD_ENABLE_BIT);
    sleep_us(DELAY_US);
}

static void lcd_send_byte(uint8_t val, int mode) {
    uint8_t high = mode | (val & 0xF0) | (LCD_BACKLIGHT * backlightEnabled);
    uint8_t low = mode | ((val << 4) & 0xF0) | (LCD_BACKLIGHT * backlightEnabled);

    i2c_write_byte(high);
    lcd_toggle_enable(high);
    i2c_write_byte(low);
    lcd_toggle_enable(low);
}

static void lcd_clear(void) { lcd_send_byte(LCD_CLEARDISPLAY, LCD_COMMAND); }

static void lcd_set_cursor(int line, int position) {
    int val = (line == 0) ? 0x80 + position : 0xC0 + position;
    lcd_send_byte(val, LCD_COMMAND);
}

static inline void lcd_char(char val) { lcd_send_byte((uint8_t)val, LCD_CHARACTER); }

static void lcd_string(const char *s) {
    while (*s) lcd_char(*s++);
}

static void lcd_init(void) {
    lcd_send_byte(0x03, LCD_COMMAND);
    lcd_send_byte(0x03, LCD_COMMAND);
    lcd_send_byte(0x03, LCD_COMMAND);
    lcd_send_byte(0x02, LCD_COMMAND);

    lcd_send_byte(LCD_ENTRYMODESET | LCD_ENTRYLEFT, LCD_COMMAND);
    lcd_send_byte(LCD_FUNCTIONSET | LCD_2LINE, LCD_COMMAND);
    lcd_send_byte(LCD_DISPLAYCONTROL | LCD_DISPLAYON, LCD_COMMAND);
    lcd_clear();
}

static void lcd_off(void) {
    backlightEnabled = false;
    lcd_send_byte(LCD_DISPLAYCONTROL, LCD_COMMAND);
}

static void lcd_on(void) {
    backlightEnabled = true;
    lcd_send_byte(LCD_DISPLAYCONTROL | LCD_DISPLAYON, LCD_COMMAND);
}

/* --- Application display and formatting helpers --- */
static void get_settings_str(uint8_t mode, uint8_t setting_option, char* str) {
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

static void draw_lcd(uint8_t mode, uint8_t setting_option, bool running) {
    if (!running) {
        lcd_set_cursor(0, 0);
        lcd_string(modes[mode]);

        lcd_set_cursor(1, 0);
        char settings_str[17];
        get_settings_str(mode, setting_option, settings_str);
        lcd_string(settings_str);
    } else {
        lcd_set_cursor(0, 0);
        if (mode != 1) {
            lcd_string(running_modes[mode]);
        } else {
            switch (heating_stage) {
                case 0:
                    lcd_string(" Preheating...  ");
                    break;
                case 1:
                    lcd_string("Ready:Press MODE");
                    break;
                case 2:
                    lcd_string(running_modes[mode]);
            }
            lcd_string(" Preheating...  ");
        }

        lcd_set_cursor(1, 0);
        float current_temp_f = current_temp * (9.0f / 5.0f) + 32;

        int remaining_time = (int)round((float)time_target / 1000.0f);

        DPRINTF("Temp %f, Time: %d\n", current_temp_f, remaining_time);
        switch (mode) {
            case 0: {
                char time_str[17];
                snprintf(time_str, 17, "Time Left: %02d:%02d", remaining_time / 60, remaining_time % 60);
                lcd_string(time_str);
                break;
            }
            case 1:
                char status_str[17];
                snprintf(status_str, 17, "%6.2fF    %02d:%02d", current_temp_f, remaining_time / 60, remaining_time % 60);
                lcd_string(status_str);
                break;
            case 2:
                lcd_string("   Running...   ");
                break;
        }
    }
}

/* Force an immediate LCD update and refresh tracking state */
static void lcd_force_update(uint8_t mode, uint8_t setting_option, bool running) {
    draw_lcd(mode, setting_option, running);
    last_lcd_update = get_absolute_time();
    last_display_seconds = running ? (int)roundf((float)time_target / 1000.0f) : -1;
}

/* Update LCD only when visible seconds change or after a timeout */
static void lcd_maybe_update(uint8_t mode, uint8_t setting_option, bool running) {
    int current_display_seconds = running ? (int)roundf((float)time_target / 1000.0f) : -1;
    int64_t since_lcd_ms = absolute_time_diff_us(last_lcd_update, get_absolute_time()) / 1000;
    if (current_display_seconds != last_display_seconds || since_lcd_ms >= LCD_UPDATE_MS) {
        lcd_force_update(mode, setting_option, running);
    }
}

static bool beeping = false;
int64_t beep_callback(alarm_id_t id, void* user_data) {
    DPRINTF("Stopping Beep\n");
    gpio_put(PIN_BUZZER, 0);
    beeping = false;
    cancel_alarm(id);
}

// Beep the buzzer for x milliseconds
static void beep(int ms, bool synchronus) {
    DPRINTF("Wanting to beep. Currently active: %d\n", beeping);
    if (beeping) return;

    gpio_put(PIN_BUZZER, 1);
    if (synchronus) {
        sleep_ms(ms);
        gpio_put(PIN_BUZZER, 0);
    } else {
        beeping = true;
        add_alarm_in_ms(ms, beep_callback, NULL, false);
    }
}

/* --- Initialization split out for clarity --- */
static void init_spi_and_sensors(void) {
    spi_init(SPI_PORT, 4000000);
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
}

static void init_buttons(ButtonState *mode_btn, ButtonState *up_btn, ButtonState *down_btn, ButtonState *start_btn) {
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

    button_init(mode_btn, PIN_BTN_MODE);
    button_init(up_btn, PIN_BTN_UP);
    button_init(down_btn, PIN_BTN_DOWN);
    button_init(start_btn, PIN_BTN_START);
}

static void init_relay() {
    gpio_set_function(PIN_RELAY, GPIO_FUNC_SIO);
    gpio_set_dir(PIN_RELAY, GPIO_OUT);
}

static void init_buzzer() {
    gpio_set_function(PIN_BUZZER, GPIO_FUNC_SIO);
    gpio_set_dir(PIN_BUZZER, GPIO_OUT);
}

static void init_i2c_and_lcd(void) {
    i2c_init(I2C_PORT, 100000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    lcd_init();
    lcd_clear();
    last_lcd_update = get_absolute_time();
    last_display_seconds = -1;
}

/* --- Main loop helpers: button handlers and timer processing --- */
static void handle_mode_button(ButtonState *b, uint8_t *mode, uint8_t *setting_option, bool running, absolute_time_t *screen_timeout) {
    if (b->cur && !b->prev && !running) {
        if (is_nil_time(*screen_timeout)) {
            lcd_on();
            *screen_timeout = make_timeout_time_ms(SCREEN_TIMEOUT);
            return;
        }
        *screen_timeout = make_timeout_time_ms(SCREEN_TIMEOUT);

        *mode = ++(*mode) % (sizeof(modes) / sizeof(modes[0]));
        *setting_option = 0;
        beep(ACTION_BEEP_LENGTH, false);

        lcd_force_update(*mode, *setting_option, running);
    }
}

static void handle_up_button(ButtonState *b, uint8_t mode, uint8_t *setting_option, bool running, absolute_time_t *screen_timeout) {
    if (mode == 1 && !b->stale && b->press_time_ms >= LONG_PRESS_MS && !running) {
        // Long press changes setting option in bake mode
        *setting_option = (uint8_t)((*setting_option + 1) % 2);
        b->stale = true;

        lcd_force_update(mode, *setting_option, running);
        return;
    }

    // Wake screen on press (rising edge) — feel responsive
    if (b->cur && !b->prev) {
        if (is_nil_time(*screen_timeout)) {
            lcd_on();
            *screen_timeout = make_timeout_time_ms(SCREEN_TIMEOUT);
            b->stale = true;
            return;
        }
        *screen_timeout = make_timeout_time_ms(SCREEN_TIMEOUT);

        beep(ACTION_BEEP_LENGTH, false);
    }

    // Short press on release
    if (!b->cur && b->prev && !b->stale && !running) {
        if (is_nil_time(*screen_timeout)) {
            lcd_on();
            *screen_timeout = make_timeout_time_ms(SCREEN_TIMEOUT);
            return;
        }
        *screen_timeout = make_timeout_time_ms(SCREEN_TIMEOUT);

        switch (mode) {
            case 0:
                toast_time = MIN(toast_time + TOAST_TIME_INC, 600);
                break;
            case 1:
                if (*setting_option == 0)
                    bake_temp = MIN(bake_temp + BAKE_TEMP_INC, 500);
                else
                    bake_time = MIN(bake_time + BAKE_TIME_INC, 1200);
                break;
        }

        lcd_force_update(mode, *setting_option, running);
    }
}

static void handle_down_button(ButtonState *b, uint8_t mode, uint8_t setting_option, bool running, absolute_time_t *screen_timeout) {
    if (b->cur && !b->prev && !running) {
        if (is_nil_time(*screen_timeout)) {
            lcd_on();
            *screen_timeout = make_timeout_time_ms(SCREEN_TIMEOUT);
            return;
        }
        *screen_timeout = make_timeout_time_ms(SCREEN_TIMEOUT);

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
        beep(ACTION_BEEP_LENGTH, false);

        lcd_force_update(mode, setting_option, running);
    }
}

static void handle_start_button(ButtonState *b, uint8_t mode, uint8_t setting_option, bool *running, absolute_time_t *screen_timeout) {
    if (b->cur && !b->prev) {
        if (!*running && is_nil_time(*screen_timeout)) {
            lcd_on();
            *screen_timeout = make_timeout_time_ms(SCREEN_TIMEOUT);
            return;
        }
        if (!*running) *screen_timeout = make_timeout_time_ms(SCREEN_TIMEOUT);

        *running = !*running;
        beep(*running ? START_BEEP_LENGTH : ACTION_BEEP_LENGTH, false);

        if (*running) {
            *screen_timeout = nil_time;
            start_time = get_absolute_time();
            temp_target = (mode == 1) ? (int)((float)(bake_temp - 32) * (5.0f / 9.0f)) : 260;
            time_target = ((mode == 0) ? toast_time : bake_time) * 1000; // convert seconds -> ms
            heating_stage = mode == 0 ? 2 : 0; // Skip preheat for toast operation
        } else {
            DPRINTF("Button stopped\n");
            gpio_put(PIN_RELAY, 0);
            lcd_force_update(mode, setting_option, *running);
            *screen_timeout = make_timeout_time_ms(SCREEN_TIMEOUT);
        }
    }
}

static void process_cycle(bool *running, uint8_t mode, uint8_t setting_option, absolute_time_t* screen_timeout, ButtonState* modeBtn) {
    prev_heating_stage = heating_stage;
    if (heating_stage == 0 && current_temp >= temp_target - TEMP_HYSTERESIS) {
        heating_stage = 1;
        beep(500, false);
    }
    
    if (heating_stage == 1 && modeBtn->cur && !modeBtn->prev) {
        beep(ACTION_BEEP_LENGTH, false);
        heating_stage = 2;
    }

    if (current_temp <= temp_target - TEMP_HYSTERESIS) {
        gpio_put(PIN_RELAY, 1);
    } else if (current_temp >= temp_target + TEMP_HYSTERESIS) {
        gpio_put(PIN_RELAY, 0);
    }
    
    if (time_target <= 0) {
        gpio_put(PIN_RELAY, 0);
        *running = false;
        lcd_force_update(mode, setting_option, *running);

        DPRINTF("Completed Cycle %d\n");
        beep(COMPLETE_BEEP_LENGTH, true);
        sleep_ms(COMPLETE_BEEP_LENGTH);
        beep(COMPLETE_BEEP_LENGTH, true);
        sleep_ms(COMPLETE_BEEP_LENGTH);
        beep(COMPLETE_BEEP_LENGTH, true);

        *screen_timeout = make_timeout_time_ms(SCREEN_TIMEOUT);
    }
}

int main(void) {
    stdio_init_all();

    // Initialize peripherals
    init_spi_and_sensors();

    ButtonState mode_btn, up_btn, down_btn, start_btn;
    init_buttons(&mode_btn, &up_btn, &down_btn, &start_btn);
    init_relay();
    init_buzzer();
    init_i2c_and_lcd();

    uint8_t mode = 0;
    uint8_t setting_option = 0; // for bake mode: 0 = temp, 1 = time
    bool running = false;
    absolute_time_t screen_timeout = make_timeout_time_ms(SCREEN_TIMEOUT);

    lcd_force_update(mode, setting_option, running);

    absolute_time_t last_time = get_absolute_time();

    while (true) {
        absolute_time_t loop_start = get_absolute_time();
        sleep_ms(LOOP_DELAY_MS);
        int32_t delta_us = absolute_time_diff_us(loop_start, get_absolute_time());
        int32_t delta_ms = (int32_t)((delta_us + 500) / 1000);

        if (!is_nil_time(screen_timeout) && absolute_time_min(get_absolute_time(), screen_timeout) == screen_timeout) {
            screen_timeout = nil_time;
            lcd_off();
        }

        update_temp();

        // Update buttons with the measured delta
        button_update(&mode_btn, delta_ms);
        button_update(&up_btn, delta_ms);
        button_update(&down_btn, delta_ms);
        button_update(&start_btn, delta_ms);

        // Handle events
        handle_mode_button(&mode_btn, &mode, &setting_option, running, &screen_timeout);
        handle_up_button(&up_btn, mode, &setting_option, running, &screen_timeout);
        handle_down_button(&down_btn, mode, setting_option, running, &screen_timeout);
        handle_start_button(&start_btn, mode, setting_option, &running, &screen_timeout);

        // Apply timer countdown when running
        if (running) {
            // Update LCD periodically or when needed
            lcd_maybe_update(mode, setting_option, running);
            
            process_cycle(&running, mode, setting_option, &screen_timeout, &mode_btn);
            int32_t delta_us = absolute_time_diff_us(loop_start, get_absolute_time());
            int32_t delta_ms = (int32_t)((delta_us + 500) / 1000);

            time_target -= delta_ms * (heating_stage == 2);
        }
    }
}
