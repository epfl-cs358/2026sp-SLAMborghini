#include "hcsr04_driver.h"

#ifdef ESP_PLATFORM
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#endif

#define ECHO_START_TIMEOUT_US 3000LL
#define ECHO_HIGH_TIMEOUT_US  6000LL
#define MIN_VALID_MM            20u
#define MAX_VALID_MM          1000u

static uint8_t s_trig_pin = 0;
static uint8_t s_echo_pin = 0;

void hcsr04_init(uint8_t trig_pin, uint8_t echo_pin)
{
    s_trig_pin = trig_pin;
    s_echo_pin = echo_pin;

#ifdef ESP_PLATFORM
    gpio_config_t trig = {
        .pin_bit_mask = 1ULL << trig_pin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&trig);
    gpio_set_level((gpio_num_t)trig_pin, 0);

    gpio_config_t echo = {
        .pin_bit_mask = 1ULL << echo_pin,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&echo);
#endif
}

float hcsr04_read_mm(void)
{
#ifdef ESP_PLATFORM
    gpio_set_level((gpio_num_t)s_trig_pin, 0);
    esp_rom_delay_us(2);
    gpio_set_level((gpio_num_t)s_trig_pin, 1);
    esp_rom_delay_us(10);
    gpio_set_level((gpio_num_t)s_trig_pin, 0);

    int64_t t0 = esp_timer_get_time();
    while (gpio_get_level((gpio_num_t)s_echo_pin) == 0) {
        if (esp_timer_get_time() - t0 > ECHO_START_TIMEOUT_US)
            return 0.0f;
    }

    int64_t rise = esp_timer_get_time();
    while (gpio_get_level((gpio_num_t)s_echo_pin) != 0) {
        if (esp_timer_get_time() - rise > ECHO_HIGH_TIMEOUT_US)
            return 0.0f;
    }

    float mm = (float)(esp_timer_get_time() - rise) * 0.1715f;
    if (mm < (float)MIN_VALID_MM || mm > (float)MAX_VALID_MM)
        return 0.0f;
    return mm;
#else
    return 0.0f;
#endif
}

bool hcsr04_read_u16_mm(uint16_t *out_mm)
{
    float mm = hcsr04_read_mm();
    if (mm <= 0.0f) {
        if (out_mm) *out_mm = 0u;
        return false;
    }
    if (out_mm) *out_mm = (uint16_t)(mm + 0.5f);
    return true;
}
