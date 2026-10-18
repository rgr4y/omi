#ifndef LED_H
#define LED_H

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

// LED color enum for PWM control
typedef enum {
    LED_RED,
    LED_GREEN,
    LED_BLUE
} led_color_t;

/**
 * @brief Initialize the LEDs
 *
 * Initializes the LEDs
 *
 * @return 0 if successful, negative errno code if error
 */
int led_start();
void set_led_red(bool on);
void set_led_green(bool on);
void set_led_blue(bool on);
void set_led_pwm(led_color_t color, uint8_t level);
void set_led_pwm_ns(led_color_t color, uint32_t pulse_width_ns);
uint32_t led_get_period_ns(led_color_t color);
void led_off(void);

#endif
