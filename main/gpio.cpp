/**
 * - Buttons: GPIO 25, 26, 27, 32, 33 (active LOW with pullup)
 * - LEDs: GPIO 2 (status), 23 (activity), 19 (playing)
 */

#include <driver/gpio.h>
#include <portmacro.h>
#include <FreeRTOSConfig.h>
#include <freertos/idf_additions.h>
#include <iostream>
#include "BLEHID.h"

// Button pins
#define BUTTON_PLAY_PAUSE   GPIO_NUM_25
#define BUTTON_NEXT         GPIO_NUM_26
#define BUTTON_PREV         GPIO_NUM_27
#define BUTTON_VOL_UP       GPIO_NUM_32
#define BUTTON_VOL_DOWN     GPIO_NUM_33

#define NUM_BUTTONS 5

// LED pins
#define LED_STATUS          GPIO_NUM_2
#define LED_ACTIVITY        GPIO_NUM_23
#define LED_PLAYING         GPIO_NUM_19

// Debounce time (milliseconds)
#define DEBOUNCE_MS         200

const gpio_num_t buttons[] = {BUTTON_PLAY_PAUSE, BUTTON_NEXT, BUTTON_PREV, BUTTON_VOL_UP, BUTTON_VOL_DOWN};

void ledOn(gpio_num_t pin) {
    gpio_set_level(pin, 1);
}

void ledOff(gpio_num_t pin) {
    gpio_set_level(pin, 0);
}

void ledBlink(gpio_num_t pin, uint32_t duration_ms) {
    ledOn(pin);
    vTaskDelay(duration_ms / portTICK_PERIOD_MS);
    ledOff(pin);
}

void initGPIO() {
    printf("Initializing GPIO...\n");
    
    // Configure button pins
    uint64_t button_mask = 0;
    for (int i = 0; i < NUM_BUTTONS; i++) {
        button_mask |= (1ULL << buttons[i]);
    }
    
    gpio_config_t btn_config = {
        .pin_bit_mask = button_mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn_config);
    
    // Configure LED pins
    gpio_config_t led_config = {
        .pin_bit_mask = (1ULL << LED_STATUS) | (1ULL << LED_ACTIVITY) | (1ULL << LED_PLAYING),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&led_config);
    
    // Turn off all LEDs
    ledOff(LED_STATUS);
    ledOff(LED_ACTIVITY);
    ledOff(LED_PLAYING);
    
    printf("✓ GPIO configured\n");
    
    // Power-on LED test
    for (int i = 0; i < 3; i++) {
        ledOn(LED_STATUS);
        ledOn(LED_ACTIVITY);
        ledOn(LED_PLAYING);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        ledOff(LED_STATUS);
        ledOff(LED_ACTIVITY);
        ledOff(LED_PLAYING);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}






