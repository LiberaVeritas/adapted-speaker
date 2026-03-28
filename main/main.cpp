/**
 * BLE Media Remote - Simple Auto-Reconnect
 * 
 * Features:
 * - Auto-reconnect after power cycle
 * - Bonds stored in NVS
 * - No power management (always on)
 */

#include "BLEHID.h"
#include "nvs_flash.h"
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

/* ========================================================================
 * GPIO CONFIGURATION
 * ======================================================================== */

#define INPUT_PLAY_PAUSE     GPIO_NUM_18
#define INPUT_PREVIOUS_TRACK GPIO_NUM_19
#define INPUT_NEXT_TRACK     GPIO_NUM_20
#define INPUT_PIN_SEL        ((1ULL<<INPUT_PLAY_PAUSE) | (1ULL<<INPUT_PREVIOUS_TRACK) | (1ULL<<INPUT_NEXT_TRACK))

#define ESP_INTR_FLAG_DEFAULT 0

/* ========================================================================
 * GLOBAL OBJECTS
 * ======================================================================== */

MusicRemote remote;
static QueueHandle_t gpio_evt_queue = NULL;

/* ========================================================================
 * GPIO ISR
 * ======================================================================== */

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

/* ========================================================================
 * GPIO SETUP
 * ======================================================================== */

void input_setup(void)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.pin_bit_mask = INPUT_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
    
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    
    printf("✓ GPIO configured\n");
}

/* ========================================================================
 * BLE SETUP
 * ======================================================================== */

void setup() {
    printf("\n");
    printf("╔════════════════════════════════════════════╗\n");
    printf("║  BLE Media Remote with Auto-Reconnect     ║\n");
    printf("╚════════════════════════════════════════════╝\n");
    printf("\n");
    
    remote.onConnect([]() { 
        printf("✓ Connected\n");
    });
    
    remote.onDisconnect([]() { 
        printf("✗ Disconnected - will auto-reconnect\n");
    });
    
    remote.begin();
}

/* ========================================================================
 * BUTTON TASK
 * ======================================================================== */

static void main_task(void* arg)
{
    int64_t last_time = 0;
    const int64_t debounce_us = 50 * 1000;
    gpio_num_t io_num;
    
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            int64_t now = esp_timer_get_time();
            if (now - last_time < debounce_us) {
                continue;
            }
            last_time = now;
            
            switch(io_num) {
                case INPUT_PLAY_PAUSE:
                    printf("Button: Play/Pause\n");
                    remote.write(KEY_MEDIA_PLAY_PAUSE);
                    break;
                    
                case INPUT_PREVIOUS_TRACK:
                    printf("Button: Previous\n");
                    remote.write(KEY_MEDIA_PREVIOUS_TRACK);
                    break;
                    
                case INPUT_NEXT_TRACK:
                    printf("Button: Next\n");
                    remote.write(KEY_MEDIA_NEXT_TRACK);
                    break;
				default:
					break;
            }
            
            while (gpio_get_level(io_num) == 0) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            
            vTaskDelay(pdMS_TO_TICKS(50));
            xQueueReset(gpio_evt_queue);
        }
    }
}

/* ========================================================================
 * MAIN
 * ======================================================================== */

extern "C" void app_main(void)
{
    printf("\n========================================\n");
    printf("BLE Media Remote Starting...\n");
    printf("========================================\n\n");
    
    // ====================================================================
    // CRITICAL: Initialize NVS for bond storage
    // ====================================================================
    printf("Initializing NVS...\n");
    esp_err_t ret = nvs_flash_init();
    
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        printf("NVS partition needs erase\n");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    
    if (ret != ESP_OK) {
        printf("✗ NVS init failed: %s\n", esp_err_to_name(ret));
        printf("⚠ Bonds will NOT persist!\n");
    } else {
        printf("✓ NVS initialized\n");
    }
    
    // DEBUG: Check NVS stats
    nvs_stats_t nvs_stats;
    ret = nvs_get_stats(NULL, &nvs_stats);
    if (ret == ESP_OK) {
        printf("\nNVS Statistics:\n");
        printf("  Used entries:     %d\n", nvs_stats.used_entries);
        printf("  Free entries:     %d\n", nvs_stats.free_entries);
        printf("  Total entries:    %d\n", nvs_stats.total_entries);
        printf("  Namespace count:  %d\n", nvs_stats.namespace_count);
    } else {
        printf("✗ Failed to get NVS stats: %s\n", esp_err_to_name(ret));
    }
    
    // DEBUG: Try to open nimble namespace
    nvs_handle_t handle;
    ret = nvs_open("nimble", NVS_READWRITE, &handle);
    if (ret == ESP_OK) {
        printf("✓ NimBLE namespace can be opened for writing\n");
        nvs_close(handle);
    } else {
        printf("✗ Cannot open NimBLE namespace: %s\n", esp_err_to_name(ret));
        printf("  This is likely why bonds aren't persisting!\n");
    }
    printf("\n");
    
    // ====================================================================
    // Initialize BLE
    // ====================================================================
    printf("\nInitializing BLE...\n");
    setup();
    
    // ====================================================================
    // Initialize GPIO
    // ====================================================================
    printf("\nInitializing GPIO...\n");
    input_setup();
    
    gpio_isr_handler_add(INPUT_PLAY_PAUSE, gpio_isr_handler, (void*)INPUT_PLAY_PAUSE);
    gpio_isr_handler_add(INPUT_NEXT_TRACK, gpio_isr_handler, (void*)INPUT_NEXT_TRACK);
    gpio_isr_handler_add(INPUT_PREVIOUS_TRACK, gpio_isr_handler, (void*)INPUT_PREVIOUS_TRACK);
    
    // ====================================================================
    // Start button task
    // ====================================================================
    xTaskCreate(&main_task, "button_handler", 2048, NULL, 5, NULL);
    
    // ====================================================================
    // Ready
    // ====================================================================
    printf("\n========================================\n");
    printf("✓ READY\n");
    printf("========================================\n\n");
    
    printf("How to pair:\n");
    printf("  1. Open Bluetooth settings on phone\n");
    printf("  2. Look for 'Music Remote'\n");
    printf("  3. Tap to pair (no PIN needed)\n\n");
    
    printf("Buttons:\n");
    printf("  GPIO %d: Play/Pause\n", INPUT_PLAY_PAUSE);
    printf("  GPIO %d: Previous Track\n", INPUT_PREVIOUS_TRACK);
    printf("  GPIO %d: Next Track\n\n", INPUT_NEXT_TRACK);
    
    printf("Auto-Reconnect:\n");
    printf("  ✓ Power off/on → Reconnects\n");
    printf("  ✓ Out of range → Reconnects when back\n");
    printf("  ✓ Phone restart → Reconnects\n\n");
    
    printf("========================================\n\n");
}