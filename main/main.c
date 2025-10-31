/*
 * Complete BLE HID Media Remote Control
 * Hardcoded for Media Control - No menuconfig needed
 * 
 * Features:
 * - 5 media control buttons with proper debouncing
 * - LED feedback for connection and activity
 * - Serial debugging/testing interface
 * - Production-ready for assistive technology
 * 
 * Hardware:
 * - Buttons: GPIO 25, 26, 27, 32, 33 (active LOW, internal pullup)
 * - LED Status: GPIO 2 (connection status)
 * - LED Activity: GPIO 23 (button press feedback)
 * - LED Playing: GPIO 19 (music playing indicator)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "driver/gpio.h"

#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "esp_hidd.h"
#include "esp_hid_gap.h"

static const char *TAG = "MEDIA_REMOTE";

// HID Consumer Usage IDs (subset of the codes available in the USB HID Usage Tables spec)
#define HID_CONSUMER_PLAY           176 // Play
#define HID_CONSUMER_PAUSE          177 // Pause
#define HID_CONSUMER_SCAN_NEXT_TRK  181 // Scan Next Track
#define HID_CONSUMER_SCAN_PREV_TRK  182 // Scan Previous Track
#define HID_CONSUMER_PLAY_PAUSE     205 // Play/Pause

#define HID_CONSUMER_VOLUME_UP      233 // Volume Increment
#define HID_CONSUMER_VOLUME_DOWN    234 // Volume Decrement

/* ========================================================================
 * GPIO CONFIGURATION
 * ======================================================================== */

// Button pins (active LOW with internal pullup)
#define BUTTON_PLAY_PAUSE   GPIO_NUM_25
#define BUTTON_NEXT         GPIO_NUM_26
#define BUTTON_PREV         GPIO_NUM_27
#define BUTTON_VOL_UP       GPIO_NUM_32
#define BUTTON_VOL_DOWN     GPIO_NUM_33

// LED pins
#define LED_STATUS          GPIO_NUM_2   // Connection status (on = connected)
#define LED_ACTIVITY        GPIO_NUM_23  // Blinks on button press
#define LED_PLAYING         GPIO_NUM_19  // On when music playing

// Debounce configuration
#define DEBOUNCE_TIME_MS    200          // Minimum time between button presses

/* ========================================================================
 * BUTTON STATE TRACKING
 * ======================================================================== */

typedef struct {
    gpio_num_t pin;
    uint8_t consumer_code;
    const char *name;
    bool last_state;
    uint32_t last_press_time;
    bool currently_pressed;
} button_state_t;

static button_state_t buttons[] = {
    {BUTTON_PLAY_PAUSE, HID_CONSUMER_PLAY_PAUSE,    "Play/Pause", true, 0, false},
    {BUTTON_NEXT,       HID_CONSUMER_SCAN_NEXT_TRK, "Next",       true, 0, false},
    {BUTTON_PREV,       HID_CONSUMER_SCAN_PREV_TRK, "Previous",   true, 0, false},
 //   {BUTTON_VOL_UP,     HID_CONSUMER_VOLUME_UP,     "Vol Up",     true, 0, false},
  //  {BUTTON_VOL_DOWN,   HID_CONSUMER_VOLUME_DOWN,   "Vol Down",   true, 0, false},
};

#define NUM_BUTTONS (sizeof(buttons) / sizeof(buttons[0]))

/* ========================================================================
 * DEVICE STATE
 * ======================================================================== */

typedef struct {
    TaskHandle_t task_hdl;
    esp_hidd_dev_t *hid_dev;
    bool connected;
    bool playing;
} device_state_t;

static device_state_t device_state = {0};

/* ========================================================================
 * HID REPORT MAP - MEDIA CONTROL
 * ======================================================================== */

// Simplified Consumer Control report map
const unsigned char mediaReportMap[] = {
    0x05, 0x0C,        // Usage Page (Consumer)
    0x09, 0x01,        // Usage (Consumer Control)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x03,        //   Report ID (3)
    0x09, 0x02,        //   Usage (Numeric Key Pad)
    0xA1, 0x02,        //   Collection (Logical)
    0x05, 0x09,        //     Usage Page (Button)
    0x19, 0x01,        //     Usage Minimum (0x01)
    0x29, 0x0A,        //     Usage Maximum (0x0A)
    0x15, 0x01,        //     Logical Minimum (1)
    0x25, 0x0A,        //     Logical Maximum (10)
    0x75, 0x04,        //     Report Size (4)
    0x95, 0x01,        //     Report Count (1)
    0x81, 0x00,        //     Input (Data,Array,Abs)
    0xC0,              //   End Collection
    0x05, 0x0C,        //   Usage Page (Consumer)
    0x09, 0x86,        //   Usage (Channel)
    0x15, 0xFF,        //   Logical Minimum (-1)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x02,        //   Report Size (2)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x46,        //   Input (Data,Var,Rel)
    0x09, 0xE9,        //   Usage (Volume Increment)
    0x09, 0xEA,        //   Usage (Volume Decrement)
    0x15, 0x00,        //   Logical Minimum (0)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x02,        //   Report Count (2)
    0x81, 0x02,        //   Input (Data,Var,Abs)
    0x09, 0xE2,        //   Usage (Mute)
    0x09, 0x30,        //   Usage (Power)
    0x09, 0x83,        //   Usage (Recall Last)
    0x09, 0x81,        //   Usage (Assign Selection)
    0x09, 0xB0,        //   Usage (Play)
    0x09, 0xB1,        //   Usage (Pause)
    0x09, 0xB2,        //   Usage (Record)
    0x09, 0xB3,        //   Usage (Fast Forward)
    0x09, 0xB4,        //   Usage (Rewind)
    0x09, 0xB5,        //   Usage (Scan Next Track)
    0x09, 0xB6,        //   Usage (Scan Previous Track)
    0x09, 0xB7,        //   Usage (Stop)
    0x15, 0x01,        //   Logical Minimum (1)
    0x25, 0x0C,        //   Logical Maximum (12)
    0x75, 0x04,        //   Report Size (4)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x00,        //   Input (Data,Array,Abs)
    0x09, 0x80,        //   Usage (Selection)
    0xA1, 0x02,        //   Collection (Logical)
    0x05, 0x09,        //     Usage Page (Button)
    0x19, 0x01,        //     Usage Minimum (0x01)
    0x29, 0x03,        //     Usage Maximum (0x03)
    0x15, 0x01,        //     Logical Minimum (1)
    0x25, 0x03,        //     Logical Maximum (3)
    0x75, 0x02,        //     Report Size (2)
    0x81, 0x00,        //     Input (Data,Array,Abs)
    0xC0,              //   End Collection
    0x81, 0x03,        //   Input (Const,Var,Abs)
    0xC0,              // End Collection
};

static esp_hid_raw_report_map_t report_maps[] = {
    {
        .data = mediaReportMap,
        .len = sizeof(mediaReportMap)
    }
};

static esp_hid_device_config_t hid_config = {
    .vendor_id          = 0x16C0,
    .product_id         = 0x05DF,
    .version            = 0x0100,
    .device_name        = "Adapted Speaker Control",
    .manufacturer_name  = "Espressif",
    .serial_number      = "000001",
    .report_maps        = report_maps,
    .report_maps_len    = 1
};

/* ========================================================================
 * HID CONSUMER CONTROL CODES
 * ======================================================================== */

// Report ID for consumer control
#define HID_RPT_ID_CC_IN        3
#define HID_CC_IN_RPT_LEN       2

// Consumer control report constants
#define HID_CC_RPT_MUTE                 1
#define HID_CC_RPT_POWER                2
#define HID_CC_RPT_LAST                 3
#define HID_CC_RPT_ASSIGN_SEL           4
#define HID_CC_RPT_PLAY                 5
#define HID_CC_RPT_PAUSE                6
#define HID_CC_RPT_RECORD               7
#define HID_CC_RPT_FAST_FWD             8
#define HID_CC_RPT_REWIND               9
#define HID_CC_RPT_SCAN_NEXT_TRK        10
#define HID_CC_RPT_SCAN_PREV_TRK        11
#define HID_CC_RPT_STOP                 12

#define HID_CC_RPT_CHANNEL_UP           0x10
#define HID_CC_RPT_CHANNEL_DOWN         0x30
#define HID_CC_RPT_VOLUME_UP            0x40
#define HID_CC_RPT_VOLUME_DOWN          0x80

// Bitmasks
#define HID_CC_RPT_NUMERIC_BITS         0xF0
#define HID_CC_RPT_CHANNEL_BITS         0xCF
#define HID_CC_RPT_VOLUME_BITS          0x3F
#define HID_CC_RPT_BUTTON_BITS          0xF0
#define HID_CC_RPT_SELECTION_BITS       0xCF

// Macros to set report values
#define HID_CC_RPT_SET_BUTTON(s, x)     (s)[1] &= HID_CC_RPT_BUTTON_BITS;    (s)[1] |= (x)
#define HID_CC_RPT_SET_VOLUME_UP(s)     (s)[0] &= HID_CC_RPT_VOLUME_BITS;    (s)[0] |= 0x40
#define HID_CC_RPT_SET_VOLUME_DOWN(s)   (s)[0] &= HID_CC_RPT_VOLUME_BITS;    (s)[0] |= 0x80

/* ========================================================================
 * LED CONTROL FUNCTIONS
 * ======================================================================== */

static void led_on(gpio_num_t pin) {
    gpio_set_level(pin, 1);
}

static void led_off(gpio_num_t pin) {
    gpio_set_level(pin, 0);
}

static void led_blink(gpio_num_t pin, uint32_t duration_ms) {
    led_on(pin);
    vTaskDelay(duration_ms / portTICK_PERIOD_MS);
    led_off(pin);
}

static void led_blink_async(gpio_num_t pin) {
    // Non-blocking blink for activity LED
    led_on(pin);
    // Will be turned off by next iteration or timer
}

/* ========================================================================
 * GPIO INITIALIZATION
 * ======================================================================== */

static void init_gpio(void) {
    ESP_LOGI(TAG, "Initializing GPIO...");
    
    // Configure button pins as inputs with pullups
    uint64_t button_mask = 0;
    for (int i = 0; i < NUM_BUTTONS; i++) {
        button_mask |= (1ULL << buttons[i].pin);
    }
    
    gpio_config_t button_conf = {
        .pin_bit_mask = button_mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&button_conf);
    
    ESP_LOGI(TAG, "✓ Buttons configured:");
    for (int i = 0; i < NUM_BUTTONS; i++) {
        ESP_LOGI(TAG, "  - GPIO %d: %s", buttons[i].pin, buttons[i].name);
    }
    
    // Configure LED pins as outputs
    gpio_config_t led_conf = {
        .pin_bit_mask = (1ULL << LED_STATUS) | (1ULL << LED_ACTIVITY) | (1ULL << LED_PLAYING),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&led_conf);
    
    // Initialize all LEDs off
    led_off(LED_STATUS);
    led_off(LED_ACTIVITY);
    led_off(LED_PLAYING);
    
    ESP_LOGI(TAG, "✓ LEDs configured:");
    ESP_LOGI(TAG, "  - GPIO %d: Status LED", LED_STATUS);
    ESP_LOGI(TAG, "  - GPIO %d: Activity LED", LED_ACTIVITY);
    ESP_LOGI(TAG, "  - GPIO %d: Playing LED", LED_PLAYING);
    
    // Power-on LED test sequence
    ESP_LOGI(TAG, "Running LED test sequence...");
    for (int i = 0; i < 3; i++) {
        led_on(LED_STATUS);
        led_on(LED_ACTIVITY);
        led_on(LED_PLAYING);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        led_off(LED_STATUS);
        led_off(LED_ACTIVITY);
        led_off(LED_PLAYING);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    
    ESP_LOGI(TAG, "✓ GPIO initialization complete");
}

/* ========================================================================
 * HID REPORT SENDING
 * ======================================================================== */

void send_consumer_key(uint8_t key_cmd, bool key_pressed) {
    if (!device_state.connected) {
        ESP_LOGW(TAG, "Not connected, cannot send key");
        return;
    }
    
    uint8_t buffer[HID_CC_IN_RPT_LEN] = {0, 0};
    
    if (key_pressed) {
        switch (key_cmd) {
       /* case HID_CONSUMER_VOLUME_UP:
            HID_CC_RPT_SET_VOLUME_UP(buffer);
            break;*/
            
       /* case HID_CONSUMER_VOLUME_DOWN:
            HID_CC_RPT_SET_VOLUME_DOWN(buffer);
            break;*/
            
        case HID_CONSUMER_PLAY_PAUSE:
            HID_CC_RPT_SET_BUTTON(buffer, HID_CC_RPT_PLAY);
            break;
            
        /*case HID_CONSUMER_PAUSE:
            HID_CC_RPT_SET_BUTTON(buffer, HID_CC_RPT_PAUSE);
            break;*/
            
        case HID_CONSUMER_SCAN_NEXT_TRK:
            HID_CC_RPT_SET_BUTTON(buffer, HID_CC_RPT_SCAN_NEXT_TRK);
            break;
            
        case HID_CONSUMER_SCAN_PREV_TRK:
            HID_CC_RPT_SET_BUTTON(buffer, HID_CC_RPT_SCAN_PREV_TRK);
            break;
           
            
        default:
            ESP_LOGW(TAG, "Unknown consumer code: %d", key_cmd);
            return;
        }
    }
    // If not pressed, buffer stays zeros (key release)
    
    esp_hidd_dev_input_set(device_state.hid_dev, 0, HID_RPT_ID_CC_IN, 
                          buffer, HID_CC_IN_RPT_LEN);
}

/* ========================================================================
 * BUTTON HANDLING WITH DEBOUNCING
 * ======================================================================== */

static void check_buttons(void) {
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    for (int i = 0; i < NUM_BUTTONS; i++) {
        button_state_t *btn = &buttons[i];
        bool current_state = gpio_get_level(btn->pin);
        
        // Detect falling edge (button press) - active LOW
        if (current_state == 0 && btn->last_state == 1) {
            // Button just pressed
            
            // Check debounce time
            if (current_time - btn->last_press_time > DEBOUNCE_TIME_MS) {
                ESP_LOGI(TAG, "Button: %s", btn->name);
                
                // Send key press
                send_consumer_key(btn->consumer_code, true);
                
                // Special handling for Play button
                if (btn->consumer_code == HID_CONSUMER_PLAY) {
                    device_state.playing = !device_state.playing;
                    if (device_state.playing) {
                        led_on(LED_PLAYING);
                    } else {
                        led_off(LED_PLAYING);
                    }
                }
                
                // Visual feedback
                led_blink_async(LED_ACTIVITY);
                
                btn->last_press_time = current_time;
                btn->currently_pressed = true;
            }
        }
        // Detect rising edge (button release)
        else if (current_state == 1 && btn->last_state == 0) {
            if (btn->currently_pressed) {
                // Send key release
                send_consumer_key(btn->consumer_code, false);
                btn->currently_pressed = false;
                
                // Turn off activity LED
                led_off(LED_ACTIVITY);
            }
        }
        
        btn->last_state = current_state;
    }
}

/* ========================================================================
 * BUTTON TASK
 * ======================================================================== */

void button_task(void *pvParameters) {
    ESP_LOGI(TAG, "Button task started");
    
    while (1) {
        if (device_state.connected) {
            check_buttons();
        }
        
        vTaskDelay(10 / portTICK_PERIOD_MS);  // Check every 10ms
    }
}

/* ========================================================================
 * SERIAL TESTING INTERFACE
 * ======================================================================== */

void test_task(void *pvParameters) {
    ESP_LOGI(TAG, "Test interface started");
    ESP_LOGI(TAG, "Commands: p=play, n=next, b=prev, u=volup, d=voldown, t=test, h=help");
    
    char c;
    while (1) {
        c = fgetc(stdin);
        
        if (c == 255 || c == 0) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }
        
        if (!device_state.connected && c != 't' && c != 'h') {
            ESP_LOGW(TAG, "Not connected! Press 't' to test LEDs or 'h' for help");
            continue;
        }
        
        switch (c) {
        case 'p':
            ESP_LOGI(TAG, "TEST: Play/Pause");
            send_consumer_key(HID_CONSUMER_PLAY_PAUSE, true);
            vTaskDelay(50 / portTICK_PERIOD_MS);
            send_consumer_key(HID_CONSUMER_PLAY, false);
            led_blink(LED_ACTIVITY, 100);
            break;
            
        case 'n':
            ESP_LOGI(TAG, "TEST: Next track");
            send_consumer_key(HID_CONSUMER_SCAN_NEXT_TRK, true);
            vTaskDelay(50 / portTICK_PERIOD_MS);
            send_consumer_key(HID_CONSUMER_SCAN_NEXT_TRK, false);
            led_blink(LED_ACTIVITY, 100);
            break;
            
        case 'b':
            ESP_LOGI(TAG, "TEST: Previous track");
            send_consumer_key(HID_CONSUMER_SCAN_PREV_TRK, true);
            vTaskDelay(50 / portTICK_PERIOD_MS);
            send_consumer_key(HID_CONSUMER_SCAN_PREV_TRK, false);
            led_blink(LED_ACTIVITY, 100);
            break;
            
        /*case 'u':
            ESP_LOGI(TAG, "TEST: Volume up");
            send_consumer_key(HID_CONSUMER_VOLUME_UP, true);
            vTaskDelay(50 / portTICK_PERIOD_MS);
            send_consumer_key(HID_CONSUMER_VOLUME_UP, false);
            led_blink(LED_ACTIVITY, 100);
            break;
            
        case 'd':
            ESP_LOGI(TAG, "TEST: Volume down");
            send_consumer_key(HID_CONSUMER_VOLUME_DOWN, true);
            vTaskDelay(50 / portTICK_PERIOD_MS);
            send_consumer_key(HID_CONSUMER_VOLUME_DOWN, false);
            led_blink(LED_ACTIVITY, 100);
            break;*/
            
        case 't':
            ESP_LOGI(TAG, "TEST: LED test sequence");
            ESP_LOGI(TAG, "  Status LED...");
            led_blink(LED_STATUS, 500);
            vTaskDelay(200 / portTICK_PERIOD_MS);
            ESP_LOGI(TAG, "  Activity LED...");
            led_blink(LED_ACTIVITY, 500);
            vTaskDelay(200 / portTICK_PERIOD_MS);
            ESP_LOGI(TAG, "  Playing LED...");
            led_blink(LED_PLAYING, 500);
            ESP_LOGI(TAG, "  Test complete");
            break;
            
        case 's':
            ESP_LOGI(TAG, "STATUS:");
            ESP_LOGI(TAG, "  Connected: %s", device_state.connected ? "YES" : "NO");
            ESP_LOGI(TAG, "  Playing: %s", device_state.playing ? "YES" : "NO");
            for (int i = 0; i < NUM_BUTTONS; i++) {
                ESP_LOGI(TAG, "  Button %d (%s): %s", 
                        buttons[i].pin, buttons[i].name,
                        gpio_get_level(buttons[i].pin) ? "Released" : "Pressed");
            }
            break;
            
        case 'h':
            ESP_LOGI(TAG, "=== HELP ===");
            ESP_LOGI(TAG, "Serial commands:");
            ESP_LOGI(TAG, "  p - Send Play/Pause");
            ESP_LOGI(TAG, "  n - Send Next Track");
            ESP_LOGI(TAG, "  b - Send Previous Track");
            ESP_LOGI(TAG, "  u - Send Volume Up");
            ESP_LOGI(TAG, "  d - Send Volume Down");
            ESP_LOGI(TAG, "  t - Test LEDs");
            ESP_LOGI(TAG, "  s - Show status");
            ESP_LOGI(TAG, "  h - Show this help");
            break;
            
        default:
            break;
        }
        
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

/* ========================================================================
 * HID EVENT CALLBACKS
 * ======================================================================== */

static void hidd_event_callback(void *handler_args, esp_event_base_t base, 
                                int32_t id, void *event_data) {
    esp_hidd_event_t event = (esp_hidd_event_t)id;
    esp_hidd_event_data_t *param = (esp_hidd_event_data_t *)event_data;

    switch (event) {
    case ESP_HIDD_START_EVENT:
        ESP_LOGI(TAG, "START - Beginning advertisement");
        esp_hid_ble_gap_adv_start();
        break;
        
    case ESP_HIDD_CONNECT_EVENT:
        ESP_LOGI(TAG, "CONNECT - Device paired!");
        device_state.connected = true;
        led_on(LED_STATUS);
        break;
        
    case ESP_HIDD_DISCONNECT_EVENT:
        ESP_LOGI(TAG, "DISCONNECT - Device unpaired");
        device_state.connected = false;
        device_state.playing = false;
        led_off(LED_STATUS);
        led_off(LED_PLAYING);
        led_off(LED_ACTIVITY);
        
        // Resume advertising
        esp_hid_ble_gap_adv_start();
        break;
        
    case ESP_HIDD_CONTROL_EVENT:
        ESP_LOGI(TAG, "CONTROL - %s", 
                param->control.control ? "EXIT_SUSPEND" : "SUSPEND");
        break;
        
    case ESP_HIDD_PROTOCOL_MODE_EVENT:
        ESP_LOGI(TAG, "PROTOCOL_MODE - %s", 
                param->protocol_mode.protocol_mode ? "REPORT" : "BOOT");
        break;
        
    default:
        break;
    }
}

/* ========================================================================
 * NIMBLE HOST TASK
 * ======================================================================== */

void ble_host_task(void *param) {
    ESP_LOGI(TAG, "BLE Host Task Started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/* ========================================================================
 * MAIN APPLICATION
 * ======================================================================== */

void app_main(void) {
    esp_err_t ret;
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║  BLE Media Remote Control                 ║");
    ESP_LOGI(TAG, "║  Assistive Technology Device              ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
    
    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "✓ NVS initialized");
    
    // Initialize GPIO (buttons and LEDs)
    init_gpio();
    
    // Initialize HID GAP
    ESP_LOGI(TAG, "Initializing Bluetooth...");
    ESP_ERROR_CHECK(esp_hid_gap_init(ESP_BT_MODE_BLE));
    ESP_LOGI(TAG, "✓ HID GAP initialized");
    
    // Initialize BLE GAP for advertising
    ESP_ERROR_CHECK(esp_hid_ble_gap_adv_init(ESP_HID_APPEARANCE_GENERIC, 
                                            hid_config.device_name));
    ESP_LOGI(TAG, "✓ BLE advertising configured");
    
    // Initialize HID device
    ESP_LOGI(TAG, "Initializing HID device...");
    ESP_ERROR_CHECK(esp_hidd_dev_init(&hid_config, ESP_HID_TRANSPORT_BLE, 
                                     hidd_event_callback, &device_state.hid_dev));
    ESP_LOGI(TAG, "✓ HID device initialized");
    
    // Start NimBLE host task
    nimble_port_freertos_init(ble_host_task);
    
    // Wait a moment for BLE stack to initialize
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    // Start button handling task
    xTaskCreate(button_task, "button_task", 4096, NULL, 5, &device_state.task_hdl);
    ESP_LOGI(TAG, "✓ Button task started");
    
    // Start test interface task
    xTaskCreate(test_task, "test_task", 4096, NULL, 3, NULL);
    ESP_LOGI(TAG, "✓ Test interface started");
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "════════════════════════════════════════════");
    ESP_LOGI(TAG, "SETUP COMPLETE");
    ESP_LOGI(TAG, "════════════════════════════════════════════");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Device Name: %s", hid_config.device_name);
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Pairing Instructions:");
    ESP_LOGI(TAG, "1. Open Bluetooth settings on your phone");
    ESP_LOGI(TAG, "2. Look for 'Adapted Speaker Control'");
    ESP_LOGI(TAG, "3. Tap to pair");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Button Functions:");
    ESP_LOGI(TAG, "  GPIO 32: Volume Up");
    ESP_LOGI(TAG, "  GPIO 33: Volume Down");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "LED Indicators:");
    ESP_LOGI(TAG, "  GPIO 2:  Status (On = Connected)");
    ESP_LOGI(TAG, "  GPIO 23: Activity (Blinks on press)");
    ESP_LOGI(TAG, "  GPIO 19: Playing (On when music playing)");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Serial Test Commands:");
    ESP_LOGI(TAG, "  Type 'h' for help");
    ESP_LOGI(TAG, "════════════════════════════════════════════");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Ready! Waiting for Bluetooth connection...");
}

