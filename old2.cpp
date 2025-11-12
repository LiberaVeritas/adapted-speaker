/**
 * - Buttons: GPIO 25, 26, 27, 32, 33 (active LOW with pullup)
 * - LEDs: GPIO 2 (status), 23 (activity), 19 (playing)
 */

#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEHIDDevice.h>
#include <driver/gpio.h>
#include "HIDtypes.h"

/* ========================================================================
 * CONFIGURATION
 * ======================================================================== */

// Button pins
#define BUTTON_PLAY_PAUSE   GPIO_NUM_25
#define BUTTON_NEXT         GPIO_NUM_26
#define BUTTON_PREV         GPIO_NUM_27
#define BUTTON_VOL_UP       GPIO_NUM_32
#define BUTTON_VOL_DOWN     GPIO_NUM_33

// LED pins
#define LED_STATUS          GPIO_NUM_2
#define LED_ACTIVITY        GPIO_NUM_23
#define LED_PLAYING         GPIO_NUM_19

// Debounce time (milliseconds)
#define DEBOUNCE_MS         200

// Device name
#define DEVICE_NAME         "Music Remote"

/* ========================================================================
 * HID REPORT MAP - CONSUMER CONTROL (MEDIA KEYS)
 * ======================================================================== */

/**
 * This report map defines a Consumer Control device with 7 media buttons.
 * Each button is a single bit in a 1-byte report.
 * 
 */
static uint8_t mediaReportMap[] = {
  USAGE_PAGE(1),       0x0C,          // USAGE_PAGE (Consumer)
  USAGE(1),            0x01,          // USAGE (Consumer Control)
  COLLECTION(1),       0x01,          // COLLECTION (Application)
  REPORT_ID(1),        0x01,          //   REPORT_ID (1)
  USAGE_PAGE(1),       0x0C,          //   USAGE_PAGE (Consumer)
  LOGICAL_MINIMUM(1), 0x00,          //   LOGICAL_MINIMUM (0)
  LOGICAL_MAXIMUM(1), 0x01,          //   LOGICAL_MAXIMUM (1)
  REPORT_SIZE(1),     0x01,          //   REPORT_SIZE (1)
  REPORT_COUNT(1),    0x08,          //   REPORT_COUNT (8)
  USAGE(1),           0xB5,          //   USAGE (Scan Next Track)     ; bit 0: 1
  USAGE(1),           0xB6,          //   USAGE (Scan Previous Track) ; bit 1: 2
  USAGE(1),           0xB7,          //   USAGE (Stop)                ; bit 2: 4
  USAGE(1),           0xCD,          //   USAGE (Play/Pause)          ; bit 3: 8
  USAGE(1),           0xE2,          //   USAGE (Mute)                ; bit 4: 16
  USAGE(1),           0xE9,          //   USAGE (Volume Increment)    ; bit 5: 32
  USAGE(1),           0xEA,          //   USAGE (Volume Decrement)    ; bit 6: 64
  USAGE(1),           0xEA,          //   USAGE (NO OP)               ; bit 7: 128
  HIDINPUT(1),        0x02,          //   INPUT (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
  END_COLLECTION(0)                       // END_COLLECTION                     // End Collection
};

/* ========================================================================
 * BIT POSITIONS FOR EACH BUTTON IN REPORT
 * ======================================================================== */

#define BIT_PLAY_PAUSE      3
#define BIT_NEXT_TRACK      0
#define BIT_PREV_TRACK      1
#define BIT_VOL_DOWN        6
#define BIT_VOL_UP          5

/* ========================================================================
 * GLOBAL OBJECTS AND STATE
 * ======================================================================== */

static NimBLEServer* pServer = nullptr;
static NimBLEHIDDevice* pHID = nullptr;
static NimBLECharacteristic* pInputCharacteristic = nullptr;

static bool deviceConnected = false;
static bool isPlaying = false;

// Button state tracking
typedef struct {
    gpio_num_t pin;
    uint8_t bit_position;
    const char* name;
    bool last_state;
    uint32_t last_press_time;
} button_t;

static button_t buttons[] = {
    {BUTTON_PLAY_PAUSE, BIT_PLAY_PAUSE, "Play/Pause", true, 0},
    {BUTTON_NEXT,       BIT_NEXT_TRACK, "Next",       true, 0},
    {BUTTON_PREV,       BIT_PREV_TRACK, "Previous",   true, 0},
    {BUTTON_VOL_UP,     BIT_VOL_UP,     "Vol Up",     true, 0},
    {BUTTON_VOL_DOWN,   BIT_VOL_DOWN,   "Vol Down",   true, 0},
};

#define NUM_BUTTONS (sizeof(buttons) / sizeof(buttons[0]))

/* ========================================================================
 * LED CONTROL FUNCTIONS
 * ======================================================================== */

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

/* ========================================================================
 * GPIO INITIALIZATION
 * ======================================================================== */

void initGPIO() {
    printf("Initializing GPIO...\n");
    
    // Configure button pins
    uint64_t button_mask = 0;
    for (int i = 0; i < NUM_BUTTONS; i++) {
        button_mask |= (1ULL << buttons[i].pin);
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

/* ========================================================================
 * SERVER CALLBACKS
 * ======================================================================== */

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
        printf("Client connected: %s\n", connInfo.getAddress().toString().c_str());
        deviceConnected = true;
        ledOn(LED_STATUS);
        
        // Update connection parameters for better responsiveness
        // Args: handle, min_interval, max_interval, latency, timeout
        // Intervals in 1.25ms units, timeout in 10ms units
        pServer->updateConnParams(connInfo.getConnHandle(), 24, 48, 0, 200);
    }
    
    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
        printf("Client disconnected, reason: %d\n", reason);
        deviceConnected = false;
        isPlaying = false;
        ledOff(LED_STATUS);
        ledOff(LED_PLAYING);
        
        // Restart advertising
        NimBLEDevice::startAdvertising();
        printf("Advertising restarted\n");
    }
    
    void onAuthenticationComplete(NimBLEConnInfo& connInfo) override {
        if (!connInfo.isEncrypted()) {
            printf("Encryption failed - disconnecting\n");
            NimBLEDevice::getServer()->disconnect(connInfo.getConnHandle());
            return;
        }
        printf("Connection secured\n");
    }
} serverCallbacks;

class CharacteristicCallbacks : public NimBLECharacteristicCallbacks {
    void onRead(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
        printf("%s : onRead(), value: %s\n",
               pCharacteristic->getUUID().toString().c_str(),
               pCharacteristic->getValue().c_str());
    }

    void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
        printf("%s : onWrite(), value: %s\n",
               pCharacteristic->getUUID().toString().c_str(),
               pCharacteristic->getValue().c_str());
    }

    /**
     *  The value returned in code is the NimBLE host return code.
     */
    void onStatus(NimBLECharacteristic* pCharacteristic, int code) override {
        printf("Notification/Indication return code: %d, %s\n", code, NimBLEUtils::returnCodeToString(code));
    }

    /** Peer subscribed to notifications/indications */
    void onSubscribe(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo, uint16_t subValue) override {
        std::string str  = "Client ID: ";
        str             += connInfo.getConnHandle();
        str             += " Address: ";
        str             += connInfo.getAddress().toString();
        if (subValue == 0) {
            str += " Unsubscribed to ";
        } else if (subValue == 1) {
            str += " Subscribed to notifications for ";
        } else if (subValue == 2) {
            str += " Subscribed to indications for ";
        } else if (subValue == 3) {
            str += " Subscribed to notifications and indications for ";
        }
        str += std::string(pCharacteristic->getUUID());

        printf("%s\n", str.c_str());
    }
} chrCallbacks;

/* ========================================================================
 * HID FUNCTIONS
 * ======================================================================== */

void sendMediaKey(uint8_t bit_position, bool pressed) {
    if (!deviceConnected || pInputCharacteristic == nullptr) {
        printf("Not connected, cannot send key\n");
        return;
    }
    
    // HID Report format: [Report ID, Data bitmap]
    uint8_t report[2];
	report[0] = 0x01;  // report id
    report[0] = 0x00;  // Data byte
    
    if (pressed) {
        // Set the bit for this button
        report[0] |= (0x01 << bit_position);
    }
    // If not pressed, report stays 0x00 (all keys released)
    
    // Send the report
	printf("sending\n");
    pInputCharacteristic->setValue(report, 1);
    pInputCharacteristic->notify();
	printf("testing 1 4\n");
	pInputCharacteristic->setValue({1, 4});
	pInputCharacteristic->notify();
    
    // Visual feedback
    if (pressed) {
        ledOn(LED_ACTIVITY);
    } else {
        ledOff(LED_ACTIVITY);
    }
}

/* ========================================================================
 * BUTTON HANDLING
 * ======================================================================== */

void checkButtons() {
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    for (int i = 0; i < NUM_BUTTONS; i++) {
        button_t* btn = &buttons[i];
        bool current_state = gpio_get_level(btn->pin);
        
        // Detect falling edge (button press) - active LOW
        if (current_state == 0 && btn->last_state == 1) {
            // Debounce check
            if (current_time - btn->last_press_time > DEBOUNCE_MS) {
                printf("Button: %s\n", btn->name);
                
                // Send key press
                sendMediaKey(btn->bit_position, true);
                
                // Special handling for Play/Pause
                if (btn->bit_position == BIT_PLAY_PAUSE) {
                    isPlaying = !isPlaying;
                    if (isPlaying) {
                        ledOn(LED_PLAYING);
                    } else {
                        ledOff(LED_PLAYING);
                    }
                }
                
                btn->last_press_time = current_time;
            }
        }
        // Detect rising edge (button release)
        else if (current_state == 1 && btn->last_state == 0) {
            // Send key release after a short delay
            vTaskDelay(50 / portTICK_PERIOD_MS);
            sendMediaKey(btn->bit_position, false);
        }
        
        btn->last_state = current_state;
    }
}

/* ========================================================================
 * TASKS
 * ======================================================================== */

void buttonTask(void* pvParameters) {
    printf("Button task started\n");
    
    while (1) {
        if (deviceConnected) {
            checkButtons();
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void testTask(void* pvParameters) {
    printf("Test interface started\n");
    printf("Commands: p=play, n=next, b=prev, u=volup, d=voldown, t=test, s=status, h=help\n");
    
    char c;
    while (1) {
        c = fgetc(stdin);
        
        if (c == 255 || c == 0) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }
        
        switch (c) {
        case 'p':
            printf("TEST: Play/Pause\n");
            sendMediaKey(BIT_PLAY_PAUSE, true);
            vTaskDelay(50 / portTICK_PERIOD_MS);
            sendMediaKey(BIT_PLAY_PAUSE, false);
            break;
            
        case 'n':
            printf("TEST: Next track\n");
            sendMediaKey(BIT_NEXT_TRACK, true);
            vTaskDelay(50 / portTICK_PERIOD_MS);
            sendMediaKey(BIT_NEXT_TRACK, false);
            break;
            
        case 'b':
            printf("TEST: Previous track\n");
            sendMediaKey(BIT_PREV_TRACK, true);
            vTaskDelay(50 / portTICK_PERIOD_MS);
            sendMediaKey(BIT_PREV_TRACK, false);
            break;
            
        case 'u':
            printf("TEST: Volume up\n");
            sendMediaKey(BIT_VOL_UP, true);
            vTaskDelay(50 / portTICK_PERIOD_MS);
            sendMediaKey(BIT_VOL_UP, false);
            break;
            
        case 'd':
            printf("TEST: Volume down\n");
            sendMediaKey(BIT_VOL_DOWN, true);
            vTaskDelay(50 / portTICK_PERIOD_MS);
            sendMediaKey(BIT_VOL_DOWN, false);
            break;
            
        case 't':
            printf("TEST: LED test\n");
            ledBlink(LED_STATUS, 500);
            vTaskDelay(200 / portTICK_PERIOD_MS);
            ledBlink(LED_ACTIVITY, 500);
            vTaskDelay(200 / portTICK_PERIOD_MS);
            ledBlink(LED_PLAYING, 500);
            printf("Test complete\n");
            break;
            
        case 's':
            printf("STATUS:\n");
            printf("  Connected: %s\n", deviceConnected ? "YES" : "NO");
            printf("  Playing: %s\n", isPlaying ? "YES" : "NO");
            for (int i = 0; i < NUM_BUTTONS; i++) {
                printf("  %s: %s\n", buttons[i].name, 
                      gpio_get_level(buttons[i].pin) ? "Released" : "Pressed");
            }
            break;
            
        case 'h':
            printf("=== HELP ===\n");
            printf("Commands:\n");
            printf("  p - Play/Pause\n");
            printf("  n - Next Track\n");
            printf("  b - Previous Track\n");
            printf("  u - Volume Up\n");
            printf("  d - Volume Down\n");
            printf("  t - Test LEDs\n");
            printf("  s - Show Status\n");
            printf("  h - Help\n");
            break;
        }
        
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

/* ========================================================================
 * MAIN APPLICATION
 * ======================================================================== */

extern "C" void app_main(void) {
    printf("\n");
    printf("╔════════════════════════════════════════════╗\n");
    printf("║  BLE HID Media Remote                     ║\n");
    printf("║  Using NimBLE-Arduino Library             ║\n");
    printf("╚════════════════════════════════════════════╝\n");
    printf("\n");
    
    // Initialize GPIO
    initGPIO();
    
    // Initialize NimBLE
    printf("Initializing NimBLE...\n");
    NimBLEDevice::init(DEVICE_NAME);
    
    // Set security - Just Works pairing (no PIN)
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
    NimBLEDevice::setSecurityAuth(false, false, true);
    
    // Create BLE Server
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(&serverCallbacks);
    
    // Create HID Device
    printf("Creating HID device...\n");
    pHID = new NimBLEHIDDevice(pServer);
    
    // Set HID info
    pHID->setManufacturer("Espressif");
    pHID->setPnp(0x02, 0x16c0, 0x05df, 0x0110);  // USB vendor, product, version
    pHID->setHidInfo(0x00, 0x01);  // HID version
    
    // Set report map
    pHID->setReportMap((uint8_t*)mediaReportMap, sizeof(mediaReportMap));
    
    // Start HID service
    pHID->startServices();
    
    // Get the input report characteristic
    // This is the characteristic we'll use to send button presses
    /*NimBLEService* pHIDService = pServer->getServiceByUUID(BLEUUID((uint16_t)0x1812));
    if (pHIDService) {
        // HID Report characteristic UUID is 0x2A4D
        pInputCharacteristic = pHIDService->getCharacteristic(BLEUUID((uint16_t)0x2A4D));
        if (pInputCharacteristic) {
            printf("✓ HID Input characteristic found\n");
        } else {
            printf("✗ Failed to find HID input characteristic\n");
        }
    }*/
	pInputCharacteristic = pHID->getInputReport(0x01);
	NimBLEService* pHIDService = pHID->getHidService();
	//NimBLECharacteristic* pCharacteristic =
    //    pHIDService->createCharacteristic("Characteristic",
    //                                       NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE |
                                               /** Require a secure connection for read and write access */
     //                                          NIMBLE_PROPERTY::READ_ENC | // only allow reading if paired / encrypted
      //                                         NIMBLE_PROPERTY::WRITE_ENC  // only allow writing if paired / encrypted
     //   );
	
	//pCharacteristic->setValue("Characteristic Value");
    pInputCharacteristic->setCallbacks(&chrCallbacks);
	
    
    // Setup advertising
    printf("Setting up advertising...\n");
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->setAppearance(HID_KEYBOARD);  // Advertise as HID device	
	pAdvertising->setName(DEVICE_NAME);
	pAdvertising->addServiceUUID(pHIDService->getUUID());
    //pAdvertising->addServiceUUID(pHID->getHidService()->getUUID());
    pAdvertising->enableScanResponse(true);
    //pAdvertising->setMinInterval(0x06);  // Functions that help with iPhone connections issue
    //pAdvertising->setMinInterval(0x12);
    
    // Start advertising
    pAdvertising->start();
    printf("✓ Advertising started\n");
    
    printf("\n");
    printf("════════════════════════════════════════════\n");
    printf("READY\n");
    printf("════════════════════════════════════════════\n");
    printf("Device Name: %s\n", DEVICE_NAME);
    printf("\n");
    printf("Pairing Instructions:\n");
    printf("1. Open Bluetooth settings on your phone\n");
    printf("2. Look for '%s'\n", DEVICE_NAME);
    printf("3. Tap to pair (no PIN needed)\n");
    printf("\n");
    printf("Button Functions:\n");
    printf("  GPIO 25: Play/Pause\n");
    printf("  GPIO 26: Next Track\n");
    printf("  GPIO 27: Previous Track\n");
    printf("  GPIO 32: Volume Up\n");
    printf("  GPIO 33: Volume Down\n");
    printf("\n");
    printf("LED Indicators:\n");
    printf("  GPIO 2:  Status (On = Connected)\n");
    printf("  GPIO 23: Activity (Blinks on press)\n");
    printf("  GPIO 19: Playing (On = Music playing)\n");
    printf("\n");
    printf("Serial Commands: Type 'h' for help\n");
    printf("════════════════════════════════════════════\n");
    printf("\n");
    
    // Start tasks
    xTaskCreate(buttonTask, "button_task", 4096, NULL, 5, NULL);
    xTaskCreate(testTask, "test_task", 4096, NULL, 3, NULL);
    
    printf("Tasks started. Ready for connection!\n");
    
    // Main loop - just keep alive
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}