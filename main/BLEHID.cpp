// BLEHID.cpp - Auto-Reconnect Only (No Power Management)

#include "BLEHID.h"
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEHIDDevice.h>
#include "HIDTypes.h"

#if defined(CONFIG_ARDUHAL_ESP_LOG)
  #include "esp32-hal-log.h"
  #define LOG_TAG ""
#else
  #include "esp_log.h"
  static const char* LOG_TAG = "MusicRemote";
#endif

// Report ID:
#define MEDIA_KEYS_ID 0x01

static const uint8_t _hidReportDescriptor[] = {
  // ------------------------------------------------- Media Keys
  USAGE_PAGE(1),      0x0C,          // USAGE_PAGE (Consumer)
  USAGE(1),           0x01,          // USAGE (Consumer Control)
  COLLECTION(1),      0x01,          // COLLECTION (Application)
  REPORT_ID(1),       MEDIA_KEYS_ID, //   REPORT_ID (1)
  USAGE_PAGE(1),      0x0C,          //   USAGE_PAGE (Consumer)
  LOGICAL_MINIMUM(1), 0x00,          //   LOGICAL_MINIMUM (0)
  LOGICAL_MAXIMUM(1), 0x01,          //   LOGICAL_MAXIMUM (1)
  REPORT_SIZE(1),     0x01,          //   REPORT_SIZE (1)
  REPORT_COUNT(1),    0x10,          //   REPORT_COUNT (16)
  USAGE(1),           0xB5,          //   USAGE (Scan Next Track)     ; bit 0: 1
  USAGE(1),           0xB6,          //   USAGE (Scan Previous Track) ; bit 1: 2
  USAGE(1),           0xB7,          //   USAGE (Stop)                ; bit 2: 4
  USAGE(1),           0xCD,          //   USAGE (Play/Pause)          ; bit 3: 8
  USAGE(1),           0xE2,          //   USAGE (Mute)                ; bit 4: 16
  USAGE(1),           0xE9,          //   USAGE (Volume Increment)    ; bit 5: 32
  USAGE(1),           0xEA,          //   USAGE (Volume Decrement)    ; bit 6: 64
  HIDINPUT(1),        0x02,          //   INPUT (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
  END_COLLECTION(0)                  // END_COLLECTION
};

MusicRemote::MusicRemote(std::string deviceName, std::string deviceManufacturer, uint8_t batteryLevel) 
    : deviceName(deviceName),
      deviceManufacturer(deviceManufacturer),
      batteryLevel(batteryLevel),
      reconnectTaskHandle(nullptr),
      hid(nullptr)
{
}

void MusicRemote::begin(void)
{
  ESP_LOGI(LOG_TAG, "========================================");
  ESP_LOGI(LOG_TAG, "Initializing BLE with auto-reconnect...");
  ESP_LOGI(LOG_TAG, "========================================");
  
  NimBLEDevice::init(deviceName);
  
  // RECONNECT FIX 1: Security settings for reliable bonding
  // BLE_HS_IO_NO_INPUT_OUTPUT = "Just Works" pairing (no PIN needed)
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
  
  // RECONNECT FIX 2: Enable bonding with secure connections
  // bonding=true: Store pairing info permanently in NVS
  // MITM=false: Don't require PIN/passkey (accessibility!)
  // secure_conn=true: Use modern BLE 4.2+ encryption
  NimBLEDevice::setSecurityAuth(true, false, true);
  
  this->pServer = NimBLEDevice::createServer();
  this->pServer->setCallbacks(this);

  hid = new NimBLEHIDDevice(pServer);
  inputMediaKeys = hid->getInputReport(MEDIA_KEYS_ID);
  
  hid->setManufacturer(deviceManufacturer);
  hid->setPnp(0x02, 0xe502, 0xa111, 0x0210);
  hid->setHidInfo(0x00, 0x01);
  hid->setReportMap((uint8_t*)_hidReportDescriptor, sizeof(_hidReportDescriptor));
  hid->startServices();

  this->pAdvertising = pServer->getAdvertising();
  this->pAdvertising->setAppearance(HID_KEYBOARD);
  this->pAdvertising->addServiceUUID(hid->getHidService()->getUUID());
  
  // RECONNECT FIX 3: Check bond status
  int bondedCount = NimBLEDevice::getNumBonds();
  ESP_LOGI(LOG_TAG, "Bonded devices: %d", bondedCount);
  
  if (bondedCount > 0) {
    // RECONNECT FIX 4: Log bonded device for debugging
    NimBLEAddress bondedAddr = NimBLEDevice::getBondedAddress(0);
    ESP_LOGI(LOG_TAG, "Will reconnect to: %s", bondedAddr.toString().c_str());
  } else {
    ESP_LOGI(LOG_TAG, "No bonds - ready for initial pairing");
  }
  
  // RECONNECT FIX 5: Use general connectable advertising
  // (Directed advertising has issues on iOS, general is more reliable)
  //pAdvertising->setAdvertisementType(BLE_GAP_CONN_MODE_UND);
  
  // RECONNECT FIX 6: Fast advertising for quick reconnect
  // Intervals in 0.625ms units: 0x20=20ms, 0x40=40ms
  pAdvertising->setMinInterval(0x20);  // 20ms
  pAdvertising->setMaxInterval(0x40);  // 40ms
  
  // RECONNECT FIX 7: Enable scan response
  //pAdvertising->setScanResponse(true);
  
  pAdvertising->start();
  hid->setBatteryLevel(batteryLevel);
  
  // RECONNECT FIX 8: Start watchdog task
  xTaskCreate(
    reconnectTaskWrapper,  // Static wrapper function
    "ble_reconnect",
    4096,
    this,                  // Pass 'this' pointer as parameter
    1,
    &reconnectTaskHandle
  );
  
  ESP_LOGI(LOG_TAG, "✓ BLE initialized and advertising");
  ESP_LOGI(LOG_TAG, "========================================");
}

void MusicRemote::end(void)
{
  if (reconnectTaskHandle) {
    vTaskDelete(reconnectTaskHandle);
    reconnectTaskHandle = nullptr;
  }
  
  if (pAdvertising) {
    pAdvertising->stop();
  }
  
  NimBLEDevice::deinit(true);
}

bool MusicRemote::isConnected(void) const {
  return connected;
}

void MusicRemote::setBatteryLevel(uint8_t level) {
  this->batteryLevel = level;
  if (hid != 0)
    this->hid->setBatteryLevel(this->batteryLevel);
}

void MusicRemote::sendReport(MediaKeyReport* keys)
{
  if (this->isConnected())
  {
    this->inputMediaKeys->setValue((uint8_t*)keys, sizeof(MediaKeyReport));
    this->inputMediaKeys->notify();
  } else {
    ESP_LOGW(LOG_TAG, "Cannot send - not connected");
  }
}

void MusicRemote::onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) {
    connected = true;
    connectionFailures = 0;
    
    ESP_LOGI(LOG_TAG, "✓ Connected: %s", connInfo.getAddress().toString().c_str());
    
    // RECONNECT FIX 9: Update connection params for low latency
    // Args: handle, min_interval, max_interval, latency, timeout
    // Intervals in 1.25ms units, timeout in 10ms units
    pServer->updateConnParams(
      connInfo.getConnHandle(),
      12,   // min: 15ms
      24,   // max: 30ms  
      0,    // latency: 0
      400   // timeout: 4s
    );
    
    // RECONNECT FIX 10: Stop advertising when connected
    if (pAdvertising->isAdvertising()) {
      pAdvertising->stop();
    }
    
    if (connectCallback) connectCallback();
}

void MusicRemote::onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) {
    connected = false;
    
    ESP_LOGI(LOG_TAG, "✗ Disconnected, reason: %d", reason);
    
    // RECONNECT FIX 11: Log disconnect reason
    switch(reason) {
      case BLE_ERR_CONN_TERM_MIC:
        ESP_LOGI(LOG_TAG, "  (Connection TERM MIC - )");
        break;
      case BLE_ERR_REM_USER_CONN_TERM:
        ESP_LOGI(LOG_TAG, "  (Remote user terminated)");
        break;
      case BLE_ERR_CONN_ESTABLISHMENT:
        ESP_LOGI(LOG_TAG, "  (Connection establishment failed)");
        connectionFailures++;
        break;
      default:
        ESP_LOGI(LOG_TAG, "  (Other: 0x%02X)", reason);
        break;
    }
    
    if (disconnectCallback) disconnectCallback();
    
    // RECONNECT FIX 12: Exponential backoff on connection failures
    uint32_t delay_ms = 0;
    if (connectionFailures > 0) {
      delay_ms = 100 << (connectionFailures - 1);
      if (delay_ms > 1000) delay_ms = 1000;
      ESP_LOGI(LOG_TAG, "Connection failures: %d, waiting %dms", connectionFailures, delay_ms);
    }
    
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
    
    // RECONNECT FIX 13: Restart advertising
    if (!pAdvertising->isAdvertising()) {
      pAdvertising->start();
      ESP_LOGI(LOG_TAG, "✓ Restarted advertising");
    }
}

void MusicRemote::onConnect(Callback cb) {
    connectCallback = cb;
}

void MusicRemote::onDisconnect(Callback cb) {
    disconnectCallback = cb;
}

void MusicRemote::onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) {
  uint8_t* value = (uint8_t*)(pCharacteristic->getValue().c_str());
  ESP_LOGD(LOG_TAG, "Write received: %d", *value);
}

void MusicRemote::onAuthenticationComplete(NimBLEConnInfo& connInfo) {
  // RECONNECT FIX 14: Verify pairing succeeded
  if (!connInfo.isEncrypted()) {
    ESP_LOGE(LOG_TAG, "✗ Pairing failed - not encrypted!");
    pServer->disconnect(connInfo.getConnHandle());
  } else {
    ESP_LOGI(LOG_TAG, "✓ Pairing complete - bonded and encrypted");
    
    // Wait a moment for NVS write to complete
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Verify bond was actually saved
    int bondCount = NimBLEDevice::getNumBonds();
    ESP_LOGI(LOG_TAG, "  Bond count after pairing: %d", bondCount);
    
    if (bondCount > 0) {
      NimBLEAddress addr = NimBLEDevice::getBondedAddress(0);
      ESP_LOGI(LOG_TAG, "  ✓ Bond saved successfully!");
      ESP_LOGI(LOG_TAG, "  Bonded to: %s", addr.toString().c_str());
      ESP_LOGI(LOG_TAG, "  Device will now auto-reconnect on power cycle");
    } else {
      ESP_LOGE(LOG_TAG, "  ✗✗✗ BOND NOT SAVED! ✗✗✗");
      ESP_LOGE(LOG_TAG, "  NVS storage error - check NVS initialization");
      ESP_LOGE(LOG_TAG, "  Device will NOT auto-reconnect after reboot!");
    }
  }
}

// RECONNECT FIX 15: Static wrapper for FreeRTOS task
void MusicRemote::reconnectTaskWrapper(void* param) {
  MusicRemote* remote = static_cast<MusicRemote*>(param);
  remote->reconnectTask();
}

// RECONNECT FIX 16: Watchdog task ensures advertising stays active
void MusicRemote::reconnectTask() {
  ESP_LOGI(LOG_TAG, "Reconnect watchdog started");
  
  const TickType_t CHECK_INTERVAL = pdMS_TO_TICKS(5000);
  
  while (true) {
    vTaskDelay(CHECK_INTERVAL);
    
    // RECONNECT FIX 16: Auto-restart advertising if stopped
    if (!connected && !pAdvertising->isAdvertising()) {
      ESP_LOGW(LOG_TAG, "⚠ Not advertising - restarting");
      pAdvertising->start();
    }
    
    // RECONNECT FIX 17: Reset failure counter after stable connection
    if (connected && connectionFailures > 0) {
      static uint32_t connectedTime = 0;
      connectedTime += 5;
      
      if (connectedTime >= 30) {
        connectionFailures = 0;
        connectedTime = 0;
        ESP_LOGI(LOG_TAG, "Connection stable - reset failure counter");
      }
    }
  }
}

size_t MusicRemote::press(const MediaKeyReport k)
{
    uint16_t k_16 = k[1] | (k[0] << 8);
    uint16_t mediaKeyReport_16 = _mediaKeyReport[1] | (_mediaKeyReport[0] << 8);

    mediaKeyReport_16 |= k_16;
    _mediaKeyReport[0] = (uint8_t)((mediaKeyReport_16 & 0xFF00) >> 8);
    _mediaKeyReport[1] = (uint8_t)(mediaKeyReport_16 & 0x00FF);

    sendReport(&_mediaKeyReport);
    return 1;
}

size_t MusicRemote::release(const MediaKeyReport k)
{
    uint16_t k_16 = k[1] | (k[0] << 8);
    uint16_t mediaKeyReport_16 = _mediaKeyReport[1] | (_mediaKeyReport[0] << 8);
    mediaKeyReport_16 &= ~k_16;
    _mediaKeyReport[0] = (uint8_t)((mediaKeyReport_16 & 0xFF00) >> 8);
    _mediaKeyReport[1] = (uint8_t)(mediaKeyReport_16 & 0x00FF);

    sendReport(&_mediaKeyReport);
    return 1;
}

size_t MusicRemote::write(const MediaKeyReport c)
{
    uint16_t p = press(c);
    vTaskDelay(3);
    release(c);
    return p;
}