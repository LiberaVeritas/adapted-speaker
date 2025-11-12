#include "BLEHID.h"

#include <NimBLEDevice.h>
#include "HIDTypes.h"

#include "sdkconfig.h"

#if defined(CONFIG_ARDUHAL_ESP_LOG)
  #include "esp32-hal-log.h"
  #define LOG_TAG ""
#else
  #include "esp_log.h"
  static const char* LOG_TAG = "NimBLEDevice";
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
  REPORT_COUNT(1),    0x10,          //   REPORT_COUNT (8)
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

MusicRemote::MusicRemote(std::string deviceName, std::string deviceManufacturer, uint8_t batteryLevel) : hid(0)
{
  this->deviceName = deviceName;
  this->deviceManufacturer = deviceManufacturer;
  this->batteryLevel = batteryLevel;
}

void MusicRemote::begin(void)
{
  NimBLEDevice::init(deviceName);
  BLEDevice::setSecurityAuth(true, true, false);

  this->pServer = NimBLEDevice::createServer();
  this->pServer->setCallbacks(this);

  hid        = new NimBLEHIDDevice(pServer);
  inputMediaKeys = hid->getInputReport(MEDIA_KEYS_ID);
  
  hid->setManufacturer(deviceManufacturer);
  hid->setPnp(0x02, 0xe502, 0xa111, 0x0210);
  hid->setHidInfo(0x00, 0x01);
  hid->setReportMap((uint8_t*)_hidReportDescriptor, sizeof(_hidReportDescriptor));
  hid->startServices();

  this->pAdvertising = pServer->getAdvertising();
  this->pAdvertising->setAppearance(HID_KEYBOARD);
  this->pAdvertising->addServiceUUID(hid->getHidService()->getUUID());
  this->pAdvertising->start();
  hid->setBatteryLevel(batteryLevel);
}

void MusicRemote::end(void)
{
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
  }
}

void MusicRemote::onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) {
    connected = true;
    if (connectCallback) connectCallback();
}

void MusicRemote::onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) {
    connected = false;
    if (disconnectCallback) disconnectCallback();
}

void MusicRemote::onConnect(Callback cb) {
    connectCallback = cb;
}

void MusicRemote::onDisconnect(Callback cb) {
    disconnectCallback = cb;
}

void MusicRemote::onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) {
  uint8_t* value = (uint8_t*)(pCharacteristic->getValue().c_str());
  ESP_LOGI(LOG_TAG, "special keys: %d", *value);
}


//uint8_t USBPutChar(uint8_t c);

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
	uint16_t p = press(c);  // Keydown
	vTaskDelay(3);
	release(c);            // Keyup
	return p;              // just return the result of press() since release() almost always returns 1
}
