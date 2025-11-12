#ifndef ESP32_MUSIC_REMOTE_H
#define ESP32_MUSIC_REMOTE_H
#include "NimBLEAdvertising.h"
#include "sdkconfig.h"
#if defined(CONFIG_BT_ENABLED)
//#include "nimconfig.h"
#if defined(CONFIG_BT_NIMBLE_ROLE_PERIPHERAL)

#include <NimBLEServer.h>
#include <NimBLECharacteristic.h>
#include <NimBLEHIDDevice.h>
#include <functional>


typedef uint8_t MediaKeyReport[2];

const MediaKeyReport KEY_MEDIA_NEXT_TRACK = {1, 0};
const MediaKeyReport KEY_MEDIA_PREVIOUS_TRACK = {2, 0};
const MediaKeyReport KEY_MEDIA_STOP = {4, 0};
const MediaKeyReport KEY_MEDIA_PLAY_PAUSE = {8, 0};
const MediaKeyReport KEY_MEDIA_MUTE = {16, 0};
const MediaKeyReport KEY_MEDIA_VOLUME_UP = {32, 0};
const MediaKeyReport KEY_MEDIA_VOLUME_DOWN = {64, 0};
const MediaKeyReport KEY_MEDIA_CONSUMER_CONTROL_CONFIGURATION = {0, 64}; // Media Selection


class MusicRemote : public NimBLEServerCallbacks, NimBLECharacteristicCallbacks
{
public:
  using Callback = std::function<void(void)>;
  NimBLEAdvertising *pAdvertising;

public:
  MusicRemote(std::string deviceName = "Music Remote", std::string deviceManufacturer = "Espressif", uint8_t batteryLevel = 100);
  void begin(void);
  void end(void);
  void sendReport(MediaKeyReport* keys);
  size_t press(const MediaKeyReport k);
  size_t release(const MediaKeyReport k);
  size_t write(const MediaKeyReport c);
  //size_t write(const uint8_t *buffer, size_t size);
  void releaseAll(void);
  bool isConnected(void) const;
  void setBatteryLevel(uint8_t level);
  void onConnect(Callback cb);
  void onDisconnect(Callback cb);

protected:
  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override;
  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override;
  virtual void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override;

protected:
  NimBLEHIDDevice*      hid;
//  NimBLECharacteristic* outputKeyboard;
  NimBLECharacteristic* inputMediaKeys;
  MediaKeyReport _mediaKeyReport;

  uint8_t batteryLevel;
  std::string deviceManufacturer;
  std::string deviceName;
  bool connected = false;

  NimBLEServer *pServer;

  Callback connectCallback    = nullptr;
  Callback disconnectCallback = nullptr;
};

#endif // CONFIG_BT_NIMBLE_ROLE_PERIPHERAL
#endif // CONFIG_BT_ENABLED
#endif // ESP32_MUSIC_REMOTE_H