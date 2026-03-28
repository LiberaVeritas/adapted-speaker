// BLEHID.h - Auto-Reconnect Only

#ifndef BLEHID_H
#define BLEHID_H

#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEHIDDevice.h>

typedef uint8_t MediaKeyReport[2];

// Media key definitions
const MediaKeyReport KEY_MEDIA_NEXT_TRACK     = {0x00, 0x01};
const MediaKeyReport KEY_MEDIA_PREVIOUS_TRACK = {0x00, 0x02};
const MediaKeyReport KEY_MEDIA_STOP           = {0x00, 0x04};
const MediaKeyReport KEY_MEDIA_PLAY_PAUSE     = {0x00, 0x08};
const MediaKeyReport KEY_MEDIA_MUTE           = {0x00, 0x10};
const MediaKeyReport KEY_MEDIA_VOLUME_UP      = {0x00, 0x20};
const MediaKeyReport KEY_MEDIA_VOLUME_DOWN    = {0x00, 0x40};

typedef std::function<void()> Callback;

class MusicRemote : public NimBLEServerCallbacks, public NimBLECharacteristicCallbacks
{
private:
    std::string deviceName;
    std::string deviceManufacturer;
    uint8_t batteryLevel;
    bool connected = false;
    uint8_t connectionFailures = 0;
    
	TaskHandle_t reconnectTaskHandle = nullptr;
    NimBLEHIDDevice* hid = nullptr;
	NimBLEServer* pServer = nullptr;
    
    NimBLEAdvertising* pAdvertising = nullptr;
    NimBLECharacteristic* inputMediaKeys = nullptr;
	
    
    MediaKeyReport _mediaKeyReport = {0x00, 0x00};
    
    Callback connectCallback = nullptr;
    Callback disconnectCallback = nullptr;
    
    
    
    void sendReport(MediaKeyReport* keys);
    void reconnectTask();
    
    // Static wrapper for FreeRTOS task
    static void reconnectTaskWrapper(void* param);
    
public:
    MusicRemote(std::string deviceName = "Music Remote", 
                std::string deviceManufacturer = "Espressif", 
                uint8_t batteryLevel = 100);
    
    void begin(void);
    void end(void);
    bool isConnected(void) const;
    void setBatteryLevel(uint8_t level);
    
    // Media key functions
    size_t press(const MediaKeyReport k);
    size_t release(const MediaKeyReport k);
    size_t write(const MediaKeyReport k);
    
    // Callback registration
    void onConnect(Callback cb);
    void onDisconnect(Callback cb);
    
    // NimBLE callbacks
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override;
    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override;
    void onAuthenticationComplete(NimBLEConnInfo& connInfo) override;
    void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override;
};

#endif // BLEHID_H