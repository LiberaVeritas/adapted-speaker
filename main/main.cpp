/**
 * https://github.com/wakwak-koba/ESP32-NimBLE-Keyboard/tree/master
 */
//#include "BLEHID.h"
#include "BLEHID.cpp"
#include <iostream>

#include "gpio.cpp"

MusicRemote remote;


void testTask() {
    printf("Test interface started\n");
    printf("Commands: p=play, n=next, b=prev, u=volup, d=voldown, t=test, h=help\n");
    
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
            remote.write(KEY_MEDIA_PLAY_PAUSE);
            break;
            
        case 'n':
            printf("TEST: Next track\n");
            remote.write(KEY_MEDIA_NEXT_TRACK);
            break;
            
        case 'b':
            printf("TEST: Previous track\n");
            remote.write(KEY_MEDIA_PREVIOUS_TRACK);
            break;
            
        case 'u':
            printf("TEST: Volume up\n");
            remote.write(KEY_MEDIA_VOLUME_UP);
            break;
            
        case 'd':
            printf("TEST: Volume down\n");
            remote.write(KEY_MEDIA_VOLUME_DOWN);
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

void setup() {
  //Serial.begin(115200);
  std::cout << "Starting BLE work!\n";
  remote.onConnect([](){ printf("Connected\n"); });
  remote.onDisconnect([](){ printf("Disconnected\n"); remote.pAdvertising->start(); });
  remote.begin();
}

extern "C" {void app_main(void);}
void app_main(void) {
  setup();
  while (1) {
  if(remote.isConnected()) {
    printf("Sending 'Hello world'...\n");

    sleep(5);


    printf("Sending Play/Pause media key...\n");
    remote.write(KEY_MEDIA_PLAY_PAUSE);
    initGPIO();
	testTask();
	}

  printf("Waiting 5 seconds...\n");
  sleep(5);
}
//return 0;
}