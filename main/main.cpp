/**
 * https://github.com/wakwak-koba/ESP32-NimBLE-Keyboard/tree/master
 */
//#include "BLEHID.h"
#include "BLEHID.cpp"
#include <iostream>

//#include "gpio.cpp"
#include "BLEHID.h"
#include "input.cpp"

MusicRemote remote;


/*void testTask() {
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
*/
void setup() {
  //Serial.begin(115200);
  std::cout << "Starting BLE work!\n";
  remote.onConnect([](){ printf("Connected\n"); });
  remote.onDisconnect([](){ printf("Disconnected\n"); remote.pAdvertising->start(); });
  remote.begin();
}

static void main_task(void* arg)
{
    int64_t last_time = 0;
    const int64_t debounce_us = 150 * 1000; // 100 ms
    gpio_num_t io_num;
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
			//vTaskDelay(1000 / portTICK_PERIOD_MS);
            //printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
			int64_t now = esp_timer_get_time();
	        if (now - last_time < debounce_us) {
	            continue;
	        }
	        last_time = now;
			/*if (!remote.isConnected()) {
	            continue;
	        }*/
			switch(io_num) {
				case INPUT_PLAY_PAUSE:
					remote.write(KEY_MEDIA_PLAY_PAUSE);
					break;
				case INPUT_PREVIOUS_TRACK:
					remote.write(KEY_MEDIA_PREVIOUS_TRACK);
					printf("pressed\n");
					break;
				case INPUT_NEXT_TRACK:
					remote.write(KEY_MEDIA_NEXT_TRACK);
					break;
				default:
					break;
				}

			// held down
	        while (gpio_get_level(io_num) == 0) {
	            vTaskDelay(pdMS_TO_TICKS(50)); // yields CPU
	        }
			printf("released\n");
			vTaskDelay(pdMS_TO_TICKS(150));
		}
    }
}

extern "C" {void app_main(void);}
void app_main(void) {
  setup();
  input_setup();
  xTaskCreate(main_task, "main_task", 2048, NULL, 10, NULL);
  //main_task(0);
/*
  while (1) {
  if(remote.isConnected()) {
    printf("Sending 'Hello world'...\n");

    sleep(5);


    printf("Sending Play/Pause media key...\n");
    remote.write(KEY_MEDIA_PLAY_PAUSE);
    //initGPIO();
	//testTask();
	}

  printf("Waiting 5 seconds...\n");
  sleep(5);
}
*/

//return 0;
}