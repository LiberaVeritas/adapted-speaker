/**
 * https://github.com/wakwak-koba/ESP32-NimBLE-Keyboard/tree/master
 */
//#include "BLEHID.h"
#include "BLEHID.cpp"
#include <cstdint>
#include <iostream>

#include "BLEHID.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "input.cpp"

MusicRemote remote;



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
  const int64_t debounce_us = 50 * 1000; // 100 ms
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
		//printf("pressed play/pause\n");
		break;
	case INPUT_PREVIOUS_TRACK:
		remote.write(KEY_MEDIA_PREVIOUS_TRACK);
		//printf("pressed prev\n");
		break;
	case INPUT_NEXT_TRACK:
		remote.write(KEY_MEDIA_NEXT_TRACK);
		//printf("pressed next\n");
		break;
	default:
		break;
	}

	// held down
    while (gpio_get_level(io_num) == 0) {
        vTaskDelay(pdMS_TO_TICKS(10)); // yields CPU
    }
	//printf("released\n");
	vTaskDelay(pdMS_TO_TICKS(50));
	xQueueReset(gpio_evt_queue);
	}
  }
}


extern "C" {void app_main(void);}
void app_main(void) {
  setup();
  input_setup();

  gpio_isr_handler_add(INPUT_PLAY_PAUSE, gpio_isr_handler, (void*)INPUT_PLAY_PAUSE);
  //hook isr handler for specific gpio pin
  gpio_isr_handler_add(INPUT_NEXT_TRACK, gpio_isr_handler, (void*)INPUT_NEXT_TRACK);

  gpio_isr_handler_add(INPUT_PREVIOUS_TRACK, gpio_isr_handler, (void*)INPUT_PREVIOUS_TRACK);
  
  xTaskCreate(&main_task, "main_task", 2048, NULL,  5, NULL);

}