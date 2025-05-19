#include <stdio.h>

#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

// LWiP
#include "lwipopts.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

bool state = false;

void toggle(void* param) {
	while (true) {
		cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, state);
		state ^= true;
		vTaskDelay(1000);
	}
}

int main() {
	stdio_init_all();

	// Initialise the Wi-Fi chip
	if (cyw43_arch_init()) {
		printf("Wi-Fi init failed\n");
		return -1;
	}

	cyw43_arch_enable_sta_mode();
	if (cyw43_arch_wifi_connect_timeout_ms("HOMER", "92378736", CYW43_AUTH_WPA3_WPA2_AES_PSK, 10000) != PICO_OK) return 0;

	TaskHandle_t ledHandle = NULL;

	uint32_t status = xTaskCreate(toggle, "Toggle LED", 256, NULL, 1, &ledHandle);

	vTaskStartScheduler();
}
