#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

#include <stdio.h>

int main() {
	stdio_init_all();

	// Initialise the Wi-Fi chip
	if (cyw43_arch_init()) {
		printf("Wi-Fi init failed\n");
		return -1;
	}

	bool state = false;
	while (true) {
		cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, state);
		state ^= true;
		sleep_ms(1000);
	}
}
