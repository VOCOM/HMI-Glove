// Hardware Headers
#include "hardware/i2c.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include <stdio.h>

// OS Headers
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

// Network Headers
#include "lwipopts.h"

// Program Headers
#include "mpu6050.hpp"

void Heartbeat(void*);
void IMUReader(void*);

static SemaphoreHandle_t mutex;
static int testVal = 0;

/// @brief HMI-Glove entry point
/// @param param
void mainTask(void* param) {
	// Init WiFi chip
	cyw43_arch_init();
	cyw43_arch_enable_sta_mode();
	cyw43_arch_wifi_connect_timeout_ms("HOMER", "92378736", CYW43_AUTH_WPA3_WPA2_AES_PSK, 10000);

	// Init I2C bus
	i2c_init(i2c_default, 100000);
	gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
	gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
	gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
	gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);

	xTaskCreate(Heartbeat, "Heartbeat", configMINIMAL_STACK_SIZE, NULL, PRIORITY_IDLE, NULL);
	xTaskCreate(IMUReader, "IMU", configMINIMAL_STACK_SIZE, NULL, PRIORITY_IDLE, NULL);

	while (true) {
		vTaskDelay(100);
	}
}

/// @brief FreeRTOS entry point
/// @return
int main() {
	stdio_init_all();

	mutex = xSemaphoreCreateMutex();

	xTaskCreate(mainTask, "Main Program", configMINIMAL_STACK_SIZE, NULL, PRIORITY_IDLE, NULL);
	vTaskStartScheduler();
}

void IMUReader(void* param) {
	printf("Initializing MPU6050\n");
	MPU6050 sensor1(i2c_default);

	while (true) {
		sensor1.GetAll();
		printf("Acc: X = %.1fg Y = %.1fg Z = %.1fg\n", sensor1.Acceleration.x, sensor1.Acceleration.y, sensor1.Acceleration.z);
		printf("Gyr: X = %.1fdeg/s Y = %.1fdeg/s Z = %.1fdeg/s\n", sensor1.Gyroscope.x, sensor1.Gyroscope.y, sensor1.Gyroscope.z);
		printf("Tmp: %.2f\n", sensor1.Temperature);

		vTaskDelay(100);
	}
}

void Heartbeat(void* param) {
	bool state = false;
	while (true) {
		cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, state);
		state ^= true;
		vTaskDelay(1000);
	}
}
