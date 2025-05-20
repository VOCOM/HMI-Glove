// C Headers
#include <math.h>
#include <stdio.h>

// Hardware Headers
#include "hardware/i2c.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

// OS Headers
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

// Network Headers
#include "lwipopts.h"

// Program Headers
#include "dataTypes.hpp"
#include "filters.hpp"
#include "mpu6050.hpp"

void Heartbeat(void*);
void IMUReader(void*);

static SemaphoreHandle_t mutex;
static Odometry fused_odom;

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

	TickType_t lastTick = xTaskGetTickCount();
	while (true) {
		TickType_t currentTick = xTaskGetTickCount();
		float deltaTime        = (currentTick - lastTick) * portTICK_RATE_MS / 1000.0;

		if (xSemaphoreTake(mutex, 0U) == pdTRUE) {
			printf("Fused Odometry\n");
			printf("Acceleration [%0.1f,%0.1f,%0.1f]m/s^2\n", fused_odom.Acceleration.x, fused_odom.Acceleration.y, fused_odom.Acceleration.z);
			printf("Orientation  [%0.1f,%0.1f,%0.1f]rad\n", fused_odom.Orientation.x, fused_odom.Orientation.y, fused_odom.Orientation.z);
			printf("\n");
			xSemaphoreGive(mutex);
		}

		lastTick = currentTick;
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
		sensor1.UpdateAll();

		// #TODO: Move this into the MPU6050, possibly abstract as static function
		float roll    = atan2f(sensor1.Acceleration.y, sensor1.Acceleration.z);
		float x2      = sensor1.Acceleration.x * sensor1.Acceleration.x;
		float y2      = sensor1.Acceleration.y * sensor1.Acceleration.y;
		float z2      = sensor1.Acceleration.z * sensor1.Acceleration.z;
		float gVector = sqrtf(x2 + y2 + z2);
		float pitch   = asinf(sensor1.Acceleration.x / gVector);

		if (xSemaphoreTake(mutex, 0U) == pdTRUE) {
			fused_odom.Orientation.x = EMA(roll, fused_odom.Orientation.x, 0.5);
			fused_odom.Orientation.y = EMA(pitch, fused_odom.Orientation.y, 0.5);
			// #TODO: Remove Gravity Vector
			fused_odom.Acceleration = EMA(sensor1.Acceleration * 9.81, fused_odom.Acceleration, 0.5);
			xSemaphoreGive(mutex);
		}

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
