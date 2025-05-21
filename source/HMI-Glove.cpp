// C Headers
#include <cmath>
#include <cstdio>

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
#include "kinematics.hpp"
#include "mpu6050.hpp"

void Heartbeat(void*);
void IMUReader(void*);

static SemaphoreHandle_t mutex;
static Odometry fused_odom;

/**
 * @brief HMI-Glove entry point
 *
 * @param param
 */
void mainTask(void* param) {
	// Init WiFi chip
	cyw43_arch_init();
	cyw43_arch_enable_sta_mode();
	cyw43_arch_wifi_connect_timeout_ms("HOMER", "92378736", CYW43_AUTH_WPA3_WPA2_AES_PSK, 10000);

	// Init I2C bus
	i2c_init(i2c_default, 400000);
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
			fused_odom.Euler += fused_odom.EulerRate * deltaTime;
			fused_odom.Euler %= PI;
			Vector2 eulerAccel = EulerFromAccel(fused_odom.Acceleration);
			fused_odom.Euler.x = EMA(eulerAccel.x, fused_odom.Euler.x, 0.75);
			fused_odom.Euler.y = EMA(eulerAccel.y, fused_odom.Euler.y, 0.75);

			printf("Fused Odometry\n");
			printf("Linear\n");
			printf("Acceleration [%6.1f,%6.1f,%6.1f]m/s^2\n",
			       fused_odom.Acceleration.x * 9.81,
			       fused_odom.Acceleration.y * 9.81,
			       fused_odom.Acceleration.z * 9.81);

			printf("Angular\n");
			printf("Euler Rate   [%6.1f,%6.1f,%6.1f]rad/s\n", fused_odom.EulerRate.x, fused_odom.EulerRate.y, fused_odom.EulerRate.z);
			printf("Euler        [%6.1f,%6.1f,%6.1f]rad\n", fused_odom.Euler.x, fused_odom.Euler.y, fused_odom.Euler.z);
			printf("\n");
			xSemaphoreGive(mutex);
		}

		lastTick = currentTick;
		vTaskDelay(100);
	}
}

/**
 * @brief FreeRTOS entry point
 *
 * @return int
 */
int main() {
	stdio_init_all();

	mutex = xSemaphoreCreateMutex();

	xTaskCreate(mainTask, "Main Program", configMINIMAL_STACK_SIZE, NULL, PRIORITY_IDLE, NULL);
	vTaskStartScheduler();
}

/**
 * @brief I2C IMU Reader & Fuser
 *
 * @param param
 */
void IMUReader(void* param) {
	printf("Initializing MPU6050\n");
	MPU6050 sensor1(i2c_default);
	// sensor1.Calibrate();

	while (true) {
		sensor1.Update();

		if (xSemaphoreTake(mutex, 0U) == pdTRUE) {
			// #TODO: Apply Frame Transform
			// #TODO: Remove Gravity Vector
			fused_odom.Acceleration = sensor1.Acceleration;
			fused_odom.EulerRate    = sensor1.Gyroscope / 180.0 * PI;
			xSemaphoreGive(mutex);
		}

		vTaskDelay(10);
	}
}

/**
 * @brief Activity Indicator
 *
 * @param param
 */
void Heartbeat(void* param) {
	bool state = false;
	while (true) {
		cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, state);
		state ^= true;
		vTaskDelay(1000);
	}
}
