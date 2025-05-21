// PICO C Headers
#include "pico/stdio.h"
#include "pico/stdlib.h"

// Hardware Headers
#include "hardware/i2c.h"
#include "pico/cyw43_arch.h"

// OS Headers
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

// Network Headers
#include "lwipopts.h"

// Program Headers
#include "math.hpp"
#include "mpu6050.hpp"

void Heartbeat(void*);
void IMUReader(void*);
void KinematicEngine(void* param);

static SemaphoreHandle_t mutex_odometry;
static Odometry odometry;

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
	xTaskCreate(IMUReader, "I2C Reader", configMINIMAL_STACK_SIZE, NULL, PRIORITY_IDLE, NULL);
	xTaskCreate(KinematicEngine, "Kinematics Engine", configMINIMAL_STACK_SIZE, NULL, PRIORITY_IDLE, NULL);

	while (true) {
		if (xSemaphoreTake(mutex_odometry, 0U) == pdTRUE) {
			// printf("Fused Odometry\n");
			// printf("Linear\n");
			// printf("Acceleration [%6.1f,%6.1f,%6.1f]m/s^2\n",
			//        odometry.Acceleration.x,
			//        odometry.Acceleration.y,
			//        odometry.Acceleration.z);

			// printf("Angular\n");
			// printf("Euler Rate   [%6.1f,%6.1f,%6.1f]rad/s\n", odometry.EulerRate.x, odometry.EulerRate.y, odometry.EulerRate.z);
			// printf("Euler        [%6.1f,%6.1f,%6.1f]rad\n", odometry.Euler.x, odometry.Euler.y, odometry.Euler.z);
			// printf("\n");
			xSemaphoreGive(mutex_odometry);
		}

		vTaskDelay(1000);
	}
}

/**
 * @brief FreeRTOS entry point
 *
 * @return int
 */
int main() {
	stdio_init_all();

	mutex_odometry = xSemaphoreCreateMutex();

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

		if (xSemaphoreTake(mutex_odometry, 0U) == pdTRUE) {
			// #TODO: Sensor frame -> Body frame
			odometry.Acceleration = sensor1.Acceleration;
			odometry.EulerRate    = sensor1.Gyroscope;
			xSemaphoreGive(mutex_odometry);
		}

		vTaskDelay(10);
	}
}

/**
 * @brief Updates Odometry with sensor data
 *
 * @param param
 */
void KinematicEngine(void* param) {
	TickType_t lastTick = xTaskGetTickCount();
	while (true) {
		TickType_t currentTick = xTaskGetTickCount();
		float deltaTime        = (currentTick - lastTick) * portTICK_RATE_MS / 1000.0;
		if (xSemaphoreTake(mutex_odometry, 0U) == pdTRUE) {
			Vector3& euler = odometry.Euler;

			// Angular
			euler += odometry.EulerRate * deltaTime;
			if (euler.z > PI) euler.z = -PI;
			else if (euler.z < -PI) euler.z = PI;

			Vector2 accEuler = EulerFromAccel(odometry.Acceleration);
			euler.x          = EMA(accEuler.x, euler.x, 0.75);
			euler.y          = EMA(accEuler.y, euler.y, 0.75);

			// Linear
			// #TODO: Remove Gravity Vector
			// 1. World frame -> Body frame
			Matrix3x3 bodyFrame{Matrix3x3::FromEuler(odometry.Euler)};
			Vector3 gravity = bodyFrame * G;
			printf("Gravity (Body Frame)\n");
			printf("%6.1f %6.1f %6.1f\n", gravity.x, gravity.y, gravity.z);

			printf("Acceleration (Body Frame)\n");
			printf("%6.1f %6.1f %6.1f\n", odometry.Acceleration.x, odometry.Acceleration.y, odometry.Acceleration.z);
			printf("\n");

			// Truncate
			odometry.Acceleration.Round(3);
			odometry.EulerRate.Round(3);
			odometry.Euler.Round(3);
			xSemaphoreGive(mutex_odometry);
		}

		lastTick = currentTick;
		vTaskDelay(100);
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
