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
#include "mpu9250.hpp"

void Heartbeat(void*);
void ReadIMUs(void*);
void UpdateOdometry(void* param);

static SemaphoreHandle_t mutex_odometry;

static Vector3 accelerometer{};
static Vector3 gyroscope{};

static Odometry odom{};
static EKF ekf{};

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
	xTaskCreate(ReadIMUs, "I2C Reader", configMINIMAL_STACK_SIZE, NULL, PRIORITY_IDLE, NULL);
	xTaskCreate(UpdateOdometry, "Kinematics Engine", configMINIMAL_STACK_SIZE, NULL, PRIORITY_IDLE, NULL);

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
void ReadIMUs(void* param) {
	printf("Initializing MPU6050\n");
	MPU9250 sensor1(i2c_default);

	// vTaskDelay(3000);
	// sensor1.Calibrate();
	// printf("Gyro  Offsets %8.3f %8.3f %8.3f\n", sensor1.OFFSET_GYROSCOPE.x, sensor1.OFFSET_GYROSCOPE.y, sensor1.OFFSET_GYROSCOPE.z);
	// printf("Accel Offsets %8.3f %8.3f %8.3f\n", sensor1.OFFSET_ACCELEROMETER.x, sensor1.OFFSET_ACCELEROMETER.y, sensor1.OFFSET_ACCELEROMETER.z);

	while (true) {
		sensor1.Update();

		if (xSemaphoreTake(mutex_odometry, 0U) == pdTRUE) {
			gyroscope     = sensor1.Gyroscope;
			accelerometer = sensor1.Acceleration;

			// printf("Gyro  %6.3f %6.3f %6.3f\n", gyroscope.x, gyroscope.y, gyroscope.z);
			// printf("Accel %6.3f %6.3f %6.3f\n", accelerometer.x, accelerometer.y, accelerometer.z);
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
void UpdateOdometry(void* param) {
	TickType_t lastTick = xTaskGetTickCount();
	while (true) {
		TickType_t currentTick = xTaskGetTickCount();
		float dt               = (currentTick - lastTick) * portTICK_RATE_MS / 1000.0;
		if (xSemaphoreTake(mutex_odometry, 0U) == pdTRUE) {

			// SLERP
			odom.Orientation = IntegrateGyro(odom.Orientation, gyroscope, dt);
			odom.Orientation = IntegrateAccel(odom.Orientation, accelerometer, 0.1f);
			printf("SLERP Orientation %6.3f %6.3f %6.3f %6.3f\n", odom.Orientation.w, odom.Orientation.x, odom.Orientation.y, odom.Orientation.z);

			// EKF
			ekf.Update(gyroscope, accelerometer, Vector3(), dt);
			Quaternion q = ekf.GetState();
			// printf("EKF   Orientation %6.3f %6.3f %6.3f %6.3f\n", q.w, q.x, q.y, q.z);

			Vector3 e = q.ToVector3();
			// printf("EKF   Orientation %6.3f %6.3f %6.3f\n", e.x, e.y, e.z);

			// printf("\n");
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
