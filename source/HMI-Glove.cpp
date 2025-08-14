/**
 * @file HMI-Glove.cpp
 * @author Muhd Syamim (Syazam33@gmail.com)
 * @brief
 * @version 0.1
 * @date 2025-05-29
 *
 * @copyright Copyright (c) 2025
 *
 * Sensor List
 * 0: Palm
 * 1: Thumb
 * 2: Index
 * 3: Middle
 * 4: Ring
 * 5: Pinky
 */

// C++
#include <new>

// PICO C
#include "pico/stdio.h"
#include "pico/stdlib.h"

// Hardware
#include "hardware/i2c.h"
#include "pico/cyw43_arch.h"

// OS
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

// Network
#include "lwipopts.h"

// Bluetooth
#include "btstack.h"

// Program
#include "bleService.hpp"
#include "icm20948.hpp"
#include "math.hpp"
#include "pca9548a.hpp"

constexpr uint I2C_RATE_HZ{400 * 1000};

void UpdateIMUs(void* param);
void UpdateOdometry(void* param);
void BluetoothService(void* param);
void BluetoothUpdate(void* param);
void Heartbeat(void* param);

static SemaphoreHandle_t mutex_sensors;
static PCA9548A* mux;
static ICM20948* sensors[6];

static SemaphoreHandle_t mutex_ekf;
static EKF ekfs[6];
static Pose poses[6];

static BLEService ble;

/**
 * @brief HMI-Glove entry point
 *
 * @param param
 */
void mainTask(void* param) {
	// Init Wifi/BLE Chip
	cyw43_arch_init();

	// Init BTstack
	l2cap_init();
	sm_init();

	// Init I2C bus
	i2c_init(i2c_default, I2C_RATE_HZ);
	gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
	gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
	gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
	gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);

	// Init Mux
	mux = static_cast<PCA9548A*>(pvPortMalloc(sizeof(PCA9548A)));
	mux = new (mux) PCA9548A(i2c0, 0x70);

	// Init IMUs
	for (int i = 0; i < 6; i++) {
		if (i > 0) continue;

		mux->Select(i);
		sensors[i] = static_cast<ICM20948*>(pvPortMalloc(sizeof(ICM20948)));
		sensors[i] = new (sensors[i]) ICM20948(i2c0);
	}

	xTaskCreate(Heartbeat, "Heartbeat", configMINIMAL_STACK_SIZE, NULL, PRIORITY_IDLE, NULL);
	xTaskCreate(BluetoothService, "Bluetooth", configMINIMAL_STACK_SIZE, NULL, PRIORITY_IDLE, NULL);
	xTaskCreate(UpdateIMUs, "I2C Reader", configMINIMAL_STACK_SIZE, NULL, PRIORITY_IDLE, NULL);
	xTaskCreate(UpdateOdometry, "Kinematics Engine", configMINIMAL_STACK_SIZE, NULL, PRIORITY_IDLE, NULL);

	while (true) {
		vTaskDelay(1000 * portTICK_PERIOD_MS);
	}
}

/**
 * @brief FreeRTOS entry point
 *
 * @return int
 */
int main() {
	stdio_init_all();

	mutex_sensors = xSemaphoreCreateMutex();
	mutex_ekf     = xSemaphoreCreateMutex();

	xTaskCreate(mainTask, "Main Program", configMINIMAL_STACK_SIZE, NULL, PRIORITY_IDLE, NULL);
	vTaskStartScheduler();
}

/**
 * @brief I2C IMU Reader & Fuser
 *
 * @param param
 */
void UpdateIMUs(void* param) {
	while (true) {
		if (xSemaphoreTake(mutex_sensors, 0U) == pdTRUE) {
			for (int i = 0; i < 6; i++) {
				if (i > 0) continue;

				mux->Select(i);

				// Recovery routine
				if (sensors[i]->Error()) {
					sensors[i]->Ping();
					if (sensors[i]->Error()) continue;
					sensors[i]->Reset();
				}

				sensors[i]->Update();
			}
			xSemaphoreGive(mutex_sensors);
		}

		vTaskDelay(10 * portTICK_PERIOD_MS);
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

		if (xSemaphoreTake(mutex_sensors, 0U) == pdTRUE) {
			if (xSemaphoreTake(mutex_ekf, 0U) == pdTRUE) {
				for (int i = 0; i < 6; i++) {
					if (i > 0) continue; // Palm Sensor test

					poses[i].Stale = sensors[i]->Error();
					if (poses[i].Stale) continue;

					// Simple complimentary filter

					// Prediction
					Quaternion& p = poses[i].Orientation;
					UpdatePrediction(poses[i].Orientation, sensors[i]->Gyroscope, dt);

					// Model
					Quaternion m;
					UpdateModel(m, sensors[i]->Acceleration, sensors[i]->Magnetometer);

					// Correct Prediction
					float alpha  = 1.0f - 0.9f;
					Quaternion e = m * p.Conjugate();
					p *= Quaternion(1, e.x * alpha, e.y * alpha, e.z * alpha);

					Vector3 mV = m.ToVector3();
					printf("Model %6.3f %6.3f %6.3f\n", mV.x, mV.y, mV.z);
				}
				xSemaphoreGive(mutex_ekf);
			}
			xSemaphoreGive(mutex_sensors);
		}
		lastTick = currentTick;
		vTaskDelay(10 * portTICK_PERIOD_MS);
	}
}

/**
 * @brief Bluetooth LE Service
 *
 */
void BluetoothService(void* param) {
	ble.Init("HMI Glove R");

	xTaskCreate(BluetoothUpdate, "BLE Update", configMINIMAL_STACK_SIZE, NULL, PRIORITY_IDLE, NULL);

	ble.Start();
}

/**
 * @brief Data Notification Service
 *
 */
void BluetoothUpdate(void* param) {
	while (true) {
		if (xSemaphoreTake(mutex_ekf, 0U) == pdTRUE) {
			ble.Publish(poses, xTaskGetTickCount() * portTICK_PERIOD_MS);

			const Quaternion& p = poses[0].Orientation;
			Vector3 pVec        = p.ToVector3();

			// if (poses[0].Stale) {
			// 	printf("Error Detected\n");
			// } else {
			// 	printf("%6.3f %6.3f %6.3f\n", pVec.x, pVec.y, pVec.z);
			// }
			xSemaphoreGive(mutex_ekf);
		}
		vTaskDelay(20 * portTICK_PERIOD_MS);
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
		vTaskDelay(1000 * portTICK_PERIOD_MS);
	}
}
