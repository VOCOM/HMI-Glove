#include <stdio.h>

#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

#include "hardware/i2c.h"

// LWiP
#include "lwipopts.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

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
	int mpu6050_addr = 0x68;
	int16_t accel[3], gyro[3], temp;
	uint8_t buf[6] = {0, 0, 0, 0, 0, 0};

	// Reset MPU6050
	buf[0] = 0x6B;
	buf[1] = 0x80;
	i2c_write_blocking(i2c_default, mpu6050_addr, buf, 2, false);
	vTaskDelay(100);

	// Wake MPU6050
	buf[1] = 0x00;
	i2c_write_blocking(i2c_default, mpu6050_addr, buf, 2, false);
	vTaskDelay(10);

	while (true) {
		// Read data
		buf[0] = 0x3B;
		i2c_write_blocking(i2c_default, mpu6050_addr, buf, 1, true);
		i2c_read_blocking(i2c_default, mpu6050_addr, buf, 6, false);
		for (int i = 0; i < 3; ++i) {
			accel[i] = (buf[i * 2] << 8 | buf[(i * 2) + 1]); // Big Endian
		}

		buf[0] = 0x43;
		i2c_write_blocking(i2c_default, mpu6050_addr, buf, 1, true);
		i2c_read_blocking(i2c_default, mpu6050_addr, buf, 6, false);
		for (int i = 0; i < 3; ++i) {
			gyro[i] = (buf[i * 2] << 8 | buf[(i * 2) + 1]); // Big Endian
		}

		buf[0] = 0x41;
		i2c_write_blocking(i2c_default, mpu6050_addr, buf, 1, true);
		i2c_read_blocking(i2c_default, mpu6050_addr, buf, 2, false);
		temp = (buf[0] << 8 | buf[1]); // Big Endian

		printf("Acc: X = %d Y = %d Z = %d\n", accel[0], accel[1], accel[2]);
		printf("Gyr: X = %d Y = %d Z = %d\n", gyro[0], gyro[1], gyro[2]);
		printf("Tmp: %.2f\n", (temp / 340.0) + 36.53);

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
