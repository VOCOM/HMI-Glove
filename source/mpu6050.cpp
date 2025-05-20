#include "mpu6050.hpp"

/**
 * ID: Range | Resolution
 * Force
 * 0: +- 2 g | 16384 LSB/g
 * 1: +- 4 g |  8192 LSB/g
 * 2: +- 8 g |  4096 LSB/g
 * 3: +-16 g |  2048 LSB/g
 *
 * Angular
 * 0: +- 250 deg/s | 131.0 LSB/deg/s
 * 1: +- 500 deg/s |  65.5 LSB/deg/s
 * 2: +-1000 deg/s |  32.8 LSB/deg/s
 * 3: +-2000 deg/s |  16.4 LSB/deg/s
 */

// Public Interface
MPU6050::MPU6050(i2c_inst_t* bus) : bus(bus) {
	Reset();
	SetAccelerationResolution(0);
	SetGryoscopeResolution(0);
}
void MPU6050::Reset() {
	// Reset MPU6050
	WriteRegister(REGISTER_PWR_MGMT, 0x80);

	// Wake MPU6050
	WriteRegister(REGISTER_PWR_MGMT, 0x00);
}
void MPU6050::SetAccelerationResolution(uint level) {
	if (level > 3) level = 3;
	WriteRegister(REGISTER_ACCEL_CONFIG, level << 3);
	RESOLUTION_FORCE = 16384.0f / (1 << level);
}
void MPU6050::SetGryoscopeResolution(uint level) {
	if (level > 3) level = 3;
	WriteRegister(REGISTER_GYRO_CONFIG, level << 3);
	RESOLUTION_ANGULAR = 131.0 / (1 << level);
}
void MPU6050::GetAcceleration() {
	ReadRegister(REGISTER_ACCEL);
	Acceleration.x = ((short)(buffer[0] << 8 | buffer[1])) / RESOLUTION_FORCE;
	Acceleration.y = ((short)(buffer[2] << 8 | buffer[3])) / RESOLUTION_FORCE;
	Acceleration.z = ((short)(buffer[4] << 8 | buffer[5])) / RESOLUTION_FORCE;
}
void MPU6050::GetGryoscope() {
	ReadRegister(REGISTER_GYRO);
	Gyroscope.x = ((short)(buffer[0] << 8 | buffer[1])) / RESOLUTION_ANGULAR;
	Gyroscope.y = ((short)(buffer[2] << 8 | buffer[3])) / RESOLUTION_ANGULAR;
	Gyroscope.z = ((short)(buffer[4] << 8 | buffer[5])) / RESOLUTION_ANGULAR;
}
void MPU6050::GetTemperature() {
	ReadRegister(REGISTER_TEMP);
	Temperature = ((int16_t)(buffer[0] << 8 | buffer[1]) / 340.0) + 36.53;
}
void MPU6050::GetAll() {
	GetAcceleration();
	GetGryoscope();
	GetTemperature();
}

// Private Interface
void MPU6050::WriteRegister(uint8_t address, uint8_t data) {
	buffer[0] = address;
	buffer[1] = data;
	i2c_write_blocking(bus, I2C_ADDRESS, buffer, 2, false);
	vTaskDelay(100);
}
void MPU6050::ReadRegister(uint8_t address) {
	buffer[0] = address;
	i2c_write_blocking(bus, I2C_ADDRESS, buffer, 1, true);
	i2c_read_blocking(bus, I2C_ADDRESS, buffer, 6, false);
}
