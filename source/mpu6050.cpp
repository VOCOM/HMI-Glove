#include "mpu6050.hpp"

const Vector3 MPU6050::OFFSET_ACCELEROMETER = {0.06, 0.007, -0.05};
const Vector3 MPU6050::OFFSET_GYROSCOPE     = {-2.104, 1.4, -1.35};

// Public Interface
MPU6050::MPU6050(i2c_inst_t* bus) : bus(bus), alpha(0.5) {
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
void MPU6050::UpdateAcceleration() {
	Vector3 old_val = Acceleration;

	ReadRegister(REGISTER_ACCEL);
	Acceleration.x = ((short)(buffer[0] << 8 | buffer[1])) / RESOLUTION_FORCE - OFFSET_ACCELEROMETER.x;
	Acceleration.y = ((short)(buffer[2] << 8 | buffer[3])) / RESOLUTION_FORCE - OFFSET_ACCELEROMETER.y;
	Acceleration.z = ((short)(buffer[4] << 8 | buffer[5])) / RESOLUTION_FORCE - OFFSET_ACCELEROMETER.z;

	// Apply Exponential Moving Average Filter
	Acceleration.x = EMA(Acceleration.x, old_val.x, alpha);
	Acceleration.y = EMA(Acceleration.y, old_val.y, alpha);
	Acceleration.z = EMA(Acceleration.z, old_val.z, alpha);
}
void MPU6050::UpdateGryoscope() {
	Vector3 old_val = Gyroscope;

	ReadRegister(REGISTER_GYRO);
	Gyroscope.x = ((short)(buffer[0] << 8 | buffer[1])) / RESOLUTION_ANGULAR - OFFSET_GYROSCOPE.x;
	Gyroscope.y = ((short)(buffer[2] << 8 | buffer[3])) / RESOLUTION_ANGULAR - OFFSET_GYROSCOPE.y;
	Gyroscope.z = ((short)(buffer[4] << 8 | buffer[5])) / RESOLUTION_ANGULAR - OFFSET_GYROSCOPE.z;

	// Apply Exponential Moving Average Filter
	Gyroscope.x = EMA(Gyroscope.x, old_val.x, alpha);
	Gyroscope.y = EMA(Gyroscope.y, old_val.y, alpha);
	Gyroscope.z = EMA(Gyroscope.z, old_val.z, alpha);
}
void MPU6050::UpdateTemperature() {
	ReadRegister(REGISTER_TEMP);
	Temperature = ((int16_t)(buffer[0] << 8 | buffer[1]) / 340.0) + 36.53;
}
void MPU6050::UpdateAll() {
	UpdateAcceleration();
	UpdateGryoscope();
	UpdateTemperature();
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
