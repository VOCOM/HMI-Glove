#include "mpu6050.hpp"

// Public Interface

MPU6050::MPU6050(i2c_inst_t* bus) : bus(bus) {
	Reset();
	SetAccelerationResolution(0);
	SetGryoscopeResolution(0);
}

void MPU6050::Reset() {
	// Reset MPU6050
	WriteRegister(REGISTER_POWER_MGMT, 0x80);

	// Wake MPU6050
	WriteRegister(REGISTER_POWER_MGMT, 0x00);
}
void MPU6050::Calibrate() {
	calibrate = true;

	int sampleCount = 51;
	Vector3 avgAcc, avgGyr;
	for (int i = 0; i < sampleCount; i++) {
		Update();
		avgAcc += Acceleration;
		avgGyr += Gyroscope;
	}
	offsetGyro  = avgGyr / sampleCount;
	offsetAccel = avgAcc / sampleCount;
	offsetAccel.z -= 1; // Gravity

	calibrate = false;
}
void MPU6050::Update() {
	Vector3 old_accel = Acceleration;
	Vector3 old_gyro  = Gyroscope;

	ReadRegister(REGISTER_MEASUREMENTS, 14);

	Acceleration.x = ((short)(buffer[0] << 8 | buffer[1])) / resolutionAccel;
	Acceleration.y = ((short)(buffer[2] << 8 | buffer[3])) / resolutionAccel;
	Acceleration.z = ((short)(buffer[4] << 8 | buffer[5])) / resolutionAccel;

	Temperature = ((int16_t)(buffer[6] << 8 | buffer[7]) / 340.0) + 36.53;

	Gyroscope.x = ((short)(buffer[8] << 8 | buffer[9])) / resolutionGyro;
	Gyroscope.y = ((short)(buffer[10] << 8 | buffer[11])) / resolutionGyro;
	Gyroscope.z = ((short)(buffer[12] << 8 | buffer[13])) / resolutionGyro;

	if (calibrate) return;
	Acceleration -= offsetAccel;
	Gyroscope -= offsetGyro;
	Gyroscope *= DEG2RAD;

	// Low Pass Filter [Complementary Filter]
	Acceleration = CF(Acceleration, old_accel, alphaAccel);
	Gyroscope    = CF(Gyroscope, old_gyro, alphaGyro);
}

void MPU6050::SetAccelerationResolution(uint level) {
	if (level > 3) level = 3;
	WriteRegister(REGISTER_CONFIG_ACCEL, level << 3);
	resolutionAccel = 16384.0f / (1 << level);
}
void MPU6050::SetGryoscopeResolution(uint level) {
	if (level > 3) level = 3;
	WriteRegister(REGISTER_CONFIG_GYRO, level << 3);
	resolutionGyro = 131.0 / (1 << level);
}

// Protected Interface

void MPU6050::WriteRegister(uint8_t address, uint8_t data) {
	buffer[0] = address;
	buffer[1] = data;
	i2c_write_blocking(bus, I2C_ADDRESS, buffer, 2, false);
	vTaskDelay(10);
}
void MPU6050::ReadRegister(uint8_t address, uint8_t bytes) {
	buffer[0] = address;
	i2c_write_blocking(bus, I2C_ADDRESS, buffer, 1, true);
	i2c_read_blocking(bus, I2C_ADDRESS, buffer, bytes, false);
}
