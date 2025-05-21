#include "mpu6050.hpp"

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
void MPU6050::Calibrate() {
	OFFSET_ACCELEROMETER = Vector3();
	OFFSET_GYROSCOPE     = Vector3();

	bool oldState = useFilters;
	useFilters    = false;

	int sampleCount = 15;
	Vector3 avgAcc, avgGyr;
	for (int i = 0; i < sampleCount; i++) {
		Update();
		avgAcc += Acceleration;
		avgGyr += Gyroscope;
	}
	OFFSET_GYROSCOPE     = avgGyr / sampleCount;
	OFFSET_ACCELEROMETER = avgAcc / sampleCount;
	OFFSET_ACCELEROMETER.z -= 1; // Gravity

	useFilters = oldState;
}
void MPU6050::Update() {
	UpdateAcceleration();
	UpdateGryoscope();
	UpdateTemperature();

	// Find Gravity Orientation
}

void MPU6050::SetAccelerationResolution(uint level) {
	if (level > 3) level = 3;
	WriteRegister(REGISTER_ACCEL_CONFIG, level << 3);
	resForce = 16384.0f / (1 << level);
}
void MPU6050::SetGryoscopeResolution(uint level) {
	if (level > 3) level = 3;
	WriteRegister(REGISTER_GYRO_CONFIG, level << 3);
	resAngular = 131.0 / (1 << level);
}

// Private Interface
void MPU6050::UpdateAcceleration() {
	Vector3 old_val = Acceleration;

	ReadRegister(REGISTER_ACCEL);
	Acceleration.x = ((short)(buffer[0] << 8 | buffer[1])) / resForce - OFFSET_ACCELEROMETER.x;
	Acceleration.y = ((short)(buffer[2] << 8 | buffer[3])) / resForce - OFFSET_ACCELEROMETER.y;
	Acceleration.z = ((short)(buffer[4] << 8 | buffer[5])) / resForce - OFFSET_ACCELEROMETER.z;
	Acceleration *= 9.81;

	if (useFilters == false) return;
	Acceleration = EMA(Acceleration, old_val, alphaAccel);
}
void MPU6050::UpdateGryoscope() {
	Vector3 old_val = Gyroscope;

	// North-West-Down -> North-East-Down
	ReadRegister(REGISTER_GYRO);
	Gyroscope.x = ((short)(buffer[0] << 8 | buffer[1])) / resAngular - OFFSET_GYROSCOPE.x;
	Gyroscope.y = -(((short)(buffer[2] << 8 | buffer[3])) / resAngular - OFFSET_GYROSCOPE.y);
	Gyroscope.z = ((short)(buffer[4] << 8 | buffer[5])) / resAngular - OFFSET_GYROSCOPE.z;
	Gyroscope   = Gyroscope / 180.0 * PI;

	if (useFilters == false) return;
	Gyroscope = EMA(Gyroscope, old_val, alphaGyro);
}
void MPU6050::UpdateTemperature() {
	ReadRegister(REGISTER_TEMP);
	Temperature = ((int16_t)(buffer[0] << 8 | buffer[1]) / 340.0) + 36.53;
}

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
