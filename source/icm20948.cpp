#include "icm20948.hpp"

#include <filters.hpp>

ICM20948::ICM20948(i2c_inst_t* bus) : bus(bus) {
	Reset();
	Ping();
}

bool ICM20948::Error() {
	return readError;
}

void ICM20948::Reset() {
	vTaskDelay(1000 * portTICK_PERIOD_MS);

	WriteRegister(REGISTER_BANK_SELECT, BANK0_IO);
	WriteRegister(REGISTER_BANK0_POWER_MGMT, 0x80);
	vTaskDelay(100 * portTICK_PERIOD_MS);

	// Wake
	WriteRegister(REGISTER_BANK0_POWER_MGMT, 0x01);
	vTaskDelay(10 * portTICK_PERIOD_MS);

	// Configure Sensors with Digital Low Pass Filters
	ConfigureGyroscope(0, 3);
	ConfigureAccelerometer(0, 3);

	// Enable I2C Master Mode
	WriteRegister(REGISTER_BANK_SELECT, BANK0_IO);
	WriteRegister(REGISTER_BANK0_USER_CONTROL, 0x20);

	// Set I2C Clock to 400kHz
	WriteRegister(REGISTER_BANK_SELECT, BANK3_I2C);
	WriteRegister(REGISTER_BANK3_MASTER_CONTROL, 0x07);

	// Set Mag Continuous 100Hz Mode
	WriteI2C0(MAG_I2C_ADDRESS, 0x31, 0x08);

	// Map Mag Output to EXT_SLV_SENS_DATA_00 [Register 0x3B]
	WriteRegister(REGISTER_BANK_SELECT, BANK3_I2C);
	WriteRegister(REGISTER_BANK3_SLAVE0_ADDRESS, MAG_I2C_ADDRESS | 0x80);
	WriteRegister(REGISTER_BANK3_SLAVE0_REGISTER, 0x10);
	WriteRegister(REGISTER_BANK3_SLAVE0_CONTROL, 0x09 | 0x80);
	vTaskDelay(10 * portTICK_PERIOD_MS);
}
void ICM20948::Ping() {
	// Load ID [Should be 0xEA]
	ReadRegister(REGISTER_BANK0_WHO_AM_I, 1);
	ID = buffer[0];
}
void ICM20948::Calibrate() {
	calibrate = true;

	Vector3 GyroSum, AccelSum;

	int count = 101;
	for (int i = 0; i < count; i++) {
		Update();
		GyroSum += Gyroscope;
		AccelSum += Acceleration;
		vTaskDelay(100 * portTICK_PERIOD_MS);
	}

	offsetGyro  = GyroSum / count;
	offsetAccel = AccelSum / count;

	calibrate = false;
}
void ICM20948::Update() {
	WriteRegister(REGISTER_BANK_SELECT, BANK0_IO);
	ReadRegister(REGISTER_BANK0_MEASUREMENTS, 23);

	Vector3 rawAccel;
	rawAccel.x = (int16_t)(buffer[0] << 8 | buffer[1]) / resolutionAccel;
	rawAccel.y = (int16_t)(buffer[2] << 8 | buffer[3]) / resolutionAccel;
	rawAccel.z = (int16_t)(buffer[4] << 8 | buffer[5]) / resolutionAccel;

	Vector3 rawGyro;
	rawGyro.x = (int16_t)(buffer[6] << 8 | buffer[7]) / resolutionGyro;
	rawGyro.y = (int16_t)(buffer[8] << 8 | buffer[9]) / resolutionGyro;
	rawGyro.z = (int16_t)(buffer[10] << 8 | buffer[11]) / resolutionGyro;
	rawGyro *= DEG2RAD;

	Temperature = (float)(buffer[12] << 8 | buffer[13]) / 333.84f + 21.0f;

	// Sensor Overflow Guard
	if (!(buffer[14] & 0x02)) {
		Magnetometer.x = (int16_t)(buffer[16] << 8 | buffer[15]);
		Magnetometer.y = (int16_t)(buffer[18] << 8 | buffer[17]);
		Magnetometer.z = (int16_t)(buffer[20] << 8 | buffer[19]);
		Magnetometer *= resolutionMag;

		if (Magnetometer.x > 4912) Magnetometer.x = 4912;
		if (Magnetometer.y > 4912) Magnetometer.y = 4912;
		if (Magnetometer.z > 4912) Magnetometer.z = 4912;
		if (Magnetometer.x < -4912) Magnetometer.x = -4912;
		if (Magnetometer.y < -4912) Magnetometer.y = -4912;
		if (Magnetometer.z < -4912) Magnetometer.z = -4912;
	}

	if (!calibrate) {
		rawAccel -= offsetAccel;
		rawGyro -= offsetGyro;
		Magnetometer -= offsetMag;

		Acceleration = CF(rawAccel, Acceleration, alphaAccel);
		Gyroscope    = CF(rawGyro, Gyroscope, alphaGyro);
	} else {
		Acceleration = rawAccel;
		Gyroscope    = rawGyro;
	}
}

void ICM20948::ConfigureGyroscope(uint8_t rangeLevel, uint8_t filterLevel) {
	if (rangeLevel > 3) rangeLevel = 3;
	if (filterLevel > 7) filterLevel = 7;
	WriteRegister(REGISTER_BANK_SELECT, BANK2_CONFIG);

	uint8_t value = (filterLevel << 3) | (rangeLevel << 1) | 0x01;
	WriteRegister(REGISTER_BANK2_GYRO_CONFIG1, value);
	resolutionGyro = 131.0 / (1 << rangeLevel);
}
void ICM20948::ConfigureAccelerometer(uint8_t rangeLevel, uint8_t filterLevel) {
	if (rangeLevel > 3) rangeLevel = 3;
	if (filterLevel > 7) filterLevel = 7;
	WriteRegister(REGISTER_BANK_SELECT, BANK2_CONFIG);

	uint8_t value = (filterLevel << 3) | (rangeLevel << 1) | 0x01;
	WriteRegister(REGISTER_BANK2_ACCEL_CONFIG1, value);
	resolutionAccel = 16384.0f / (1 << rangeLevel);
}

void ICM20948::WriteRegister(uint8_t regAddress, uint8_t value) {
	buffer[0] = regAddress;
	buffer[1] = value;
	i2c_write_blocking(bus, I2C_ADDRESS, buffer, 2, false);
	vTaskDelay(10 * portTICK_PERIOD_MS);
}
void ICM20948::ReadRegister(uint8_t regAddress, uint8_t bytes) {
	readError = false;
	buffer[0] = regAddress;
	i2c_write_blocking(bus, I2C_ADDRESS, buffer, 1, true);
	int retVal = i2c_read_blocking(bus, I2C_ADDRESS, buffer, bytes, false);
	if (retVal == PICO_ERROR_GENERIC) readError = true;
}

void ICM20948::WriteI2C0(uint8_t address, uint8_t regAddress, uint8_t value) {
	WriteRegister(REGISTER_BANK_SELECT, BANK3_I2C);
	WriteRegister(REGISTER_BANK3_SLAVE0_ADDRESS, address);
	WriteRegister(REGISTER_BANK3_SLAVE0_REGISTER, regAddress);
	WriteRegister(REGISTER_BANK3_SLAVE0_DATA_OUT, value);
	WriteRegister(REGISTER_BANK3_SLAVE0_CONTROL, 0x81);
	vTaskDelay(10 * portTICK_PERIOD_MS);
}
void ICM20948::ReadI2C0(uint8_t address, uint8_t regAddress, uint8_t bytes) {
	if (bytes > 0x0F) bytes = 0x0F;

	WriteRegister(REGISTER_BANK_SELECT, BANK3_I2C);
	WriteRegister(REGISTER_BANK3_SLAVE0_ADDRESS, address | 0x80);
	WriteRegister(REGISTER_BANK3_SLAVE0_REGISTER, regAddress);
	WriteRegister(REGISTER_BANK3_SLAVE0_CONTROL, bytes | 0x80);
	vTaskDelay(10 * portTICK_PERIOD_MS);

	WriteRegister(REGISTER_BANK_SELECT, BANK0_IO);
	ReadRegister(0x3B, bytes);
}
