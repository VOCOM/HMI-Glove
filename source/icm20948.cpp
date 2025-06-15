#include "icm20948.hpp"

ICM20948::ICM20948(i2c_inst_t* bus) : bus(bus) {
	Reset();

	// Load ID [Should be 0xEA]
	ReadRegister(REGISTER_BANK0_WHO_AM_I, 1);
	ID = buffer[0];

	// Configure Sensors with Digital Low Pass Filters
	ConfigureGyroscope(0, 3);
	ConfigureAccelerometer(0, 3);

	// Enable I2C Master Mode
	WriteRegister(REGISTER_BANK_SELECT, BANK0_IO);
	WriteRegister(REGISTER_BANK0_USER_CONTROL, 0x20);

	// Set I2C Clock to 400kHz
	WriteRegister(REGISTER_BANK_SELECT, BANK3_I2C);
	WriteRegister(REGISTER_BANK3_MASTER_CONTROL, 0x07);

	// Set I2C Slave for read
	WriteRegister(REGISTER_BANK3_SLAVE0_ADDRESS, 0x0C | 0x80);

	// Set I2C Register Mapping
	WriteRegister(REGISTER_BANK3_SLAVE0_REGISTER, 0x11);
	WriteRegister(REGISTER_BANK3_SLAVE0_CONTROL, 0x87);
}

void ICM20948::Reset() {
	WriteRegister(REGISTER_BANK_SELECT, BANK0_IO);
	WriteRegister(REGISTER_BANK0_POWER_MGMT, 0x80);
	vTaskDelay(100 * portTICK_PERIOD_MS);
}
void ICM20948::Calibrate() {
	calibrate = true;

	calibrate = false;
}
void ICM20948::Update() {
	WriteRegister(REGISTER_BANK_SELECT, BANK0_IO);
	ReadRegister(REGISTER_BANK0_MEASUREMENTS, 21);

	Acceleration.x = (int16_t)(buffer[0] << 8 | buffer[1]);
	Acceleration.y = (int16_t)(buffer[2] << 8 | buffer[3]);
	Acceleration.z = (int16_t)(buffer[4] << 8 | buffer[5]);
	Acceleration /= resolutionAccel;

	Gyroscope.x = (int16_t)(buffer[6] << 8 | buffer[7]);
	Gyroscope.y = (int16_t)(buffer[8] << 8 | buffer[9]);
	Gyroscope.z = (int16_t)(buffer[10] << 8 | buffer[11]);
	Gyroscope /= resolutionGyro;

	Temperature = (float)(buffer[12] << 8 | buffer[13]) / 333.84f + 21.0f;

	// Magnetometer data ready flag
	if (!(buffer[14] & 0x01)) return;
	Magnetometer.x = (int16_t)(buffer[15] << 8 | buffer[16]);
	Magnetometer.y = (int16_t)(buffer[17] << 8 | buffer[18]);
	Magnetometer.z = (int16_t)(buffer[19] << 8 | buffer[20]);
	Magnetometer /= resolutionMag;
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
	buffer[0] = regAddress;
	i2c_write_blocking(bus, I2C_ADDRESS, buffer, 1, true);
	i2c_read_blocking(bus, I2C_ADDRESS, buffer, bytes, false);
}
