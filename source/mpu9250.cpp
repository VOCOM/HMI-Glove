#include "mpu9250.hpp"

// Public Interface

MPU9250::MPU9250(i2c_inst_t* bus) : MPU6050(bus) {
	return; // #TODO: Remove when sensors arrive

	// Enable I2C Bypass
	WriteRegister(REGISTER_CONFIG_PIN, 0x02);

	// Obtain sensitivity adjustment in Fuse ROM mode
	WriteMagnetometer(REGISTER_CONTROL_MAG, 0x0F);
	ReadMagnetometer(REGISTER_ASA_MAG, 3);
	asa.x = buffer[0];
	asa.y = buffer[1];
	asa.z = buffer[2];
	asa   = (asa - 128) / 256.0f + 1.0f;

	// Set to continuous mode 100Hz 16-bit
	WriteMagnetometer(REGISTER_CONTROL_MAG, 0x16);
}

void MPU9250::Update() {
	MPU6050::Update();

	return; // #TODO: Remove when sensors arrive

	// Save cycles if data not ready
	ReadMagnetometer(REGISTER_STATUS_1_MAG, 1);
	if (!(buffer[0] & 0x01)) return;

	ReadMagnetometer(REGISTER_MEASUREMENTS_MAG, 7);

	// Flush stale data
	if (!(buffer[6] & 0x08)) return;

	Magnetometer.x = (short)(buffer[1] << 8 | buffer[0]) * asa.x;
	Magnetometer.y = (short)(buffer[3] << 8 | buffer[2]) * asa.y;
	Magnetometer.z = (short)(buffer[5] << 8 | buffer[4]) * asa.z;

	// #TODO: Selectable resolution
	Magnetometer *= 0.15;
}

// Protected Interface

void MPU9250::WriteMagnetometer(uint8_t address, uint8_t data) {
	buffer[0] = address;
	buffer[1] = data;
	i2c_write_blocking(bus, I2C_ADDRESS_MAG, buffer, 2, false);
	vTaskDelay(10);
}
void MPU9250::ReadMagnetometer(uint8_t address, uint8_t bytes) {
	buffer[0] = address;
	i2c_write_blocking(bus, I2C_ADDRESS_MAG, buffer, 1, true);
	i2c_read_blocking(bus, I2C_ADDRESS_MAG, buffer, bytes, false);
}
