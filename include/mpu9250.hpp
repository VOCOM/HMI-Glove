#ifndef MPU9250_HPP
#define MPU9250_HPP

// Previous Generation
#include "mpu6050.hpp"

/**
 * @brief MPU9250 Wrapper
 * Frame axis
 * ENU [East  North Up  ] (Accelerometer & Gyroscope)
 * NED [North East  Down] (Magnetometer)
 *
 * Variance:
 *
 * Gyroscope
 * 0: +- 250 deg/s | 131.0 LSB/deg/s
 * 1: +- 500 deg/s |  65.5 LSB/deg/s
 * 2: +-1000 deg/s |  32.8 LSB/deg/s
 * 3: +-2000 deg/s |  16.4 LSB/deg/s
 *
 * Accelerometer
 * 0: +- 2 g | 16384 LSB/g
 * 1: +- 4 g |  8192 LSB/g
 * 2: +- 8 g |  4096 LSB/g
 * 3: +-16 g |  2048 LSB/g
 *
 * Magnetometer
 * +- 4800 uT | 0.6[12-bit] / 0.15[16-bit] uT/LSB
 */
class MPU9250 : public MPU6050 {
public:
	MPU9250(i2c_inst_t* bus);

	void Update();

	Vector3 Magnetometer;

protected:
	// Addresses
	static const int I2C_ADDRESS_MAG           = 0x0C;
	static const int REGISTER_STATUS_1_MAG     = 0x02;
	static const int REGISTER_MEASUREMENTS_MAG = 0x03;
	static const int REGISTER_CONTROL_MAG      = 0x0A;
	static const int REGISTER_ASA_MAG          = 0x10;

private:
	void WriteMagnetometer(uint8_t address, uint8_t data);
	void ReadMagnetometer(uint8_t address, uint8_t bytes);

	uint8_t buffer[7];

	Vector3 asa;
};

#endif /* MPU9250 */
