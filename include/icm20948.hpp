#ifndef ICM20948_HPP
#define ICM20948_HPP

// Hardware Headers
#include "hardware/i2c.h"

// OS Headers
#include "FreeRTOS.h"
#include "task.h"

// Program Headers
#include "math.hpp"

class ICM20948 {
public:
	ICM20948(i2c_inst_t* bus);

	void Reset();
	void Calibrate();
	void Update();

	void ConfigureGyroscope(uint8_t rangeLevel, uint8_t filterLevel);
	void ConfigureAccelerometer(uint8_t rangeLevel, uint8_t filterLevel);

	uint8_t ID           = {};
	float Temperature    = {};
	Vector3 Gyroscope    = {};
	Vector3 Acceleration = {};
	Vector3 Magnetometer = {};

private:
	void WriteRegister(uint8_t regAddress, uint8_t value);
	void ReadRegister(uint8_t regAddress, uint8_t bytes);

	i2c_inst_t* bus;
	bool calibrate     = {};
	uint8_t buffer[14] = {};

	// Offsets
	Vector3 offsetGyro  = {}; //{-2.000, 1.414, -1.330};
	Vector3 offsetAccel = {}; //{0.058, 0.006, -0.049};
	Vector3 offsetMag   = {}; //

	// Resolution
	float resolutionGyro  = {};
	float resolutionAccel = {};
	float resolutionMag   = {};

	// Low Pass Filter Gain
	float alphaGyro  = 1.0f; // 0 > a > 1 | LF <-> HF
	float alphaAccel = 1.0f; // 0 > a > 1 | LF <-> HF
	float alphaMag   = 1.0f; // 0 > a > 1 | LF <-> HF

	// Addresses
	static const uint8_t I2C_ADDRESS  = 0x68;
	static const uint8_t BANK0_IO     = 0x00;
	static const uint8_t BANK1_OFFSET = 0x10;
	static const uint8_t BANK2_CONFIG = 0x20;
	static const uint8_t BANK3_I2C    = 0x30;

	// Registers
	static const uint8_t REGISTER_BANK_SELECT           = 0x7F;
	static const uint8_t REGISTER_BANK0_WHO_AM_I        = 0x00;
	static const uint8_t REGISTER_BANK0_USER_CONTROL    = 0x03;
	static const uint8_t REGISTER_BANK0_POWER_MGMT      = 0x06;
	static const uint8_t REGISTER_BANK0_MEASUREMENTS    = 0x2D;
	static const uint8_t REGISTER_BANK2_GYRO_CONFIG1    = 0x01;
	static const uint8_t REGISTER_BANK2_GYRO_CONFIG2    = 0x02;
	static const uint8_t REGISTER_BANK2_ACCEL_CONFIG1   = 0x14;
	static const uint8_t REGISTER_BANK2_ACCEL_CONFIG2   = 0x15;
	static const uint8_t REGISTER_BANK3_MASTER_CONTROL  = 0x01;
	static const uint8_t REGISTER_BANK3_SLAVE0_ADDRESS  = 0x03;
	static const uint8_t REGISTER_BANK3_SLAVE0_REGISTER = 0x04;
	static const uint8_t REGISTER_BANK3_SLAVE0_CONTROL  = 0x05;
};

#endif /* ICM20948 */
